"""Immutable trajectory plans and their execution as vessel commands.

A :class:`Plan` is the pure-data hand-off between a trajectory optimiser
(:mod:`recovery.guidance.scvx`) and the control loop.  It stores the node
states/controls in the target frame plus the game time the plan starts at, so
it can be executed without any solver or kRPC object.

:class:`PlanExecutor` turns a plan into a ``(throttle, nose)`` command at the
current time, using first-order-hold interpolation between nodes.  Only the
thrust *vector* is stored; the throttle is reconstructed as
``|T| / available_thrust`` at execution time, so a plan stays valid when the
engine set changes (the caller supplies the current thrust capability).

:func:`plan_to_virtual_control` converts a plan back into the piecewise
constant :class:`VirtualControl` consumed by the high-fidelity predictor, which
is what the model-mismatch corrector uses to re-fly a plan under drag/lift.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

import numpy as np

from ..types import Vector3
from .control import (
    ConstantThrottle,
    ControlSegment,
    FixedNose,
    NoseRule,
    RetrogradeNose,
    Trigger,
    VirtualControl,
)

Vec3 = tuple[float, float, float]

#: Numerical tolerance below which a thrust vector is treated as "engine off".
#: Purely a floating-point guard — not a physical threshold.
THRUST_EPS = 1e-9


@dataclass(frozen=True, eq=False)
class Plan:
    """A time-stamped, immutable trajectory plan in the target frame.

    Attributes:
        t0_ut: Game universal time of node 0 (seconds).  The control loop
            computes ``elapsed = state.ut - t0_ut`` before sampling the plan.
        times: Physical time of each node relative to ``t0_ut`` (s), shape
            ``(N,)``, non-decreasing, starting at 0.
        positions: Node positions in the target frame (m), shape ``(N, 3)``.
        velocities: Node velocities in the target frame (m/s), shape ``(N, 3)``.
        masses: Node masses (kg), shape ``(N,)``.
        thrusts: Node thrust force vectors in the target frame (N), shape
            ``(N, 3)``.  Direction is the thrust (nose) direction; magnitude is
            ``throttle * available_thrust`` at solve time.
        iterations: SCP iterations spent on this plan.
        cost: Solver cost at the final iteration (informational).
        converged: Whether the solver reported convergence for this plan.
    """

    t0_ut: float
    times: np.ndarray = field(repr=False)
    positions: np.ndarray = field(repr=False)
    velocities: np.ndarray = field(repr=False)
    masses: np.ndarray = field(repr=False)
    thrusts: np.ndarray = field(repr=False)
    iterations: int = 0
    cost: float = float("nan")
    converged: bool = False
    attitudes: np.ndarray | None = field(default=None, repr=False)
    """Node body→target quaternions ``(x, y, z, w)``, shape ``(N, 4)``.

    Present for 6-DOF plans; ``None`` for point-mass plans.
    """

    angular_velocities: np.ndarray | None = field(default=None, repr=False)
    """Node body-frame angular velocities (rad/s), shape ``(N, 3)``."""

    torques: np.ndarray | None = field(default=None, repr=False)
    """Node body-frame control torques (N·m), shape ``(N, 3)``."""

    ignition_time: float | None = field(default=None, repr=False)
    """Free-ignition time relative to the plan start (s), when optimized."""

    def __post_init__(self) -> None:
        n = int(np.asarray(self.times).shape[0])
        if n < 2:
            raise ValueError(f"a plan needs at least 2 nodes, got {n}")
        expected: dict[str, tuple[int, ...]] = {
            "positions": (n, 3),
            "velocities": (n, 3),
            "thrusts": (n, 3),
            "masses": (n,),
        }
        for name, shape in expected.items():
            arr = np.asarray(getattr(self, name))
            if arr.shape != shape:
                raise ValueError(f"{name} must have shape {shape}, got {arr.shape}")
        optional: dict[str, tuple[int, ...]] = {
            "attitudes": (n, 4),
            "angular_velocities": (n, 3),
            "torques": (n, 3),
        }
        for name, shape in optional.items():
            value = getattr(self, name)
            if value is not None and np.asarray(value).shape != shape:
                raise ValueError(f"{name} must have shape {shape}, got {np.asarray(value).shape}")
        if not np.all(np.diff(np.asarray(self.times)) >= 0.0):
            raise ValueError("plan times must be non-decreasing")

    @property
    def n(self) -> int:
        """Number of nodes in the plan."""
        return int(np.asarray(self.times).shape[0])

    @property
    def duration(self) -> float:
        """Planned duration from the first to the last node (s)."""
        return float(np.asarray(self.times)[-1])

    def final_position(self) -> np.ndarray:
        """Terminal node position (m) as a fresh array."""
        return np.array(self.positions[-1], dtype=float)

    def final_velocity(self) -> np.ndarray:
        """Terminal node velocity (m/s) as a fresh array."""
        return np.array(self.velocities[-1], dtype=float)

    def _sample_linear(self, values: np.ndarray, t: float) -> np.ndarray:
        times = np.asarray(self.times, dtype=float)
        table = np.asarray(values, dtype=float)
        if table.ndim == 1:
            return np.array([np.interp(t, times, table)], dtype=float)
        return np.array(
            [np.interp(t, times, table[:, i]) for i in range(table.shape[1])],
            dtype=float,
        )

    def thrust_at(self, t: float) -> np.ndarray:
        """Thrust vector at time *t* (s from ``t0_ut``), linearly interpolated.

        Times outside the plan are clamped to the first/last node.
        """
        return self._sample_linear(self.thrusts, float(t))

    def state_at(self, t: float) -> tuple[np.ndarray, np.ndarray, float]:
        """Interpolated ``(position, velocity, mass)`` at time *t*.

        Used for warm-starting the next solve and for diagnostics; the control
        loop itself only needs the thrust.
        """
        pos = self._sample_linear(self.positions, float(t))
        vel = self._sample_linear(self.velocities, float(t))
        mass = float(
            np.interp(
                float(t),
                np.asarray(self.times, dtype=float),
                np.asarray(self.masses, dtype=float),
            )
        )
        return pos, vel, mass

    def attitude_at(self, t: float) -> np.ndarray | None:
        """Interpolated unit quaternion ``(x, y, z, w)`` at *t*, or ``None``."""
        if self.attitudes is None:
            return None
        quaternion = self._sample_linear(self.attitudes, float(t))
        norm = float(np.linalg.norm(quaternion))
        return quaternion / norm if norm > 1e-12 else None

    def attitude_axes(self, t: float) -> tuple[np.ndarray, np.ndarray] | None:
        """Return ``(nose, dorsal)`` unit vectors in the target frame at *t*.

        The body frame matches the KSP vessel frame: the nose is body ``+y``
        and the dorsal axis is body ``-z``.
        """
        quaternion = self.attitude_at(t)
        if quaternion is None:
            return None
        x, y, z, w = quaternion
        rotation = np.array(
            [
                [w * w + x * x - y * y - z * z, 2.0 * (x * y - w * z), 2.0 * (w * y + x * z)],
                [2.0 * (w * z + x * y), w * w - x * x + y * y - z * z, 2.0 * (y * z - w * x)],
                [2.0 * (x * z - w * y), 2.0 * (w * x + y * z), w * w - x * x - y * y + z * z],
            ]
        )
        return rotation[:, 1], -rotation[:, 2]

    def shifted(self, elapsed: float) -> Plan | None:
        """Return the remaining plan starting at *elapsed* seconds.

        The new plan's node 0 is the interpolated state/thrust at *elapsed*;
        later nodes are the original ones, re-based to ``t=0``.  Returns
        ``None`` when *elapsed* is at or beyond the end of the plan.

        This is what the model corrector needs: it re-flies the *remaining*
        control from the current measurement, not the already-executed head.
        """
        elapsed = float(elapsed)
        times = np.asarray(self.times, dtype=float)
        if elapsed <= 0.0:
            return self
        if elapsed >= times[-1]:
            return None
        positions = np.asarray(self.positions, dtype=float)
        velocities = np.asarray(self.velocities, dtype=float)
        masses = np.asarray(self.masses, dtype=float)
        thrusts = np.asarray(self.thrusts, dtype=float)
        keep = times > elapsed

        def _shifted_optional(values: np.ndarray | None) -> np.ndarray | None:
            if values is None:
                return None
            table = np.asarray(values, dtype=float)
            return np.vstack((self._sample_linear(table, elapsed), table[keep]))

        attitudes = _shifted_optional(self.attitudes)
        if attitudes is not None:
            norms = np.linalg.norm(attitudes, axis=1, keepdims=True)
            attitudes = attitudes / np.maximum(norms, 1e-12)
        return Plan(
            t0_ut=self.t0_ut + elapsed,
            times=np.concatenate(([0.0], times[keep] - elapsed)),
            positions=np.vstack((self._sample_linear(positions, elapsed), positions[keep])),
            velocities=np.vstack((self._sample_linear(velocities, elapsed), velocities[keep])),
            masses=np.concatenate(
                ([float(np.interp(elapsed, times, masses))], masses[keep])
            ),
            thrusts=np.vstack((self._sample_linear(thrusts, elapsed), thrusts[keep])),
            iterations=self.iterations,
            cost=self.cost,
            converged=self.converged,
            attitudes=attitudes,
            angular_velocities=_shifted_optional(self.angular_velocities),
            torques=_shifted_optional(self.torques),
            ignition_time=(
                None
                if self.ignition_time is None
                else max(float(self.ignition_time) - elapsed, 0.0)
            ),
        )


@dataclass(frozen=True)
class TerminalTarget:
    """Desired terminal pose in the target frame.

    Attributes:
        position: Touchdown position of the centre of mass (m).
        velocity: Touchdown velocity (m/s); zero for a soft landing.
    """

    position: tuple[float, float, float]
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class GuidanceCommand:
    """One control command produced by a :class:`PlanExecutor`."""

    throttle: float
    """Commanded throttle in ``[0, 1]``."""

    nose: Vec3
    """Unit nose (= thrust) direction in the target frame."""

    thrust: Vec3
    """Commanded thrust vector (N) in the target frame."""

    status: Literal["plan", "coast", "min-throttle-cutoff", "terminal"]
    """Why this command was produced (diagnostics)."""


class PlanExecutor:
    """Convert a :class:`Plan` into commands for the vessel.

    The executor is deliberately stateless: it only needs the plan, the elapsed
    game time, and the *current* thrust capability.  This keeps it usable with
    any plan source and any engine configuration.

    Keyword Args:
        min_throttle: Engine minimum throttle fraction (default ``0.0`` — KSP
            engines accept any throttle).  A commanded thrust below this
            fraction is executed as engine-off, because a real engine cannot
            hold it.  Set it from the vehicle data when applicable.
    """

    def __init__(self, *, min_throttle: float = 0.0) -> None:
        if not 0.0 <= min_throttle <= 1.0:
            raise ValueError(f"min_throttle must be in [0, 1], got {min_throttle}")
        self._min_throttle = float(min_throttle)

    def command(
        self,
        plan: Plan,
        elapsed: float,
        *,
        available_thrust: float,
        coast_nose: Vec3,
    ) -> GuidanceCommand:
        """Return the command for *elapsed* seconds after ``plan.t0_ut``.

        Args:
            plan: The trajectory to execute.
            elapsed: Seconds since ``plan.t0_ut`` (may be negative or beyond the
                plan; values are clamped to the endpoint controls).
            available_thrust: Current maximum thrust of the active engine set
                (N).  ``<= 0`` means the engines cannot fire.
            coast_nose: Nose direction to hold when the plan commands no thrust
                (the caller owns this policy, e.g. retrograde or velocity
                aligned).
        """
        thrust = plan.thrust_at(elapsed)
        magnitude = float(np.linalg.norm(thrust))
        if magnitude <= THRUST_EPS or available_thrust <= 0.0:
            return GuidanceCommand(0.0, coast_nose, (0.0, 0.0, 0.0), "coast")

        throttle = min(magnitude / float(available_thrust), 1.0)
        if throttle < self._min_throttle:
            return GuidanceCommand(0.0, coast_nose, (0.0, 0.0, 0.0), "min-throttle-cutoff")

        unit = thrust / magnitude
        nose = (float(unit[0]), float(unit[1]), float(unit[2]))
        applied_magnitude = throttle * float(available_thrust)
        applied = (
            nose[0] * applied_magnitude,
            nose[1] * applied_magnitude,
            nose[2] * applied_magnitude,
        )
        return GuidanceCommand(throttle, nose, applied, "plan")


def plan_to_virtual_control(
    plan: Plan,
    *,
    max_thrust: float,
    isp: float,
    dry_mass: float,
    coast_nose: NoseRule | None = None,
    terminate_with_coast: bool = True,
) -> VirtualControl:
    """Convert *plan* into the piecewise-constant control used by the predictor.

    Each node becomes one :class:`ControlSegment` active from that node's time.
    The throttle is ``|T_k| / max_thrust`` and the nose is the thrust direction
    (or *coast_nose* when the node commands no thrust).  This is a lossless
    representation of the plan as long as ``max_thrust`` covers every node's
    thrust magnitude.

    Args:
        plan: Trajectory to convert.
        max_thrust: Reference maximum thrust (N) used to normalise throttle.
            Must be at least the largest node thrust; a smaller value is
            rejected rather than silently clipped.
        isp: Specific impulse (s) to use for the mass flow.
        dry_mass: Vessel dry mass (kg), the predictor's mass floor.
        coast_nose: Steering rule for zero-thrust nodes (default retrograde).
        terminate_with_coast: Append a final engine-off segment at the last
            node time, so the predictor does not keep applying the last thrust
            after the plan ends.  Enabled by default for re-flying a plan.
    """
    peak = float(np.max(np.linalg.norm(np.asarray(plan.thrusts, dtype=float), axis=1)))
    if max_thrust <= 0.0:
        raise ValueError(f"max_thrust must be positive, got {max_thrust}")
    if peak > max_thrust * (1.0 + 1e-9):
        raise ValueError(
            f"plan peak thrust {peak:.1f} N exceeds max_thrust {max_thrust:.1f} N"
        )
    coast_rule = coast_nose if coast_nose is not None else RetrogradeNose()
    segments: list[ControlSegment] = []
    times = np.asarray(plan.times, dtype=float)
    for k in range(plan.n):
        thrust = np.asarray(plan.thrusts[k], dtype=float)
        magnitude = float(np.linalg.norm(thrust))
        if magnitude <= THRUST_EPS:
            nose: NoseRule = coast_rule
        else:
            nose = FixedNose(Vector3(*(thrust / magnitude)))
        segments.append(
            ControlSegment(
                throttle=ConstantThrottle(min(magnitude / max_thrust, 1.0)),
                max_thrust=float(max_thrust),
                isp=float(isp),
                nose=nose,
                trigger=None if k == 0 else Trigger.at_time(float(times[k])),
            )
        )
    if terminate_with_coast:
        segments.append(
            ControlSegment(
                throttle=ConstantThrottle(0.0),
                max_thrust=0.0,
                isp=0.0,
                nose=coast_rule,
                trigger=Trigger.at_time(float(times[-1])),
            )
        )
    return VirtualControl(tuple(segments), dry_mass=float(dry_mass))
