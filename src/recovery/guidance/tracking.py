"""Plan tracking with explicit coast and tilt guards.

The SCvx plan is a trajectory in the target frame.  The controller has two
jobs that are separate from the optimiser:

* follow the plan's thrust, not invent a burn during a planned coast; and
* keep the commanded thrust direction inside the vehicle's tilt envelope.

The OCP constrains the *plan* (``ScvxConfig.pointing_constraint`` and
``max_tilt_deg``).  This module constrains the *command* actually sent to the
vessel, which is what matters when feedback, a stale plan, or a numerical
artefact would otherwise point the engine below the horizon.

:class:`PlanTracker` also implements the receding-horizon policy used by the
scripts: periodically re-seed the SCvx problem from the current measurement,
accept a new plan only when its terminal state is usable, and keep the previous
plan while the solver is refining or when a refresh fails.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from ..types import FlightState
from .plan import Plan
from .scvx import AttitudeModel, EngineModel


@dataclass(frozen=True)
class TrackingConfig:
    """Policy for turning a :class:`Plan` into a vessel command."""

    min_throttle: float = 0.40
    """Engine lower throttle limit."""

    kp_position: float = 0.20
    """Position-error gain (1/s²)."""

    kd_velocity: float = 1.00
    """Velocity-error gain (1/s)."""

    max_feedback: float = 2.0
    """Feedback acceleration limit (m/s²)."""

    max_tilt_deg: float = 30.0
    """Hard command tilt limit from local vertical (deg).

    This mirrors the OCP's pointing constraint; it is a command guard, not a
    flight-termination system.
    """

    light_fraction: float = 1e-3
    """Plan thrust fraction needed to start a burn from a coast.

    For a free-ignition plan this should be the executable engine band (about
    0.4); otherwise a gate tail can make the controller toggle 0/40 % around
    the threshold.  Fixed-ignition plans may use a tiny threshold.
    """

    cutoff_fraction: float = 0.0
    """Plan thrust fraction below which an already-burning engine is cut.

    Once lit, the engine stays lit until the plan drops below this value.
    This hysteresis prevents the 0/40 % chattering seen when a plan's thrust
    fraction hovers around the light threshold.
    """


@dataclass(frozen=True)
class PlanThrust:
    """One plan-tracking command."""

    throttle: float
    nose: tuple[float, float, float]
    desired_position: np.ndarray
    planned_fraction: float
    coast: bool
    tilt_limited: bool


class SolveService(Protocol):
    """The slice of :class:`ScvxSolveService` used by :class:`PlanTracker`."""

    def refresh(
        self,
        booster_id: str,
        state: FlightState,
        *,
        engine: EngineModel,
        tf_hint: float | None = None,
        accel_bias: np.ndarray | None = None,
        attitude: AttitudeModel | None = None,
    ) -> None: ...

    def latest_plan(self, booster_id: str) -> Plan | None: ...


def _unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        return np.array([0.0, 0.0, 1.0], dtype=float)
    return vector / norm


def _as_tuple(vector: np.ndarray) -> tuple[float, float, float]:
    return (float(vector[0]), float(vector[1]), float(vector[2]))


def _clamp_to_cone(direction: np.ndarray, min_z: float) -> np.ndarray:
    """Return *direction* projected into the upward cone ``z >= min_z``."""
    min_z = min(max(float(min_z), -1.0), 1.0)
    unit = _unit(np.asarray(direction, dtype=float))
    if float(unit[2]) >= min_z:
        return unit
    horizontal = np.array([unit[0], unit[1]], dtype=float)
    horizontal_norm = float(np.linalg.norm(horizontal))
    if horizontal_norm < 1e-9:
        return np.array([0.0, 0.0, 1.0], dtype=float)
    horizontal_target = math.sqrt(max(1.0 - min_z * min_z, 0.0))
    return np.array(
        [
            horizontal[0] / horizontal_norm * horizontal_target,
            horizontal[1] / horizontal_norm * horizontal_target,
            min_z,
        ],
        dtype=float,
    )


def plan_thrust(
    plan: Plan,
    state: FlightState,
    elapsed: float,
    *,
    config: TrackingConfig,
    burning: bool = False,
) -> PlanThrust:
    """Track *plan* at *elapsed* seconds with coast and tilt guards.

    The returned direction is never below the horizon and never more than
    ``config.max_tilt_deg`` from local vertical.  When feedback would demand
    such a direction, the command falls back to the plan's own (constrained)
    attitude and only scales thrust along that direction.
    """
    desired_p, desired_v, _ = plan.state_at(elapsed)
    feedforward = np.asarray(plan.thrust_at(elapsed), dtype=float)
    capability = max(float(state.available_thrust), float(state.max_thrust))
    planned_magnitude = float(np.linalg.norm(feedforward))
    planned_fraction = planned_magnitude / capability if capability > 0.0 else 0.0

    axes = plan.attitude_axes(elapsed)
    plan_nose = (
        _unit(np.asarray(axes[0], dtype=float))
        if axes is not None
        else np.array([0.0, 0.0, 1.0], dtype=float)
    )

    # A relative threshold above the sigmoid gate tail, plus hysteresis once
    # the engine is actually lit: do not fall back to a 0/40 % limit cycle
    # when the plan's thrust fraction wanders around the light threshold.
    if burning:
        should_coast = planned_fraction <= config.cutoff_fraction
    else:
        should_coast = planned_fraction < config.light_fraction

    if capability <= 0.0 or should_coast:
        return PlanThrust(
            throttle=0.0,
            nose=_as_tuple(plan_nose),
            desired_position=np.asarray(desired_p, dtype=float),
            planned_fraction=planned_fraction,
            coast=True,
            tilt_limited=False,
        )

    error_p = np.asarray(desired_p, dtype=float) - np.asarray(state.position, dtype=float)
    error_v = np.asarray(desired_v, dtype=float) - np.asarray(state.velocity, dtype=float)
    feedback = config.kp_position * error_p + config.kd_velocity * error_v
    feedback_norm = float(np.linalg.norm(feedback))
    if feedback_norm > config.max_feedback:
        feedback *= config.max_feedback / feedback_norm

    acceleration = feedforward / max(float(state.mass), 1e-3) + feedback
    acceleration_norm = float(np.linalg.norm(acceleration))
    min_z = math.cos(math.radians(config.max_tilt_deg))

    if acceleration_norm <= 1e-9:
        direction = _clamp_to_cone(plan_nose, min_z)
        tilt_limited = False
        magnitude = config.min_throttle * capability
    else:
        candidate = acceleration / acceleration_norm
        if float(candidate[2]) >= min_z:
            direction = candidate
            tilt_limited = False
            magnitude = float(state.mass) * acceleration_norm
        else:
            # Do not invert the engine to chase a coast that the plan did not
            # ask for.  Stay on the plan's attitude and scale thrust along it.
            direction = _clamp_to_cone(plan_nose, min_z)
            along = max(float(np.dot(acceleration, direction)), 0.0)
            magnitude = float(state.mass) * along
            tilt_limited = True

    magnitude = min(
        max(magnitude, config.min_throttle * capability),
        capability,
    )
    return PlanThrust(
        throttle=magnitude / capability,
        nose=_as_tuple(direction),
        desired_position=np.asarray(desired_p, dtype=float),
        planned_fraction=planned_fraction,
        coast=False,
        tilt_limited=tilt_limited,
    )


def plan_is_usable(
    plan: Plan,
    *,
    target_height: float,
    terminal_alt: float = 500.0,
    max_xy_error: float = 100.0,
    max_speed: float = 150.0,
    min_descent_speed: float = 5.0,
    max_below_pad: float = 10.0,
) -> bool:
    """Whether *plan* is a sensible horizon to hand to terminal guidance.

    The SCvx plan is only responsible down to ``target_height + terminal_alt``;
    below that the polynomial terminal law takes over.  Checking the terminal
    *node* is both too strict (a valid plan may continue descending through the
    hand-off point) and too weak (an early iterate can end at the pad while
    still moving tens of m/s).  This checks the crossing of the hand-off
    altitude instead: lateral error, descent rate, speed, and that the plan
    does not cross below the pad.
    """
    if not math.isfinite(float(plan.duration)) or float(plan.duration) <= 1.0:
        return False
    positions = np.asarray(plan.positions, dtype=float)
    velocities = np.asarray(plan.velocities, dtype=float)
    if not (np.all(np.isfinite(positions)) and np.all(np.isfinite(velocities))):
        return False
    heights = positions[:, 2]
    handoff_z = float(target_height) + float(terminal_alt)
    if not (float(np.max(heights)) >= handoff_z >= float(np.min(heights))):
        return False
    below = heights <= handoff_z
    index = int(np.argmax(below))
    if index == 0:
        position = positions[0]
        velocity = velocities[0]
    else:
        z0 = float(heights[index - 1])
        z1 = float(heights[index])
        alpha = (handoff_z - z0) / (z1 - z0) if z1 != z0 else 0.0
        alpha = min(max(alpha, 0.0), 1.0)
        position = positions[index - 1] + alpha * (positions[index] - positions[index - 1])
        velocity = velocities[index - 1] + alpha * (
            velocities[index] - velocities[index - 1]
        )
    return (
        math.hypot(float(position[0]), float(position[1])) <= max_xy_error
        and float(position[2]) >= float(target_height) - max_below_pad
        and float(velocity[2]) <= -min_descent_speed
        and float(np.linalg.norm(velocity)) <= max_speed
    )


class PlanTracker:
    """Receding-horizon plan tracker with periodic SCvx re-seeding.

    The tracker owns the "latest usable plan" policy.  It refreshes the solve
    service from the current measurement every ``mpc_period`` seconds while
    the vehicle is above the terminal hand-off altitude, accepts a newer plan
    only when :func:`plan_is_usable` passes, and keeps the previous plan while
    the solver is refining or a refresh fails.
    """

    def __init__(
        self,
        *,
        booster_id: str,
        service: SolveService,
        tracking: TrackingConfig,
        target_height: float,
        mpc_period: float = 2.0,
        terminal_alt: float = 500.0,
        tf_hint: float | None = None,
        max_plan_age: float = 15.0,
        max_xy_error: float = 100.0,
        max_speed: float = 150.0,
        min_descent_speed: float = 5.0,
        min_plan_iterations: int = 5,
    ) -> None:
        if mpc_period <= 0.0:
            raise ValueError(f"mpc_period must be positive, got {mpc_period}")
        self._booster_id = booster_id
        self._service = service
        self._tracking = tracking
        self._target_height = float(target_height)
        self._mpc_period = float(mpc_period)
        self._terminal_alt = float(terminal_alt)
        self._tf_hint = tf_hint
        self._max_plan_age = float(max_plan_age)
        self._max_xy_error = float(max_xy_error)
        self._max_speed = float(max_speed)
        self._min_descent_speed = float(min_descent_speed)
        self._min_plan_iterations = max(int(min_plan_iterations), 0)
        self._plan: Plan | None = None
        self._started = False
        self._started_ut = 0.0
        self._last_refresh_wall = 0.0
        self._last_accept_wall = 0.0
        self._elapsed = 0.0
        self._replans = 0

    @property
    def plan(self) -> Plan | None:
        """The plan currently being tracked."""
        return self._plan

    @property
    def started_ut(self) -> float:
        """Game time of the first tracker step (the descent hand-off)."""
        return self._started_ut

    @property
    def elapsed(self) -> float:
        """Plan time used for the most recent :meth:`step` (s)."""
        return self._elapsed

    @property
    def replans(self) -> int:
        """Number of plans accepted so far."""
        return self._replans

    @property
    def last_accept_wall(self) -> float:
        """Wall-clock time when the current plan was accepted."""
        return self._last_accept_wall

    def set_tf_hint(self, tf_hint: float | None) -> None:
        """Update the time-of-flight seed used by the next refresh."""
        self._tf_hint = None if tf_hint is None else float(tf_hint)

    def step(
        self,
        state: FlightState,
        *,
        now: float | None = None,
    ) -> PlanThrust | None:
        """Return the next command, re-seeding the solver when due.

        Returns ``None`` until a usable plan exists.  Once a plan is tracked, a
        refresh failure or a non-usable iterate keeps the previous plan; the
        next refresh is still attempted on schedule.
        """
        now_wall = time.monotonic() if now is None else float(now)
        if not self._started:
            self.seed(state, now=now_wall)

        candidate = self._service.latest_plan(self._booster_id)
        if self._plan is None:
            if candidate is not None and self._usable(candidate, state):
                self._accept(candidate, now_wall)
            else:
                return None
        else:
            if self._should_refresh(state) and (
                now_wall - self._last_refresh_wall >= self._mpc_period
            ):
                self._refresh(state)
                self._last_refresh_wall = now_wall
                candidate = self._service.latest_plan(self._booster_id)
            if (
                candidate is not None
                and candidate is not self._plan
                and self._is_newer(candidate)
                and self._usable(candidate, state)
            ):
                self._accept(candidate, now_wall)

        if self._plan is None:  # pragma: no cover - guarded by the branches
            return None
        elapsed = max(0.0, float(state.ut) - float(self._plan.t0_ut))
        if elapsed >= float(self._plan.duration):
            if (
                candidate is not None
                and candidate is not self._plan
                and self._is_newer(candidate)
                and self._usable(candidate, state)
            ):
                self._accept(candidate, now_wall)
                elapsed = max(0.0, float(state.ut) - float(self._plan.t0_ut))
            else:
                raise RuntimeError("SCvx plan expired before the terminal hand-off")
        self._elapsed = elapsed
        return plan_thrust(
            self._plan,
            state,
            elapsed,
            config=self._tracking,
            burning=float(state.throttle) > 0.01,
        )

    def seed(self, state: FlightState, *, now: float | None = None) -> None:
        """Send the first solve request from *state* (idempotent)."""
        if self._started:
            return
        self._started = True
        self._started_ut = float(state.ut)
        self._refresh(state)
        self._last_refresh_wall = time.monotonic() if now is None else float(now)

    def _refresh(self, state: FlightState) -> None:
        self._service.refresh(
            self._booster_id,
            state,
            engine=EngineModel.from_state(state),
            tf_hint=self._tf_hint,
            attitude=AttitudeModel.from_state(state),
        )

    def _should_refresh(self, state: FlightState) -> bool:
        return (
            self._plan is not None
            and float(state.position.z) > self._target_height + self._terminal_alt
        )

    def _is_newer(self, plan: Plan) -> bool:
        # Only a new refresh (a strictly newer t0) should replace the tracked
        # plan.  Intermediate SCP iterates share t0; accepting every one of
        # them makes the command jump on every solver iteration.
        return self._plan is None or float(plan.t0_ut) > float(self._plan.t0_ut)

    def _usable(self, plan: Plan, state: FlightState) -> bool:
        if int(plan.iterations) < self._min_plan_iterations:
            return False
        age = float(state.ut) - float(plan.t0_ut)
        if age < -1.0 or age > self._max_plan_age:
            return False
        return plan_is_usable(
            plan,
            target_height=self._target_height,
            terminal_alt=self._terminal_alt,
            max_xy_error=self._max_xy_error,
            max_speed=self._max_speed,
            min_descent_speed=self._min_descent_speed,
        )

    def _accept(self, plan: Plan, now_wall: float) -> None:
        first = self._plan is None
        self._plan = plan
        self._last_accept_wall = now_wall
        self._replans += 1
        if first:
            # Give the solver a full period from the first usable plan instead
            # of immediately replacing it with the next refresh.
            self._last_refresh_wall = now_wall
