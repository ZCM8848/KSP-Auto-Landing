"""Polynomial terminal guidance for the final metres of the descent.

The terminal phase is deliberately *not* solved by SCvx: close to the ground the
convex subproblem becomes ill-conditioned and the minimum-throttle
discretisation dominates.  Instead, a cubic position profile drives both
position and velocity to their target values.

For a cubic profile ``r(t) = r0 + v0 t + a2 t² + a3 t³`` that satisfies
``r(T) = rf`` and ``v(T) = vf``, the initial acceleration is

    a(0) = -6 (r0 - rf) / T² - 4 v0 / T - 2 vf / T

(the coefficients 6, 4, 2 are the cubic boundary conditions, not tunables).
The commanded thrust acceleration is ``a(0) - g`` so the vessel follows the
profile under gravity.

The time-to-go ``T`` may be supplied by the caller (e.g. from the trajectory
planner); otherwise it is estimated from the remaining range and closing speed.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from ..types import FlightState
from .plan import GuidanceCommand, TerminalTarget, Vec3

#: Numerical threshold below which the commanded acceleration is treated as
#: zero (m/s²).  Pure floating-point guard, far below any real acceleration.
ACCEL_EPS = 1e-9


@dataclass(frozen=True)
class PolynomialConfig:
    """Policy for the terminal polynomial guidance."""

    min_time_to_go: float = 5.0
    """Lower bound on the time-to-go (s): avoids an unbounded acceleration
    command when the vessel is very close to the target."""

    time_to_go_gain: float = 2.0
    """Multiplier on the range/closing-speed estimate.  A value > 1 makes the
    profile more conservative (slower final approach)."""

    min_closing_speed: float = 1e-3
    """Speed floor (m/s) used when estimating the time-to-go at rest."""


class PolynomialTerminalGuidance:
    """Cubic terminal guidance to a position/velocity target.

    The guidance is stateless; the caller decides when to hand over from the
    planner (typically on a range threshold) and how often to call it.
    """

    def __init__(self, config: PolynomialConfig | None = None) -> None:
        self._config = config if config is not None else PolynomialConfig()

    def command(
        self,
        state: FlightState,
        *,
        target: TerminalTarget,
        gravity: Sequence[float],
        available_thrust: float,
        time_to_go: float | None = None,
    ) -> GuidanceCommand:
        """Return the terminal thrust command for *state*.

        Args:
            state: Current flight snapshot.
            target: Desired terminal position and velocity.
            gravity: Gravitational acceleration vector in the target frame
                (m/s²), e.g. ``(0, 0, -g_local)``.
            available_thrust: Maximum thrust of the active engine set (N).
            time_to_go: Optional explicit time-to-go (s).  When omitted, it is
                estimated from the remaining range and closing speed.
        """
        position = np.asarray(state.position, dtype=float)
        velocity = np.asarray(state.velocity, dtype=float)
        target_position = np.asarray(target.position, dtype=float)
        target_velocity = np.asarray(target.velocity, dtype=float)
        gravity_vec = np.asarray(gravity, dtype=float)

        if time_to_go is None:
            time_to_go = self._estimate_time_to_go(
                position, velocity, target_position, target_velocity
            )
        if not np.isfinite(time_to_go) or time_to_go <= 0.0:
            raise ValueError(f"time_to_go must be positive and finite, got {time_to_go}")

        # Cubic boundary conditions: initial acceleration for r(0)=p, v(0)=v,
        # r(T)=pf, v(T)=vf.
        accel_cmd = (
            -6.0 * (position - target_position) / (time_to_go * time_to_go)
            - 4.0 * velocity / time_to_go
            - 2.0 * target_velocity / time_to_go
        )
        accel_thrust = accel_cmd - gravity_vec
        mass = float(state.mass)
        if mass > 0.0 and available_thrust > 0.0:
            # Allocate the available thrust to vertical braking first; lateral
            # correction only gets the remaining authority.  With a large
            # lateral error the unconstrained 3-D command would tilt the
            # vehicle so far that it stops braking vertically.
            accel_max = float(available_thrust) / mass
            vertical = float(accel_thrust[2])
            if abs(vertical) >= accel_max:
                accel_thrust[2] = math.copysign(accel_max, vertical)
                accel_thrust[0] = 0.0
                accel_thrust[1] = 0.0
            else:
                remaining = math.sqrt(max(accel_max * accel_max - vertical * vertical, 0.0))
                horizontal = accel_thrust[:2]
                horizontal_norm = float(np.linalg.norm(horizontal))
                if horizontal_norm > remaining > 0.0:
                    accel_thrust[:2] = horizontal * (remaining / horizontal_norm)
        magnitude = float(np.linalg.norm(accel_thrust))

        if magnitude <= ACCEL_EPS or available_thrust <= 0.0 or mass <= 0.0:
            coast_nose = (
                _unit(target_velocity)
                if np.linalg.norm(target_velocity) > 0.0
                else (0.0, 0.0, 1.0)
            )
            return GuidanceCommand(
                throttle=0.0,
                nose=coast_nose,
                thrust=(0.0, 0.0, 0.0),
                status="coast",
            )

        unit = accel_thrust / magnitude
        throttle = min(mass * magnitude / float(available_thrust), 1.0)
        force = tuple(float(component) * mass for component in accel_thrust)
        return GuidanceCommand(
            throttle=throttle,
            nose=(float(unit[0]), float(unit[1]), float(unit[2])),
            thrust=(force[0], force[1], force[2]),
            status="terminal",
        )

    def _estimate_time_to_go(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        target_position: np.ndarray,
        target_velocity: np.ndarray,
    ) -> float:
        """Estimate T from **vertical** range and speed.

        The thrust budget is shared between vertical braking and lateral
        correction.  Using the 3-D range with a large lateral offset inflates
        T (the closing speed is mostly vertical), which removes vertical
        braking exactly when it is needed.  The vertical time-to-go keeps the
        descent profile tied to altitude, independent of lateral error.
        """
        altitude = float(position[2] - target_position[2])
        if altitude <= 0.0:
            return self._config.min_time_to_go
        vertical_speed = abs(float(velocity[2] - target_velocity[2]))
        vertical_speed = max(vertical_speed, self._config.min_closing_speed)
        return max(
            self._config.min_time_to_go,
            self._config.time_to_go_gain * altitude / vertical_speed,
        )


def _unit(vector: np.ndarray) -> Vec3:
    """Return *vector* normalised, or local up if it is zero."""
    norm = float(np.linalg.norm(vector))
    if norm <= ACCEL_EPS:
        return (0.0, 0.0, 1.0)
    return (float(vector[0] / norm), float(vector[1] / norm), float(vector[2] / norm))
