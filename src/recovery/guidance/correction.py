"""Predictor-in-the-loop model correction for trajectory plans.

The planner uses a reduced model (point mass + drag).  The real vessel also has
lift, attitude lag, engine transients and atmosphere-table error.  This module
re-flies a plan through the high-fidelity :class:`ControlledPredictor` and
estimates the *constant acceleration bias* that would explain the terminal
mismatch.  The bias is fed back into the planner as a parameter, so the next
plan compensates for the unmodelled dynamics instead of merely reacting to
them.

The estimator is deliberately generic:

* any object implementing :class:`PlantModel` (the predictor) can be used;
* any :class:`Plan` source works — the plan is converted to a piecewise
  constant control with :func:`plan_to_virtual_control`;
* no vehicle constants are baked in: thrust, Isp, dry mass and the target are
  arguments.

For a constant bias ``a`` acting over a flight time ``T``, the terminal errors
are ``Δp = ½ a T²`` and ``Δv = a T``.  Two unbiased estimates are therefore
``2Δp/T²`` and ``Δv/T``; the mean of the two is used.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from ..types import FlightState
from .control import NoseRule, VirtualControl
from .controlled import Trajectory
from .plan import Plan, TerminalTarget, plan_to_virtual_control

#: Numerical floor on the simulated flight time (s) to avoid dividing by zero
#: for a degenerate plan.  Never reached by a real descent.
MIN_FLIGHT_TIME = 1e-6


class PlantModel(Protocol):
    """Structural type for the high-fidelity predictor used by the corrector."""

    def predict_from(
        self,
        state: FlightState,
        control: VirtualControl,
        *,
        dt: float | None = None,
        t_max: float | None = None,
        stop_on_endpoint: bool = False,
    ) -> Trajectory: ...


@dataclass(frozen=True)
class Correction:
    """Result of re-flying a plan through the high-fidelity model."""

    accel_bias: np.ndarray
    """Estimated constant acceleration bias in the target frame (m/s²).

    Add it to the planner dynamics to make the reduced model match the plant.
    """

    position_error: np.ndarray
    """Predicted terminal position minus target position (m)."""

    velocity_error: np.ndarray
    """Predicted terminal velocity minus target velocity (m/s)."""

    flight_time: float
    """Time from the plan start to the predicted touchdown (s)."""

    predicted_position: np.ndarray
    """Predicted touchdown position in the target frame (m)."""

    predicted_velocity: np.ndarray
    """Predicted touchdown velocity in the target frame (m/s)."""


class TerminalCorrector:
    """Estimate the model-mismatch acceleration for a plan.

    Keyword Args:
        coast_nose: Steering rule assumed during zero-thrust nodes.  Use the
            same policy the executor uses on the real vessel, otherwise the
            correction would model a different attitude history.
    """

    def __init__(self, plant: PlantModel, *, coast_nose: NoseRule | None = None) -> None:
        self._plant = plant
        self._coast_nose = coast_nose

    def estimate(
        self,
        plan: Plan,
        state: FlightState,
        *,
        target: TerminalTarget,
        max_thrust: float,
        isp: float,
        dry_mass: float,
    ) -> Correction:
        """Re-fly *plan* from *state* and return the estimated correction.

        The plan must start at the same time as *state* (call this at a planner
        refresh, before the vessel has advanced past ``plan.t0_ut``).
        """
        peak = float(np.max(np.linalg.norm(np.asarray(plan.thrusts, dtype=float), axis=1)))
        effective_max = max(float(max_thrust), peak)
        control = plan_to_virtual_control(
            plan,
            max_thrust=effective_max,
            isp=isp,
            dry_mass=dry_mass,
            coast_nose=self._coast_nose,
        )
        trajectory = self._plant.predict_from(state, control)

        if trajectory.impact is not None:
            predicted_position = np.asarray(trajectory.impact.position, dtype=float)
            flight_time = float(trajectory.impact.time)
        else:
            predicted_position = np.asarray(trajectory.final_position, dtype=float)
            flight_time = float(np.asarray(trajectory.times)[-1])
        predicted_velocity = np.asarray(trajectory.final_velocity, dtype=float)

        position_error = predicted_position - np.asarray(target.position, dtype=float)
        velocity_error = predicted_velocity - np.asarray(target.velocity, dtype=float)
        flight_time = max(flight_time, MIN_FLIGHT_TIME)
        accel_bias = 0.5 * (
            velocity_error / flight_time + 2.0 * position_error / (flight_time * flight_time)
        )
        return Correction(
            accel_bias=accel_bias,
            position_error=position_error,
            velocity_error=velocity_error,
            flight_time=flight_time,
            predicted_position=predicted_position,
            predicted_velocity=predicted_velocity,
        )
