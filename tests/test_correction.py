"""Tests for the predictor-in-the-loop model correction."""

import numpy as np

from recovery.guidance import (
    ControlledPredictor,
    ControlSegment,
    LiftDragModel,
    Plan,
    RetrogradeNose,
    TerminalCorrector,
    TerminalTarget,
    VirtualControl,
)
from recovery.specs import BodySpec, DragSpec
from recovery.types import FlightState, Quaternion, Situation, Vector3

BODY = BodySpec(
    mu=3.5316e12,
    omega=(0.0, 0.0, 0.0),
    body_center=(0.0, 0.0, -600000.0),
    body_radius=600000.0,
    surface_gravity=9.81,
)
ALTS = np.linspace(0.0, 70000.0, 64)
DENSITIES = 1.225 * np.exp(-ALTS / 5600.0)
DENSITIES[-1] = 0.0
DRAG = DragSpec(
    ballistic_coefficient=3000.0,
    density_alts=ALTS,
    density_vals=DENSITIES,
    body_center=(0.0, 0.0, -600000.0),
    sea_level_radius=600000.0,
    cd_area=6.0,
)


def _state() -> FlightState:
    return FlightState(
        ut=100.0,
        met=100.0,
        position=Vector3(0.0, 0.0, 1000.0),
        velocity=Vector3(0.0, 0.0, -100.0),
        velocity_surface=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(0.0, 0.0, 0.0),
        altitude=1000.0,
        surface_altitude=1000.0,
        mass=10000.0,
        dry_mass=2000.0,
        thrust=0.0,
        available_thrust=200000.0,
        max_thrust=200000.0,
        max_vacuum_thrust=220000.0,
        specific_impulse=300.0,
        max_acceleration=20.0,
        throttle=0.0,
        situation=Situation.FLYING,
        loaded=True,
        packed=False,
        landed=False,
        atmosphere_density=0.5,
        atmosphere_depth=70000.0,
        frame=None,
    )


def _ballistic_plan(
    predictor: ControlledPredictor, state: FlightState
) -> tuple[Plan, object, np.ndarray]:
    control = VirtualControl((ControlSegment.coast(nose=RetrogradeNose()),), dry_mass=2000.0)
    trajectory = predictor.predict_from(state, control)
    assert trajectory.impact is not None
    plan = Plan(
        t0_ut=state.ut,
        times=np.asarray(trajectory.times, dtype=float),
        positions=np.asarray(trajectory.positions, dtype=float),
        velocities=np.asarray(trajectory.velocities, dtype=float),
        masses=np.asarray(trajectory.masses, dtype=float),
        thrusts=np.zeros((trajectory.n, 3)),
    )
    return plan, trajectory.impact, np.asarray(trajectory.final_velocity, dtype=float)


def test_correction_is_zero_when_plant_matches_plan() -> None:
    """Re-flying a plan with the same model reproduces it (zero bias)."""
    state = _state()
    predictor = ControlledPredictor.from_body_spec(BODY)
    plan, impact, final_velocity = _ballistic_plan(predictor, state)
    correction = TerminalCorrector(predictor).estimate(
        plan,
        state,
        target=TerminalTarget(tuple(impact.position), tuple(final_velocity)),
        max_thrust=state.available_thrust,
        isp=state.specific_impulse,
        dry_mass=state.dry_mass,
    )
    assert np.linalg.norm(correction.accel_bias) < 1e-6
    assert np.linalg.norm(correction.position_error) < 1e-3
    assert np.linalg.norm(correction.velocity_error) < 1e-3


def test_correction_detects_unmodelled_drag() -> None:
    """Flying a drag-free plan through a drag model yields an upward bias."""
    state = _state()
    predictor = ControlledPredictor.from_body_spec(BODY)
    plan, impact, final_velocity = _ballistic_plan(predictor, state)

    drag_model = LiftDragModel.from_drag_spec(DRAG, mass_ref=state.mass)
    drag_predictor = ControlledPredictor.from_body_spec(BODY, aero=drag_model)
    correction = TerminalCorrector(drag_predictor).estimate(
        plan,
        state,
        target=TerminalTarget(tuple(impact.position), tuple(final_velocity)),
        max_thrust=state.available_thrust,
        isp=state.specific_impulse,
        dry_mass=state.dry_mass,
    )
    # Drag slows the descent, so the plant reaches the ground with a velocity
    # closer to zero than the drag-free plan assumed: the bias points up.
    assert correction.velocity_error[2] > 0.0
    assert correction.accel_bias[2] > 0.0
    assert correction.flight_time > 0.0


def test_correction_clips_plan_peak_thrust() -> None:
    """The corrector uses the plan's peak thrust when it exceeds the engine."""
    state = _state()
    predictor = ControlledPredictor.from_body_spec(BODY)
    plan, impact, final_velocity = _ballistic_plan(predictor, state)
    # Replace the zero-thrust plan with a nonzero one; the engine limit passed
    # in is smaller, and the corrector must not reject the plan.
    plan = Plan(
        t0_ut=plan.t0_ut,
        times=plan.times,
        positions=plan.positions,
        velocities=plan.velocities,
        masses=plan.masses,
        thrusts=np.tile(np.array([0.0, 0.0, 250000.0]), (plan.n, 1)),
    )
    correction = TerminalCorrector(predictor).estimate(
        plan,
        state,
        target=TerminalTarget(tuple(impact.position), tuple(final_velocity)),
        max_thrust=100000.0,
        isp=state.specific_impulse,
        dry_mass=state.dry_mass,
    )
    assert np.all(np.isfinite(correction.accel_bias))
