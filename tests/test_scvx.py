"""Integration tests for the OpenSCvx descent planner (skipped if unavailable)."""

import time

import numpy as np
import pytest

pytest.importorskip("openscvx")

from recovery.guidance import (  # noqa: E402 - after importorskip
    AttitudeModel,
    EngineModel,
    PlanExecutor,
    ScvxConfig,
    ScvxPlanner,
    build_descent_problem,
    build_six_dof_problem,
)
from recovery.specs import BodySpec, DragSpec  # noqa: E402
from recovery.types import (  # noqa: E402
    FlightState,
    Quaternion,
    Situation,
    TorquePair,
    Vector3,
)

BODY = BodySpec(
    mu=3.5316e12,
    omega=(0.0, 0.0, 0.0),
    body_center=(0.0, 0.0, -600000.0),
    body_radius=600000.0,
    surface_gravity=9.81,
)
ALTS = np.linspace(0.0, 70000.0, 48)
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


def _state(
    *,
    ut: float = 100.0,
    z: float = 3000.0,
    vz: float = -150.0,
    px: float = 0.0,
    py: float = 0.0,
) -> FlightState:
    return FlightState(
        ut=ut,
        met=ut,
        position=Vector3(px, py, z),
        velocity=Vector3(0.0, 0.0, vz),
        velocity_surface=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(0.0, 0.0, 0.0),
        altitude=z,
        surface_altitude=z,
        mass=20000.0,
        dry_mass=2000.0,
        thrust=0.0,
        available_thrust=300000.0,
        max_thrust=300000.0,
        max_vacuum_thrust=320000.0,
        specific_impulse=300.0,
        max_acceleration=15.0,
        throttle=0.0,
        situation=Situation.FLYING,
        loaded=True,
        packed=False,
        landed=False,
        atmosphere_density=0.5,
        atmosphere_depth=70000.0,
        frame=None,
        available_reaction_wheel_torque=TorquePair(
            Vector3(-1000.0, -1000.0, -1000.0),
            Vector3(1000.0, 1000.0, 1000.0),
        ),
        available_engine_torque=TorquePair(
            Vector3(-100000.0, 0.0, -100000.0),
            Vector3(100000.0, 0.0, 100000.0),
        ),
        moment_of_inertia=Vector3(1.6e5, 2.2e4, 1.6e5),
    )


def test_builder_requires_drag_area() -> None:
    """A DragSpec without CdA cannot be used for the OCP."""
    drag = DragSpec(
        ballistic_coefficient=3000.0,
        density_alts=ALTS,
        density_vals=DENSITIES,
        body_center=(0.0, 0.0, -600000.0),
        sea_level_radius=600000.0,
        cd_area=None,
    )
    with pytest.raises(ValueError, match="cd_area"):
        build_descent_problem(
            BODY,
            drag,
            state=_state(),
            target_height=10.0,
            engine=EngineModel.from_state(_state()),
            config=ScvxConfig(n=8),
        )


def test_planner_builds_and_publishes_a_plan() -> None:
    """The async planner initialises, executes SCP steps and publishes a Plan."""
    state = _state()
    engine = EngineModel.from_state(state)
    config = ScvxConfig(n=8)
    built = build_descent_problem(
        BODY,
        DRAG,
        state=state,
        target_height=10.0,
        engine=engine,
        config=config,
    )
    assert built.xy_limit > 100.0
    planner = ScvxPlanner(built, config=config)
    planner.initialize()
    try:
        planner.start()
        planner.refresh(
            state,
            engine=engine,
            tf_hint=40.0,
            accel_bias=np.array([0.0, 0.0, 0.5]),
        )
        deadline = time.monotonic() + 15.0
        while planner.latest_plan() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        plan = planner.latest_plan()
        assert plan is not None, f"no plan produced; error={planner.last_error}"
        assert plan.n == config.n
        assert plan.t0_ut == pytest.approx(state.ut)
        assert plan.times[0] == pytest.approx(0.0)
        assert np.all(np.diff(plan.times) >= 0.0)
        assert np.all(np.isfinite(plan.thrusts))
        assert np.all(np.isfinite(plan.positions))
        assert np.all(np.linalg.norm(plan.thrusts, axis=1) <= engine.available_thrust * 1.001)

        cmd = PlanExecutor().command(
            plan,
            0.0,
            available_thrust=engine.available_thrust,
            coast_nose=(0.0, 0.0, -1.0),
        )
        assert 0.0 <= cmd.throttle <= 1.0
        assert np.linalg.norm(cmd.nose) == pytest.approx(1.0)
    finally:
        planner.stop()


def test_attitude_model_from_state() -> None:
    """Torque authority is summed across the four actuator sources."""
    model = AttitudeModel.from_state(_state())
    assert model.inertia == pytest.approx((1.6e5, 2.2e4, 1.6e5))
    assert model.max_torque == pytest.approx((101000.0, 1000.0, 101000.0))


def test_six_dof_problem_builds_and_steps() -> None:
    """The 6-DOF OCP constructs, initializes and publishes attitude in the plan."""
    state = _state(px=50.0)
    engine = EngineModel.from_state(state)
    config = ScvxConfig(n=6)
    built = build_six_dof_problem(
        BODY,
        DRAG,
        state=state,
        target_height=10.0,
        engine=engine,
        attitude=AttitudeModel.from_state(state),
        config=config,
    )
    assert built.dof == "6dof"
    assert {"attitude", "angular_velocity", "thrust", "torque"} <= set(built.slices)
    # The lateral box must cover the offset and the authority to correct it,
    # not collapse to the numerical floor when starting over the target.
    assert built.xy_limit > 50.0

    built.initialize()
    built.set_engine(engine)
    built.set_attitude_model(AttitudeModel.from_state(state))
    built.set_initial(state, tf_hint=20.0)
    built.reset()
    result = built.step()
    plan = built.read_plan(state.ut)
    assert result is not None
    assert plan.attitudes is not None
    assert plan.attitudes.shape == (config.n, 4)
    assert np.allclose(np.linalg.norm(plan.attitudes, axis=1), 1.0, atol=1e-6)
    assert plan.angular_velocities is not None
    assert plan.angular_velocities.shape == (config.n, 3)
    assert plan.torques is not None
    assert plan.torques.shape == (config.n, 3)
    assert np.all(np.isfinite(plan.thrusts))
    assert plan.positions[0, 0] == pytest.approx(50.0)
    assert plan.positions[-1, 0] == pytest.approx(0.0, abs=1e-6)
