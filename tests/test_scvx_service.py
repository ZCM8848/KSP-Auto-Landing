"""Queue/isolation tests for the multi-booster SCvx solve service.

These use a fake ``DescentProblem`` so the scheduling logic is exercised
without compiling a JAX problem; the real solver is covered by
``tests/test_scvx.py``.
"""

import time

import numpy as np
import pytest

from recovery.guidance import (
    EngineModel,
    Plan,
    ScvxConfig,
    ScvxSolveService,
)
from recovery.types import FlightState, Quaternion, Situation, Vector3


class FakeDescent:
    """Minimal stand-in for :class:`DescentProblem`."""

    def __init__(self, config: ScvxConfig) -> None:
        self.config = config
        self.initial_z = 0.0
        self.steps = 0
        self.engine: EngineModel | None = None
        self.bias: np.ndarray | None = None
        self.initialized = False
        self.previous: Plan | None = None

    def initialize(self) -> None:
        self.initialized = True

    def set_engine(self, engine: EngineModel) -> None:
        self.engine = engine

    def set_accel_bias(self, bias: np.ndarray) -> None:
        self.bias = np.asarray(bias, dtype=float)

    def set_initial(
        self, state: FlightState, *, previous: Plan | None = None, tf_hint=None
    ) -> None:
        self.initial_z = float(state.position.z)
        self.previous = previous

    def reset(self) -> None:
        self.steps = 0

    def step(self) -> dict[str, object]:
        self.steps += 1
        return {"converged": False}

    def read_plan(self, t0_ut: float, *, converged: bool = False) -> Plan:
        z = self.initial_z
        return Plan(
            t0_ut=t0_ut,
            times=np.array([0.0, 1.0]),
            positions=np.array([[0.0, 0.0, z], [0.0, 0.0, max(z - 1.0, 0.0)]]),
            velocities=np.zeros((2, 3)),
            masses=np.array([1000.0, 900.0]),
            thrusts=np.zeros((2, 3)),
            iterations=self.steps,
            converged=converged,
        )


def _state(*, ut: float, z: float) -> FlightState:
    return FlightState(
        ut=ut,
        met=ut,
        position=Vector3(0.0, 0.0, z),
        velocity=Vector3(0.0, 0.0, -50.0),
        velocity_surface=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(0.0, 0.0, 0.0),
        altitude=z,
        surface_altitude=z,
        mass=1000.0,
        dry_mass=200.0,
        thrust=0.0,
        available_thrust=20000.0,
        max_thrust=20000.0,
        max_vacuum_thrust=20000.0,
        specific_impulse=300.0,
        max_acceleration=20.0,
        throttle=0.0,
        situation=Situation.FLYING,
        loaded=True,
        packed=False,
        landed=False,
        atmosphere_density=0.0,
        atmosphere_depth=70000.0,
        frame=None,
    )


def _wait_for(predicate, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("condition not met before timeout")


def test_service_keeps_boosters_isolated() -> None:
    """Each booster gets its own plan and refresh updates stay separate."""
    config = ScvxConfig(max_iters_per_refresh=2)
    first = FakeDescent(config)
    second = FakeDescent(config)
    service = ScvxSolveService(workers=1)
    service.register("a", first, config=config)
    service.register("b", second, config=config)
    service.initialize()
    service.start()
    try:
        a_state = _state(ut=10.0, z=1000.0)
        b_state = _state(ut=20.0, z=2000.0)
        service.refresh("a", a_state, engine=EngineModel.from_state(a_state))
        service.refresh("b", b_state, engine=EngineModel.from_state(b_state))

        _wait_for(
            lambda: service.latest_plan("a") is not None
            and service.latest_plan("b") is not None
        )
        plan_a = service.latest_plan("a")
        plan_b = service.latest_plan("b")
        assert plan_a is not None and plan_b is not None
        assert plan_a.t0_ut == pytest.approx(10.0)
        assert plan_b.t0_ut == pytest.approx(20.0)
        assert plan_a.positions[0, 2] == pytest.approx(1000.0)
        assert plan_b.positions[0, 2] == pytest.approx(2000.0)

        # Refreshing one booster must not touch the other's plan.
        service.refresh("b", _state(ut=30.0, z=1500.0), engine=EngineModel.from_state(b_state))

        def _b_updated() -> bool:
            plan = service.latest_plan("b")
            return plan is not None and plan.t0_ut == pytest.approx(30.0)

        _wait_for(_b_updated)
        still_a = service.latest_plan("a")
        assert still_a is not None
        assert still_a.t0_ut == pytest.approx(10.0)
        assert still_a.positions[0, 2] == pytest.approx(1000.0)
        updated_b = service.latest_plan("b")
        assert updated_b is not None
        assert updated_b.positions[0, 2] == pytest.approx(1500.0)
    finally:
        service.stop()


def test_service_reports_stats_and_errors() -> None:
    """Stats expose iterations/age and unknown ids are rejected."""
    config = ScvxConfig(max_iters_per_refresh=3)
    service = ScvxSolveService(workers=1)
    service.register("a", FakeDescent(config), config=config)
    with pytest.raises(ValueError, match="already registered"):
        service.register("a", FakeDescent(config), config=config)
    state = _state(ut=0.0, z=10.0)
    with pytest.raises(RuntimeError, match="initialize"):
        service.refresh("a", state, engine=EngineModel.from_state(state))
    service.initialize()
    stats = service.stats("a")
    assert stats.iterations == 0
    assert stats.plan_age is None
    assert stats.queue_depth == 0
    with pytest.raises(KeyError):
        service.latest_plan("missing")
    with pytest.raises(KeyError):
        service.stats("missing")
    with pytest.raises(RuntimeError, match="after initialize"):
        service.register("b", FakeDescent(config), config=config)


def test_service_rejects_mixed_float_dtype() -> None:
    """The process-wide JAX dtype must be uniform across problems."""
    service = ScvxSolveService(workers=1)
    service.register("a", FakeDescent(ScvxConfig(float_dtype="float64")))
    with pytest.raises(ValueError, match="float_dtype"):
        service.register("b", FakeDescent(ScvxConfig(float_dtype="float32")))
