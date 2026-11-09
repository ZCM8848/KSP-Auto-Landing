"""Tests for coast/tilt-guarded plan tracking and receding-horizon updates."""

from __future__ import annotations

import math

import numpy as np
import pytest

from recovery.guidance import (
    AttitudeModel,
    EngineModel,
    Plan,
    PlanTracker,
    TrackingConfig,
    plan_is_usable,
    plan_thrust,
)
from recovery.types import FlightState, Quaternion, Situation, Vector3


def _state(
    *,
    ut: float = 100.0,
    z: float = 8000.0,
    vz: float = -200.0,
    mass: float = 1000.0,
) -> FlightState:
    return FlightState(
        ut=ut,
        met=ut,
        position=Vector3(0.0, 0.0, z),
        velocity=Vector3(0.0, 0.0, vz),
        velocity_surface=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(0.0, 0.0, 0.0),
        altitude=z,
        surface_altitude=z,
        mass=mass,
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


def _plan(
    *,
    thrust: tuple[float, float, float] = (0.0, 0.0, 10000.0),
    t0: float = 100.0,
    final_z: float = 6.66,
    duration: float = 20.0,
    ignition_time: float | None = None,
    iterations: int = 10,
) -> Plan:
    n = 5
    times = np.linspace(0.0, duration, n)
    positions = np.zeros((n, 3), dtype=float)
    positions[:, 2] = np.linspace(8000.0, final_z, n)
    velocities = np.zeros((n, 3), dtype=float)
    velocities[:, 2] = np.linspace(-200.0, 0.0, n)
    masses = np.linspace(1000.0, 900.0, n)
    thrusts = np.tile(np.asarray(thrust, dtype=float), (n, 1))
    # 90° about body +x maps the nose (body +y) to target +z.
    q_up = math.sqrt(0.5)
    attitudes = np.tile(np.array([q_up, 0.0, 0.0, q_up]), (n, 1))
    return Plan(
        t0_ut=t0,
        times=times,
        positions=positions,
        velocities=velocities,
        masses=masses,
        thrusts=thrusts,
        attitudes=attitudes,
        ignition_time=ignition_time,
        iterations=iterations,
    )


def test_gate_tail_is_a_coast_not_a_burn() -> None:
    """A sigmoid gate tail of a few newtons must not light the engine."""
    plan = _plan(thrust=(0.0, 0.0, 0.9))
    state = _state(z=8001.0, vz=0.0)
    cmd = plan_thrust(plan, state, 10.0, config=TrackingConfig())
    assert cmd.coast
    assert cmd.throttle == 0.0
    assert cmd.planned_fraction < 1e-3
    assert cmd.nose[2] == pytest.approx(1.0)


def test_free_ignition_ramp_waits_for_the_executable_band() -> None:
    """A gate ramp below half the engine band is still a coast."""
    config = TrackingConfig(light_fraction=0.4, cutoff_fraction=0.05)
    low = _plan(thrust=(0.0, 0.0, 2000.0), ignition_time=10.0)
    assert plan_thrust(low, _state(z=8001.0, vz=0.0), 10.0, config=config).coast

    high = _plan(thrust=(0.0, 0.0, 10000.0), ignition_time=10.0)
    cmd = plan_thrust(high, _state(z=8001.0, vz=0.0), 10.0, config=config)
    assert not cmd.coast


def test_burn_hysteresis_prevents_zero_forty_chatter() -> None:
    """Once lit, a low-thrust plan stays lit until it drops below cutoff."""
    config = TrackingConfig(light_fraction=0.4, cutoff_fraction=0.05)
    low = _plan(thrust=(0.0, 0.0, 2000.0), ignition_time=10.0)

    assert plan_thrust(
        low, _state(z=8001.0, vz=0.0), 10.0, config=config, burning=False
    ).coast
    lit = plan_thrust(
        low, _state(z=8001.0, vz=0.0), 10.0, config=config, burning=True
    )
    assert not lit.coast
    assert lit.throttle >= 0.4

    cut = _plan(thrust=(0.0, 0.0, 200.0), ignition_time=10.0)
    assert plan_thrust(
        cut, _state(z=8001.0, vz=0.0), 10.0, config=config, burning=True
    ).coast


def test_feedback_cannot_invert_the_engine() -> None:
    """Downward feedback falls back to the plan attitude, never below 30°."""
    plan = _plan(thrust=(0.0, 0.0, 1000.0))
    # Well above the plan and slower: feedback wants to accelerate downward.
    state = _state(z=9000.0, vz=0.0)
    cmd = plan_thrust(plan, state, 10.0, config=TrackingConfig())
    assert not cmd.coast
    assert cmd.tilt_limited
    assert cmd.nose[2] >= math.cos(math.radians(30.0)) - 1e-9
    assert 0.4 <= cmd.throttle <= 1.0


def test_upward_burn_keeps_feedback_direction() -> None:
    """Ordinary upward feedback is still allowed to correct the plan."""
    plan = _plan(thrust=(0.0, 0.0, 10000.0))
    state = _state(z=1000.0, vz=-200.0)
    cmd = plan_thrust(plan, state, 10.0, config=TrackingConfig())
    assert not cmd.coast
    assert not cmd.tilt_limited
    assert cmd.nose[2] > 0.0
    assert cmd.throttle >= 0.4


def test_plan_is_usable_rejects_early_high_terminal_error() -> None:
    assert plan_is_usable(_plan(final_z=6.66), target_height=6.66)
    assert not plan_is_usable(_plan(final_z=500.0), target_height=6.66)


class _FakeService:
    """Synchronous stand-in for the refresh/latest-plan slice of the service."""

    def __init__(self) -> None:
        self.plan: Plan | None = None
        self.next_plan: Plan | None = None
        self.refreshes = 0

    def refresh(
        self,
        booster_id: str,
        state: FlightState,
        *,
        engine: EngineModel,
        tf_hint: float | None = None,
        accel_bias: np.ndarray | None = None,
        attitude: AttitudeModel | None = None,
    ) -> None:
        del booster_id, state, engine, tf_hint, accel_bias, attitude
        self.refreshes += 1
        if self.next_plan is not None:
            self.plan = self.next_plan
            self.next_plan = None

    def latest_plan(self, booster_id: str) -> Plan | None:
        del booster_id
        return self.plan


def test_tracker_waits_for_a_usable_plan_then_replans() -> None:
    """The first usable plan is tracked; a later refresh replaces it."""
    service = _FakeService()
    tracker = PlanTracker(
        booster_id="b",
        service=service,
        tracking=TrackingConfig(),
        target_height=6.66,
        mpc_period=2.0,
        terminal_alt=500.0,
    )
    state = _state(ut=100.0, z=8000.0)
    assert tracker.step(state, now=0.0) is None
    assert service.refreshes == 1

    service.plan = _plan(t0=100.0, iterations=2)
    assert tracker.step(state, now=1.0) is None

    service.plan = _plan(t0=100.0)
    first = tracker.step(state, now=1.0)
    assert first is not None
    assert tracker.replans == 1
    assert tracker.plan is not None

    # A newer SCP iterate with the same t0 must not replace the plan.
    service.plan = _plan(t0=100.0)
    assert tracker.step(state, now=1.5) is not None
    assert tracker.replans == 1

    state2 = _state(ut=102.5, z=7000.0)
    service.next_plan = _plan(t0=102.5)
    second = tracker.step(state2, now=3.2)
    assert second is not None
    assert service.refreshes == 2
    assert tracker.replans == 2
