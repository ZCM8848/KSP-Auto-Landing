"""Tests for immutable plans and their execution."""

from dataclasses import replace

import numpy as np
import pytest

from recovery.guidance.control import ConstantThrottle, FixedNose, RetrogradeNose
from recovery.guidance.plan import (
    Plan,
    PlanExecutor,
    plan_to_virtual_control,
)


def _plan(
    *,
    times: tuple[float, ...] = (0.0, 1.0, 2.0),
    thrusts: np.ndarray | None = None,
) -> Plan:
    n = len(times)
    if thrusts is None:
        thrusts = np.zeros((n, 3))
        thrusts[:, 2] = np.linspace(0.0, 200.0, n)
    positions = np.zeros((n, 3))
    positions[:, 2] = np.linspace(1000.0, 100.0, n)
    velocities = np.zeros((n, 3))
    velocities[:, 2] = -50.0
    masses = np.linspace(1000.0, 800.0, n)
    return Plan(
        t0_ut=100.0,
        times=np.asarray(times, dtype=float),
        positions=positions,
        velocities=velocities,
        masses=masses,
        thrusts=np.asarray(thrusts, dtype=float),
    )


def test_plan_rejects_bad_shapes() -> None:
    """A plan validates node arrays eagerly."""
    with pytest.raises(ValueError, match="positions"):
        Plan(
            t0_ut=0.0,
            times=np.array([0.0, 1.0]),
            positions=np.zeros((2, 2)),
            velocities=np.zeros((2, 3)),
            masses=np.zeros(2),
            thrusts=np.zeros((2, 3)),
        )


def test_plan_rejects_non_monotonic_times() -> None:
    """Node times must be non-decreasing."""
    with pytest.raises(ValueError, match="non-decreasing"):
        _plan(times=(0.0, 2.0, 1.0))


def test_thrust_at_interpolates_and_clamps() -> None:
    """Thrust is linearly interpolated and clamped outside the horizon."""
    thrusts = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 100.0], [0.0, 0.0, 200.0]])
    plan = _plan(thrusts=thrusts)
    assert plan.thrust_at(0.5)[2] == pytest.approx(50.0)
    assert plan.thrust_at(-1.0)[2] == pytest.approx(0.0)
    assert plan.thrust_at(99.0)[2] == pytest.approx(200.0)
    assert plan.duration == pytest.approx(2.0)


def test_state_at_interpolates() -> None:
    """Position/velocity/mass interpolation uses the same node table."""
    plan = _plan()
    pos, vel, mass = plan.state_at(1.0)
    assert pos[2] == pytest.approx(550.0)
    assert vel[2] == pytest.approx(-50.0)
    assert mass == pytest.approx(900.0)


def test_executor_reconstructs_throttle_and_nose() -> None:
    """Throttle is |T|/available_thrust and the nose is the thrust direction."""
    thrusts = np.array([[0.0, 0.0, 150.0], [0.0, 0.0, 150.0], [0.0, 0.0, 150.0]])
    plan = _plan(thrusts=thrusts)
    cmd = PlanExecutor().command(
        plan, 1.0, available_thrust=300.0, coast_nose=(0.0, 0.0, -1.0)
    )
    assert cmd.throttle == pytest.approx(0.5)
    assert cmd.nose == pytest.approx((0.0, 0.0, 1.0))
    assert cmd.thrust == pytest.approx((0.0, 0.0, 150.0))
    assert cmd.status == "plan"


def test_executor_coasts_on_zero_thrust() -> None:
    """Zero commanded thrust keeps the caller's coast attitude."""
    plan = _plan()
    cmd = PlanExecutor().command(
        plan, 0.0, available_thrust=300.0, coast_nose=(0.0, 0.0, -1.0)
    )
    assert cmd.throttle == 0.0
    assert cmd.status == "coast"
    assert cmd.nose == (0.0, 0.0, -1.0)


def test_executor_cuts_below_min_throttle() -> None:
    """A thrust below the engine minimum is executed as engine-off."""
    thrusts = np.full((3, 3), [0.0, 0.0, 10.0])
    plan = _plan(thrusts=thrusts)
    cmd = PlanExecutor(min_throttle=0.1).command(
        plan, 0.0, available_thrust=300.0, coast_nose=(0.0, 0.0, -1.0)
    )
    assert cmd.throttle == 0.0
    assert cmd.status == "min-throttle-cutoff"


def test_executor_handles_engine_out() -> None:
    """No available thrust forces a coast even when the plan asks for thrust."""
    plan = _plan()
    cmd = PlanExecutor().command(
        plan, 1.0, available_thrust=0.0, coast_nose=(0.0, 0.0, -1.0)
    )
    assert cmd.throttle == 0.0
    assert cmd.status == "coast"


def test_shifted_rebases_and_interpolates() -> None:
    """Shifting a plan interpolates the current state and re-bases the times."""
    plan = _plan()
    shifted = plan.shifted(0.5)
    assert shifted is not None
    assert shifted.t0_ut == pytest.approx(100.5)
    assert shifted.times[0] == pytest.approx(0.0)
    assert shifted.times[-1] == pytest.approx(1.5)
    assert shifted.positions[0, 2] == pytest.approx(775.0)
    assert shifted.thrusts[0, 2] == pytest.approx(50.0)
    assert plan.shifted(0.0) is plan
    assert plan.shifted(2.0) is None


def test_plan_attitude_axes_and_shift() -> None:
    """6-DOF plans expose nose/dorsal axes and preserve attitude when shifted."""
    plan = _plan()
    quaternion = np.tile(
        [np.sin(np.pi / 4.0), 0.0, 0.0, np.cos(np.pi / 4.0)],
        (plan.n, 1),
    )
    plan = replace(
        plan,
        attitudes=quaternion,
        angular_velocities=np.zeros((plan.n, 3)),
        torques=np.zeros((plan.n, 3)),
    )
    axes = plan.attitude_axes(0.5)
    assert axes is not None
    nose, dorsal = axes
    assert nose == pytest.approx((0.0, 0.0, 1.0), abs=1e-9)
    assert dorsal == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)

    shifted = plan.shifted(0.5)
    assert shifted is not None
    assert shifted.attitudes is not None
    assert shifted.attitudes.shape == (3, 4)
    assert np.allclose(np.linalg.norm(shifted.attitudes, axis=1), 1.0)
    assert shifted.angular_velocities is not None
    assert shifted.torques is not None


def test_plan_to_virtual_control_segments() -> None:
    """Each node becomes a segment with the thrust direction and node time."""
    thrusts = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 100.0], [0.0, 0.0, 200.0]])
    plan = _plan(thrusts=thrusts)
    vc = plan_to_virtual_control(plan, max_thrust=400.0, isp=300.0, dry_mass=500.0)
    assert len(vc.segments) == 4  # 3 nodes + terminating coast
    assert vc.dry_mass == pytest.approx(500.0)
    first, second, third, coast = vc.segments
    assert first.trigger is None
    assert isinstance(first.nose, RetrogradeNose)
    assert isinstance(second.throttle, ConstantThrottle)
    assert second.throttle.throttle == pytest.approx(0.25)
    assert isinstance(second.nose, FixedNose)
    assert second.nose.direction == (0.0, 0.0, 1.0)
    assert second.trigger is not None and second.trigger.value == pytest.approx(1.0)
    assert third.throttle.throttle == pytest.approx(0.5)
    assert coast.throttle.throttle == pytest.approx(0.0)
    assert coast.trigger is not None and coast.trigger.value == pytest.approx(2.0)


def test_plan_to_virtual_control_can_skip_terminating_coast() -> None:
    """The terminating coast segment is optional."""
    plan = _plan()
    vc = plan_to_virtual_control(
        plan,
        max_thrust=400.0,
        isp=300.0,
        dry_mass=500.0,
        terminate_with_coast=False,
    )
    assert len(vc.segments) == 3


def test_plan_to_virtual_control_rejects_peak_above_limit() -> None:
    """A plan that exceeds the reference thrust is rejected, not clipped."""
    plan = _plan()
    with pytest.raises(ValueError, match="exceeds max_thrust"):
        plan_to_virtual_control(plan, max_thrust=100.0, isp=300.0, dry_mass=500.0)
