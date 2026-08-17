import pytest

from recovery.ksp.control import VesselControls
from tests.fakes import FakeVessel


def test_throttle_clamped() -> None:
    vessel = FakeVessel()
    controls = VesselControls(vessel)
    controls.apply(throttle=1.5)
    assert vessel.control.throttle == 1.0
    controls.apply(throttle=-0.5)
    assert vessel.control.throttle == 0.0


def test_target_direction_normalized_and_engages() -> None:
    vessel = FakeVessel()
    controls = VesselControls(vessel)
    frame = object()
    controls.apply(
        target_direction=(0.0, 0.0, -5.0),
        up=(0.0, 1.0, 0.0),
        roll_angle=10.0,
        reference_frame=frame,
    )
    auto_pilot = vessel.auto_pilot
    assert auto_pilot.reference_frame is frame
    assert auto_pilot.target_direction == (0.0, 0.0, -1.0)
    assert auto_pilot.up_reference == (0.0, 1.0, 0.0)
    assert auto_pilot.target_roll == 10.0
    assert auto_pilot.engaged is True


def test_target_direction_requires_reference_frame() -> None:
    controls = VesselControls(FakeVessel())
    with pytest.raises(ValueError):
        controls.apply(target_direction=(1.0, 0.0, 0.0))


def test_target_direction_zero_rejected() -> None:
    controls = VesselControls(FakeVessel())
    with pytest.raises(ValueError):
        controls.apply(target_direction=(0.0, 0.0, 0.0), reference_frame=object())


def test_mixed_apply() -> None:
    vessel = FakeVessel()
    controls = VesselControls(vessel)
    frame = object()
    controls.apply(throttle=0.4, target_direction=(0.0, 1.0, 0.0), reference_frame=frame)
    assert vessel.control.throttle == 0.4
    assert vessel.auto_pilot.target_direction == (0.0, 1.0, 0.0)


def test_repoint_does_not_toggle_engaged() -> None:
    vessel = FakeVessel()
    controls = VesselControls(vessel)
    frame = object()
    controls.apply(target_direction=(0.0, 1.0, 0.0), reference_frame=frame, throttle=0.5)
    assert vessel.auto_pilot.engaged is True
    controls.apply(target_direction=(0.0, 0.0, 1.0), reference_frame=frame, throttle=0.3)
    assert vessel.auto_pilot.engaged is True


def test_repoint_after_manual_disengage() -> None:
    vessel = FakeVessel()
    controls = VesselControls(vessel)
    frame = object()
    controls.apply(target_direction=(0.0, 1.0, 0.0), reference_frame=frame, throttle=0.5)
    controls.disengage_auto_pilot()
    assert vessel.auto_pilot.engaged is False
    controls.apply(target_direction=(0.0, 0.0, 1.0), reference_frame=frame, throttle=0.3)
    assert vessel.auto_pilot.engaged is True
    assert vessel.auto_pilot.target_direction == (0.0, 0.0, 1.0)


def test_target_smoothing_time() -> None:
    vessel = FakeVessel()
    controls = VesselControls(vessel)
    controls.target_smoothing_time = 0.3
    assert controls.target_smoothing_time == 0.3


def test_cut_thrust() -> None:
    vessel = FakeVessel()
    controls = VesselControls(vessel)
    controls.apply(throttle=0.9)
    controls.engage_auto_pilot()
    controls.cut_thrust()
    assert vessel.control.throttle == 0.0
    assert vessel.auto_pilot.engaged is False


def test_staging_and_action_groups() -> None:
    vessel = FakeVessel()
    controls = VesselControls(vessel)
    controls.activate_next_stage()
    assert vessel.control.stage_calls == 1
    controls.set_action_group(1, True)
    assert controls.get_action_group(1) is True
    controls.toggle_action_group(1)
    assert controls.get_action_group(1) is False


def test_reference_frame_switches_on_change() -> None:
    vessel = FakeVessel()
    controls = VesselControls(vessel)
    frame_a = object()
    frame_b = object()
    controls.apply(target_direction=(0.0, 1.0, 0.0), reference_frame=frame_a)
    assert vessel.auto_pilot.reference_frame is frame_a
    controls.apply(target_direction=(0.0, 0.0, 1.0), reference_frame=frame_b)
    assert vessel.auto_pilot.reference_frame is frame_b
