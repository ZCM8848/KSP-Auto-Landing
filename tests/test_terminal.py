"""Tests for the cubic terminal guidance."""

import numpy as np
import pytest

from recovery.guidance.plan import TerminalTarget
from recovery.guidance.terminal import PolynomialTerminalGuidance
from recovery.types import FlightState, Quaternion, Situation, Vector3


def _state(
    *,
    position: tuple[float, float, float] = (0.0, 0.0, 100.0),
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0),
    mass: float = 1000.0,
) -> FlightState:
    return FlightState(
        ut=0.0,
        met=0.0,
        position=Vector3(*position),
        velocity=Vector3(*velocity),
        velocity_surface=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(0.0, 0.0, 0.0),
        altitude=position[2],
        surface_altitude=position[2],
        mass=mass,
        dry_mass=mass,
        thrust=0.0,
        available_thrust=100000.0,
        max_thrust=100000.0,
        max_vacuum_thrust=100000.0,
        specific_impulse=300.0,
        max_acceleration=0.0,
        throttle=0.0,
        situation=Situation.FLYING,
        loaded=True,
        packed=False,
        landed=False,
        atmosphere_density=0.0,
        atmosphere_depth=70000.0,
        frame=None,
    )


def test_cubic_initial_acceleration() -> None:
    """At rest above the target, the profile asks for 6h/T² upward thrust."""
    cmd = PolynomialTerminalGuidance().command(
        _state(),
        target=TerminalTarget((0.0, 0.0, 0.0)),
        gravity=(0.0, 0.0, -9.81),
        available_thrust=100000.0,
        time_to_go=10.0,
    )
    # a_cmd = -6 * 100 / 10^2 = -6 m/s²; thrust = a_cmd - g = +3.81 m/s².
    assert cmd.nose == pytest.approx((0.0, 0.0, 1.0))
    assert cmd.throttle == pytest.approx(1000.0 * 3.81 / 100000.0)
    assert cmd.status == "terminal"


def test_cubic_target_velocity_term() -> None:
    """A non-zero target velocity adds -2·v_f/T to the command."""
    cmd = PolynomialTerminalGuidance().command(
        _state(),
        target=TerminalTarget((0.0, 0.0, 0.0), (0.0, 0.0, -2.0)),
        gravity=(0.0, 0.0, -9.81),
        available_thrust=100000.0,
        time_to_go=10.0,
    )
    # a_cmd = -6 + 0.4 = -5.6; thrust = -5.6 + 9.81 = 4.21.
    assert cmd.throttle == pytest.approx(1000.0 * 4.21 / 100000.0)


def test_time_to_go_estimate() -> None:
    """Range and closing speed produce T = gain·range/closing."""
    cmd = PolynomialTerminalGuidance().command(
        _state(velocity=(0.0, 0.0, -10.0)),
        target=TerminalTarget((0.0, 0.0, 0.0)),
        gravity=(0.0, 0.0, -9.81),
        available_thrust=100000.0,
    )
    # a_cmd = -6*100/400 - 4*(-10)/20 = -1.5 + 2.0 = +0.5; thrust = 10.31.
    assert cmd.throttle == pytest.approx(1000.0 * 10.31 / 100000.0)


def test_time_to_go_uses_vertical_range() -> None:
    """A large lateral offset must not inflate the vertical time-to-go."""
    guidance = PolynomialTerminalGuidance()
    time_to_go = guidance._estimate_time_to_go(  # noqa: SLF001 - unit test
        np.array([200.0, 0.0, 100.0]),
        np.array([0.0, 0.0, -10.0]),
        np.zeros(3),
        np.zeros(3),
    )
    assert time_to_go == pytest.approx(20.0)


def test_vertical_braking_has_priority_over_lateral() -> None:
    """When the lateral demand saturates the budget, vertical braking wins."""
    cmd = PolynomialTerminalGuidance().command(
        _state(position=(1000.0, 0.0, 100.0), velocity=(0.0, 0.0, -50.0)),
        target=TerminalTarget((0.0, 0.0, 0.0)),
        gravity=(0.0, 0.0, -9.81),
        available_thrust=10000.0,
        time_to_go=5.0,
    )
    assert cmd.throttle == pytest.approx(1.0)
    assert cmd.nose[2] == pytest.approx(1.0)
    assert abs(cmd.nose[0]) < 1e-9


def test_large_lateral_error_keeps_terminal_tilt_bounded() -> None:
    """The cubic never commands a near-horizontal thrust near the ground."""
    cmd = PolynomialTerminalGuidance().command(
        _state(position=(100.0, 0.0, 20.0), velocity=(0.0, 0.0, -10.0)),
        target=TerminalTarget((0.0, 0.0, 0.0)),
        gravity=(0.0, 0.0, -9.81),
        available_thrust=100000.0,
        time_to_go=5.0,
    )
    assert np.linalg.norm(cmd.nose) == pytest.approx(1.0)
    assert cmd.nose[2] >= np.cos(np.radians(30.0)) - 1e-9


def test_downward_terminal_demand_coasts() -> None:
    """A cubic that wants to dive does not point the engine downward."""
    cmd = PolynomialTerminalGuidance().command(
        _state(position=(0.0, 0.0, 100.0), velocity=(0.0, 0.0, 0.0)),
        target=TerminalTarget((0.0, 0.0, 0.0)),
        gravity=(0.0, 0.0, -9.81),
        available_thrust=100000.0,
        time_to_go=5.0,
    )
    # a_cmd = -24 m/s²; engine demand is -14.19 m/s² (downward): coast.
    assert cmd.status == "coast"
    assert cmd.throttle == 0.0


def test_rejects_invalid_time_to_go() -> None:
    """A non-positive or non-finite time-to-go is a programming error."""
    with pytest.raises(ValueError, match="time_to_go"):
        PolynomialTerminalGuidance().command(
            _state(),
            target=TerminalTarget((0.0, 0.0, 0.0)),
            gravity=(0.0, 0.0, -9.81),
            available_thrust=100000.0,
            time_to_go=0.0,
        )


def test_engine_out_coasts() -> None:
    """No available thrust returns a coast command instead of dividing."""
    cmd = PolynomialTerminalGuidance().command(
        _state(),
        target=TerminalTarget((0.0, 0.0, 0.0)),
        gravity=(0.0, 0.0, -9.81),
        available_thrust=0.0,
        time_to_go=10.0,
    )
    assert cmd.throttle == 0.0
    assert cmd.status == "coast"


def test_throttle_is_clipped() -> None:
    """A command above the engine capability saturates at full throttle."""
    cmd = PolynomialTerminalGuidance().command(
        _state(position=(0.0, 0.0, 100.0), velocity=(0.0, 0.0, -50.0)),
        target=TerminalTarget((0.0, 0.0, 0.0)),
        gravity=(0.0, 0.0, -9.81),
        available_thrust=1000.0,
        time_to_go=5.0,
    )
    assert cmd.throttle == pytest.approx(1.0)
    assert np.linalg.norm(cmd.nose) == pytest.approx(1.0)
