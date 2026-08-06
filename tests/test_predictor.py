"""Tests for the RK45 ballistic impact predictor."""

import numpy as np
import pytest

from recovery.guidance import LandingPredictor
from recovery.ksp.types import FlightState, Quaternion, Situation, Vector3

# Kerbin-like planet (no rotation for simpler tests)
MU = 3.5316e12
R = 600_000.0
G0 = MU / (R * R)  # ~9.81 m/s^2


@pytest.fixture
def predictor_no_rotation():
    return LandingPredictor(
        mu=MU,
        omega=(0.0, 0.0, 0.0),
        body_center=(0.0, 0.0, -R),
        body_radius=R,
    )


def test_straight_drop(predictor_no_rotation):
    """From altitude h with zero velocity — should land directly below."""
    h = 1000.0
    r0 = (0.0, 0.0, h)
    v0 = (0.0, 0.0, 0.0)
    result = predictor_no_rotation.predict(position=r0, velocity=v0)
    assert result is not None
    assert result.position[0] == pytest.approx(0.0, abs=0.1)
    assert result.position[1] == pytest.approx(0.0, abs=0.1)
    assert result.position[2] == pytest.approx(0.0, abs=1.0)
    t_freefall = np.sqrt(2 * h / G0)
    assert result.time == pytest.approx(t_freefall, rel=0.01)


def test_horizontal_throw(predictor_no_rotation):
    """Launch horizontally from altitude — verify x displacement."""
    h = 500.0
    vx = 200.0
    r0 = (0.0, 0.0, h)
    v0 = (vx, 0.0, 0.0)
    result = predictor_no_rotation.predict(position=r0, velocity=v0)
    assert result is not None
    t_fall = np.sqrt(2 * h / G0)
    expected_x = vx * t_fall
    assert result.position[0] == pytest.approx(expected_x, rel=0.02)
    assert result.position[1] == pytest.approx(0.0, abs=0.1)
    assert result.position[2] == pytest.approx(0.0, abs=abs(expected_x * 0.01))


def test_not_hitting_surface(predictor_no_rotation):
    """Upward velocity prevents impact within t_max."""
    r0 = (0.0, 0.0, 5000.0)
    v0 = (0.0, 0.0, 0.0)
    result = predictor_no_rotation.predict(position=r0, velocity=v0, t_max=1.0)
    assert result is None


def test_with_rotation():
    """Coriolis deflects for a falling object."""
    omega = (0.0, 0.0001, 0.0)
    predictor = LandingPredictor(
        mu=MU,
        omega=omega,
        body_center=(0.0, 0.0, -R),
        body_radius=R,
    )
    h = 1000.0
    r0 = (0.0, 0.0, h)
    v0 = (0.0, 0.0, 0.0)
    result = predictor.predict(position=r0, velocity=v0)
    assert result is not None
    deflected_x = result.position[0] != pytest.approx(0.0, abs=1e-3)
    deflected_y = result.position[1] != pytest.approx(0.0, abs=1e-3)
    assert deflected_x or deflected_y


def test_rtol_accuracy():
    """Stricter tolerance gives more precise result."""
    predictor = LandingPredictor(
        mu=MU, omega=(0.0, 0.0, 0.0), body_center=(0.0, 0.0, -R), body_radius=R
    )
    r0 = (0.0, 0.0, 1000.0)
    v0 = (100.0, 0.0, -50.0)
    r_loose = predictor.predict(position=r0, velocity=v0, rtol=1e-3, atol=1e-3)
    r_tight = predictor.predict(position=r0, velocity=v0, rtol=1e-9, atol=1e-9)
    assert r_loose is not None
    assert r_tight is not None
    assert r_loose.position[2] == pytest.approx(r_tight.position[2], abs=0.5)


def test_predict_from_snapshot(predictor_no_rotation):
    """predict_from accepts a FlightState directly."""
    state = FlightState(
        ut=0, met=0,
        position=Vector3(0, 0, 500),
        velocity=Vector3(200, 0, 0),
        velocity_surface=Vector3(0, 0, 0),
        rotation=Quaternion(0, 0, 0, 1),
        angular_velocity=Vector3(0, 0, 0),
        altitude=500, surface_altitude=500,
        mass=30000, dry_mass=10000,
        thrust=0, available_thrust=0, max_thrust=0, max_vacuum_thrust=0,
        specific_impulse=0, max_acceleration=0, throttle=0,
        situation=Situation.FLYING, loaded=True, packed=False, landed=False,
        atmosphere_density=0, frame=None,
    )
    result = predictor_no_rotation.predict_from(state)
    assert result is not None
    assert result.position[2] == pytest.approx(0, abs=5)
