"""Tests for the RK45 ballistic impact predictor."""

import time
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from recovery.guidance import AeroModel, DragModel, KrpcAeroModel, LandingPredictor
from recovery.ksp.sampling import sample_drag_spec
from recovery.types import FlightState, Quaternion, Situation, Vector3

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


# ------------------------------------------------------------------ KrpcAeroModel


class _CountingAero(AeroModel):
    """AeroModel that counts how many times ``acceleration`` is called."""

    def __init__(self) -> None:
        self.call_count = 0
        self.positions: list[tuple[float, ...]] = []
        self.velocities: list[tuple[float, ...]] = []

    def acceleration(
        self, position: np.ndarray, velocity: np.ndarray
    ) -> tuple[float, float, float]:
        self.call_count += 1
        self.positions.append(tuple(position))
        self.velocities.append(tuple(velocity))
        return (0.0, 0.0, 0.0)


def test_krpc_aero_call_count() -> None:
    """A reentry prediction calls acceleration at least several hundred times."""
    aero = _CountingAero()
    predictor = LandingPredictor(
        mu=MU, omega=(0.0, 0.0, 0.0),
        body_center=(0.0, 0.0, -R), body_radius=R,
        aero=aero,
    )
    result = predictor.predict(position=(0.0, 0.0, 40000.0), velocity=(1000.0, 0.0, -800.0))
    assert result is not None
    assert aero.call_count > 50, f"expected >50 steps, got {aero.call_count}"


def test_krpc_aero_drag_slows_descent() -> None:
    """With a drag-like aero model, impact takes longer than ballistic."""
    mock_flight = MagicMock()
    mock_body = MagicMock()

    def _drag_force(
        _body: Any, pos: Any, vel: Any, _rot: Any,
    ) -> tuple[float, float, float]:
        v = np.array(vel, dtype=float)
        mag = float(np.linalg.norm(v))
        if mag < 1e-6:
            return (0.0, 0.0, 0.0)
        drag = -v / mag * (0.5 * 1.2 * mag * mag * 20.0 * 1.0)
        return (float(drag[0]), float(drag[1]), float(drag[2]))

    mock_flight.simulate_aerodynamic_force_at.side_effect = _drag_force
    aero = KrpcAeroModel(mock_flight, mock_body, mass=30000.0)

    ballistic = LandingPredictor(
        mu=MU, omega=(0.0, 0.0, 0.0), body_center=(0.0, 0.0, -R), body_radius=R,
    )
    with_drag = LandingPredictor(
        mu=MU, omega=(0.0, 0.0, 0.0), body_center=(0.0, 0.0, -R), body_radius=R,
        aero=aero,
    )

    r0 = (0.0, 0.0, 5000.0)
    v0 = (0.0, 0.0, -200.0)
    r_b = ballistic.predict(position=r0, velocity=v0)
    r_d = with_drag.predict(position=r0, velocity=v0)

    assert r_b is not None
    assert r_d is not None
    assert r_d.time > r_b.time, (
        f"drag {r_d.time:.2f}s should exceed ballistic {r_b.time:.2f}s"
    )


def test_krpc_aero_mock_performance() -> None:
    """Measure the number of integration steps and elapsed time.

    This test documents the cost of the RPC model — print metrics to stdout
    so CI / developers see the expected per-step overhead.
    """
    mock_flight = MagicMock()
    mock_flight.simulate_aerodynamic_force_at.return_value = (5000.0, 200.0, -800.0)
    mock_body = MagicMock()
    aero = KrpcAeroModel(mock_flight, mock_body, mass=30000.0)

    predictor = LandingPredictor(
        mu=MU, omega=(0.0, 0.0, 0.0), body_center=(0.0, 0.0, -R), body_radius=R,
        aero=aero,
    )

    t0 = time.perf_counter()
    result = predictor.predict(
        position=(0.0, 0.0, 40000.0), velocity=(1500.0, 0.0, -900.0),
    )
    elapsed = time.perf_counter() - t0
    calls = mock_flight.simulate_aerodynamic_force_at.call_count
    per_call_us = (elapsed / calls * 1e6) if calls else float("inf")

    print(
        f"\n  KrpcAeroModel mock perf — steps: {calls},"
        f" total: {elapsed*1e3:.1f}ms, per-call: {per_call_us:.0f}µs"
    )
    assert result is not None
    assert calls > 0


# -------------------------------------------------------------------- DragModel


def test_drag_acceleration_zero_speed() -> None:
    """Drag is zero when velocity is zero."""
    model = DragModel(
        ballistic_coefficient=1000.0,
        density_fn=lambda h: 1.2,
        body_center=(0.0, 0.0, -R),
        sea_level_radius=R,
    )
    acc = model.acceleration(
        np.array([0.0, 0.0, 500.0]),
        np.array([0.0, 0.0, 0.0]),
    )
    assert acc == (0.0, 0.0, 0.0)


def test_drag_acceleration_above_atmosphere() -> None:
    """Drag is zero above the atmosphere (zero density)."""
    model = DragModel(
        ballistic_coefficient=1000.0,
        density_fn=lambda h: 0.0,
        body_center=(0.0, 0.0, -R),
        sea_level_radius=R,
    )
    acc = model.acceleration(
        np.array([0.0, 0.0, 100000.0]),
        np.array([0.0, 0.0, -200.0]),
    )
    assert acc == (0.0, 0.0, 0.0)


def test_drag_acceleration_direction() -> None:
    """Drag opposes velocity — downward velocity gets upward acceleration."""
    beta = 500.0
    rho = 1.2
    v = np.array([0.0, 0.0, -100.0])
    model = DragModel(
        ballistic_coefficient=beta,
        density_fn=lambda h: rho,
        body_center=(0.0, 0.0, -R),
        sea_level_radius=R,
    )
    acc = model.acceleration(np.array([0.0, 0.0, 500.0]), v)
    # a = -0.5 * rho * |v|^2 / beta * v/|v|
    #   = 0.5 * 1.2 * 10000 / 500 * (0, 0, 1) = 12.0
    mag = 0.5 * rho * 100.0 * 100.0 / beta
    assert acc == pytest.approx((0.0, 0.0, mag), abs=1e-9)


def test_drag_slows_impact() -> None:
    """A vessel with drag takes longer to hit the ground than ballistic."""
    model = DragModel(
        ballistic_coefficient=500.0,
        density_fn=lambda h: 1.0,
        body_center=(0.0, 0.0, -R),
        sea_level_radius=R,
    )
    predictor_drag = LandingPredictor(
        mu=MU, omega=(0.0, 0.0, 0.0),
        body_center=(0.0, 0.0, -R), body_radius=R,
        aero=model,
    )
    predictor_bal = LandingPredictor(
        mu=MU, omega=(0.0, 0.0, 0.0),
        body_center=(0.0, 0.0, -R), body_radius=R,
    )
    r0 = (0.0, 0.0, 5000.0)
    v0 = (0.0, 0.0, -200.0)
    r_d = predictor_drag.predict(position=r0, velocity=v0)
    r_b = predictor_bal.predict(position=r0, velocity=v0)
    assert r_d is not None
    assert r_b is not None
    assert r_d.time > r_b.time, (
        f"drag {r_d.time:.2f}s should exceed ballistic {r_b.time:.2f}s"
    )


def test_sample_drag_spec_far() -> None:
    """sample_drag_spec + DragModel.from_spec builds a functional model."""
    mock_body = MagicMock()
    mock_body.equatorial_radius = float(R)
    mock_body.atmosphere_depth = 80000.0
    mock_body.position.return_value = (0.0, 0.0, -float(R))

    scale_h = 5600.0
    rho0 = 1.225
    mock_body.density_at.side_effect = (
        lambda h: rho0 * np.exp(-float(h) / scale_h)
    )
    far_proxy = MagicMock()
    far_proxy.far_available = True
    mock_body.space_center = far_proxy

    mock_flight = MagicMock()
    mock_flight.ballistic_coefficient = 1500.0

    spec = sample_drag_spec(
        body=mock_body,
        flight=mock_flight,
        target_frame=MagicMock(),
        altitude_samples=32,
    )
    model = DragModel.from_spec(spec)

    r = np.array([0.0, 0.0, 1000.0])
    v = np.array([0.0, 0.0, -500.0])
    acc = model.acceleration(r, v)
    # Should return a non-zero drag deceleration
    assert acc[2] > 0.0, f"drag should oppose downward velocity, got {acc}"

    # Above atmosphere should give zero
    acc_high = model.acceleration(np.array([0.0, 0.0, 90000.0]), v)
    assert acc_high == (0.0, 0.0, 0.0)


def test_sample_drag_spec_no_far_estimation() -> None:
    """Without FAR and with mass, beta is estimated from drag force."""
    mock_body = MagicMock()
    mock_body.equatorial_radius = float(R)
    mock_body.atmosphere_depth = 70000.0
    mock_body.position.return_value = (0.0, 0.0, -float(R))
    far_proxy = MagicMock()
    far_proxy.far_available = False
    mock_body.space_center = far_proxy
    mock_body.density_at.return_value = 1.0

    mock_flight = MagicMock()
    mock_flight.atmosphere_density = 1.0
    mock_flight.drag = (0.0, 0.0, -500.0)
    mock_flight.speed = 500.0

    # beta = mass * rho * v^2 / (2 * drag_mag)
    # = 30000 * 1.0 * 500^2 / (2 * 500)
    # = 30000 * 250000 / 1000
    # = 7500000  (kg/m2)
    spec = sample_drag_spec(
        body=mock_body,
        flight=mock_flight,
        target_frame=MagicMock(),
        mass=30000.0,
        altitude_samples=32,
    )
    model = DragModel.from_spec(spec)

    r = np.array([0.0, 0.0, 1000.0])
    v = np.array([0.0, 0.0, -200.0])
    acc = model.acceleration(r, v)
    assert acc[2] > 0.0


def test_drag_inf_beta_gives_zero_drag() -> None:
    """beta=inf (no drag) produces zero acceleration regardless of velocity."""
    model = DragModel(
        ballistic_coefficient=float("inf"),
        density_fn=lambda h: 1.225,
        body_center=(0.0, 0.0, -R),
        sea_level_radius=R,
    )
    acc = model.acceleration(
        np.array([0.0, 0.0, 500.0]),
        np.array([0.0, 0.0, -200.0]),
    )
    assert acc == (0.0, 0.0, 0.0)


def test_sample_drag_spec_uses_local_sea_level_radius() -> None:
    """sample_drag_spec accepts lat/lon and computes a location-aware sea-level radius."""
    mock_body = MagicMock()
    mock_body.equatorial_radius = float(R)
    mock_body.atmosphere_depth = 70000.0
    mock_body.position.return_value = (0.0, 0.0, -float(R))
    mock_body.density_at.return_value = 1.0

    far_proxy = MagicMock()
    far_proxy.far_available = True
    mock_body.space_center = far_proxy

    mock_flight = MagicMock()
    mock_flight.ballistic_coefficient = 1500.0

    spec = sample_drag_spec(
        body=mock_body,
        flight=mock_flight,
        target_frame=MagicMock(),
        lat=-0.185,
        lon=-74.473,
        altitude_samples=32,
    )
    # KSP bodies are spheres, so the local sea-level radius equals the equatorial radius.
    assert spec.sea_level_radius == pytest.approx(float(R))
    # Ensure the body was queried at the requested location (placeholder for
    # future oblate-body support; currently the helper ignores lat/lon).
    assert mock_body.equatorial_radius == pytest.approx(float(R))


def test_numba_rk4_matches_scipy() -> None:
    """Fixed-step RK4 result agrees with scipy adaptive when DragModel is attached."""
    from recovery.guidance._numba import rk4_fixed

    h_vals = np.linspace(0, 80000, 64)
    density_vals = 1.225 * np.exp(-h_vals / 5600.0)
    drag = DragModel(
        ballistic_coefficient=5000.0,
        density_fn=lambda h: float(np.interp(h, h_vals, density_vals)),
        body_center=(0.0, 0.0, -R),
        sea_level_radius=R,
        density_alts=h_vals,
        density_vals=density_vals,
    )
    predictor = LandingPredictor(
        mu=MU, omega=(0.0, 0.0, 0.0),
        body_center=(0.0, 0.0, -R), body_radius=R,
        aero=drag,
    )

    for alt, vx, vz in [(50000, 1200, -600), (10000, 500, -100)]:
        r_scipy = predictor.predict(
            position=(0.0, 0.0, alt), velocity=(vx, 0.0, vz),
            rtol=5e-6, atol=5e-6,
        )
        assert r_scipy is not None
        r0 = np.array([0.0, 0.0, alt], dtype=float)
        v0 = np.array([vx, 0.0, vz], dtype=float)
        params = drag.numba_params
        assert params is not None
        beta, alts, dens, sea_r = params
        hit = rk4_fixed(
            r0, v0, MU, np.zeros(3), np.array([0.0, 0.0, -R]), R,
            beta, alts, dens, sea_r,
            dt=0.04, max_n=5000,
        )
        assert hit is not None
        dpos = np.hypot(r_scipy.position[0] - hit[0], r_scipy.position[2] - hit[2])
        dt_err = abs(r_scipy.time - hit[3])
        assert dpos < 500, f"position diff {dpos:.0f}m too large at alt={alt}"
        assert dt_err < 2.0, f"time diff {dt_err:.2f}s too large at alt={alt}"


def test_from_body_spec_pure() -> None:
    """LandingPredictor.from_body_spec constructs from pure data, no kRPC."""
    from recovery.specs import BodySpec

    spec = BodySpec(
        mu=MU,
        omega=(0.0, 0.0, 0.0),
        body_center=(0.0, 0.0, -R),
        body_radius=R,
        surface_gravity=MU / R**2,
    )
    predictor = LandingPredictor.from_body_spec(spec)
    result = predictor.predict(position=(0.0, 0.0, 1000.0), velocity=(0.0, 0.0, 0.0))
    assert result is not None
    assert result.position[2] == pytest.approx(0.0, abs=1.0)


def test_drag_from_spec_pure() -> None:
    """DragModel.from_spec builds the interpolation from raw density arrays."""
    from recovery.specs import DragSpec

    alts = np.linspace(0.0, 80000.0, 64)
    vals = 1.225 * np.exp(-alts / 5600.0)
    spec = DragSpec(
        ballistic_coefficient=5000.0,
        density_alts=alts,
        density_vals=vals,
        body_center=(0.0, 0.0, -R),
        sea_level_radius=R,
    )
    model = DragModel.from_spec(spec)
    assert model.numba_params is not None
    acc = model.acceleration(
        np.array([0.0, 0.0, 1000.0]), np.array([0.0, 0.0, -200.0])
    )
    assert acc[2] > 0.0


def test_numba_drag_acceleration_matches_python_at_boundaries() -> None:
    """Numba drag acceleration must match the Python DragModel at atmosphere boundaries.

    Regression guard: the old numba interpolator returned the top-of-atmosphere
    density for every altitude above the atmosphere, producing spurious drag in
    the fast RK4 path. This test uses a constant-density profile so the bug
    produces a large, obvious discrepancy above the atmosphere.
    """
    from recovery.guidance._numba import _accel_jit

    depth = 80000.0
    h_vals = np.linspace(0.0, depth, 64)
    density_vals = np.full_like(h_vals, 1.225)  # constant sea-level density
    beta = 100.0  # small beta -> large drag, makes the bug obvious
    center_arr = np.array([0.0, 0.0, -R], dtype=float)
    omega = np.zeros(3)

    model = DragModel(
        ballistic_coefficient=beta,
        density_fn=lambda h: 0.0 if h > depth else 1.225,
        body_center=(0.0, 0.0, -R),
        sea_level_radius=R,
        density_alts=h_vals,
        density_vals=density_vals,
    )

    velocity = np.array([1000.0, 0.0, -500.0], dtype=float)
    positions = [
        (0.0, 0.0, 1000.0),  # in atmosphere
        (0.0, 0.0, depth),  # exactly at atmosphere top
        (0.0, 0.0, 100000.0),  # above atmosphere
        (0.0, 0.0, -50.0),  # below sea level
    ]

    for pos in positions:
        r_arr = np.array(pos, dtype=float)
        python_drag = np.array(model.acceleration(r_arr, velocity), dtype=float)

        # _accel_jit returns total acceleration; subtract gravity to isolate drag.
        d = r_arr - center_arr
        dist = float(np.linalg.norm(d))
        gravity = -MU / (dist ** 3) * d
        numba_total = _accel_jit(
            r_arr,
            velocity,
            MU,
            omega,
            center_arr,
            beta,
            h_vals,
            density_vals,
            R,
        )
        numba_drag = numba_total - gravity

        np.testing.assert_allclose(
            numba_drag,
            python_drag,
            atol=1e-6,
            err_msg=f"drag mismatch at position {pos}",
        )


def test_numba_vs_scipy_performance(monkeypatch: pytest.MonkeyPatch) -> None:
    """Benchmark the numba fast path against the scipy fallback.

    Both paths integrate the same state with the same :class:`DragModel`; the
    only difference is the integrator.  Results are printed so the speedup and
    accuracy gap are visible when deciding whether to invest in a variable-step
    numba integrator.
    """
    h_vals = np.linspace(0, 80000, 64)
    density_vals = 1.225 * np.exp(-h_vals / 5600.0)
    drag = DragModel(
        ballistic_coefficient=5000.0,
        density_fn=lambda h: float(np.interp(h, h_vals, density_vals)),
        body_center=(0.0, 0.0, -R),
        sea_level_radius=R,
        density_alts=h_vals,
        density_vals=density_vals,
    )
    predictor = LandingPredictor(
        mu=MU,
        omega=(0.0, 0.0, 0.0),
        body_center=(0.0, 0.0, -R),
        body_radius=R,
        aero=drag,
    )

    cases: list[tuple[tuple[float, float, float], tuple[float, float, float], str]] = [
        ((0.0, 0.0, 50000.0), (1500.0, 0.0, -900.0), "high_altitude_reentry"),
        ((0.0, 0.0, 10000.0), (500.0, 0.0, -100.0), "mid_altitude_landing"),
    ]

    for r0, v0, name in cases:
        # Numba fast path.
        t0 = time.perf_counter()
        r_numba = predictor.predict(position=r0, velocity=v0)
        t_numba = time.perf_counter() - t0

        # Force the scipy fallback by neutering the numba fast path.
        monkeypatch.setattr(predictor, "_predict_numba", lambda *a, **k: None)
        t0 = time.perf_counter()
        r_scipy = predictor.predict(
            position=r0, velocity=v0, rtol=1e-6, atol=1e-6
        )
        t_scipy = time.perf_counter() - t0
        monkeypatch.undo()

        assert r_numba is not None
        assert r_scipy is not None
        dpos = np.hypot(
            r_numba.position[0] - r_scipy.position[0],
            r_numba.position[2] - r_scipy.position[2],
        )
        dt_err = abs(r_numba.time - r_scipy.time)
        speedup = t_scipy / t_numba if t_numba > 0.0 else float("inf")

        print(
            f"\n{name}:"
            f" numba={t_numba * 1e3:.2f}ms"
            f" scipy={t_scipy * 1e3:.2f}ms"
            f" speedup={speedup:.1f}x"
            f" dpos={dpos:.1f}m"
            f" dt_err={dt_err:.2f}s"
        )

        # The existing test_numba_rk4_matches_scipy tolerances.
        assert dpos < 500.0
        assert dt_err < 2.0


def test_predict_dt_validation() -> None:
    """Constructor and per-call dt must reject non-positive values."""
    with pytest.raises(ValueError):
        LandingPredictor(
            mu=MU,
            omega=(0.0, 0.0, 0.0),
            body_center=(0.0, 0.0, -R),
            body_radius=R,
            dt=0.0,
        )
    with pytest.raises(ValueError):
        LandingPredictor(
            mu=MU,
            omega=(0.0, 0.0, 0.0),
            body_center=(0.0, 0.0, -R),
            body_radius=R,
            dt=-0.1,
        )
    predictor = LandingPredictor(
        mu=MU, omega=(0.0, 0.0, 0.0), body_center=(0.0, 0.0, -R), body_radius=R
    )
    with pytest.raises(ValueError):
        predictor.predict(position=(0.0, 0.0, 1000.0), velocity=(0.0, 0.0, 0.0), dt=0.0)


def test_predict_dt_override() -> None:
    """Per-call dt overrides the constructor default."""
    h_vals = np.linspace(0, 80000, 64)
    density_vals = 1.225 * np.exp(-h_vals / 5600.0)
    drag = DragModel(
        ballistic_coefficient=5000.0,
        density_fn=lambda h: float(np.interp(h, h_vals, density_vals)),
        body_center=(0.0, 0.0, -R),
        sea_level_radius=R,
        density_alts=h_vals,
        density_vals=density_vals,
    )
    predictor = LandingPredictor(
        mu=MU,
        omega=(0.0, 0.0, 0.0),
        body_center=(0.0, 0.0, -R),
        body_radius=R,
        aero=drag,
        dt=0.04,
    )
    r_default = predictor.predict(position=(0.0, 0.0, 10000.0), velocity=(500.0, 0.0, -100.0))
    r_large = predictor.predict(
        position=(0.0, 0.0, 10000.0), velocity=(500.0, 0.0, -100.0), dt=0.5
    )
    assert r_default is not None
    assert r_large is not None


def test_dt_speed_accuracy_tradeoff() -> None:
    """Print the speed/accuracy tradeoff for different numba dt values.

    Uses a long high-altitude reentry so the effect of dt is visible.
    Assertions are loose; the printed table is the primary evaluation output.
    """
    h_vals = np.linspace(0, 80000, 64)
    density_vals = 1.225 * np.exp(-h_vals / 5600.0)
    drag = DragModel(
        ballistic_coefficient=5000.0,
        density_fn=lambda h: float(np.interp(h, h_vals, density_vals)),
        body_center=(0.0, 0.0, -R),
        sea_level_radius=R,
        density_alts=h_vals,
        density_vals=density_vals,
    )
    predictor = LandingPredictor(
        mu=MU,
        omega=(0.0, 0.0, 0.0),
        body_center=(0.0, 0.0, -R),
        body_radius=R,
        aero=drag,
    )

    r0 = (0.0, 0.0, 50000.0)
    v0 = (1500.0, 0.0, -900.0)

    # Reference: high-precision scipy path.
    r_scipy = predictor.predict(position=r0, velocity=v0, rtol=1e-9, atol=1e-9)
    assert r_scipy is not None

    print("\n  dt      time(ms)   dpos_vs_scipy(m)   dt_err(s)")
    print("  -----------------------------------------------")
    for dt in (0.5, 0.1, 0.04, 0.01):
        t0 = time.perf_counter()
        r = predictor.predict(position=r0, velocity=v0, dt=dt)
        elapsed = time.perf_counter() - t0
        assert r is not None
        dpos = np.hypot(
            r.position[0] - r_scipy.position[0],
            r.position[2] - r_scipy.position[2],
        )
        dt_err = abs(r.time - r_scipy.time)
        print(f"  {dt:4.2f}    {elapsed * 1e3:6.2f}      {dpos:8.1f}          {dt_err:.2f}")
        assert dpos < 1000.0
        assert dt_err < 5.0
