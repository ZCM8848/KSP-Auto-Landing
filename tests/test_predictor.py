"""Tests for the ControlledPredictor ballistic / coast impact prediction.

After the predictor unification the ballistic case is a coast segment
(``throttle = 0``) under a drag-only :class:`LiftDragModel`; with ``aero=None``
it is pure gravity + Coriolis + centrifugal.  Accuracy against a near-exact
adaptive reference is provided by
:class:`tests.scipy_controlled.ScipyControlledPredictor`; these tests focus on
analytic geometry, the sampling -> model path, and numba kernel boundaries.
"""

from __future__ import annotations

import time
from unittest.mock import MagicMock

import numpy as np
import pytest

from recovery.guidance import (
    ControlledPredictor,
    ControlSegment,
    LiftDragModel,
    RetrogradeNose,
    VirtualControl,
)
from recovery.guidance._numba import _interp_jit
from recovery.ksp.sampling import sample_drag_spec
from recovery.specs import BodySpec
from recovery.types import FlightState, Quaternion, Situation, Vector3
from tests.scipy_controlled import ScipyControlledPredictor

MU = 3.5316e12
R = 600_000.0
G0 = MU / (R * R)  # ~9.81 m/s^2
BETA = 1900.0
M0 = 22800.0
DRY = 10000.0
CENTER = (0.0, 0.0, -R)


def _density(h: float) -> float:
    return float(1.225 * np.exp(-max(h, 0.0) / 8500.0))


def _density_table() -> tuple[np.ndarray, np.ndarray]:
    alts = np.linspace(0.0, 80000.0, 400)
    vals = 1.225 * np.exp(-alts / 8500.0)
    return alts, vals


def _coast(dry_mass: float = DRY) -> VirtualControl:
    return VirtualControl((ControlSegment.coast(nose=RetrogradeNose()),), dry_mass)


def _impact(
    predictor: ControlledPredictor,
    pos: tuple[float, float, float],
    vel: tuple[float, float, float],
    *,
    mass: float = M0,
    dt: float | None = None,
    t_max: float | None = None,
):
    tr = predictor.predict(pos, vel, mass, _coast(), dt=dt, t_max=t_max)
    return tr.impact


@pytest.fixture
def body() -> BodySpec:
    return BodySpec(mu=MU, omega=(0.0, 0.0, 0.0), body_center=CENTER,
                     body_radius=R, surface_gravity=G0)


@pytest.fixture
def aero() -> LiftDragModel:
    alts, vals = _density_table()
    return LiftDragModel(cd0_area=M0 / BETA, cl_area=0.0, k_ind=0.0, clamp_aoa=1.0,
                          density_fn=_density, body_center=CENTER, sea_level_radius=R,
                          density_alts=alts, density_vals=vals)


@pytest.fixture
def predictor(body: BodySpec) -> ControlledPredictor:
    """Pure-ballistic predictor (no aero), tight step for analytic checks."""
    return ControlledPredictor.from_body_spec(body, dt=0.04)


@pytest.fixture
def drag_predictor(body: BodySpec, aero: LiftDragModel) -> ControlledPredictor:
    return ControlledPredictor.from_body_spec(body, aero=aero, dt=0.04)


# ---------------------------------------------------------------------------
# Analytic geometry (no aero, no rotation)
# ---------------------------------------------------------------------------


def test_straight_drop(predictor: ControlledPredictor) -> None:
    """From altitude h with zero velocity — lands directly below in ~free-fall."""
    h = 1000.0
    r = _impact(predictor, (0.0, 0.0, h), (0.0, 0.0, 0.0))
    assert r is not None
    assert r.position[0] == pytest.approx(0.0, abs=0.1)
    assert r.position[1] == pytest.approx(0.0, abs=0.1)
    assert r.position[2] == pytest.approx(0.0, abs=1.0)
    t_freefall = float(np.sqrt(2 * h / G0))
    assert r.time == pytest.approx(t_freefall, rel=0.01)


def test_horizontal_throw(predictor: ControlledPredictor) -> None:
    """Launch horizontally from altitude — verify x displacement."""
    h = 500.0
    vx = 200.0
    r = _impact(predictor, (0.0, 0.0, h), (vx, 0.0, 0.0))
    assert r is not None
    t_fall = float(np.sqrt(2 * h / G0))
    expected_x = vx * t_fall
    assert r.position[0] == pytest.approx(expected_x, rel=0.02)
    assert r.position[1] == pytest.approx(0.0, abs=0.1)
    assert r.position[2] == pytest.approx(0.0, abs=abs(expected_x * 0.01))


def test_not_hitting_surface(predictor: ControlledPredictor) -> None:
    """t_max too short -> no surface crossing -> impact is None."""
    r = _impact(predictor, (0.0, 0.0, 5000.0), (0.0, 0.0, 0.0), t_max=1.0)
    assert r is None


def test_with_rotation() -> None:
    """Coriolis deflects a falling object off the vertical."""
    body_rot = BodySpec(mu=MU, omega=(0.0, 0.0001, 0.0), body_center=CENTER,
                        body_radius=R, surface_gravity=G0)
    predictor_rot = ControlledPredictor.from_body_spec(body_rot, dt=0.04)
    r = _impact(predictor_rot, (0.0, 0.0, 1000.0), (0.0, 0.0, 0.0))
    assert r is not None
    deflected = not pytest.approx(0.0, abs=1e-3) == r.position[0]
    assert deflected or not pytest.approx(0.0, abs=1e-3) == r.position[1]


def test_predict_from_snapshot(predictor: ControlledPredictor) -> None:
    """predict_from accepts a FlightState directly."""
    state = FlightState(
        ut=0, met=0,
        position=Vector3(0, 0, 500), velocity=Vector3(200, 0, 0),
        velocity_surface=Vector3(0, 0, 0), rotation=Quaternion(0, 0, 0, 1),
        angular_velocity=Vector3(0, 0, 0), altitude=500, surface_altitude=500,
        mass=M0, dry_mass=DRY, thrust=0, available_thrust=0, max_thrust=0,
        max_vacuum_thrust=0, specific_impulse=0, max_acceleration=0, throttle=0,
        situation=Situation.FLYING, loaded=True, packed=False, landed=False,
        atmosphere_density=0, atmosphere_depth=0, frame=None,
    )
    tr = predictor.predict_from(state, _coast())
    assert tr.impact is not None
    assert tr.impact.position[2] == pytest.approx(0, abs=5)


# ---------------------------------------------------------------------------
# Drag-only aero model
# ---------------------------------------------------------------------------


def _drag_aero(cd0_area: float, density_fn) -> LiftDragModel:
    return LiftDragModel(cd0_area=cd0_area, cl_area=0.0, k_ind=0.0, clamp_aoa=1.0,
                         density_fn=density_fn, body_center=CENTER, sea_level_radius=R)


def test_drag_acceleration_zero_speed(aero: LiftDragModel) -> None:
    """Drag is zero when velocity is zero."""
    acc = aero.acceleration(np.array([0.0, 0.0, 500.0]), np.array([0.0, 0.0, 0.0]),
                            np.array([0.0, 0.0, 1.0]), M0)
    assert acc == (0.0, 0.0, 0.0)


def test_drag_acceleration_above_atmosphere() -> None:
    """Drag is zero above the atmosphere (zero density)."""
    model = _drag_aero(12.0, lambda h: 0.0)
    acc = model.acceleration(np.array([0.0, 0.0, 100000.0]), np.array([0.0, 0.0, -200.0]),
                              np.array([0.0, 0.0, 1.0]), M0)
    assert acc == (0.0, 0.0, 0.0)


def test_drag_acceleration_direction() -> None:
    """Drag opposes velocity — a downward velocity gets an upward acceleration."""
    rho = 1.2
    cd0_area = M0 / 500.0  # cd0_area/mass = 1/500 -> a = 0.5*rho*v^2/500
    model = _drag_aero(cd0_area, lambda h: rho)
    v = np.array([0.0, 0.0, -100.0])
    nose = np.array([0.0, 0.0, 1.0])  # retrograde to v -> AoA = 0
    acc = model.acceleration(np.array([0.0, 0.0, 500.0]), v, nose, M0)
    mag = 0.5 * rho * 100.0 * 100.0 / 500.0
    assert acc == pytest.approx((0.0, 0.0, mag), abs=1e-9)


def test_drag_slows_impact(
    predictor: ControlledPredictor, drag_predictor: ControlledPredictor,
) -> None:
    """A vessel with drag takes longer to hit the ground than ballistic."""
    pos = (0.0, 0.0, 5000.0)
    vel = (0.0, 0.0, -200.0)
    r_b = _impact(predictor, pos, vel)
    r_d = _impact(drag_predictor, pos, vel)
    assert r_b is not None and r_d is not None
    assert r_d.time > r_b.time, (
        f"drag {r_d.time:.2f}s should exceed ballistic {r_b.time:.2f}s"
    )


def test_zero_cd0_area_gives_zero_drag() -> None:
    """cd0_area = 0 (no drag area) produces zero drag regardless of velocity."""
    model = _drag_aero(0.0, lambda h: 1.225)
    acc = model.acceleration(np.array([0.0, 0.0, 500.0]), np.array([0.0, 0.0, -200.0]),
                             np.array([0.0, 0.0, 1.0]), M0)
    assert acc == (0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# sample_drag_spec -> LiftDragModel.from_drag_spec
# ---------------------------------------------------------------------------


def test_sample_drag_spec_far() -> None:
    """sample_drag_spec (FAR) + LiftDragModel.from_drag_spec builds a model."""
    mock_body = MagicMock()
    mock_body.equatorial_radius = float(R)
    mock_body.atmosphere_depth = 80000.0
    mock_body.position.return_value = (0.0, 0.0, -float(R))
    scale_h = 5600.0
    rho0 = 1.225
    mock_body.density_at.side_effect = lambda h: rho0 * np.exp(-float(h) / scale_h)
    far_proxy = MagicMock()
    far_proxy.far_available = True
    mock_flight = MagicMock()
    mock_flight.ballistic_coefficient = 1500.0

    spec = sample_drag_spec(body=mock_body, flight=mock_flight,
                            target_frame=MagicMock(), space_center=far_proxy,
                            altitude_samples=32)
    model = LiftDragModel.from_drag_spec(spec, mass_ref=30000.0)
    nose = np.array([0.0, 0.0, 1.0])
    acc = model.acceleration(np.array([0.0, 0.0, 1000.0]), np.array([0.0, 0.0, -500.0]),
                             nose, 30000.0)
    assert acc[2] > 0.0, f"drag should oppose downward velocity, got {acc}"
    acc_high = model.acceleration(np.array([0.0, 0.0, 90000.0]),
                                  np.array([0.0, 0.0, -500.0]), nose, 30000.0)
    assert acc_high == (0.0, 0.0, 0.0)


def test_sample_drag_spec_no_far_estimation() -> None:
    """Without FAR and with mass, beta is estimated from a simulated force."""
    mock_body = MagicMock()
    mock_body.equatorial_radius = float(R)
    mock_body.atmosphere_depth = 70000.0
    mock_body.position.return_value = (0.0, 0.0, -float(R))
    far_proxy = MagicMock()
    far_proxy.far_available = False
    mock_body.density_at.return_value = 1.0
    mock_flight = MagicMock()
    # Simulated aerodynamic force at the reference state (vertical tail-first
    # descent): the drag opposes the downward velocity, so it points up (+z).
    mock_flight.simulate_aerodynamic_force_at.return_value = (0.0, 0.0, 500.0)

    # beta = mass * rho * v^2 / (2 * drag_mag) = 30000 * 1.0 * 300^2 / 1000 = 2.7e6
    spec = sample_drag_spec(body=mock_body, flight=mock_flight,
                            target_frame=MagicMock(), space_center=far_proxy,
                            mass=30000.0, altitude_samples=32)
    assert spec.ballistic_coefficient == pytest.approx(2.7e6, rel=1e-9)
    model = LiftDragModel.from_drag_spec(spec, mass_ref=30000.0)
    nose = np.array([0.0, 0.0, 1.0])
    acc = model.acceleration(np.array([0.0, 0.0, 1000.0]), np.array([0.0, 0.0, -200.0]),
                             nose, 30000.0)
    assert acc[2] > 0.0


def test_sample_drag_spec_uses_local_sea_level_radius() -> None:
    """sample_drag_spec accepts lat/lon and computes a location-aware sea-level radius."""
    mock_body = MagicMock()
    mock_body.equatorial_radius = float(R)
    mock_body.atmosphere_depth = 70000.0
    mock_body.position.return_value = (0.0, 0.0, -float(R))
    mock_body.density_at.return_value = 1.0
    far_proxy = MagicMock()
    far_proxy.far_available = True
    mock_flight = MagicMock()
    mock_flight.ballistic_coefficient = 1500.0

    spec = sample_drag_spec(body=mock_body, flight=mock_flight,
                            target_frame=MagicMock(), space_center=far_proxy,
                            lat=-0.185, lon=-74.473, altitude_samples=32)
    # KSP bodies are spheres, so the local sea-level radius equals the equatorial radius.
    assert spec.sea_level_radius == pytest.approx(float(R))


# ---------------------------------------------------------------------------
# Numba kernel boundaries
# ---------------------------------------------------------------------------


def test_interp_jit_density_boundaries() -> None:
    """Density interpolation clamps below, zeros above the atmosphere top.

    Regression guard: the old numba interpolator returned the top-of-atmosphere
    density for every altitude above the atmosphere, producing spurious drag in
    the fast RK4 path.  A constant-density profile makes the bug obvious.
    """
    depth = 80000.0
    alts = np.linspace(0.0, depth, 64)
    vals = np.full_like(alts, 1.225)  # constant sea-level density

    assert _interp_jit(50000.0, alts, vals) == pytest.approx(1.225)  # in atmosphere
    assert _interp_jit(float(depth), alts, vals) == pytest.approx(1.225)  # at top
    assert _interp_jit(100000.0, alts, vals) == 0.0  # above -> zero (the bug)
    assert _interp_jit(-50.0, alts, vals) == pytest.approx(1.225)  # below -> clamp


def test_coast_matches_scipy_reference(body: BodySpec, aero: LiftDragModel) -> None:
    """The numba coast+drag impact agrees with the scipy reference."""
    predictor = ControlledPredictor.from_body_spec(body, aero=aero, dt=0.04)
    ref = ScipyControlledPredictor.from_body_spec(body, aero=aero, rtol=1e-10, atol=1e-10)
    ctl = _coast()

    for alt, vx, vz in [(50000, 1200, -600), (10000, 500, -100)]:
        pos = (0.0, 0.0, alt)
        vel = (vx, 0.0, vz)
        tr = predictor.predict(pos, vel, M0, ctl)
        r = ref.predict(pos, vel, M0, ctl)
        assert tr.impact is not None and r.impact is not None
        dpos = float(np.hypot(tr.impact.position[0] - r.impact.position[0],
                              tr.impact.position[2] - r.impact.position[2]))
        dt_err = abs(tr.impact.time - r.impact.time)
        assert dpos < 500.0, f"position diff {dpos:.0f}m too large at alt={alt}"
        assert dt_err < 2.0, f"time diff {dt_err:.2f}s too large at alt={alt}"


# ---------------------------------------------------------------------------
# Construction & dt
# ---------------------------------------------------------------------------


def test_from_body_spec_pure(body: BodySpec) -> None:
    """ControlledPredictor.from_body_spec constructs from pure data, no kRPC."""
    predictor = ControlledPredictor.from_body_spec(body, dt=0.1)
    r = _impact(predictor, (0.0, 0.0, 1000.0), (0.0, 0.0, 0.0))
    assert r is not None
    assert r.position[2] == pytest.approx(0.0, abs=1.0)


def test_drag_from_spec_pure(aero: LiftDragModel) -> None:
    """LiftDragModel.numba_params is populated and drag is non-zero."""
    from recovery.specs import DragSpec

    alts = np.linspace(0.0, 80000.0, 64)
    vals = 1.225 * np.exp(-alts / 5600.0)
    spec = DragSpec(ballistic_coefficient=5000.0, density_alts=alts, density_vals=vals,
                    body_center=CENTER, sea_level_radius=R)
    model = LiftDragModel.from_drag_spec(spec, mass_ref=M0)
    assert model.numba_params is not None
    nose = np.array([0.0, 0.0, 1.0])
    acc = model.acceleration(np.array([0.0, 0.0, 1000.0]), np.array([0.0, 0.0, -200.0]),
                             nose, M0)
    assert acc[2] > 0.0


def test_predict_dt_validation(body: BodySpec) -> None:
    """Constructor and per-call dt must reject non-positive values."""
    with pytest.raises(ValueError):
        ControlledPredictor(mu=MU, omega=(0.0, 0.0, 0.0), body_center=CENTER,
                            body_radius=R, dt=0.0)
    with pytest.raises(ValueError):
        ControlledPredictor(mu=MU, omega=(0.0, 0.0, 0.0), body_center=CENTER,
                            body_radius=R, dt=-0.1)
    predictor = ControlledPredictor(mu=MU, omega=(0.0, 0.0, 0.0), body_center=CENTER,
                                    body_radius=R)
    with pytest.raises(ValueError):
        predictor.predict((0.0, 0.0, 1000.0), (0.0, 0.0, 0.0), M0, _coast(), dt=0.0)


def test_predict_dt_override(body: BodySpec, aero: LiftDragModel) -> None:
    """Per-call dt overrides the constructor default; both reach the surface."""
    predictor = ControlledPredictor.from_body_spec(body, aero=aero, dt=0.04)
    pos = (0.0, 0.0, 10000.0)
    vel = (500.0, 0.0, -100.0)
    r_default = _impact(predictor, pos, vel)
    r_large = _impact(predictor, pos, vel, dt=0.5)
    assert r_default is not None and r_large is not None


def test_dt_speed_accuracy_tradeoff(body: BodySpec, aero: LiftDragModel) -> None:
    """Print the speed/accuracy tradeoff for different numba dt values (coast).

    Reference is the scipy adaptive integrator; assertions are loose, the
    printed table is the primary output.
    """
    predictor = ControlledPredictor.from_body_spec(body, aero=aero)
    ref = ScipyControlledPredictor.from_body_spec(body, aero=aero, rtol=1e-11, atol=1e-11)
    ctl = _coast()
    pos = (0.0, 0.0, 50000.0)
    vel = (1500.0, 0.0, -900.0)
    r_ref = ref.predict(pos, vel, M0, ctl)
    assert r_ref.impact is not None

    print("\n  dt      time(ms)   dpos_vs_scipy(m)   dt_err(s)")
    print("  -----------------------------------------------")
    for dt in (0.5, 0.1, 0.04, 0.01):
        t0 = time.perf_counter()
        tr = predictor.predict(pos, vel, M0, ctl, dt=dt)
        elapsed = time.perf_counter() - t0
        assert tr.impact is not None
        dpos = float(np.hypot(tr.impact.position[0] - r_ref.impact.position[0],
                              tr.impact.position[2] - r_ref.impact.position[2]))
        dt_err = abs(tr.impact.time - r_ref.impact.time)
        print(f"  {dt:4.2f}    {elapsed * 1e3:6.2f}      {dpos:8.1f}          {dt_err:.2f}")
        assert dpos < 1000.0
        assert dt_err < 5.0
