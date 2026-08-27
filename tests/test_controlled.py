"""Tests for the controlled trajectory predictor and its aero/control model.

Covers the two regression guards added with the LiftDragModel + virtual-control
work:

* :class:`LiftDragModel` with zero lift (``cl_area=0``) is ballistically
  equivalent to :class:`DragModel` — same drag acceleration, same impact point.
* The impact point is interpolated to the exact surface crossing, so the
  controlled and ballistic predictors agree (rather than being ~|v|·dt apart).
"""

import numpy as np
import pytest

from recovery.guidance import (
    BrakeToThrottle,
    ConstantThrottle,
    ControlledPredictor,
    ControlSegment,
    FixedNose,
    LandingPredictor,
    LiftDragModel,
    RetrogradeNose,
    TowardTargetNose,
    Trigger,
    UpNose,
    VirtualControl,
)
from recovery.guidance.aerodynamics import DragModel
from recovery.specs import BodySpec

MU = 3.5316e12
R = 600_000.0
BETA = 1900.0
M0 = 22800.0  # ballistic mass (constant; cd0_area = M0 / BETA)

CENTER = (0.0, 0.0, -R)


def _density(h: float) -> float:
    return 1.225 * np.exp(-max(h, 0.0) / 8500.0)


def _density_table():
    alts = np.linspace(0.0, 80000.0, 400)
    vals = 1.225 * np.exp(-alts / 8500.0)
    return alts, vals


@pytest.fixture
def body() -> BodySpec:
    return BodySpec(mu=MU, omega=(0.0, 0.0, 0.0),
                    body_center=CENTER, body_radius=R, surface_gravity=MU / R**2)


@pytest.fixture
def drag_model() -> DragModel:
    alts, vals = _density_table()
    return DragModel(ballistic_coefficient=BETA, density_fn=_density,
                     body_center=CENTER, sea_level_radius=R,
                     density_alts=alts, density_vals=vals)


@pytest.fixture
def lift_model() -> LiftDragModel:
    alts, vals = _density_table()
    return LiftDragModel(cd0_area=M0 / BETA, cl_area=0.0, k_ind=0.0, clamp_aoa=1.0,
                         density_fn=_density, body_center=CENTER, sea_level_radius=R,
                         density_alts=alts, density_vals=vals)


# ---------------------------------------------------------------------------
# Regression: LiftDragModel (cl_area=0) == DragModel
# ---------------------------------------------------------------------------


def test_liftdrag_zero_lift_matches_dragmodel_acceleration(
    drag_model: DragModel, lift_model: LiftDragModel,
) -> None:
    """Zero-lift LiftDragModel and DragModel produce the same drag acceleration.

    cd0_area = M0 / beta, so for retrograde (AoA=0) and constant mass m=M0 the
    two models reduce to the same -0.5*rho*|v|^2/beta term.
    """
    states = [
        ((15000.0, 0.0, 50000.0), (150.0, 0.0, -500.0)),
        ((12000.0, 0.0, 15000.0), (120.0, 0.0, -350.0)),
        ((8000.0, 0.0, 5000.0), (80.0, 0.0, -220.0)),
    ]
    for pos, vel in states:
        p = np.asarray(pos, dtype=float)
        v = np.asarray(vel, dtype=float)
        vm = np.linalg.norm(v)
        nose = -v / vm  # retrograde -> AoA = 0
        a_drag = np.asarray(drag_model.acceleration(p, v), dtype=float)
        a_lift = np.asarray(lift_model.acceleration(p, v, nose, M0), dtype=float)
        np.testing.assert_allclose(a_lift, a_drag, atol=1e-12, rtol=1e-12)


def test_liftdrag_ballistic_impact_matches_dragmodel(
    body: BodySpec, drag_model: DragModel, lift_model: LiftDragModel,
) -> None:
    """A ballistic coast under LiftDragModel lands at the DragModel impact point.

    With the impact-point interpolation, the controlled (LiftDragModel) and
    ballistic (DragModel) predictors agree to well under a metre (previously
    they differed by ~|v|·dt, i.e. tens of metres at dt=0.1).
    """
    landing = LandingPredictor.from_body_spec(body, aero=drag_model)
    controlled = ControlledPredictor.from_body_spec(body, aero=lift_model,
                                                    dt=0.1, t_max=600.0)
    pos = (15000.0, 0.0, 50000.0)
    vel = (150.0, 0.0, -500.0)

    r_ball = landing.predict(position=pos, velocity=vel, dt=0.1)
    tr = controlled.predict(pos, vel, M0, VirtualControl((ControlSegment.coast(),), M0))
    assert r_ball is not None
    assert tr.impact is not None

    d = np.linalg.norm(np.asarray(r_ball.position) - np.asarray(tr.impact.position))
    assert d < 2.0, f"impact points differ by {d:.3f} m"


def test_impact_state_lies_on_surface(body: BodySpec, lift_model: LiftDragModel) -> None:
    """The reported impact position sits on the surface sphere (alt ~ 0)."""
    controlled = ControlledPredictor.from_body_spec(body, aero=lift_model,
                                                    dt=0.1, t_max=600.0)
    tr = controlled.predict((15000.0, 0.0, 50000.0), (150.0, 0.0, -500.0), M0,
                            VirtualControl((ControlSegment.coast(),), M0))
    assert tr.impact is not None
    center = np.asarray(CENTER, dtype=float)
    alt = np.linalg.norm(np.asarray(tr.impact.position) - center) - R
    assert abs(alt) < 0.5, f"impact altitude {alt:.3f} m should be ~0"


# ---------------------------------------------------------------------------
# brake_to throttle rule (merged ENERGY + CONSTANT_DECEL)
# ---------------------------------------------------------------------------


def test_braketo_surface_is_suicide_burn(body: BodySpec, lift_model: LiftDragModel) -> None:
    """brake_to(v_terminal, h_terminal=0) brakes to v_terminal at the surface."""
    controlled = ControlledPredictor.from_body_spec(body, aero=lift_model,
                                                    dt=0.05, t_max=600.0)
    THREE = 3 * 269913.0
    ctl = VirtualControl((
        ControlSegment.coast(),
        ControlSegment(throttle=BrakeToThrottle(0.1), max_thrust=THREE, isp=300.0,
                       nose=RetrogradeNose(), trigger=Trigger.at_altitude(5000.0)),
    ), dry_mass=10000.0)
    tr = controlled.predict((15000.0, 0.0, 50000.0), (150.0, 0.0, -500.0), M0, ctl)
    assert tr.impact is not None
    v_td = float(np.linalg.norm(tr.final_velocity))
    assert v_td < 1.0, f"touchdown {v_td:.2f} m/s should be a soft landing"


def test_braketo_reaches_v_terminal_at_h_terminal(
    body: BodySpec, lift_model: LiftDragModel,
) -> None:
    """brake_to(v_terminal, h_terminal>0) reduces vertical speed to v_terminal
    by the time the vessel descends through h_terminal (the handoff point)."""
    controlled = ControlledPredictor.from_body_spec(body, aero=lift_model,
                                                    dt=0.05, t_max=600.0)
    THREE = 3 * 269913.0
    vt, ht = 140.0, 1000.0
    ctl = VirtualControl((
        ControlSegment.coast(),
        ControlSegment(throttle=BrakeToThrottle(vt, ht), max_thrust=THREE, isp=300.0,
                       nose=RetrogradeNose(), trigger=Trigger.at_altitude(5000.0)),
    ), dry_mass=10000.0)
    tr = controlled.predict((15000.0, 0.0, 50000.0), (150.0, 0.0, -500.0), M0, ctl)
    center = np.asarray(CENTER, dtype=float)
    alts = np.linalg.norm(tr.positions - center, axis=1) - R
    # first step where altitude descends below the handoff altitude
    crossing = int(np.argmax(alts < ht))
    assert crossing > 0, "vessel never reached the handoff altitude"
    # up is +z in this frame, so the vertical speed is |vz|.
    vrad = abs(float(tr.velocities[crossing, 2]))
    assert abs(vrad - vt) < 0.15 * vt, (
        f"vertical speed {vrad:.1f} m/s at h_terminal should be ~{vt} m/s"
    )


# ---------------------------------------------------------------------------
# Union API construction
# ---------------------------------------------------------------------------


def test_union_rules_construct_and_predict(body: BodySpec, lift_model: LiftDragModel) -> None:
    """Every nose-rule variant and throttle-rule variant constructs and predicts."""
    controlled = ControlledPredictor.from_body_spec(body, aero=lift_model,
                                                    dt=0.1, t_max=600.0)
    pos = (15000.0, 0.0, 50000.0)
    vel = (150.0, 0.0, -500.0)
    for nose in (RetrogradeNose(), UpNose(), TowardTargetNose(),
                 FixedNose((0.0, 0.0, 1.0))):
        tr = controlled.predict(pos, vel, M0,
                                VirtualControl((ControlSegment.coast(nose=nose),), M0))
        assert tr.impact is not None

    for throttle in (ConstantThrottle(0.0), ConstantThrottle(0.5), BrakeToThrottle(0.1)):
        seg = ControlSegment(throttle=throttle, max_thrust=0.0, isp=0.0,
                             nose=RetrogradeNose())
        tr = controlled.predict(pos, vel, M0, VirtualControl((seg,), M0))
        assert tr.impact is not None


def test_constant_throttle_out_of_range_rejected() -> None:
    """ConstantThrottle must be in 0..1."""
    with pytest.raises(ValueError):
        VirtualControl((ControlSegment(throttle=ConstantThrottle(1.5), max_thrust=0.0,
                                       isp=0.0, nose=RetrogradeNose()),), 10000.0)


# ---------------------------------------------------------------------------
# Endpoint termination (stop_on_endpoint)
# ---------------------------------------------------------------------------


def test_stop_on_endpoint_halts_at_v_dot_up_zero(body: BodySpec, lift_model: LiftDragModel) -> None:
    """With ``stop_on_endpoint=True`` the propagator stops when radial velocity
    crosses zero from below, and the reported endpoint state has v_z ~ 0.
    """
    controlled = ControlledPredictor.from_body_spec(body, aero=lift_model,
                                                    dt=0.1, t_max=600.0)
    pos = (0.0, 0.0, 50000.0)
    vel = (0.0, 0.0, -100.0)
    thrust = M0 * 15.0
    ctl = VirtualControl((
        ControlSegment(throttle=ConstantThrottle(1.0), max_thrust=thrust,
                       isp=300.0, nose=RetrogradeNose()),
    ), dry_mass=10000.0)

    tr = controlled.predict(pos, vel, M0, ctl, stop_on_endpoint=True)
    assert tr.endpoint is not None
    assert tr.impact is None
    endpoint_v = tr.final_velocity
    assert abs(float(endpoint_v[2])) < 1.0, (
        f"endpoint vertical velocity {endpoint_v[2]:.3f} m/s should be ~0"
    )

    center = np.asarray(CENTER, dtype=float)
    endpoint_alt = float(np.linalg.norm(np.asarray(tr.final_position) - center) - R)
    assert 48500.0 < endpoint_alt < 50000.0, f"endpoint altitude {endpoint_alt:.1f} m out of range"


def test_endpoint_impact_without_crossing(body: BodySpec, lift_model: LiftDragModel) -> None:
    r"""With ``stop_on_endpoint=True`` but insufficient thrust, the vessel hits the
    surface before ``v \cdot up`` can cross zero; the impact point is still
    reported.
    """
    controlled = ControlledPredictor.from_body_spec(body, aero=lift_model,
                                                    dt=0.1, t_max=600.0)
    pos = (0.0, 0.0, 50000.0)
    vel = (0.0, 0.0, -1000.0)
    ctl = VirtualControl((ControlSegment.coast(),), dry_mass=10000.0)

    tr = controlled.predict(pos, vel, M0, ctl, stop_on_endpoint=True)
    assert tr.endpoint is None
    assert tr.impact is not None
