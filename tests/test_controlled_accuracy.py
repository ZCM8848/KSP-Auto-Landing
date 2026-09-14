"""Accuracy of :class:`ControlledPredictor` vs a scipy variable-mass reference.

The production predictor (:func:`recovery.guidance._numba.rk4_controlled`)
uses a fixed-step RK4 with an operator split: mass is held constant within
each RK4 step and decremented by forward Euler afterwards.  In the powered
regime this is **1st-order (linear) in ``dt``**: the step-top mass is
systematically higher than the step-average, so the thrust acceleration is
biased low by an amount proportional to ``dt``; integrated over the burn that
constant bias produces a position error proportional to ``dt`` (measured
``dpos ≈ 3.9·dt`` for the ConstantThrottle endpoint).  These tests quantify
the error against an adaptive RK45 reference
(:class:`tests.scipy_controlled.ScipyControlledPredictor`) that integrates
``[r, v, m]`` as a continuous state, and confirm the linear ``dt`` scaling.

Scenarios (mirroring ``tests/test_controlled.py``):

* ConstantThrottle retrograde burn stopped at the ``v·up = 0`` endpoint —
  isolates the frozen-mass effect (throttle, hence ``mdot``, is constant).
* BrakeToThrottle suicide burn to the surface — exercises the state-dependent
  throttle law (combined scheme error).
* A tilted-FixedNose burn under a :class:`LiftTableModel` — exercises the
  attitude-dependent aero path in both integrators.

The reference itself is first sanity-checked against an analytic free-fall
solution, so the ground truth is validated before it is used.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from recovery.guidance import (
    BrakeToThrottle,
    ConstantThrottle,
    ControlledPredictor,
    ControlSegment,
    FixedNose,
    LiftDragModel,
    LiftTableModel,
    RetrogradeNose,
    Trigger,
    VirtualControl,
)
from recovery.specs import BodySpec
from recovery.types import Vector3
from tests.scipy_controlled import ScipyControlledPredictor

MU = 3.5316e12
R = 600_000.0
BETA = 1900.0
M0 = 22800.0  # ballistic mass (constant; cd0_area = M0 / BETA)
CENTER = (0.0, 0.0, -R)
THREE = 3 * 269913.0  # three "Reliant"-class engines, full thrust (N)


def _density(h: float) -> float:
    return float(1.225 * np.exp(-max(h, 0.0) / 8500.0))


def _density_table() -> tuple[np.ndarray, np.ndarray]:
    alts = np.linspace(0.0, 80000.0, 400)
    vals = 1.225 * np.exp(-alts / 8500.0)
    return alts, vals


@pytest.fixture
def body() -> BodySpec:
    return BodySpec(mu=MU, omega=(0.0, 0.0, 0.0),
                     body_center=CENTER, body_radius=R, surface_gravity=MU / R**2)


@pytest.fixture
def lift_model() -> LiftDragModel:
    alts, vals = _density_table()
    return LiftDragModel(cd0_area=M0 / BETA, cl_area=0.0, k_ind=0.0, clamp_aoa=1.0,
                         density_fn=_density, body_center=CENTER, sea_level_radius=R,
                         density_alts=alts, density_vals=vals)


@pytest.fixture
def lift_table_model() -> LiftTableModel:
    """A nonlinear lift/drag table with a small positive lift slope."""
    alts, vals = _density_table()
    alpha_pts = np.linspace(0.0, 0.2618, 14)  # 0 .. 15 deg
    # cd0_area ~ M0 / BETA; mild induced drag; small positive body lift.
    cd0 = M0 / BETA
    cl_table = 0.8 * alpha_pts          # C_L·A (m^2), signed
    cd_table = cd0 * (1.0 + 1.5 * alpha_pts**2)
    return LiftTableModel(alpha_pts=alpha_pts.tolist(), cl_table=cl_table.tolist(),
                          cd_table=cd_table.tolist(), clamp_aoa=0.2618,
                          density_fn=_density, body_center=CENTER,
                          sea_level_radius=R, density_alts=alts, density_vals=vals)


# ---------------------------------------------------------------------------
# Reference sanity check: scipy-controlled coast == LandingPredictor (scipy)
# ---------------------------------------------------------------------------


def test_reference_ballistic_matches_analytic(body: BodySpec) -> None:
    """The scipy reference's free-fall time matches the analytic value, so the
    reference is a sound ground truth for the rest of this suite."""
    ref = ScipyControlledPredictor.from_body_spec(body, aero=None, rtol=1e-11, atol=1e-11)
    h = 1000.0
    ctl = VirtualControl((ControlSegment.coast(),), M0)
    r = ref.predict((0.0, 0.0, h), (0.0, 0.0, 0.0), M0, ctl)
    assert r.impact is not None
    t_freefall = float(np.sqrt(2 * h / (MU / (R * R))))
    assert r.impact.time == pytest.approx(t_freefall, rel=0.005)
    assert r.impact.position[0] == pytest.approx(0.0, abs=0.1)
    assert r.impact.position[1] == pytest.approx(0.0, abs=0.1)


# ---------------------------------------------------------------------------
# Scenario A: ConstantThrottle retrograde burn, stop at v·up = 0 endpoint
# ---------------------------------------------------------------------------


def _const_throttle_endpoint_control() -> VirtualControl:
    thrust = M0 * 15.0
    return VirtualControl((
        ControlSegment(throttle=ConstantThrottle(1.0), max_thrust=thrust, isp=300.0,
                       nose=RetrogradeNose()),
    ), dry_mass=10000.0)


def test_constant_throttle_endpoint_matches_scipy(
    body: BodySpec, lift_model: LiftDragModel,
) -> None:
    """At dt=0.1 the frozen-mass endpoint error is sub-metre / sub-m/s,
    and the forward-Euler mass update is near-exact (ConstantThrottle -> mdot
    is constant -> Euler is exact)."""
    controlled = ControlledPredictor.from_body_spec(body, aero=lift_model, dt=0.1, t_max=600.0)
    ref = ScipyControlledPredictor.from_body_spec(body, aero=lift_model, rtol=1e-10, atol=1e-10)

    pos = (0.0, 0.0, 50000.0)
    vel = (0.0, 0.0, -100.0)
    ctl = _const_throttle_endpoint_control()

    tr = controlled.predict(pos, vel, M0, ctl, stop_on_endpoint=True)
    r = ref.predict(pos, vel, M0, ctl, stop_on_endpoint=True)

    assert tr.endpoint is not None and r.endpoint is not None
    dpos = float(np.linalg.norm(np.asarray(tr.endpoint.position) - np.asarray(r.endpoint.position)))
    dvel = float(np.linalg.norm(tr.final_velocity - r.final_velocity))
    dmass = abs(tr.final_mass - r.final_mass)
    print(f"\n[ConstThr endpoint dt=0.1] dpos={dpos:.4e} m  dvel={dvel:.4e} m/s  "
          f"dmass={dmass:.4e} kg")
    assert dpos < 0.5, f"endpoint position error {dpos:.3f} m exceeds 0.5 m"
    assert dvel < 0.5, f"endpoint velocity error {dvel:.3f} m/s exceeds 0.5 m/s"
    assert dmass < 1.0, f"mass error {dmass:.3e} kg (ConstantThrottle mass must be near-exact)"


def test_constant_mass_freeze_isolated(
    body: BodySpec, lift_model: LiftDragModel,
) -> None:
    """With ``isp = 0`` (``mdot = 0`` so mass is constant in BOTH integrators)
    the only numba-vs-scipy difference is RK4 truncation, so the endpoint error
    must collapse to ~RK4 truncation level (sub-millimetre).  This confirms the
    ~0.4 m error seen in ``test_constant_throttle_endpoint_matches_scipy`` is
    entirely the frozen-mass operator split, not a gravity/aero discrepancy."""
    controlled = ControlledPredictor.from_body_spec(body, aero=lift_model, dt=0.1, t_max=600.0)
    ref = ScipyControlledPredictor.from_body_spec(body, aero=lift_model, rtol=1e-10, atol=1e-10)

    pos = (0.0, 0.0, 50000.0)
    vel = (0.0, 0.0, -100.0)
    thrust = M0 * 15.0
    ctl = VirtualControl((
        ControlSegment(throttle=ConstantThrottle(1.0), max_thrust=thrust, isp=0.0,
                       nose=RetrogradeNose()),
    ), dry_mass=10000.0)

    tr = controlled.predict(pos, vel, M0, ctl, stop_on_endpoint=True)
    r = ref.predict(pos, vel, M0, ctl, stop_on_endpoint=True)

    assert tr.endpoint is not None and r.endpoint is not None
    dpos = float(np.linalg.norm(np.asarray(tr.endpoint.position) - np.asarray(r.endpoint.position)))
    print(f"\n[ConstThr isp=0 (mass constant) dt=0.1] dpos={dpos:.4e} m")
    assert dpos < 1e-2, (
        f"with mdot=0 the error should be ~RK4 truncation (<1 cm), got {dpos:.3e} m"
    )


def test_constant_throttle_dt_scaling(
    body: BodySpec, lift_model: LiftDragModel,
) -> None:
    """Position error scales linearly with ``dt`` (operator split, 1st-order) in
    the powered regime.  Sweeps dt and fits log-log; slope must be ~1 once the
    frozen-mass bias dominates the reference tolerance floor."""
    ref = ScipyControlledPredictor.from_body_spec(body, aero=lift_model, rtol=1e-11, atol=1e-11)
    pos = (0.0, 0.0, 50000.0)
    vel = (0.0, 0.0, -100.0)
    ctl = _const_throttle_endpoint_control()
    r = ref.predict(pos, vel, M0, ctl, stop_on_endpoint=True)
    assert r.endpoint is not None
    ref_pos = np.asarray(r.endpoint.position)

    dts = [0.5, 0.1, 0.05, 0.01]
    errs: list[float] = []
    rows = []
    for dt in dts:
        controlled = ControlledPredictor.from_body_spec(body, aero=lift_model, dt=dt, t_max=600.0)
        tr = controlled.predict(pos, vel, M0, ctl, stop_on_endpoint=True)
        assert tr.endpoint is not None
        dpos = float(np.linalg.norm(np.asarray(tr.endpoint.position) - ref_pos))
        errs.append(dpos)
        rows.append(f"  dt={dt:<5}  dpos={dpos:.4e} m")
    print("\n[ConstThr dt-sweep]\n" + "\n".join(rows))

    # monotonic convergence
    for i in range(len(errs) - 1):
        assert errs[i] > errs[i + 1], f"error not decreasing: {errs[i]} -> {errs[i+1]}"

    # log-log slope over points above the reference tolerance floor (~1e-7 m).
    log_dt = np.log(np.array(dts, dtype=float))
    log_err = np.log(np.array(errs, dtype=float))
    mask = np.array(errs) > 1e-7
    if int(mask.sum()) >= 2:
        slope = float(np.polyfit(log_dt[mask], log_err[mask], 1)[0])
        print(f"  fitted slope (log dpos vs log dt) = {slope:.2f}")
        assert 0.8 <= slope <= 1.2, (
            f"dt-scaling slope {slope:.2f} not ~1 (frozen-mass operator split)"
        )


# ---------------------------------------------------------------------------
# Scenario B: BrakeToThrottle suicide burn to the surface (impact)
# ---------------------------------------------------------------------------


def _braketo_suicide_control() -> VirtualControl:
    return VirtualControl((
        ControlSegment.coast(),
        ControlSegment(throttle=BrakeToThrottle(0.1), max_thrust=THREE, isp=300.0,
                       nose=RetrogradeNose(), trigger=Trigger.at_altitude(5000.0)),
    ), dry_mass=10000.0)


def test_braketo_impact_matches_scipy(
    body: BodySpec, lift_model: LiftDragModel,
) -> None:
    """BrakeToThrottle suicide burn: numba vs scipy impact agree to well under
    a metre at dt=0.05 (the production landing-burn step).  The state-dependent
    throttle means the combined scheme error (frozen mass + frozen throttle
    within step) is measured here, not just the mass split."""
    controlled = ControlledPredictor.from_body_spec(body, aero=lift_model, dt=0.05, t_max=600.0)
    ref = ScipyControlledPredictor.from_body_spec(body, aero=lift_model, rtol=1e-10, atol=1e-10)

    pos = (15000.0, 0.0, 50000.0)
    vel = (150.0, 0.0, -500.0)
    ctl = _braketo_suicide_control()

    tr = controlled.predict(pos, vel, M0, ctl)
    r = ref.predict(pos, vel, M0, ctl)

    assert tr.impact is not None and r.impact is not None
    dpos = float(np.linalg.norm(np.asarray(tr.impact.position) - np.asarray(r.impact.position)))
    dvel = float(np.linalg.norm(tr.final_velocity - r.final_velocity))
    dmass = abs(tr.final_mass - r.final_mass)
    print(f"\n[BrakeTo impact dt=0.05] dpos={dpos:.4e} m  dvel={dvel:.4e} m/s  "
          f"dmass={dmass:.4e} kg")
    assert dpos < 1.0, f"impact position error {dpos:.3f} m exceeds 1 m"
    assert dvel < 1.0, f"impact velocity error {dvel:.3f} m/s exceeds 1 m/s"
    # BrakeTo mdot ∝ m (exponential decay); forward-Euler drift is ~0.05% of
    # the mass (a few kg to ~10 kg at dt=0.05).  Loose by design.
    assert dmass < 25.0, f"mass error {dmass:.3f} kg exceeds 25 kg"


# ---------------------------------------------------------------------------
# Scenario C: tilted FixedNose burn under a LiftTableModel (exercises lift)
# ---------------------------------------------------------------------------


def _tilted_lifted_control() -> VirtualControl:
    thrust = M0 * 12.0
    # Nose tilted ~6 deg off retrograde toward +x, so AoA is nonzero and the
    # attitude-dependent lift/drag tables are exercised in both integrators.
    # Retrograde here is +z (v points down), so tilt toward +x keeps +z up.
    tilt = 0.1  # rad
    direction = Vector3(math.sin(tilt), 0.0, math.cos(tilt))
    return VirtualControl((
        ControlSegment(throttle=ConstantThrottle(0.9), max_thrust=thrust, isp=300.0,
                       nose=FixedNose(direction)),
    ), dry_mass=10000.0)


def test_lifted_burn_endpoint_matches_scipy(
    body: BodySpec, lift_table_model: LiftTableModel,
) -> None:
    """A powered burn with a tilted nose under a :class:`LiftTableModel`:
    the scipy reference's per-call ``aero.acceleration()`` matches the numba
    kernel's inline table interpolation, and the frozen-mass error stays small
    at the production step size."""
    controlled = ControlledPredictor.from_body_spec(
        body, aero=lift_table_model, dt=0.1, t_max=600.0)
    ref = ScipyControlledPredictor.from_body_spec(
        body, aero=lift_table_model, rtol=1e-10, atol=1e-10)

    pos = (0.0, 0.0, 50000.0)
    vel = (0.0, 0.0, -100.0)
    ctl = _tilted_lifted_control()

    tr = controlled.predict(pos, vel, M0, ctl, stop_on_endpoint=True)
    r = ref.predict(pos, vel, M0, ctl, stop_on_endpoint=True)

    assert tr.endpoint is not None and r.endpoint is not None
    dpos = float(np.linalg.norm(np.asarray(tr.endpoint.position) - np.asarray(r.endpoint.position)))
    dvel = float(np.linalg.norm(tr.final_velocity - r.final_velocity))
    dmass = abs(tr.final_mass - r.final_mass)
    print(f"\n[Lifted burn dt=0.1] dpos={dpos:.4e} m  dvel={dvel:.4e} m/s  "
          f"dmass={dmass:.4e} kg")
    # A fixed (non-retrograde) nose + longer tilted burn accumulates more of
    # both the mass-freeze bias and RK4 truncation; still sub-2 m and far below
    # the production ignition margin (hundreds of metres).
    assert dpos < 2.0, f"endpoint position error {dpos:.3f} m exceeds 2 m"
    assert dvel < 0.5, f"endpoint velocity error {dvel:.3f} m/s exceeds 0.5 m/s"
