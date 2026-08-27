"""Regression tests for the snapshot-driven local attitude controller.

The roll convention is kRPC-aligned: ``roll == 0`` aligns the vessel dorsal
axis with the *up* reference (default frame +x), positive roll banks right,
and ``LocalAttitudeController.step`` takes ``roll_target`` in **degrees**.

``_ref_roll`` is an independent reference implementation (arccos + signed
axis projection) used to cross-check ``roll_from_axes`` (atan2 form); both
must agree on the kRPC test cases and on sampled poses.
"""

import math
from collections.abc import Sequence
from typing import Any

import numpy as np
import pytest

from recovery.control import AutoPilot, LocalAttitudeController
from recovery.control.control_utils import normalize
from recovery.control.local_attitude import (
    RollAuthorityEstimator,
    max_acc_from_snapshot,
    roll_from_axes,
)
from recovery.types import FlightState, Quaternion, Situation, TorquePair, Vector3


def _ref_roll(
    direction: Sequence[float],
    bottom: Sequence[float],
    up: Sequence[float] = (1.0, 0.0, 0.0),
) -> float | None:
    """Independent roll reference: arccos of the alignment plus the signed
    rotation direction about the nose (kRPC convention)."""
    nose = np.asarray(direction, dtype=float)
    n = np.linalg.norm(nose)
    if n == 0.0:
        return None
    nose = nose / n
    dorsal = -np.asarray(bottom, dtype=float)
    # Same perpendicular projection as the production formula: the nose
    # component of the dorsal axis does not contribute to roll.
    dorsal = dorsal - np.dot(dorsal, nose) * nose
    dorsal = dorsal / np.linalg.norm(dorsal)
    u = np.asarray(up, dtype=float)
    u_perp = u - np.dot(u, nose) * nose
    n_up = np.linalg.norm(u_perp)
    if n_up < 1e-9:
        return None
    u_perp = u_perp / n_up
    ang = math.acos(np.clip(np.dot(u_perp, dorsal), -1.0, 1.0))
    sgn = np.sign(np.dot(np.cross(dorsal, u_perp), nose))
    if sgn == 0.0:
        sgn = 1.0
    return float(ang * sgn)


def _ref_max_acc(s: FlightState) -> tuple[float, float, float]:
    torques = [
        np.abs(s.available_reaction_wheel_torque.negative),
        np.abs(s.available_rcs_torque.negative),
        np.abs(s.available_engine_torque.negative),
        np.abs(s.available_control_surface_torque.negative),
    ]
    moi = np.array(s.moment_of_inertia)
    acc = (sum(torques) / moi).tolist()
    return (acc[1], acc[2], acc[0])


def _state(
    *,
    ut: float,
    direction: Sequence[float],
    bottom_axis: Sequence[float],
    angular_velocity: Sequence[float],
    rw: Sequence[Sequence[float]] = ((1000.0, 800.0, 500.0), (1000.0, 800.0, 500.0)),
    rcs: Sequence[Sequence[float]] = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    eng: Sequence[Sequence[float]] = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    cs: Sequence[Sequence[float]] = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    moi: Sequence[float] = (1000.0, 1000.0, 1000.0),
) -> FlightState:
    return FlightState(
        ut=ut,
        met=0.0,
        position=Vector3(0.0, 0.0, 0.0),
        velocity=Vector3(0.0, 0.0, 0.0),
        velocity_surface=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(*angular_velocity),
        altitude=0.0,
        surface_altitude=0.0,
        mass=1.0,
        dry_mass=1.0,
        thrust=0.0,
        available_thrust=0.0,
        max_thrust=0.0,
        max_vacuum_thrust=0.0,
        specific_impulse=0.0,
        max_acceleration=0.0,
        throttle=0.0,
        situation=Situation.FLYING,
        loaded=True,
        packed=False,
        landed=False,
        atmosphere_density=0.0,
        atmosphere_depth=0.0,
        frame=None,
        direction=Vector3(*direction),
        bottom_axis=Vector3(*bottom_axis),
        available_reaction_wheel_torque=TorquePair(Vector3(*rw[0]), Vector3(*rw[1])),
        available_rcs_torque=TorquePair(Vector3(*rcs[0]), Vector3(*rcs[1])),
        available_engine_torque=TorquePair(Vector3(*eng[0]), Vector3(*eng[1])),
        available_control_surface_torque=TorquePair(Vector3(*cs[0]), Vector3(*cs[1])),
        moment_of_inertia=Vector3(*moi),
    )


def test_roll_from_axes_krpc_cases() -> None:
    """kRPC-convention anchor cases (nose = +z, default up = +x)."""
    # bottom = -x -> dorsal = +x (north, aligned with up) -> roll 0
    assert roll_from_axes((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0)) == pytest.approx(0.0, abs=1e-9)
    # bottom = +x -> dorsal = -x -> ±180
    assert abs(roll_from_axes((0.0, 0.0, 1.0), (1.0, 0.0, 0.0))) == pytest.approx(math.pi, abs=1e-9)
    # bottom = +y -> dorsal = -y -> +90 (kRPC positive roll)
    assert roll_from_axes((0.0, 0.0, 1.0), (0.0, 1.0, 0.0)) == pytest.approx(math.pi / 2, abs=1e-9)
    # bottom = -y -> dorsal = +y -> -90
    assert roll_from_axes((0.0, 0.0, 1.0), (0.0, -1.0, 0.0)) == pytest.approx(-math.pi / 2, abs=1e-9)


def test_roll_from_axes_up_parallel_nose_returns_none() -> None:
    assert roll_from_axes((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), up=(1.0, 0.0, 0.0)) is None
    assert roll_from_axes((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), up=(-1.0, 0.0, 0.0)) is None


def test_roll_from_axes_matches_reference() -> None:
    cases = [
        ((0.0, 0.0, 1.0), (0.0, -1.0, 0.0)),
        ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),
        ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0)),
        ((0.0, 0.0, 1.0), (0.5, 0.5, 0.0)),
        ((0.5, 0.5, 0.0), (0.0, 0.0, 1.0)),
        ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0)),
        ((-0.3, 0.4, 0.9), (0.1, 0.2, -0.5)),
        ((0.7, -0.2, 0.6), (-0.4, 0.8, 0.1)),
    ]
    for direction, bottom in cases:
        got = roll_from_axes(direction, bottom)
        want = _ref_roll(direction, bottom)
        if want is None:
            assert got is None
        else:
            assert got is not None
            assert float(got) == pytest.approx(float(want), abs=1e-9)


def test_max_acc_from_snapshot_matches_reference() -> None:
    s = _state(
        ut=0.0,
        direction=(1.0, 0.0, 0.0),
        bottom_axis=(0.0, 1.0, 0.0),
        angular_velocity=(0.0, 0.0, 0.0),
        rw=((1000.0, 800.0, 500.0), (1000.0, 800.0, 500.0)),
        rcs=((500.0, 400.0, 300.0), (500.0, 400.0, 300.0)),
        cs=((200.0, 100.0, 50.0), (200.0, 100.0, 50.0)),
        moi=(1000.0, 2000.0, 3000.0),
    )
    got = max_acc_from_snapshot(s)
    want = _ref_max_acc(s)
    assert tuple(got) == tuple(float(v) for v in want)


def test_step_retunes_max_acc_by_game_time() -> None:
    target = (0.0, -1.0, 0.0)
    ctrl = LocalAttitudeController(config_interval=0.5)
    base: dict[str, Any] = dict(
        direction=(0.707, 0.707, 0.0), bottom_axis=(0.0, 0.0, 1.0),
        angular_velocity=(0.1, -0.2, 0.3),
    )
    out_a = ctrl.step(_state(ut=0.0, **base), target)
    # Same attitude, different torque, but ut advanced only 0.2s -> no re-tune.
    out_b = ctrl.step(
        _state(ut=0.2, rw=((5000.0, 5000.0, 5000.0), (5000.0, 5000.0, 5000.0)), **base),
        target,
    )
    assert out_a == out_b
    # ut advanced >= 0.5s -> max_acc re-tuned -> output changes.
    out_c = ctrl.step(
        _state(ut=0.7, rw=((5000.0, 5000.0, 5000.0), (5000.0, 5000.0, 5000.0)), **base),
        target,
    )
    assert out_c != out_a


def test_step_roll_target_switches_branch() -> None:
    target = (0.0, -1.0, 0.0)
    ctrl = LocalAttitudeController()
    s = _state(
        ut=0.0, direction=(0.0, 0.0, 1.0), bottom_axis=(0.0, 1.0, 0.0),
        angular_velocity=(0.0, 0.0, 0.0),
    )  # dorsal = -y -> current roll = +90°, zero roll rate
    damp = ctrl.step(s, target)
    hold = ctrl.step(s, target, roll_target=0.0)
    # damping branch sees no roll rate -> no roll command; the angle-hold
    # branch must actively roll back toward 0°.
    assert damp.roll == pytest.approx(0.0, abs=1e-9)
    assert hold.roll != pytest.approx(0.0, abs=1e-9)


def test_step_roll_target_degrees_convention() -> None:
    """roll_target is in degrees; 90° rolls toward +90, 0° holds at zero."""
    target = (0.0, 0.0, 1.0)
    ctrl = LocalAttitudeController()
    s = _state(
        ut=0.0,
        direction=(0.0, 0.0, 1.0),   # nose at +z (zenith)
        bottom_axis=(-1.0, 0.0, 0.0),  # dorsal = +x (north) -> current roll = 0°
        angular_velocity=(0.0, 0.0, 0.0),
    )
    hold_zero = ctrl.step(s, target, roll_target=0.0)
    hold_90 = ctrl.step(s, target, roll_target=90.0)
    hold_neg90 = ctrl.step(s, target, roll_target=-90.0)
    # Roll stick sign follows the commanded direction; zero target is the
    # smallest command.
    assert abs(hold_zero.roll) < abs(hold_90.roll)
    assert hold_90.roll > 0.0
    assert hold_neg90.roll < 0.0


def test_step_up_parallel_nose_warns_and_degrades() -> None:
    """up parallel to the nose degrades the roll channel to rate-only."""
    target = (0.0, 0.0, 1.0)
    ctrl = LocalAttitudeController()
    s = _state(
        ut=0.0,
        direction=(1.0, 0.0, 0.0),   # nose = +x = default up -> singular
        bottom_axis=(0.0, 1.0, 0.0),
        angular_velocity=(0.5, 0.0, 0.0),
    )
    with pytest.warns(RuntimeWarning, match="parallel to the nose"):
        cmd_hold = ctrl.step(s, target, roll_target=45.0)
    cmd_damp = ctrl.step(s, target)  # no second warning
    # Degraded: the commanded roll angle must be ignored (rate-only), so
    # both calls produce the same roll stick.
    assert cmd_hold.roll == pytest.approx(cmd_damp.roll)


def test_update_basis_zero_roll_aligns_dorsal_with_up() -> None:
    """AutoPilot.update builds y_ (bottom) = -u_perp at roll 0."""
    ap = AutoPilot()
    ap.update_max_acc((1.0, 1.0, 1.0))
    # nose = +z, up = +x -> u_perp = +x; at roll 0 bottom basis = -x
    out = ap.update(
        (0.0, 0.0, 0.0, 1.0),
        (None, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0),
        rot_flag=-1,
        roll_flag=1.0,
        up=(1.0, 0.0, 0.0),
        debug=True,
    )
    _, y_, _, _ = out[3], out[4], out[5], out[6]
    np.testing.assert_allclose(y_, (-1.0, 0.0, 0.0), atol=1e-9)


def test_roll_authority_estimator_recovers_max_acc_from_saturated_ramp() -> None:
    """A saturated slew (stick=±1, rate ramping at the real authority) lets
    the estimator recover the true max angular acceleration."""
    est = RollAuthorityEstimator(stick_threshold=0.5, tau=0.2)
    real = 3.0  # rad/s² the vessel actually delivers
    rate = 0.0
    ut = 0.0
    dt = 0.02
    est.update(ut, rate, 1.0)  # seed previous tick
    for _ in range(200):  # 4 s saturated ramp
        ut += dt
        rate += real * dt
        est.update(ut, rate, 1.0)
    assert est.estimate == pytest.approx(real, rel=0.02)
    assert est.measurements > 0


def test_roll_authority_estimator_ignores_coast_and_sign_mismatch() -> None:
    """Small commands, sign mismatches and telemetry gaps must not feed the
    estimate — those ratios are noise, not authority."""
    est = RollAuthorityEstimator()
    assert est.update(0.0, 0.0, 0.0) is None  # seeds prev tick
    assert est.update(0.02, 0.01, 0.05) is None  # |stick| < threshold
    assert est.update(0.04, 0.02, -0.5) is None  # acc & stick sign disagree
    assert est.update(0.30, 0.02, 0.8) is None  # dt > max_dt (stall)
    assert est.measurements == 0
    assert est.skipped == 1


def test_step_self_tunes_roll_authority_from_large_slew() -> None:
    """Plan A: a large roll error saturates the stick, the self-tuner measures
    the real roll authority in flight, and the overrated theoretical estimate
    converges to it — closing the gain gap the limit cycle needs."""
    real = 3.0  # rad/s² the vessel actually delivers (Booster 1 order)
    theo = 10.0  # overrated theoretical estimate (~3x)
    ctrl = LocalAttitudeController(self_tune_roll=True)
    theta = math.radians(130.0)  # boosterback-sized initial roll error
    theta_dot = 0.0
    dt = 1.0 / 50.0
    ut = 0.0
    target = (0.0, 0.0, 1.0)
    rw = ((1000.0, 1000.0 * theo, 500.0), (1000.0, 1000.0 * theo, 500.0))
    for _ in range(int(30.0 / dt)):
        ut += dt
        dorsal = (math.cos(theta), -math.sin(theta), 0.0)
        s = _state(
            ut=ut,
            direction=(0.0, 0.0, 1.0),
            bottom_axis=(-dorsal[0], -dorsal[1], 0.0),
            angular_velocity=(0.0, 0.0, -theta_dot),
            rw=rw,
        )
        cmd = ctrl.step(s, target, roll_target=0.0)
        theta_dot += cmd.roll * real * dt
        theta += theta_dot * dt
    assert ctrl.roll_authority_measurements > 0
    assert ctrl.roll_max_acc == pytest.approx(real, rel=0.1)
    assert abs(theta) < math.radians(1.0)
