"""Regression tests for the snapshot-driven local attitude controller.

The reference helpers below are independent, verbatim copies of the inline
logic that previously lived in ``scripts/zem_boosterback_local.py`` (and, for
``_ref_max_acc``/``_ref_roll``, the legacy ``Rocket.update_ap`` math).  They
guard that the extraction into :class:`LocalAttitudeController` reproduces the
exact same stick outputs.
"""

import math
from collections.abc import Sequence
from typing import Any

import numpy as np
import pytest

from recovery.control import AutoPilot, LocalAttitudeController
from recovery.control.control_utils import angle_between, normalize, rotate
from recovery.control.local_attitude import (
    max_acc_from_snapshot,
    roll_from_axes,
)
from recovery.types import FlightState, Quaternion, Situation, TorquePair, Vector3


def _ref_roll(direction: Sequence[float], bottom: Sequence[float]) -> float:
    x = np.array(direction)
    y = np.array(bottom)
    x0 = np.array((1.0, 0.0, 0.0))
    rot_axis = normalize(np.cross(x, x0))
    rot_ang = angle_between(x, x0)
    y0 = rotate(rot_axis, y, rot_ang)
    ang1 = angle_between(y0, (0.0, 1.0, 0.0))
    ang2 = angle_between(y0, (0.0, 0.0, 1.0))
    roll = ang1
    if ang2 > math.pi / 2:
        roll = -roll
    return roll


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


def _ref_step(
    s: FlightState,
    target_dir: Sequence[float],
    ap: AutoPilot,
) -> tuple[float, float, float]:
    cur_roll = _ref_roll(s.direction, s.bottom_axis)
    return ap.update(
        (cur_roll, *s.direction),
        (None, *target_dir),
        tuple(-c for c in s.angular_velocity),
        rot_flag=-1,
    )


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
        frame=None,
        direction=Vector3(*direction),
        bottom_axis=Vector3(*bottom_axis),
        available_reaction_wheel_torque=TorquePair(Vector3(*rw[0]), Vector3(*rw[1])),
        available_rcs_torque=TorquePair(Vector3(*rcs[0]), Vector3(*rcs[1])),
        available_engine_torque=TorquePair(Vector3(*eng[0]), Vector3(*eng[1])),
        available_control_surface_torque=TorquePair(Vector3(*cs[0]), Vector3(*cs[1])),
        moment_of_inertia=Vector3(*moi),
    )


def test_roll_from_axes_matches_reference() -> None:
    cases = [
        ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        ((0.0, 0.0, -1.0), (0.0, 1.0, 0.0)),
        ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        ((0.5, 0.5, 0.0), (0.0, 0.0, 1.0)),
        ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
        ((-0.3, 0.4, 0.9), (0.1, 0.2, -0.5)),
    ]
    for direction, bottom in cases:
        got = roll_from_axes(Vector3(*direction), Vector3(*bottom))
        want = _ref_roll(direction, bottom)
        # Production uses scipy Rotation while _ref_roll keeps the handwritten
        # Rodrigues baseline; agreement is required to machine precision, not
        # bit-for-bit (float64 rounding order differs).
        assert float(got) == pytest.approx(float(want), abs=1e-12)


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


def test_step_matches_reference() -> None:
    target = (0.0, -1.0, 0.0)
    cases: list[dict[str, Any]] = [
        dict(
            ut=0.0, direction=(1.0, 0.0, 0.0), bottom_axis=(0.0, 1.0, 0.0),
            angular_velocity=(0.0, 0.0, 0.0),
        ),
        dict(
            ut=0.5, direction=(0.707, 0.707, 0.0), bottom_axis=(0.0, 0.0, 1.0),
            angular_velocity=(0.1, -0.2, 0.3),
        ),
        dict(
            ut=1.0, direction=(0.0, 0.0, -1.0), bottom_axis=(0.0, 1.0, 0.0),
            angular_velocity=(-0.4, 0.5, 0.1),
            rw=((2000.0, 1500.0, 900.0), (2000.0, 1500.0, 900.0)),
            moi=(500.0, 800.0, 1200.0),
        ),
        dict(
            ut=1.5, direction=(-0.3, 0.4, 0.9), bottom_axis=(0.1, 0.2, -0.5),
            angular_velocity=(0.7, -0.1, -0.6),
        ),
    ]
    ctrl = LocalAttitudeController()
    ap = AutoPilot(settling_time=0.5)
    for case in cases:
        s = _state(**case)
        ap.update_max_acc(max_acc_from_snapshot(s))
        want = _ref_step(s, target, ap)
        got = ctrl.step(s, target)
        np.testing.assert_allclose(
            np.asarray(got, dtype=float),
            np.asarray(want, dtype=float),
            rtol=1e-9,
            atol=1e-12,
        )


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
        ut=0.0, direction=(1.0, 0.0, 0.0), bottom_axis=(0.0, 0.0, 1.0),
        angular_velocity=(0.3, 0.0, 0.0),
    )
    damp = ctrl.step(s, target)
    hold = ctrl.step(s, target, roll_target=0.0)
    assert damp.roll != hold.roll
