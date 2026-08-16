"""Snapshot-driven local attitude controller.

Wraps the legacy :class:`AutoPilot` behind a pure, rate-agnostic :meth:`step`
that consumes a :class:`~recovery.types.FlightState` snapshot and returns raw
stick values.  No kRPC object crosses this boundary — the caller owns the I/O
and the loop timing (the orchestration scheduler).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import NamedTuple

import numpy as np
from scipy.spatial.transform import Rotation

from ..types import FlightState, Vector3
from .auto_pilot import AutoPilot


class StickCommand(NamedTuple):
    """Raw stick outputs ``(roll, yaw, pitch)``, unclipped (kRPC clips at ±1)."""

    roll: float
    yaw: float
    pitch: float


def roll_from_axes(direction: Vector3, bottom: Vector3) -> float:
    """Roll angle (rad) about the nose axis, matching the legacy convention.

    *direction* is the vessel nose and *bottom* the vessel +z axis, both
    expressed in the snapshot frame.
    """
    x = np.asarray(direction, dtype=float)
    y = np.asarray(bottom, dtype=float)
    x0 = np.array((1.0, 0.0, 0.0))
    # Rotation that aligns the nose *x* with the +x axis.
    axis = np.cross(x, x0)
    n = np.linalg.norm(axis)
    if n < 1e-15:
        # x is (anti-)parallel to +x: no rotation axis exists.  The legacy
        # handwritten Rodrigues formula degenerates to a scaling by cos(ang):
        #   cos(0) = +1 -> *bottom* unchanged ; cos(pi) = -1 -> *bottom* negated.
        y0 = y if np.dot(x, x0) >= 0.0 else -y
    else:
        axis = axis / n
        ang = float(
            np.arccos(np.clip(np.dot(x, x0) / np.linalg.norm(x), -1.0, 1.0))
        )
        y0 = Rotation.from_rotvec(ang * axis).apply(y)
    ang1 = float(
        np.arccos(np.clip(np.dot(y0, (0.0, 1.0, 0.0)) / np.linalg.norm(y0), -1.0, 1.0))
    )
    ang2 = float(
        np.arccos(np.clip(np.dot(y0, (0.0, 0.0, 1.0)) / np.linalg.norm(y0), -1.0, 1.0))
    )
    roll = ang1
    if ang2 > math.pi / 2:
        roll = -roll
    return roll


def max_acc_from_snapshot(s: FlightState) -> tuple[float, float, float]:
    """Per-axis max angular acceleration ``(roll, yaw, pitch)`` in rad/s².

    Reproduces the legacy ``_ap_auto_config`` estimate: element-wise absolute
    of each source's negative-direction torque, summed and divided by the
    moment of inertia, then reordered to ``(roll, yaw, pitch)``.
    """
    torques = [
        np.abs(s.available_reaction_wheel_torque.negative),
        np.abs(s.available_rcs_torque.negative),
        np.abs(s.available_engine_torque.negative),
        np.abs(s.available_control_surface_torque.negative),
    ]
    moi = np.array(s.moment_of_inertia)
    acc = (sum(torques) / moi).tolist()
    return (acc[1], acc[2], acc[0])


class LocalAttitudeController:
    """Pure, snapshot-driven wrapper around the legacy :class:`AutoPilot`.

    The control law is rate-agnostic: it uses a fixed settling time, and the
    snapshot's game time (``ut``) to throttle its periodic ``max_acc``
    re-estimation.  The caller decides how often :meth:`step` runs.
    """

    def __init__(
        self,
        *,
        settling_time: float = 0.5,
        config_interval: float = 0.5,
    ) -> None:
        self._ap = AutoPilot(settling_time=settling_time)
        self._config_interval = float(config_interval)
        self._last_cfg_ut = float("-inf")

    def step(
        self,
        s: FlightState,
        target_dir: Sequence[float],
        *,
        roll_target: float | None = None,
    ) -> StickCommand:
        """Compute raw stick values to point the nose toward *target_dir*.

        ``roll_target=None`` (default) damps the roll rate rather than holding
        an angle; a float holds roll at that angle (radians).
        """
        if s.ut - self._last_cfg_ut >= self._config_interval:
            self._ap.update_max_acc(max_acc_from_snapshot(s))
            self._last_cfg_ut = s.ut

        cur_roll = roll_from_axes(s.direction, s.bottom_axis)
        ctrl_x, ctrl_y, ctrl_z = self._ap.update(
            (cur_roll, s.direction.x, s.direction.y, s.direction.z),
            (roll_target, *target_dir),
            (-s.angular_velocity.x, -s.angular_velocity.y, -s.angular_velocity.z),
            rot_flag=-1,
        )
        return StickCommand(float(ctrl_x), float(ctrl_y), float(ctrl_z))
