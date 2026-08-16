"""Client-side velocity-profile attitude auto-pilot.

Ported from the legacy control layer; the control law is preserved
bit-for-bit.  Typically driven through the snapshot-friendly
:class:`~recovery.control.local_attitude.LocalAttitudeController`.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Literal, overload

import numpy as np

from .control_utils import angle_between, normalize, rotate
from .dynamics import ApproachingModel


class AutoPilot:
    """Velocity-profile attitude controller.

    Decomposes the pointing error into a roll channel and a direction
    (yaw/pitch) channel, each driven by an :class:`ApproachingModel` and
    normalised by the per-axis maximum angular acceleration.
    """

    def __init__(
        self,
        max_acc: tuple[float, float, float] | None = None,
        max_spd: float = math.radians(360),
        accuracy: float = math.radians(1),
        settling_time: float = 0.5,
        lock_accuracy_ratio: float = 0.85,
    ) -> None:
        self.settling_time = settling_time
        self.dir_model = ApproachingModel(
            -1e-3, -100.0, 0.0, max_spd, accuracy, lock_accuracy_ratio
        )
        self.roll_model = ApproachingModel(
            -1e-3, 0.0, 0.0, max_spd, accuracy, lock_accuracy_ratio
        )
        self.max_acc_ratio = 1.0
        self.max_acc = max_acc
        self.update_max_acc(max_acc)

    def update_max_acc(self, max_acc: tuple[float, float, float] | None) -> None:
        """Update the per-axis maximum angular acceleration ``(roll, yaw, pitch)``."""
        if max_acc is None:
            return
        self.max_acc = max_acc
        self.dir_model.max_acc = float(np.linalg.norm(max_acc[1:3]))
        self.roll_model.max_acc = max_acc[0]

    def update_config(
        self,
        max_acc: tuple[float, float, float] | None = None,
        settling_time: float | None = None,
    ) -> None:
        """Update ``max_acc`` and/or ``settling_time`` (``None`` leaves them unchanged)."""
        if max_acc is not None:
            self.update_max_acc(max_acc)
        if settling_time is not None:
            self.settling_time = settling_time

    @overload
    def update(
        self,
        cur: Sequence[float],
        target: Sequence[float | None],
        angular_velocity: Sequence[float],
        rot_flag: float = 1.0,
        debug: Literal[False] = False,
    ) -> tuple[float, float, float]: ...

    @overload
    def update(
        self,
        cur: Sequence[float],
        target: Sequence[float | None],
        angular_velocity: Sequence[float],
        rot_flag: float = 1.0,
        debug: Literal[True] = True,
    ) -> tuple[float, float, float, np.ndarray, np.ndarray, np.ndarray, np.ndarray]: ...

    def update(
        self,
        cur: Sequence[float],
        target: Sequence[float | None],
        angular_velocity: Sequence[float],
        rot_flag: float = 1.0,
        debug: bool = False,
    ) -> tuple[float, float, float] | tuple[
        float, float, float, np.ndarray, np.ndarray, np.ndarray, np.ndarray
    ]:
        """Compute stick-level acceleration commands.

        Args:
            cur: ``(roll, nose_x, nose_y, nose_z)`` — current attitude.
            target: ``(roll_target | None, dir_x, dir_y, dir_z)`` — desired
                attitude; a ``None`` roll target only damps the roll rate.
            angular_velocity: Current angular velocity (rad/s).
            rot_flag: Sign multiplier for the pointing-error rotation axis.
            debug: When true, also return the computed basis vectors and the
                target direction.

        Returns:
            ``(roll, yaw, pitch)`` command levels, or, with ``debug``, those
            three levels plus ``(x_, y_, z_, target_dir)``.
        """
        roll = cur[0]
        target_roll = target[0] or 0.0
        cur_dir = np.array(cur[1:4])
        target_dir = np.array(target[1:4], dtype=float)
        ang_vel = np.array(angular_velocity)
        x = np.array((1.0, 0.0, 0.0))
        y = np.array((0.0, math.cos(roll), math.sin(roll)))
        x_ = normalize(cur_dir)
        ang = angle_between(x, x_)
        x_rot_axis = normalize(np.cross(x, x_))
        y_ = rotate(x_rot_axis, y, ang)
        z_ = np.cross(x_, y_)
        dir_ang = angle_between(cur_dir, target_dir)
        dir_rot_axis = rot_flag * np.cross(cur_dir, target_dir)
        if np.linalg.norm(dir_rot_axis) == 0.0:
            # Current and target direction are collinear; fall back to y_.
            dir_rot_axis = y_
        dir_rot_axis = normalize(dir_rot_axis)
        # Pointing error along the rotation axis; the sign is folded into the
        # axis so the desired angular velocity is positive along it.
        dir_err = dir_rot_axis * dir_ang
        v_x = np.dot(ang_vel, x_) * x_
        v_yz = ang_vel - v_x
        acc_yz = self.dir_model.next_acc(dir_err, v_yz, self.settling_time)
        if target[0] is None:
            # No roll target: only damp the roll rate.
            acc_x = -v_x / self.settling_time
        else:
            roll_err = target_roll - roll
            v_x_proj = np.dot(v_x, x_)
            if roll_err > math.pi:
                roll_err -= math.pi * 2
            elif roll_err < -math.pi:
                roll_err += math.pi * 2
            roll_err *= rot_flag
            acc_x = self.roll_model.next_acc(roll_err, v_x_proj, self.settling_time) * x_
        max_acc = self.max_acc
        assert max_acc is not None
        acc_level_x = float(np.dot(acc_x, x_) / max_acc[0])
        acc_level_y = float(np.dot(acc_yz, y_) / max_acc[1])
        acc_level_z = float(np.dot(acc_yz, z_) / max_acc[2])
        if not debug:
            return acc_level_x, acc_level_y, acc_level_z
        return acc_level_x, acc_level_y, acc_level_z, x_, y_, z_, target_dir
