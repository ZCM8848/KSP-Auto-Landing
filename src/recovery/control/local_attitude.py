"""Snapshot-driven local attitude controller.

Wraps :class:`AutoPilot` behind a pure, rate-agnostic :meth:`step` that
consumes a :class:`~recovery.types.FlightState` snapshot and returns raw
stick values.  No kRPC object crosses this boundary — the caller owns the
I/O and the loop timing (the orchestration scheduler).

Roll convention (kRPC-aligned, empirically verified against a live KSP):

* ``roll == 0`` means the vessel dorsal axis is aligned with the *up*
  reference (default: the snapshot frame's +x axis, matching the kRPC
  AutoPilot's default ``up_reference``).  Positive roll banks right.
* ``roll_target`` on :meth:`LocalAttitudeController.step` is in **degrees**
  (matching ``controls.apply(roll_angle=...)``); the internal control law
  stays in radians.
* The singularity is ``up ∥ nose`` (u_perp ≈ 0), not "nose at zenith".
  Near it the roll channel degrades to rate-only damping and warns once.
"""

from __future__ import annotations

import math
import warnings
from collections.abc import Sequence
from typing import NamedTuple

import numpy as np
from scipy.spatial.transform import Rotation

from ..types import FlightState, Vector3
from .auto_pilot import AutoPilot

DEFAULT_UP = (1.0, 0.0, 0.0)  # kRPC default up_reference = frame +x


class StickCommand(NamedTuple):
    """Raw stick outputs ``(roll, yaw, pitch)``, unclipped (kRPC clips at ±1)."""

    roll: float
    yaw: float
    pitch: float


def roll_from_axes(
    direction: Vector3,
    bottom: Vector3,
    up: Sequence[float] = DEFAULT_UP,
) -> float | None:
    """Roll angle (rad) about the nose axis, kRPC convention.

    *direction* is the vessel nose and *bottom* the vessel +z axis, both
    expressed in the snapshot frame; *up* is the roll reference direction
    in the same frame (default ``(1, 0, 0)`` = the frame's +x, matching the
    kRPC ``AutoPilot`` default ``up_reference``).

    ``roll == 0`` when the dorsal axis (``-bottom``) aligns with the
    component of *up* perpendicular to the nose; positive roll is the kRPC
    "banks right" direction.

    Returns ``None`` when *up* is (anti-)parallel to the nose (u_perp ≈ 0):
    the roll angle is undefined there.  Callers should treat ``None`` as
    "degrade to rate-only roll damping" rather than inventing an angle.
    """
    nose = np.asarray(direction, dtype=float)
    nose_norm = np.linalg.norm(nose)
    if nose_norm == 0.0:
        return None
    nose_hat = nose / nose_norm
    dorsal = -np.asarray(bottom, dtype=float)
    # Project out the nose component: only the perpendicular part of the
    # dorsal axis is relevant to roll about the nose.
    dorsal = dorsal - np.dot(dorsal, nose_hat) * nose_hat
    d_norm = np.linalg.norm(dorsal)
    if d_norm < 1e-9:
        return None
    dorsal = dorsal / d_norm
    up_v = np.asarray(up, dtype=float)
    u_perp = up_v - np.dot(up_v, nose_hat) * nose_hat
    up_norm = np.linalg.norm(u_perp)
    if up_norm < 1e-9:
        return None
    u_perp = u_perp / up_norm
    return float(
        math.atan2(
            np.dot(np.cross(dorsal, u_perp), nose_hat),
            np.dot(u_perp, dorsal),
        )
    )


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


class RollAuthorityEstimator:
    """Online roll-axis max-acceleration estimator (self-tuning, "Plan A").

    Measures the angular acceleration the vessel actually delivers about its
    nose and, whenever the roll command is large enough and the vessel is
    rotating the commanded way, folds the implied authority
    (``|acc| / |stick|``) into an EMA.  A few seconds of hard slew therefore
    converge the roll channel's ``max_acc`` from the (over-estimated)
    theoretical torque sum down to the vessel's real roll authority — no
    per-vessel derate constant required, and a vessel whose theoretical
    estimate is already correct is left effectively unchanged.

    The rate is measured in the *controller* convention
    (``v = -dot(angular_velocity, direction)``), the same quantity the
    control law damps, so the stick sign aligns with ``dv/dt`` directly.

    ``estimate`` is ``None`` until the first accepted measurement; the caller
    owns the fallback (the theoretical estimate).  The estimator is
    deliberately silent — a missed tick (telemetry stall, deadband coast)
    is a normal, expected event, not an error; the ``measurements`` and
    ``skipped`` counters are exposed so callers can surface "never latched"
    as a diagnostic if they choose.
    """

    def __init__(
        self,
        *,
        stick_threshold: float = 0.5,
        tau: float = 0.5,
        max_dt: float = 0.25,
        min_estimate: float = 1e-3,
    ) -> None:
        self._stick_threshold = float(stick_threshold)
        self._tau = float(tau)
        self._max_dt = float(max_dt)
        self._min_estimate = float(min_estimate)
        self._estimate: float | None = None
        self._prev_rate: float | None = None
        self._prev_ut: float | None = None
        self.measurements = 0
        self.skipped = 0

    @property
    def estimate(self) -> float | None:
        return self._estimate

    def update(self, ut: float, rate: float, stick: float) -> float | None:
        """Feed one tick of telemetry plus the just-computed roll stick.

        Args:
            ut: Game time of the snapshot (s).
            rate: Controller-convention roll rate (rad/s),
                ``-dot(angular_velocity, direction)``.
            stick: Raw roll command for this tick (clipped to ±1).

        Returns:
            The current estimate (rad/s²), or ``None`` until the first
            accepted measurement.
        """
        stick = float(np.clip(float(stick), -1.0, 1.0))
        if self._prev_ut is None:
            self._prev_ut = float(ut)
            self._prev_rate = float(rate)
            return self._estimate
        dt = float(ut) - self._prev_ut
        prev_rate = self._prev_rate
        self._prev_ut = float(ut)
        self._prev_rate = float(rate)
        if not 0.0 < dt <= self._max_dt:
            self.skipped += 1
            return self._estimate
        acc = (float(rate) - prev_rate) / dt
        # Accept only while the vessel is rotating the commanded way under a
        # meaningful command; the ratio is otherwise dominated by noise (a
        # coast in the deadband, or a command the craft is not yet tracking).
        if abs(stick) >= self._stick_threshold and acc * stick > 0.0:
            implied = abs(acc) / abs(stick)
            alpha = 1.0 - math.exp(-dt / self._tau)
            if self._estimate is None:
                self._estimate = implied
            else:
                self._estimate += alpha * (implied - self._estimate)
            self._estimate = max(self._estimate, self._min_estimate)
            self.measurements += 1
        return self._estimate


class LocalAttitudeController:
    """Pure, snapshot-driven wrapper around :class:`AutoPilot`.

    The control law is rate-agnostic: it uses a fixed settling time, and the
    snapshot's game time (``ut``) to throttle its periodic ``max_acc``
    re-estimation.  The caller decides how often :meth:`step` runs.
    """

    def __init__(
        self,
        *,
        settling_time: float = 0.5,
        config_interval: float = 0.5,
        self_tune_roll: bool = True,
    ) -> None:
        self._ap = AutoPilot(settling_time=settling_time)
        self._config_interval = float(config_interval)
        self._last_cfg_ut = float("-inf")
        self._warned_singular = False
        self._self_tune_roll = bool(self_tune_roll)
        self._roll_est = RollAuthorityEstimator()

    def step(
        self,
        s: FlightState,
        target_dir: Sequence[float],
        *,
        roll_target: float | None = None,
        up: Sequence[float] | None = None,
    ) -> StickCommand:
        """Compute raw stick values to point the nose toward *target_dir*.

        Args:
            s: Latest telemetry snapshot.
            target_dir: Nose direction in the snapshot frame.
            roll_target: Roll angle in **degrees** (kRPC convention):
                ``0`` = dorsal aligned with *up*; positive banks right.
                ``None`` (default) damps the roll rate without holding an
                angle (matches the kRPC AutoPilot with no ``target_roll``).
            up: Roll reference direction in the snapshot frame.  Defaults to
                ``(1, 0, 0)`` (the frame's +x) — the kRPC default
                ``up_reference``.  Pass ``(0, 0, 1)`` in the target frame to
                hold the dorsal toward the zenith.

        The roll channel degrades to rate-only damping (with a one-time
        warning) when *up* is parallel to the nose, where roll is undefined.
        """
        if s.ut - self._last_cfg_ut >= self._config_interval:
            theo = max_acc_from_snapshot(s)
            if self._self_tune_roll:
                est = self._roll_est.estimate
                roll_acc = theo[0] if est is None else est
            else:
                roll_acc = theo[0]
            self._ap.update_max_acc((roll_acc, theo[1], theo[2]))
            self._last_cfg_ut = s.ut

        up_v = np.asarray(DEFAULT_UP if up is None else up, dtype=float)
        nose = np.asarray(s.direction, dtype=float)
        nose_norm = np.linalg.norm(nose)
        singular = nose_norm == 0.0
        if not singular:
            u_perp = up_v - np.dot(up_v, nose) * nose / (nose_norm * nose_norm)
            singular = np.linalg.norm(u_perp) < 1e-9

        if singular:
            if not self._warned_singular:
                warnings.warn(
                    "roll reference is parallel to the nose (roll undefined); "
                    "degrading the roll channel to rate-only damping",
                    RuntimeWarning,
                    stacklevel=2,
                )
                self._warned_singular = True
            effective_roll: float | None = None
        elif roll_target is None:
            effective_roll = None
        else:
            effective_roll = math.radians(float(roll_target))

        cur_roll = roll_from_axes(s.direction, s.bottom_axis, up_v)
        ctrl_x, ctrl_y, ctrl_z = self._ap.update(
            (0.0 if cur_roll is None else cur_roll, s.direction.x, s.direction.y, s.direction.z),
            (effective_roll, *target_dir),
            (-s.angular_velocity.x, -s.angular_velocity.y, -s.angular_velocity.z),
            rot_flag=-1,
            roll_flag=1.0,
            up=up_v,
        )
        if self._self_tune_roll:
            nose = np.asarray(s.direction, dtype=float)
            n = float(np.linalg.norm(nose))
            if n > 0.0:
                self._roll_est.update(
                    s.ut,
                    -float(np.dot(np.asarray(s.angular_velocity), nose / n)),
                    ctrl_x,
                )
        return StickCommand(float(ctrl_x), float(ctrl_y), float(ctrl_z))

    @property
    def roll_max_acc(self) -> float:
        """Roll-axis ``max_acc`` currently in effect (rad/s²); NaN if unset."""
        acc = self._ap.max_acc
        return float(acc[0]) if acc is not None else float("nan")

    @property
    def roll_authority_measurements(self) -> int:
        """In-flight roll-authority measurements the self-tuner has accepted
        (0 while it is still on the theoretical fallback)."""
        return self._roll_est.measurements
