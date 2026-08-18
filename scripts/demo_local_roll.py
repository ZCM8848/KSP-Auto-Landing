"""Demonstrate how the local AutoPilot handles roll (requires a live KSP).

Four acts, all flying the same nose direction (TARGET_DIR):

  Act 1 - Reference pose: point the nose via the kRPC AutoPilot at
          target_roll=0 and read the local ``roll_from_axes`` — with the
          kRPC-aligned convention this should read ≈ 0°.
  Act 2 - ``roll_target=None``: roll-rate damping only.  We start rolled
          to target_roll=+90° (kRPC semantics), hand over to the local AP,
          and watch the roll *stay put* - rate is damped to zero, but no
          angle is held.
  Act 3 - ``roll_target=0.0`` (degrees): angle hold.  Same start pose,
          command roll 0°, watch the vessel actively roll back so the
          dorsal aligns with the frame +x (default up) and converge.
  Act 4 - Cross-check: command target_roll = 0/90/180/270° through the
          kRPC AutoPilot and tabulate the local ``roll_from_axes`` reading
          for each.  With the unified convention the offset column should
          read ≈ 0° for every row.

Run from the repo root with the KRPC conda env:

    PYTHONPATH=src /d/miniforge3/envs/KRPC/python.exe scripts/demo_local_roll.py
"""
import math
import time

import numpy as np

from recovery import ConnectionManager, FramePacer
from recovery.control.local_attitude import (
    LocalAttitudeController,
    max_acc_from_snapshot,
    roll_from_axes,
)
from recovery.data.targets import LAUNCHPAD_JNSQ

VESSEL = "Booster 1"
TARGET_DIR = (0.0, 0.0, 1.0)  # nose direction in the target frame
SETTLE_S = 8.0  # wait after commanding a kRPC AP pose (RLV-VTVL rolls slowly)
ACT_S = 8.0  # duration of each local-AP act
SPIN_START_DEG = 90.0  # kRPC target_roll used to build the starting pose


def _local_roll_deg(s) -> float:
    """Local roll reading (deg), kRPC convention: 0 = dorsal aligned with
    the default up (frame +x), positive banks right."""
    return float(np.degrees(roll_from_axes(s.direction, s.bottom_axis)))


def _roll_rate_deg(s) -> float:
    """Roll rate about the nose axis (deg/s): angular velocity projected on the nose."""
    return float(np.degrees(np.dot(np.asarray(s.angular_velocity), np.asarray(s.direction))))


def _pose(controls, frame, *, roll_deg: float, wait_s: float = SETTLE_S) -> None:
    """Point the nose at TARGET_DIR with the given kRPC target_roll, wait, disengage."""
    controls.apply(target_direction=TARGET_DIR, reference_frame=frame, roll_angle=roll_deg)
    ap = controls._auto_pilot  # noqa: SLF001 - read-only peek, matches compare_autopilots style
    deadline = time.monotonic() + wait_s
    settled = False
    while time.monotonic() < deadline:
        if ap.error < 3.0 and abs(ap.roll_error) < 3.0:
            settled = True
            break
        time.sleep(0.1)
    if not settled:
        print(f"  !! pose settle TIMEOUT after {wait_s:.0f}s "
              f"(error={ap.error:.1f}°, roll_error={ap.roll_error:.1f}°)")
    time.sleep(0.5)
    controls.disengage_auto_pilot()
    time.sleep(0.2)


def _bar(value: float, lo: float = -180.0, hi: float = 180.0, width: int = 48) -> str:
    frac = min(1.0, max(0.0, (value - lo) / (hi - lo)))
    n = int(frac * width)
    return "|" + "#" * n + "-" * (width - n) + "|"


def _act_reference(booster, controls, frame):
    """Act 1: settle at kRPC roll 0, show what the local roll reads there."""
    print("\n=== Act 1: reference pose (kRPC target_roll=0) ===")
    _pose(controls, frame, roll_deg=0.0)
    s = booster.snapshot()
    print(f"  local roll_from_axes = {_local_roll_deg(s):+7.1f}°   "
          f"(kRPC roll 0 = dorsal aligned with frame up)")
    print("  -> this reading is your zero-point offset between conventions")


def _act_damping(booster, frame):
    """Act 2: roll_target=None - rate damping only, angle NOT held."""
    print(f"\n=== Act 2: roll_target=None (rate damping only) ===")
    print(f"  start pose: kRPC target_roll=+{SPIN_START_DEG:.0f}°; local AP takes over,")
    print("  expects: roll rate -> 0, roll angle *stays* near the start value")
    ctrl = LocalAttitudeController(settling_time=0.5)
    pacer = FramePacer(hz=50)
    raw = booster.raw
    start = time.monotonic()
    rows = []
    while time.monotonic() - start < ACT_S:
        pacer.tick()
        s = booster.snapshot()
        cmd = ctrl.step(s, TARGET_DIR, roll_target=None)
        raw.control.roll = cmd.roll
        raw.control.yaw = cmd.yaw
        raw.control.pitch = cmd.pitch
        rows.append((time.monotonic() - start, _local_roll_deg(s), _roll_rate_deg(s)))
    _report(rows, "roll(°)", "rate(°/s)")


def _sat_accel_deg_s2(rows, t_lo: float = 0.5, t_hi: float = 3.0) -> float:
    """Measured saturated roll acceleration: linear fit of the rate ramp.

    During a saturated (stick=±1) slew the displayed rate rises ~linearly,
    so the slope is the *actual* max roll acceleration — the value the
    profile should have been planned with.
    """
    pts = [(t, r) for t, _, r in rows if t_lo <= t <= t_hi]
    if len(pts) < 2:
        return float("nan")
    (t0, r0), (t1, r1) = pts[0], pts[-1]
    return (r1 - r0) / (t1 - t0)


def _act_angle_hold(booster, frame):
    """Act 3: roll_target=0.0 - angle hold, actively rolls back to local 0°."""
    print(f"\n=== Act 3: roll_target=0.0 (angle hold) ===")
    print("  same start pose; expects the roll to *actively* converge to 0°")
    s0 = booster.snapshot()
    est = max_acc_from_snapshot(s0)
    print(f"  max_acc ESTIMATE (snapshot torques): roll={math.degrees(est[0]):6.1f}°/s² "
          f"yaw={math.degrees(est[1]):6.1f}°/s² pitch={math.degrees(est[2]):6.1f}°/s²")
    raw = booster.raw
    moi_y = float(np.asarray(s0.moment_of_inertia)[1])
    for name, tq in (
        ("reaction-wheel", s0.available_reaction_wheel_torque.negative),
        ("rcs", s0.available_rcs_torque.negative),
        ("engine", s0.available_engine_torque.negative),
        ("control-surface", s0.available_control_surface_torque.negative),
    ):
        print(f"    {name:>15}: {math.degrees(abs(tq[1]) / moi_y):7.1f}°/s² "
              f"(y-axis / roll contribution)")
    print(f"  RCS currently {'ON' if raw.control.rcs else 'OFF'}"
          f" -> enabling RCS for this act")
    raw.control.rcs = True
    ctrl = LocalAttitudeController(settling_time=0.5)
    pacer = FramePacer(hz=50)
    start = time.monotonic()
    rows = []
    while time.monotonic() - start < ACT_S:
        pacer.tick()
        s = booster.snapshot()
        cmd = ctrl.step(s, TARGET_DIR, roll_target=0.0)
        raw.control.roll = cmd.roll
        raw.control.yaw = cmd.yaw
        raw.control.pitch = cmd.pitch
        rows.append((time.monotonic() - start, _local_roll_deg(s), _roll_rate_deg(s)))
    _report(rows, "roll(°)", "rate(°/s)")
    meas = _sat_accel_deg_s2(rows)
    if math.isfinite(meas):
        ratio = meas / math.degrees(est[0])
        print(f"  measured saturated roll accel = {meas:+6.1f}°/s² "
              f"(estimate overrates by {ratio:.1f}x)"
              if ratio < 1.0 else
              f"  measured saturated roll accel = {meas:+6.1f}°/s² "
              f"(estimate underrates by {1.0 / ratio:.1f}x)")
    else:
        print("  (could not measure saturated roll accel from this run)")
    overshoot = min(r for _, r, _ in rows)
    print(f"  min roll reached = {overshoot:+6.1f}° (overshoot past 0° if negative)")


def _report(rows, label1, label2, step=20):
    """Print a decimated table + ASCII curve of the second column."""
    print(f"  {'t(s)':>6}  {label1:>10}  {label2:>10}   roll curve (-180 .. +180)")
    for i in range(0, len(rows), step):
        t, roll, rate = rows[i]
        print(f"{t:6.2f}  {roll:+10.2f}  {rate:+10.2f}   {_bar(roll)}")


def _act_crosscheck(booster, controls, frame):
    """Act 4: kRPC target_roll vs local reading - reveal the zero-point offset."""
    print("\n=== Act 4: kRPC target_roll -> local roll_from_axes reading ===")
    controls.target_smoothing_time = 0.0
    print(f"  {'kRPC target_roll':>17}  {'local roll_from_axes':>20}  {'local - kRPC':>12}")
    for target_roll in (0.0, 90.0, 180.0, 270.0):
        controls.apply(
            target_direction=TARGET_DIR, reference_frame=frame, roll_angle=target_roll
        )
        ap = booster.raw.auto_pilot
        deadline = time.monotonic() + SETTLE_S
        settled = False
        while time.monotonic() < deadline:
            if ap.error < 3.0 and abs(ap.roll_error) < 3.0:
                settled = True
                break
            time.sleep(0.1)
        if not settled:
            print(f"  !! Act4 settle TIMEOUT (error={ap.error:.1f}°, roll_error={ap.roll_error:.1f}°)")
        time.sleep(0.4)
        s = booster.snapshot()
        local = _local_roll_deg(s)
        diff = (local - target_roll) % 360.0
        if diff > 180.0:
            diff -= 360.0
        print(f"  {target_roll:17.0f}  {local:+20.1f}  {diff:+12.1f}")
    controls.disengage_auto_pilot()
    controls.target_smoothing_time = 0.2


def _zero_sticks(booster):
    raw = booster.raw
    raw.control.roll = 0.0
    raw.control.pitch = 0.0
    raw.control.yaw = 0.0


def main() -> None:
    with ConnectionManager(address="127.0.0.1") as km:
        booster = km.add_booster("booster-01", VESSEL, control_hz=50.0, telemetry_hz=30.0)
        km.register_target("booster-01", lon=LAUNCHPAD_JNSQ.lon, lat=LAUNCHPAD_JNSQ.lat)
        km.start()
        frame = km.frame("booster-01", "target")
        controls = booster.controls
        controls.target_smoothing_time = 0.2

        print(f"Vessel: {VESSEL} | nose target: {TARGET_DIR} (target frame)")
        _act_reference(booster, controls, frame)
        _pose(controls, frame, roll_deg=SPIN_START_DEG)
        _act_damping(booster, frame)
        _zero_sticks(booster)
        _pose(controls, frame, roll_deg=SPIN_START_DEG)
        _act_angle_hold(booster, frame)
        _zero_sticks(booster)
        _act_crosscheck(booster, controls, frame)
        _zero_sticks(booster)

    print("\nDone.")


if __name__ == "__main__":
    main()
