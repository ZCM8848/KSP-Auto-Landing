"""Demonstrate how the local AutoPilot handles roll (requires a live KSP).

Four acts, all flying the same nose direction (TARGET_DIR):

  Act 1 - Reference pose: point the nose via the kRPC AutoPilot and read
          the live roll angle as measured by ``roll_from_axes`` (vessel
          bottom axis vs frame +y, nose aligned to +x).
  Act 2 - ``roll_target=None``: roll-rate damping only.  We start rolled
          to target_roll=+90° (kRPC semantics), hand over to the local AP,
          and watch the roll *stay put* - rate is damped to zero, but no
          angle is held.
  Act 3 - ``roll_target=0.0``: angle hold.  Same start pose, command local
          roll 0°, watch the vessel actively roll back and converge.
  Act 4 - Cross-check: command target_roll = 0/90/180/270° through the
          kRPC AutoPilot and tabulate the local ``roll_from_axes`` reading
          for each - this exposes the zero-point relationship between the
          two roll conventions.

Run from the repo root with the KRPC conda env:

    PYTHONPATH=src /d/miniforge3/envs/KRPC/python.exe scripts/demo_local_roll.py
"""
import math
import time

import numpy as np

from recovery import ConnectionManager, FramePacer
from recovery.control.local_attitude import LocalAttitudeController, roll_from_axes
from recovery.data.targets import LAUNCHPAD_JNSQ

VESSEL = "Booster 1"
TARGET_DIR = (0.0, 0.0, 1.0)  # nose direction in the target frame
SETTLE_S = 3.0  # wait after commanding a kRPC AP pose
ACT_S = 4.0  # duration of each local-AP act
SPIN_START_DEG = 90.0  # kRPC target_roll used to build the starting pose


def _local_roll_deg(s) -> float:
    """Local roll reading (deg): bottom axis vs frame +y, nose aligned to +x."""
    return float(np.degrees(roll_from_axes(s.direction, s.bottom_axis)))


def _roll_rate_deg(s) -> float:
    """Roll rate about the nose axis (deg/s): angular velocity projected on the nose."""
    return float(np.degrees(np.dot(np.asarray(s.angular_velocity), np.asarray(s.direction))))


def _pose(controls, frame, *, roll_deg: float, wait_s: float = SETTLE_S) -> None:
    """Point the nose at TARGET_DIR with the given kRPC target_roll, wait, disengage."""
    controls.apply(target_direction=TARGET_DIR, reference_frame=frame, roll_angle=roll_deg)
    ap = controls._auto_pilot  # noqa: SLF001 - read-only peek, matches compare_autopilots style
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        if ap.error < 3.0 and abs(ap.roll_error) < 3.0:
            break
        time.sleep(0.1)
    time.sleep(0.3)
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


def _act_angle_hold(booster, frame):
    """Act 3: roll_target=0.0 - angle hold, actively rolls back to local 0°."""
    print(f"\n=== Act 3: roll_target=0.0 (angle hold) ===")
    print("  same start pose; expects the roll to *actively* converge to 0°")
    ctrl = LocalAttitudeController(settling_time=0.5)
    pacer = FramePacer(hz=50)
    raw = booster.raw
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
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if ap.error < 3.0 and abs(ap.roll_error) < 3.0:
                break
            time.sleep(0.1)
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
