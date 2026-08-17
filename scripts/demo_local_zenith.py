"""Point RLV-VTVL at the zenith using the local AutoPilot (live KSP).

Commands the nose toward the zenith direction in the target frame and
logs the pointing error / roll reading (kRPC convention: 0 = dorsal
aligned with frame +x) / angular-rate magnitude.

Zenith in the target frame is +z (empirically confirmed by
``demo_debug_target_frame.py``: zenith=(0,0,1), north=(1,0,0), east=(0,1,0)).

Run from the repo root with the KRPC conda env:

    PYTHONPATH=src /d/miniforge3/envs/KRPC/python.exe scripts/demo_local_zenith.py
"""
import time

import numpy as np

from recovery import ConnectionManager, FramePacer
from recovery.control.local_attitude import LocalAttitudeController, roll_from_axes
from recovery.data.targets import LAUNCHPAD_JNSQ
from recovery.control.control_utils import angle_between

VESSEL = "RLV-VTVL"
ZENITH = (0.0, 0.0, 1.0)  # nose-up in the target frame
DURATION_S = 8.0


def _bar(value: float, lo: float = 0.0, hi: float = 180.0, width: int = 48) -> str:
    frac = min(1.0, max(0.0, (value - lo) / (hi - lo)))
    n = int(frac * width)
    return "|" + "#" * n + "-" * (width - n) + "|"


def main() -> None:
    with ConnectionManager(address="127.0.0.1") as km:
        booster = km.add_booster("booster-01", VESSEL, control_hz=50.0, telemetry_hz=30.0)
        km.register_target("booster-01", lon=LAUNCHPAD_JNSQ.lon, lat=LAUNCHPAD_JNSQ.lat)
        km.start()
        raw = booster.raw
        frame = km.frame("booster-01", "target")

        ctrl = LocalAttitudeController(settling_time=0.5)
        pacer = FramePacer(hz=50)
        start = time.monotonic()
        rows = []

        print(f"Vessel: {VESSEL} | target: zenith = {ZENITH} (target frame)")
        print(f"{'t(s)':>6}  {'nose_err(°)':>12}  {'roll(°)':>9}  {'|ω|(°/s)':>10}   nose error (0..180)")

        while time.monotonic() - start < DURATION_S:
            pacer.tick()
            s = booster.snapshot()
            cmd = ctrl.step(s, ZENITH, roll_target=0.0)  # hold roll 0° (degrees, kRPC convention)
            raw.control.roll = cmd.roll
            raw.control.yaw = cmd.yaw
            raw.control.pitch = cmd.pitch

            nose = np.asarray(s.direction)
            err = float(np.degrees(angle_between(nose, np.asarray(ZENITH))))
            roll = float(np.degrees(roll_from_axes(s.direction, s.bottom_axis)))
            angvel = float(
                np.degrees(np.linalg.norm(np.asarray(s.angular_velocity)))
            )
            t = time.monotonic() - start
            rows.append((t, err, roll, angvel))
            if len(rows) % 25 == 0:
                print(f"{t:6.2f}  {err:12.2f}  {roll:+9.1f}  {angvel:10.2f}   {_bar(err)}")

        # zero sticks
        raw.control.roll = 0.0
        raw.control.pitch = 0.0
        raw.control.yaw = 0.0

        final = rows[-1]
        print(f"\nfinal: nose_err={final[1]:.2f}°  roll={final[2]:+.1f}°  |ω|={final[3]:.2f}°/s")
        print("Done.")


if __name__ == "__main__":
    main()
