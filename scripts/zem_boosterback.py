"""ZEM boosterback guidance using DragModel impact predictor.

High-altitude, full-throttle burn to null the zero-effort miss.  The vessel
points its nose horizontally toward the target so that engine thrust pushes
the impact point onto the landing site.  The burn ends when the predicted
miss stops decreasing (local minimum).  A dead-band of 50 km ignores noise
during the initial steering transient.

Usage::

    python scripts/zem_boosterback.py
"""

import sys
import time

import numpy as np

sys.path.insert(0, "src")
from recovery import ConnectionManager, FramePacer
from recovery.data.targets import LAUNCHPAD_JNSQ

VESSEL = "RLV-1 Probe"

MIN_ALT = 8000.0     # boosterback window (m)
ROI_MISS = 50000.0   # ignore miss-increase below this threshold



def main() -> None:
    with ConnectionManager(address="127.0.0.1") as km:
        b = km.add_booster("zem", VESSEL)
        km.register_target("zem", lon=LAUNCHPAD_JNSQ.lon, lat=LAUNCHPAD_JNSQ.lat)
        km.start()

        frame = km.frame("zem", "target")
        predictor = b.init_predictor()

        b.physics_range = 200000.0
        b.controls.target_smoothing_time = 0.3
        b.controls.rcs = True

        # set frame and initial throttle once (outside the hot loop)
        b.controls.apply(reference_frame=frame, throttle=1.0)

        log = open("zem_boosterback_debug.log", "w", encoding="utf-8")
        error_hist: list[float] = [float("inf")]
        t_start = time.monotonic()
        t_last_flush = t_start
        buf: list[str] = []
        pacer = FramePacer(hz=50)
        header = (
            f"{'t':>6s}  {'loop_us':>7s}  {'alt':>6s}"
            f"  {'miss':>8s}  {'tti':>6s}  {'rpc':>3s}  {'kN':>8s}"
        )
        print(header)
        print(header, file=log)

        while True:
            t_loop = time.perf_counter_ns()
            pacer.tick()
            s = b.snapshot()
            if s is None:
                continue

            if s.surface_altitude < MIN_ALT:
                msg = f"Below {MIN_ALT:.0f}m boosterback window — coasting."
                print(msg)
                print(msg, file=log)
                break

            result = predictor.predict_from(s, rtol=5e-6, atol=5e-6)
            if result is None:
                continue

            mx = float(result.position[0])
            my = float(result.position[1])
            miss = float(np.hypot(mx, my))

            # nose horizontally toward target
            if miss < 1.0:
                target_direction = (0.0, 0.0, -1.0)
            else:
                target_direction = (-mx / miss, -my / miss, 0.0)

            b.controls.apply(target_direction=target_direction, reference_frame=frame)

            loop_us = (time.perf_counter_ns() - t_loop) // 1000
            line = (
                f"{time.monotonic() - t_start:6.1f}  "
                f"{loop_us:7d}  "
                f"{s.surface_altitude:6.0f}  "
                f"{miss:8.0f}  "
                f"{result.time:6.1f}  "
                f"   -  "
                f"{s.thrust * 1e-3:8.0f}"
            )
            buf.append(line)

            now = time.monotonic()
            if now - t_last_flush >= 1.0 or (miss < ROI_MISS and miss > min(error_hist)):
                block = "\n".join(buf)
                print(block)
                print(block, file=log)
                log.flush()
                buf.clear()
                t_last_flush = now

            if miss < ROI_MISS and miss > min(error_hist):
                b.controls.cut_thrust()
                msg = (
                    f"Miss stopped decreasing — boosterback complete "
                    f"(min={min(error_hist):.0f}m)."
                )
                print(msg)
                print(msg, file=log)
                break

            error_hist.append(miss)

        b.controls.cut_thrust()
        msg = "Throttle zeroed.  Script exiting."
        print(msg)
        print(msg, file=log)
        log.close()


if __name__ == "__main__":
    main()
