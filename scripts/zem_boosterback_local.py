"""ZEM boosterback guidance using local (Python-side) AutoPilot.

Closed-loop, full-throttle boostback burn.  The impact point is predicted
via the offline :class:`DragModel`; the local :class:`AutoPilot` steers the
vessel to a purely horizontal nose direction that nulls the miss.

Unlike the kRPC server-side AutoPilot the local controller outputs raw
stick values every tick, giving direct algorithmic control over the
attitude response.  Roll is damped (not constrained).

Usage::

    python scripts/zem_boosterback_local.py
"""

import time

import numpy as np

from recovery import ConnectionManager, FramePacer
from recovery.control import LocalAttitudeController
from recovery.control.control_utils import angle_between
from recovery.data.targets import LAUNCHPAD_JNSQ
from recovery.guidance import DragModel, LandingPredictor

VESSEL = "Booster 1"

MIN_ALT = 8000.0     # boosterback window (m)
ROI_MISS = 50000.0   # ignore miss-increase below this threshold


def _zero_sticks(b: object) -> None:
    b.controls.apply(roll=0.0, yaw=0.0, pitch=0.0)


def main() -> None:
    with ConnectionManager(address="127.0.0.1", telemetry_hz=50) as km:
        b = km.add_booster("zem", VESSEL, control_hz=30)
        km.register_target("zem", lon=LAUNCHPAD_JNSQ.lon, lat=LAUNCHPAD_JNSQ.lat)
        km.start()

        deadline = time.monotonic() + 5.0
        while km.snapshot("zem") is None:
            if time.monotonic() > deadline:
                raise RuntimeError("telemetry not ready")
            time.sleep(0.02)

        body_spec, drag_spec = b.sample_predictor_specs()
        predictor = LandingPredictor.from_body_spec(
            body_spec, aero=DragModel.from_spec(drag_spec)
        )

        b.physics_range = 200000.0
        b.controls.rcs = True
        b.controls.throttle = 1.0

        ctrl = LocalAttitudeController(settling_time=0.5)

        log = open("zem_boosterback_debug.log", "w", encoding="utf-8")
        error_hist: list[float] = [float("inf")]
        t_start = time.monotonic()
        t_last_flush = t_start
        buf: list[str] = []
        # Loop pacing is owned by the booster's control_hz knob; the future
        # orchestration scheduler will replace this FramePacer entirely.
        pacer = FramePacer(hz=b.control_hz)
        header = (
            f"{'t':>6s}  {'loop_us':>7s}  {'alt':>6s}"
            f"  {'miss':>8s}  {'tti':>6s}  {'err(deg)':>9s}  {'kN':>8s}"
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

            result = predictor.predict_from(s, rtol=1e-6, atol=1e-6)
            if result is None:
                continue

            mx = float(result.position[0])
            my = float(result.position[1])
            miss = float(np.hypot(mx, my))

            # target direction: purely horizontal toward target
            if miss < 1.0:
                target_dir = np.array((0.0, 0.0, -1.0))
            else:
                target_dir = np.array((-mx / miss, -my / miss, 0.0))

            # ---------- local AutoPilot step ---------------------------------
            sticks = ctrl.step(s, target_dir)
            b.controls.apply(roll=sticks.roll, yaw=sticks.yaw, pitch=sticks.pitch)
            # -----------------------------------------------------------------

            att_err = float(np.degrees(angle_between(np.array(s.direction), target_dir)))

            loop_us = (time.perf_counter_ns() - t_loop) // 1000
            line = (
                f"{time.monotonic() - t_start:6.1f}  "
                f"{loop_us:7d}  "
                f"{s.surface_altitude:6.0f}  "
                f"{miss:8.0f}  "
                f"{result.time:6.1f}  "
                f"{att_err:9.2f}  "
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
                _zero_sticks(b)
                b.controls.cut_thrust()
                msg = (
                    f"Miss stopped decreasing — boosterback complete "
                    f"(min={min(error_hist):.0f}m)."
                )
                print(msg)
                print(msg, file=log)
                break

            error_hist.append(miss)

        _zero_sticks(b)
        b.controls.cut_thrust()
        msg = "Throttle zeroed.  Script exiting."
        print(msg)
        print(msg, file=log)
        log.close()


if __name__ == "__main__":
    main()
