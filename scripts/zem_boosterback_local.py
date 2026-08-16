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

import sys
import time

import numpy as np

sys.path.insert(0, "src")
from recovery import ConnectionManager, FramePacer
from recovery.control import AutoPilot as LocalAP
from recovery.control.control_utils import angle_between, cross, normalize, pi, rotate
from recovery.data.targets import LAUNCHPAD_JNSQ
from recovery.guidance import DragModel, LandingPredictor
from recovery.ksp.sampling import sample_body_spec, sample_drag_spec

VESSEL = "RLV Probe 2"

MIN_ALT = 8000.0     # boosterback window (m)
ROI_MISS = 50000.0   # ignore miss-increase below this threshold


def _roll_from_direction(direction: object, bottom: object) -> float:
    x = np.array(direction)
    y = np.array(bottom)
    x0 = np.array((1.0, 0.0, 0.0))
    rot_axis = normalize(cross(x, x0))
    rot_ang = angle_between(x, x0)
    y0 = rotate(rot_axis, y, rot_ang)
    ang1 = angle_between(y0, (0.0, 1.0, 0.0))
    ang2 = angle_between(y0, (0.0, 0.0, 1.0))
    roll = ang1
    if ang2 > pi / 2:
        roll = -roll
    return roll


def _local_max_acc(s: object) -> tuple[float, float, float]:
    torques = [
        np.abs(s.available_reaction_wheel_torque.negative),
        np.abs(s.available_rcs_torque.negative),
        np.abs(s.available_engine_torque.negative),
        np.abs(s.available_control_surface_torque.negative),
    ]
    moi = np.array(s.moment_of_inertia)
    acc = (sum(torques) / moi).tolist()
    return (acc[1], acc[2], acc[0])  # reorder to (roll, yaw, pitch)


def _zero_sticks(b: object) -> None:
    b.controls.apply(roll=0.0, yaw=0.0, pitch=0.0)


def main() -> None:
    with ConnectionManager(address="127.0.0.1", telemetry_hz=50) as km:
        b = km.add_booster("zem", VESSEL)
        km.register_target("zem", lon=LAUNCHPAD_JNSQ.lon, lat=LAUNCHPAD_JNSQ.lat)
        km.start()

        deadline = time.monotonic() + 5.0
        while km.snapshot("zem") is None:
            if time.monotonic() > deadline:
                raise RuntimeError("telemetry not ready")
            time.sleep(0.02)

        frame = km.frame("zem", "target")

        body = b.raw.orbit.body
        flight = b.raw.flight(frame)
        body_spec = sample_body_spec(body, frame, LAUNCHPAD_JNSQ.lat, LAUNCHPAD_JNSQ.lon)
        drag_spec = sample_drag_spec(body, flight, frame, mass=float(b.raw.mass))
        predictor = LandingPredictor.from_body_spec(
            body_spec, aero=DragModel.from_spec(drag_spec)
        )

        b.physics_range = 200000.0
        b.controls.rcs = True
        b.controls.throttle = 1.0

        ctrl = LocalAP(settling_time=0.5)
        ctrl.update_max_acc(_local_max_acc(b.snapshot()))

        log = open("zem_boosterback_debug.log", "w", encoding="utf-8")
        error_hist: list[float] = [float("inf")]
        ap_cfg_at = 0.0
        t_start = time.monotonic()
        t_last_flush = t_start
        buf: list[str] = []
        pacer = FramePacer(hz=30)
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
            if time.monotonic() - ap_cfg_at >= 0.5:
                ctrl.update_max_acc(_local_max_acc(s))
                ap_cfg_at = time.monotonic()

            cur_dir = np.array(s.direction)
            cur_roll = _roll_from_direction(s.direction, s.bottom_axis)
            ang_vel = np.array(s.angular_velocity)
            ctrl_x, ctrl_y, ctrl_z = ctrl.update(
                (cur_roll, *cur_dir),
                (None, *target_dir),
                -ang_vel,
                rot_flag=-1,
            )
            b.controls.apply(roll=ctrl_x, yaw=ctrl_y, pitch=ctrl_z)
            # -----------------------------------------------------------------

            att_err = float(np.degrees(angle_between(cur_dir, target_dir)))

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
