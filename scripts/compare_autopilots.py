"""Compare local (legacy) AutoPilot against kRPC server-side AutoPilot.

Both tests start from the SAME initial orientation, then command a re-point
to TARGET_DIR.  Error convergence is logged side by side.
"""
import math
import sys
import time

import numpy as np

sys.path.insert(0, "src")
from recovery import ConnectionManager, FramePacer
from recovery.control import AutoPilot as LocalAP
from recovery.control.control_utils import angle_between, normalize, rotate
from recovery.data.targets import LAUNCHPAD_JNSQ

VESSEL = "VTVL-Cam"
DURATION = 5.0
TARGET_DIR = (0.0, 0.0, 1.0)          # nose-up  in target frame  (where we want to go)
START_DIR = (0.0, 0.0, -1.0)           # nose-down in target frame  (initial, far away)


def _roll_from_direction(vessel, space_center, frame):
    x = np.array(vessel.direction(frame))
    y = np.array(
        space_center.transform_direction((0.0, 0.0, 1.0), vessel.reference_frame, frame)
    )
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


def _local_max_acc(vessel):
    torques = [
        np.abs(vessel.available_reaction_wheel_torque[0]),
        np.abs(vessel.available_rcs_torque[0]),
        np.abs(vessel.available_engine_torque[0]),
        np.abs(vessel.available_control_surface_torque[0]),
    ]
    moi = np.array(vessel.moment_of_inertia)
    acc = (sum(torques) / moi).tolist()
    return (acc[1], acc[2], acc[0])


def _reset_to_start(raw_v, frame):
    """Use kRPC AP to point the vessel to START_DIR, wait for settle, disengage."""
    ap = raw_v.auto_pilot
    ap.reference_frame = frame
    ap.target_direction = START_DIR
    ap.engaged = True
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if ap.error < 2.0:
            break
        time.sleep(0.1)
    time.sleep(1.0)
    ap.engaged = False
    time.sleep(0.3)


def run_krpc_ap(raw_v, controls, frame):
    """kRPC AP: begin from START_DIR, command TARGET_DIR, log."""
    _reset_to_start(raw_v, frame)
    controls.target_smoothing_time = 0.2
    controls.apply(target_direction=TARGET_DIR, reference_frame=frame)
    ap = raw_v.auto_pilot

    pacer = FramePacer(hz=50)
    start = time.monotonic()
    log = []
    while time.monotonic() - start < DURATION:
        pacer.tick()
        controls.apply(throttle=0.3)
        elapsed = time.monotonic() - start
        ang_vel = np.linalg.norm(np.array(raw_v.angular_velocity(frame)))
        log.append((elapsed, ap.error, ang_vel))
    raw_v.auto_pilot.engaged = False
    return log


def run_local_ap(raw_v, controls, space_center, frame):
    """Local AP: begin from START_DIR, command TARGET_DIR, log."""
    _reset_to_start(raw_v, frame)

    ctrl = LocalAP(settling_time=0.5)
    ctrl.update_max_acc(_local_max_acc(raw_v))

    pacer = FramePacer(hz=50)
    start = time.monotonic()
    log = []
    while time.monotonic() - start < DURATION:
        pacer.tick()
        cur_dir = np.array(raw_v.direction(frame))
        cur_roll = _roll_from_direction(raw_v, space_center, frame)
        ang_vel_vec = np.array(raw_v.angular_velocity(frame))
        target = (None, *np.array(TARGET_DIR))
        ctrl_x, ctrl_y, ctrl_z = ctrl.update(
            (cur_roll, *cur_dir), target, -ang_vel_vec, rot_flag=-1
        )
        raw_v.control.throttle = 0.3
        raw_v.control.roll = ctrl_x
        raw_v.control.yaw = ctrl_y
        raw_v.control.pitch = ctrl_z
        elapsed = time.monotonic() - start
        err = float(np.degrees(angle_between(cur_dir, np.array(TARGET_DIR))))
        log.append((elapsed, err, np.linalg.norm(ang_vel_vec)))
    # zero sticks
    raw_v.control.roll = 0.0
    raw_v.control.pitch = 0.0
    raw_v.control.yaw = 0.0
    return log


def print_table(krpc_log, local_log):
    print()
    header = (
        f"{'Time(s)':>8}  {'kRPC_err(°)':>12}  {'kRPC_ω(rad/s)':>14}"
        f"  {'Local_err(°)':>13}  {'Local_ω(rad/s)':>14}"
    )
    print(header)
    print("-" * 72)
    step = max(1, len(krpc_log) // 20)
    for i in range(0, len(krpc_log), step):
        if i < len(local_log):
            print(
                f"{krpc_log[i][0]:8.2f}"
                f"  {krpc_log[i][1]:12.2f}"
                f"  {krpc_log[i][2]:14.4f}"
                f"  {local_log[i][1]:13.2f}"
                f"  {local_log[i][2]:14.4f}"
            )

    krpc_final = np.mean([e[1] for e in krpc_log[-10:]])
    local_final = np.mean([e[1] for e in local_log[-10:]])
    krpc_final_av = np.mean([e[2] for e in krpc_log[-10:]])
    local_final_av = np.mean([e[2] for e in local_log[-10:]])
    print(f"\n{'final 10 avg':>8}  {krpc_final:12.2f}  {krpc_final_av:14.4f}"
          f"  {local_final:13.2f}  {local_final_av:14.4f}")


def main():
    with ConnectionManager(address="127.0.0.1") as km:
        booster = km.add_booster("booster-01", VESSEL, control_hz=50.0, telemetry_hz=30.0)
        km.register_target("booster-01", lon=LAUNCHPAD_JNSQ.lon, lat=LAUNCHPAD_JNSQ.lat)
        km.start()
        frame = km.frame("booster-01", "target")
        raw_v = booster.raw
        sc = booster.client.space_center

        print(f"=== kRPC AutoPilot   (START_DIR → TARGET_DIR, {DURATION}s) ===")
        krpc_log = run_krpc_ap(raw_v, booster.controls, frame)

        print(f"=== Local AutoPilot  (START_DIR → TARGET_DIR, {DURATION}s) ===")
        local_log = run_local_ap(raw_v, booster.controls, sc, frame)

        print_table(krpc_log, local_log)

        print("\n=== Key differences ===")
        print("kRPC:  server-side closed-loop, auto_tune PID, oscillation filter, smoothing_time")
        print(
            "Local: velocity-profile (ApproachingModel) + stick-level bang-bang,"
            " runs on Python thread"
        )
        print("       Latency: ~1 RPC + 1 physics tick per iteration.")
        print("       Manual max_acc config from torque/MOI each step.")

    print("Done.")


if __name__ == "__main__":
    main()
