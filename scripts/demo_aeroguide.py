"""Aerodynamic guidance: steer a free-falling rocket to the landing target.

Main engine OFF.  The rocket tilts its body to generate aerodynamic lift,
shifting the predicted impact point toward the launch pad.  Roll is kept
wind-aligned (belly into the airflow) via the ``up`` parameter.
"""
import time

import numpy as np

from recovery import ConnectionManager, FramePacer
from recovery.data.targets import LAUNCHPAD_JNSQ
from recovery.guidance import LandingPredictor
from recovery.ksp.sampling import sample_body_spec

VESSEL = "Booster 1"   # renamed to match the current vessel in KSP
MAX_AOA = 20.0       # degrees — max angle of attack for body lift
AOA_GAIN = 0.02      # deg per metre of horizontal miss distance


def _wind_up(nose, vel):
    """Return ``up`` so belly faces the airflow (into *vel*)."""
    d = np.asarray(nose, dtype=float)
    v = np.asarray(vel, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-6:
        return (1.0, 0.0, 0.0)
    belly = v / n
    perp = belly - np.dot(belly, d) * d
    n2 = np.dot(perp, perp)
    if n2 < 1e-12:
        return (1.0, 0.0, 0.0)
    roof = -perp / np.sqrt(n2)
    return (float(roof[0]), float(roof[1]), float(roof[2]))


def _nose_for_lift(velocity, miss_xy, max_aoa_deg, gain_deg_per_m):
    """Tilt the engine-end-down (nose-up) direction toward the desired lift."""
    v = np.asarray(velocity, dtype=float)
    speed = np.linalg.norm(v)
    if speed < 1e-6:
        return (0.0, 0.0, 1.0)
    v_dir = v / speed
    base = -v_dir                     # engine-end down = nose opposite to velocity
    miss = np.array([miss_xy[0], miss_xy[1], 0.0])
    miss_dist = np.linalg.norm(miss)
    if miss_dist < 1e-3:
        return tuple(base)
    lift_dir = miss / miss_dist       # nose tilts toward miss → lift pushes toward target
    aoa_deg = min(miss_dist * gain_deg_per_m, max_aoa_deg)
    aoa = np.radians(aoa_deg)
    axis = np.cross(base, lift_dir)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-12:
        return tuple(base)
    axis = axis / axis_norm
    nose = base * np.cos(aoa) + np.cross(axis, base) * np.sin(aoa)
    return (float(nose[0]), float(nose[1]), float(nose[2]))


def main():
    with ConnectionManager(address="127.0.0.1") as km:
        b = km.add_booster("aero", VESSEL)
        km.register_target("aero", lon=LAUNCHPAD_JNSQ.lon, lat=LAUNCHPAD_JNSQ.lat)
        km.enable_debug()
        km.start()

        deadline = time.monotonic() + 5.0
        while km.snapshot("aero") is None:
            if time.monotonic() > deadline:
                raise RuntimeError("telemetry not ready")
            time.sleep(0.02)

        raw = b.raw
        body = raw.orbit.body
        frame = km.frame("aero", "target")

        # build predictor
        body_spec = sample_body_spec(body, frame, LAUNCHPAD_JNSQ.lat, LAUNCHPAD_JNSQ.lon)
        predictor = LandingPredictor.from_body_spec(body_spec)

        # debug: target frame axes + vessel frame
        b.debug.reference_frame(frame_name="target", length=10)
        b.debug.reference_frame(frame_name="vessel", length=5)

        b.controls.target_smoothing_time = 0.3
        b.controls.apply(throttle=0.0, roll_angle=0.0)       # engine OFF, roll reference locked

        pacer = FramePacer(hz=20)
        print("t(s)    miss-xy(m)  aoa(deg)  time-to-impact(s)  pred(x,y)")
        start = time.monotonic()

        while True:
            pacer.tick()
            s = b.snapshot()
            if s is None:
                continue

            elapsed = time.monotonic() - start
            vel = np.array([s.velocity[0], s.velocity[1], s.velocity[2]])

            result = predictor.predict_from(s)
            if result is None:
                print(f"{elapsed:6.1f}  no impact predicted")
                s = float(np.linalg.norm(vel))
                v_dir = vel / s if s > 1e-6 else np.array([0.0, 0.0, 1.0])
                nose = tuple(-v_dir)
                b.controls.apply(
                    target_direction=nose,
                    reference_frame=frame,
                    up=_wind_up(nose, vel),
                    throttle=0.0,
                )
                continue

            miss_x = float(result.position[0])
            miss_y = float(result.position[1])
            miss_dist = np.hypot(miss_x, miss_y)
            miss = (miss_x, miss_y)
            aoa_deg = min(miss_dist * AOA_GAIN, MAX_AOA)

            nose = _nose_for_lift(vel, miss, MAX_AOA, AOA_GAIN)
            roof = _wind_up(nose, vel)

            b.controls.apply(
                target_direction=nose,
                reference_frame=frame,
                up=roof,
                throttle=0.0,
            )

            print(
                f"{elapsed:6.1f}  {miss_dist:10.1f}  {aoa_deg:7.1f}"
                f"  {result.time:16.2f}  ({miss_x:6.0f}, {miss_y:6.0f})"
            )

            if s.altitude < 500 and miss_dist < 10:
                print("Within 10m — terminal.")
                break


if __name__ == "__main__":
    main()
