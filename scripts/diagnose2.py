"""Live diagnostic round 2: monitor attitude error and actuator output over time."""
import sys
import time

sys.path.insert(0, "src")
from recovery import ConnectionManager
from recovery.data.targets import LAUNCHPAD_JNSQ

with ConnectionManager(address="127.0.0.1") as km:
    b = km.add_booster("booster-01", "VTVL-Cam", control_hz=60.0, telemetry_hz=30.0)
    km.register_target("booster-01", lon=LAUNCHPAD_JNSQ.lon, lat=LAUNCHPAD_JNSQ.lat)
    km.start()

    deadline = time.monotonic() + 5
    while km.snapshot("booster-01") is None:
        if time.monotonic() > deadline:
            raise RuntimeError("telemetry not ready")
        time.sleep(0.02)

    ap = b.raw.auto_pilot
    ctrl = b.raw.control
    raw_v = b.raw

    b.controls.apply(
        target_direction=(0.0, 0.0, 1.0),
        reference_frame=km.frame("booster-01", "target"),
        throttle=1.0,
    )

    print("t(s), err(deg), pitch, yaw, roll, throttle, ang_vel(rad/s), osc_py, osc_r")
    t0 = time.monotonic()
    for _ in range(20):
        time.sleep(0.1)
        t = time.monotonic() - t0
        err = ap.error
        pe = ap.attitude_error
        av = raw_v.angular_velocity(raw_v.surface_reference_frame)
        osc_py = ap.pitch_yaw_control_oscillation
        osc_r = ap.roll_control_oscillation
        print(
            f"{t:.2f}, {err:.2f}, {pe[0]:.2f}, {pe[1]:.2f}, {pe[2]:.2f}, "
            f"{ctrl.throttle:.2f}, ({av[0]:.3f}, {av[1]:.3f}, {av[2]:.3f}), "
            f"{osc_py:.3f}, {osc_r:.3f}"
        )
