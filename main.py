import numpy as np
import krpc
import gfold
import time

from recovery import ConnectionManager, FramePacer, PID
from recovery.data.targets import LAUNCHPAD_JNSQ

pid_controller = PID()
pid_controller.kp = 0.5
pid_controller.ki = 0.
pid_controller.kd = 0.1

with ConnectionManager(address="127.0.0.1") as km:
    booster = km.add_booster("booster-01", "VTVL-Cam", control_hz=60.0, telemetry_hz=30.0)
    km.register_target("booster-01", lon=LAUNCHPAD_JNSQ.lon, lat=LAUNCHPAD_JNSQ.lat)
    km.enable_debug()
    km.start()

    frame = km.frame("booster-01", "target")
    booster.controls.target_smoothing_time = 0.2
    booster.controls.apply(
        target_direction=(0.0, 0.0, 1.0),
        reference_frame=frame,
    )
    booster.debug.reference_frame(frame_name="target", length=10)
    booster.debug.reference_frame(frame_name="vessel", length=10)

    pacer = FramePacer(hz=50)
    while True:
        dt = pacer.tick()
        s = booster.snapshot()
        if s is None:
            continue
        throttle = pid_controller.update((s.position.z + s.velocity.z) - 1000, dt)
        booster.controls.apply(throttle=throttle)
