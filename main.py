import numpy as np
import krpc
import gfold
import time

from recovery import ConnectionManager, FramePacer
from internal import Targets_JNSQ
from control.pid import PID

pid_controller = PID()
pid_controller.kp = 2.
pid_controller.ki = 0.4
pid_controller.kd = 0.5

with ConnectionManager(address="127.0.0.1") as km:
    booster = km.add_booster("booster-01", "VTVL-Cam", control_hz=60.0, telemetry_hz=30.0)
    km.register_target("booster-01", lon=Targets_JNSQ.launchpad[0], lat=Targets_JNSQ.launchpad[1])   # 必须在 start 前
    km.enable_debug()
    km.start()

    state = km.snapshot("booster-01")
    booster.controls.apply(
        target_direction=(0.0, -1.0, 0.0),
        reference_frame=km.frame("booster-01", "target"),
        throttle=0.85,
    )
    booster.debug.reference_frame(frame_name="target", length=10)
    booster.debug.reference_frame(frame_name="vessel", length=10)
    booster.controls.apply(
        target_direction=(0.0, 1.0, 0.0),
        reference_frame=km.frame("booster-01", "target"),
        throttle=0.,
    )
    time.sleep(10.0)
    booster.controls.apply(
        target_direction=(0.0, -1.0, 0.0),
        reference_frame=km.frame("booster-01", "target"),
        throttle=0.,
    )
    time.sleep(10.0)