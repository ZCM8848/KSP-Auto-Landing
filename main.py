import numpy as np
import krpc
import gfold
import time

from recovery import ConnectionManager
from internal import Targets_JNSQ

with ConnectionManager(address="127.0.0.1") as km:
    booster = km.add_booster("booster-01", "VTVL-Cam", control_hz=50.0, telemetry_hz=20.0)
    km.register_target("booster-01", lon=Targets_JNSQ.launchpad[0], lat=Targets_JNSQ.launchpad[1])   # 必须在 start 前
    km.enable_debug()
    km.start()

    state = km.snapshot("booster-01")        # FlightState | None（首帧前为 None）
    booster.controls.apply(
        target_direction=(0.0, -1.0, 0.0),
        reference_frame=km.frame("booster-01", "target"),
        throttle=0.85,
    )
    booster.debug.reference_frame(frame_name="target", length=10)
    booster.debug.reference_frame(frame_name="vessel", length=10)
    for i in range(100):
        time.sleep(0.1)
        s = km.snapshot("booster-01")
        if s is not None:
            print(s.mass)
    km.abort_all()                           # 全船急停：油门归零 + AutoPilot 解除