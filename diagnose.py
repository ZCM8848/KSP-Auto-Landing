"""Live diagnostic: check why the AutoPilot isn't controlling attitude."""
import sys
import time

sys.path.insert(0, "src")
from recovery import ConnectionManager
from internal import Targets_JNSQ

with ConnectionManager(address="127.0.0.1") as km:
    b = km.add_booster("booster-01", "VTVL-Cam", control_hz=60.0, telemetry_hz=30.0)
    km.register_target("booster-01", lon=Targets_JNSQ.launchpad[0], lat=Targets_JNSQ.launchpad[1])
    km.start()

    deadline = time.monotonic() + 5
    while km.snapshot("booster-01") is None:
        if time.monotonic() > deadline:
            raise RuntimeError("telemetry not ready")
        time.sleep(0.02)

    raw_v = b.raw
    ap = raw_v.auto_pilot
    ctrl = raw_v.control

    print("--- vessel state ---")
    print("loaded:", raw_v.loaded)
    print("packed:", raw_v.packed)
    print("situation:", raw_v.situation)
    print("control_state:", ctrl.state)
    print("control_source:", ctrl.source)
    print()
    print("--- actuators ---")
    print("reaction_wheels:", ctrl.reaction_wheels)
    print("engine_gimbals:", ctrl.engine_gimbals)
    print("rcs:", ctrl.rcs)
    print("sas:", ctrl.sas)
    print()
    print("--- auto-pilot detail ---")
    print("engaged:", ap.engaged)
    print("reference_frame:", ap.reference_frame)
    print("target_direction:", ap.target_direction)
    print("target_roll:", ap.target_roll)
    print("attitude_error(p-r-y deg):", ap.attitude_error)
    print("error(deg):", ap.error)
    print("auto_tune:", ap.auto_tune)
    print("soft_start_time:", ap.soft_start_time)
    print("max_angular_velocity:", ap.max_angular_velocity)
    print()
    print("--- current throttle ---")
    print("throttle:", ctrl.throttle)
    print("thrust:", raw_v.thrust)
    print("available_thrust:", raw_v.available_thrust)
    print("mass:", raw_v.mass)
    print()
    print("--- target frame origin ---")
    snap = b.snapshot()
    if snap:
        print("position:", tuple(snap.position))
        print("rotation:", tuple(snap.rotation))
