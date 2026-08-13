"""Live validation: landing prediction against actual vessel state."""
import sys
import time

sys.path.insert(0, "src")
from recovery import ConnectionManager
from recovery.data.targets import LAUNCHPAD_JNSQ
from recovery.guidance import LandingPredictor
from recovery.ksp.sampling import sample_body_spec

with ConnectionManager(address="127.0.0.1") as km:
    b = km.add_booster("pred", "RLV-1 VTVL")
    km.register_target("pred", lon=LAUNCHPAD_JNSQ.lon, lat=LAUNCHPAD_JNSQ.lat)
    km.start()

    deadline = time.monotonic() + 5
    while km.snapshot("pred") is None:
        if time.monotonic() > deadline:
            raise RuntimeError("telemetry not ready")
        time.sleep(0.02)

    raw = b.raw
    body = raw.orbit.body
    tf = km.frame("pred", "target")

    body_spec = sample_body_spec(body, tf, LAUNCHPAD_JNSQ.lat, LAUNCHPAD_JNSQ.lon)

    print("omega        =", body_spec.omega)
    print("body_center  =", body_spec.body_center)
    print(f"body_radius  = {body_spec.body_radius:.1f}")

    predictor = LandingPredictor.from_body_spec(body_spec)

    s = b.snapshot()
    position = tuple(s.position)
    velocity = tuple(s.velocity)
    print("\nposition     =", position)
    print("velocity     =", velocity)

    result = predictor.predict(position, velocity)
    if result:
        print("\nIMPACT:")
        pos = result.position
        print(f"  position = ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
        print(f"  time     = {result.time:.2f} s")
    else:
        print("\nNo impact within t_max")
