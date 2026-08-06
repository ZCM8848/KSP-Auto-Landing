"""Live validation: landing prediction against actual vessel state."""
import sys
import time

sys.path.insert(0, "src")
import numpy as np
from recovery import ConnectionManager
from recovery.data.targets import LAUNCHPAD_JNSQ
from recovery.guidance import LandingPredictor

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

    omega = np.array(body.direction(tf)) * body.rotational_speed
    body_center = np.array(body.position(tf))
    body_radius = body.equatorial_radius + body.surface_height(
        LAUNCHPAD_JNSQ.lat, LAUNCHPAD_JNSQ.lon
    )

    print("omega        =", tuple(omega))
    print("body_center  =", tuple(body_center))
    print("body_radius  = %.1f" % body_radius)

    predictor = LandingPredictor(
        mu=body.gravitational_parameter,
        omega=tuple(omega),
        body_center=tuple(body_center),
        body_radius=body_radius,
    )

    s = b.snapshot()
    position = tuple(s.position)
    velocity = tuple(s.velocity)
    print("\nposition     =", position)
    print("velocity     =", velocity)

    result = predictor.predict(position, velocity)
    if result:
        print("\nIMPACT:")
        print("  position = (%.1f, %.1f, %.1f)" % result.position)
        print("  time     = %.2f s" % result.time)
    else:
        print("\nNo impact within t_max")
