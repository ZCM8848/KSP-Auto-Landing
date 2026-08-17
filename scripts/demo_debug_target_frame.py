"""Draw the target reference frame axes in-game via DebugProxy (live KSP).

Red/green/blue   = target frame +x / +y / +z axes (from the landing site).
White            = zenith (radial-out) reference.
Yellow           = north reference.
Cyan             = east reference.
The three world references are expressed in target-frame coordinates and
drawn from the same origin, so you can read off at a glance which target
axis points at zenith / north / east.

Run from the repo root with the KRPC conda env:

    PYTHONPATH=src /d/miniforge3/envs/KRPC/python.exe scripts/demo_debug_target_frame.py
"""
import time

from recovery import ConnectionManager
from recovery.data.targets import LAUNCHPAD_JNSQ

VESSEL = "RLV-VTVL"
HOLD_S = 30.0   # how long the axes stay visible
LENGTH = 200.0  # axis length in metres


def main() -> None:
    with ConnectionManager(address="127.0.0.1") as km:
        booster = km.add_booster("booster-01", VESSEL, control_hz=10.0, telemetry_hz=10.0)
        km.register_target("booster-01", lon=LAUNCHPAD_JNSQ.lon, lat=LAUNCHPAD_JNSQ.lat)
        km.enable_debug()
        km.start()
        debug = booster.debug

        # World references (surface frame: +x=zenith, +y=north, +z=east),
        # expressed in target-frame coordinates so they share the origin.
        sc = booster.client.space_center
        srf = km.frame("booster-01", "surface")
        tgt = km.frame("booster-01", "target")
        zenith = sc.transform_direction((1.0, 0.0, 0.0), srf, tgt)
        north = sc.transform_direction((0.0, 1.0, 0.0), srf, tgt)
        east = sc.transform_direction((0.0, 0.0, 1.0), srf, tgt)

        print("target axes: R=+x  G=+y  B=+z   (from landing site)")
        print("world refs:  W=zenith  Y=north  C=east")
        print(
            "(in target-frame coordinates) zenith=%s north=%s east=%s"
            % (tuple(zenith), tuple(north), tuple(east))
        )
        axes = debug.reference_frame(frame_name="target", length=LENGTH)
        refs = [
            debug.direction(zenith, frame_name="target", length=LENGTH, color=(1.0, 1.0, 1.0)),
            debug.direction(north, frame_name="target", length=LENGTH, color=(1.0, 1.0, 0.0)),
            debug.direction(east, frame_name="target", length=LENGTH, color=(0.0, 1.0, 1.0)),
        ]
        print(f"\nAxes visible for {HOLD_S}s — check the flight scene now.")
        time.sleep(HOLD_S)

        axes.clear()
        for ref in refs:
            ref.remove()
    print("Done.")


if __name__ == "__main__":
    main()
