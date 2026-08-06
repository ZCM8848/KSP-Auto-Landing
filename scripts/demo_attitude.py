"""Demonstrate ``up`` and ``roll_angle`` in ``controls.apply()``.

Part 3 — Cone sweep with wind-aligned ``up``:  nose traces a circle on a
         cone while ``up`` is recomputed each frame from a virtual
         velocity (0,0,-1) so the belly always faces the wind.

Part 4 — Glide → Landing mode switch:  nose transitions from tilted to
         vertical, ``up`` switches from wind-aligned to horizontal at the
         singular boundary.
"""
import sys
import time
from math import cos, pi, radians, sin

sys.path.insert(0, "src")
from recovery import ConnectionManager
from recovery.data.targets import LAUNCHPAD_JNSQ

VESSEL = "VTVL-Cam"
HOLD = 3.0

VIRTUAL_VEL = (0.0, 0.0, -1.0)   # straight down in target frame


def _wind_up(nose):
    """Compute an ``up_reference`` vector so the belly faces the virtual wind.

    The desired belly direction is the wind-facing direction (opposite to
    *VIRTUAL_VEL*).  This direction is projected onto the plane perpendicular
    to *nose*, then the roof vector (opposite) is returned — so when the
    AutoPilot rolls the roof toward this vector, the belly faces the airflow.
    """
    dx, dy, dz = nose
    belly = (-VIRTUAL_VEL[0], -VIRTUAL_VEL[1], -VIRTUAL_VEL[2])
    dot_bn = dx * belly[0] + dy * belly[1] + dz * belly[2]
    px = belly[0] - dot_bn * dx
    py = belly[1] - dot_bn * dy
    pz = belly[2] - dot_bn * dz
    n2 = px * px + py * py + pz * pz
    if n2 < 1e-12:
        return (1.0, 0.0, 0.0)
    inv = 1.0 / (n2**0.5)
    return (-px * inv, -py * inv, -pz * inv)


def _tilt(angle_deg, axis=1):
    a = radians(angle_deg)
    if axis == 1:
        return (sin(a), 0.0, cos(a))
    return (0.0, sin(a), cos(a))


def _cone(angle_deg, phi):
    a = radians(angle_deg)
    return (sin(a) * cos(phi), sin(a) * sin(phi), cos(a))


def wait_telemetry(km, booster_id):
    deadline = time.monotonic() + 5.0
    while km.snapshot(booster_id) is None:
        if time.monotonic() > deadline:
            raise RuntimeError("telemetry not ready")
        time.sleep(0.02)


def run_cone_sweep_wind(controls, frame, tilt_deg, segments=16, total_s=12.0):
    """Cone sweep where ``up`` is recomputed per frame from virtual wind.

    Only *target_direction* changes across tick — ``up`` is derived from
    *direction* + VIRTUAL_VEL, never inherited.
    """
    a = radians(tilt_deg)
    seg_dt = total_s / segments
    start = time.monotonic()
    for i in range(segments + 1):
        phi = 2.0 * pi * i / segments
        direction = (sin(a) * cos(phi), sin(a) * sin(phi), cos(a))
        up = _wind_up(direction)
        controls.apply(
            target_direction=direction,
            reference_frame=frame,
            up=up,
            roll_angle=0.0,
            throttle=0.3,
        )
        if i == 0:
            print(f">>> cone sweep {tilt_deg}°  up=wind_aligned")
        deadline = start + seg_dt * (i + 1)
        while time.monotonic() < deadline:
            time.sleep(0.05)
            controls.apply(throttle=0.3)


def run_glide_to_landing_wind(controls, frame, total_s=24.0):
    """Glide → landing with wind-aligned ``up`` during tilted flight.

    A)  0– 8s: cone sweep at tilt=40°,  up = wind-aligned,   roll = 0
    B)  8–16s: nose tilts 40° → 15°,   up = wind-aligned,   roll = 0
    C) 16 s:   mode-switch — up = horizontal (1,0,0),        roll = capture
    D) 16–24s: nose tilts 15° →  0°,   up = horizontal,      roll locked
    """
    SWEEP_END = 8.0
    TILT_END = 16.0
    SWITCH_TILT = 15
    CAPTURE_ROLL = 42.0

    steps = 120
    dt = total_s / steps
    start = time.monotonic()
    switched = False

    for i in range(steps + 1):
        target_time = start + dt * i
        now = time.monotonic()
        if target_time > now:
            time.sleep(target_time - now)
        elapsed = dt * i

        if elapsed < SWEEP_END:
            # Phase A: cone sweep at 40°, wind-aligned up
            if i == 0:
                direction = _tilt(40)
                controls.apply(
                    target_direction=direction,
                    reference_frame=frame,
                    up=_wind_up(direction),
                    roll_angle=0.0,
                )
                print(">>> glide  tilt=40°  up=wind_aligned  roll=0")
            else:
                phi = 2.0 * pi * elapsed / SWEEP_END
                direction = _cone(40, phi)
                controls.apply(
                    target_direction=direction,
                    reference_frame=frame,
                    up=_wind_up(direction),
                    roll_angle=0.0,
                )
        elif elapsed < TILT_END:
            # Phase B: tilt down, wind-aligned
            frac = (elapsed - SWEEP_END) / (TILT_END - SWEEP_END)
            tilt_now = 40.0 - frac * (40.0 - SWITCH_TILT)
            direction = _tilt(tilt_now)
            controls.apply(
                target_direction=direction,
                reference_frame=frame,
                up=_wind_up(direction),
                roll_angle=0.0,
            )
        else:
            if not switched:
                # Phase C: mode switch
                switched = True
                direction = _tilt(SWITCH_TILT)
                controls.apply(
                    target_direction=direction,
                    reference_frame=frame,
                    up=(1.0, 0.0, 0.0),
                    roll_angle=CAPTURE_ROLL,
                )
                print(
                    f">>> MODE SWITCH  tilt={SWITCH_TILT}°"
                    f"  up→horizontal  roll→{CAPTURE_ROLL}°"
                )
            else:
                # Phase D: tilt to vertical, horizontal up, roll locked
                frac = (elapsed - TILT_END) / (total_s - TILT_END)
                tilt_now = SWITCH_TILT * (1.0 - frac)
                direction = _tilt(max(tilt_now, 0.0))
                controls.apply(
                    target_direction=direction,
                    reference_frame=frame,
                    up=(1.0, 0.0, 0.0),
                    roll_angle=CAPTURE_ROLL,
                )

        controls.apply(throttle=0.3)


def main():
    with ConnectionManager(address="127.0.0.1") as km:
        b = km.add_booster("booster-01", VESSEL)
        km.register_target("booster-01", lon=LAUNCHPAD_JNSQ.lon, lat=LAUNCHPAD_JNSQ.lat)
        km.start()
        wait_telemetry(km, "booster-01")
        frame = km.frame("booster-01", "target")

        b.controls.target_smoothing_time = 0.3
        b.controls.apply(throttle=0.3)

        print("\n=== Part 3: Cone sweep with wind-aligned up ===")
        run_cone_sweep_wind(b.controls, frame, tilt_deg=30)

        print("\n=== Part 4: Glide → Landing mode switch (wind-aligned) ===")
        run_glide_to_landing_wind(b.controls, frame)

        b.controls.apply(throttle=0.0)
        b.controls.disengage_auto_pilot()
        print("\nDone.")


if __name__ == "__main__":
    main()
