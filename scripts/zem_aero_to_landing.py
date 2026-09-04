"""Merged boosterback + aero guidance + landing burn.

Phase 1 (boosterback): high-altitude full-throttle burn to null the zero-effort
miss; the nose points horizontally toward the target.  Ends when the predicted
miss stops decreasing or the altitude drops below ``MIN_ALT``.

Phase 2 (coast): after boosterback ends but before aero entry, the nose is held
toward the target frame's +z direction (engines-down attitude) while the booster
descends into the atmosphere.

Phase 3 (correction burn): at aero entry, a ballistic prediction with the
drag-only boosterback predictor checks the horizontal landing miss; if it
exceeds ``CORRECTION_MARGIN``, the nose slews toward the target and, once it
is within ``CORRECTION_LATCH_ANGLE_DEG`` of that direction (latch), burns at
full throttle until the miss closes below the margin.

Phase 4 (aero + landing burn): once descending inside the atmosphere, the
booster glides unpowered with aerodynamic lift, then ignites for landing-burn
phases A and B.

Usage::

    python scripts/zem_aero_to_landing.py
"""

from __future__ import annotations

import math
import time
from collections import Counter
from typing import Any

import numpy as np

from recovery import ConnectionManager, FramePacer
from recovery.data.targets import LANDSPACE_LZ
from recovery.guidance import (
    ConstantThrottle,
    ControlledPredictor,
    ControlSegment,
    DragModel,
    LandingPredictor,
    LiftTableModel,
    RetrogradeNose,
    VirtualControl,
)
from recovery.ksp.sampling import sample_lift_table

# ---------------------------------------------------------------------------
# User-tunable parameters
# ---------------------------------------------------------------------------

VESSEL = "ZQ-3 demo Probe"
TARGET_LON = LANDSPACE_LZ.lon
TARGET_LAT = LANDSPACE_LZ.lat

# Boosterback phase
MIN_ALT = 8000.0                # boosterback window floor (m)
ROI_MISS = 50000.0              # ignore miss-increase below this threshold (m)

# Correction burn (aero-entry ballistic check)
CORRECTION_MARGIN = 500.0        # ballistic landing horizontal miss to close (m)
CORRECTION_LATCH_ANGLE_DEG = 2.0  # nose must be within this angle of target dir before firing (deg)

# Aero + landing-burn phases
TARGET_THROTTLE = 0.9           # landing-burn throttle used in endpoint prediction
IGNITE_MARGIN = 300.0           # ignite when predicted endpoint drops below this height (m)
PEG_VELOCITY = 30.0             # target absolute speed at IGNITE_MARGIN (m/s)
T_GAIN = 2.0                    # time-to-go gain for phase-B polynomial guidance
T_MIN = 5.0                     # minimum time-to-go for phase-B (s)
ALPHA_MAX_DEG = 15.0            # maximum angle of attack (deg)
KP = 0.2                        # position gain on predicted endpoint miss (1/s^2)
KD = 0.15                       # velocity-damping gain (1/s)
DAMP_BLEND_ENDPOINT = 0.5       # endpoint-vs-current horizontal velocity blend
R_DEADBAND = 0.0                # horizontal endpoint miss considered "on target" (m)
NOSE_SMOOTHING = 0.2            # EMA weight for nose direction
ENTRY_VZ = 10.0                 # |vertical speed| threshold to enter aero (m/s)
AERO_ENTRY_Q = 200.0            # minimum dynamic pressure (Pa) to enter aero
LOOP_HZ = 50.0                  # control-loop rate

# Action group numbers used by toggle_action_group in this vessel setup.
AG2_THREE_TO_FIVE = 2
AG3_THREE_TO_ONE = 3


def _normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return v
    return v / n


def _perpendicular_to(v: np.ndarray) -> np.ndarray:
    """Return a unit vector perpendicular to *v*."""
    v = _normalize(v)
    if abs(v[2]) < 0.9:
        perp = np.cross(v, np.array([0.0, 0.0, 1.0]))
    else:
        perp = np.cross(v, np.array([1.0, 0.0, 0.0]))
    return _normalize(perp)


def _build_density_fn(drag_spec: Any) -> tuple[Any, np.ndarray, np.ndarray]:
    """Build an atmospheric-density interpolator from a DragSpec table."""
    alts = drag_spec.density_alts
    vals = drag_spec.density_vals
    depth = float(alts[-1])

    def _interp(h: float) -> float:
        if h < 0.0:
            return float(vals[0])
        if h > depth:
            return 0.0
        return float(np.interp(h, alts, vals))

    return _interp, alts, vals


def get_half_rocket_length(rocket: Any) -> float:
    """Return the average distance of parts below the CoM to the CoM.

    This estimates how far the rocket's bottom is below its center of mass,
    which is used as the target touchdown height for the CoM.
    """
    part_distance = [
        float(np.linalg.norm(part.position(rocket.reference_frame)))
        for part in rocket.parts.all
        if part.position(rocket.reference_frame)[1] < 0
    ]
    if not part_distance:
        return 0.0
    value_weight_dict = dict(Counter(part_distance))
    total_weight = len(part_distance)
    weighted_sum = sum(value * weight for value, weight in value_weight_dict.items())
    return weighted_sum / total_weight


def _run_boosterback(b: Any, *, frame: Any, predictor: Any) -> None:
    """Phase 1: full-throttle ZEM boosterback; returns when the burn ends."""
    b.controls.apply(reference_frame=frame, throttle=1.0)
    error_hist: list[float] = [float("inf")]
    pacer = FramePacer(hz=LOOP_HZ)
    print("Boosterback: full-throttle burn to null the zero-effort miss...")
    while True:
        pacer.tick()
        s = b.snapshot()
        if s is None:
            continue

        if s.surface_altitude < MIN_ALT:
            print(f"Below {MIN_ALT:.0f}m boosterback window — coasting.")
            break

        result = predictor.predict_from(s, rtol=5e-6, atol=5e-6)
        if result is None:
            continue

        mx = float(result.position[0])
        my = float(result.position[1])
        miss = float(np.hypot(mx, my))

        # Nose horizontally toward the target so thrust pushes the impact
        # point onto the landing site.
        if miss < 1.0:
            target_direction = (0.0, 0.0, -1.0)
        else:
            target_direction = (-mx / miss, -my / miss, 0.0)
        b.controls.apply(
            target_direction=target_direction,
            reference_frame=frame,
            up=(0.0, 0.0, 1.0),
            roll_angle=0.0,
        )

        if miss < ROI_MISS and miss > min(error_hist):
            b.controls.cut_thrust()
            print(
                f"Miss stopped decreasing — boosterback complete "
                f"(min={min(error_hist):.0f}m)."
            )
            break

        error_hist.append(miss)

    b.controls.cut_thrust()
    # Switch to 5-engine mode for the landing burn and deploy the air brakes
    # (grid fins).
    b.controls.toggle_action_group(AG2_THREE_TO_FIVE)
    b.controls.brakes = True
    print("Toggled action group 2 (3-engine -> 5-engine switch); brakes (grid fins) deployed.")


def _wait_for_aero_entry(b: Any, *, frame: Any) -> Any:
    """Phase 2: hold the nose toward target-frame +z until aero entry."""
    print("Coast: holding nose toward target-frame +z until aero entry...")
    while True:
        b.controls.apply(
            target_direction=(0.0, 0.0, 1.0),
            reference_frame=frame,
            throttle=0.0,
        )
        s = b.snapshot()
        if s is None:
            time.sleep(0.05)
            continue
        speed = float(np.linalg.norm(np.asarray(s.velocity, dtype=float)))
        q = 0.5 * s.atmosphere_density * speed * speed
        if s.velocity.z < -ENTRY_VZ and q >= AERO_ENTRY_Q:
            return s
        time.sleep(0.05)


def _run_correction_burn(b: Any, *, frame: Any, predictor: Any) -> None:
    """Aero-entry ballistic check + correction burn.

    Uses the drag-only boosterback predictor to compute the ballistic landing
    miss.  If the horizontal miss exceeds ``CORRECTION_MARGIN``, the nose is
    slewed horizontally toward the target; once it is within
    ``CORRECTION_LATCH_ANGLE_DEG`` of that direction (a one-way latch) the
    engine fires at full throttle until the miss closes below
    ``CORRECTION_MARGIN``.
    """
    pacer = FramePacer(hz=LOOP_HZ)
    latched = False
    print("Correction burn: ballistic landing check with boosterback predictor...")
    while True:
        pacer.tick()
        s = b.snapshot()
        if s is None:
            continue

        result = predictor.predict_from(s, rtol=5e-6, atol=5e-6)
        if result is None:
            continue

        mx = float(result.position[0])
        my = float(result.position[1])
        miss = float(np.hypot(mx, my))

        if miss <= CORRECTION_MARGIN:
            b.controls.cut_thrust()
            if latched:
                print(
                    f"Correction burn complete — ballistic miss "
                    f"{miss:.0f}m <= {CORRECTION_MARGIN:.0f}m."
                )
            else:
                print(
                    f"Ballistic miss {miss:.0f}m within {CORRECTION_MARGIN:.0f}m "
                    f"— skipping correction burn."
                )
            break

        if miss < 1.0:
            target_direction = (0.0, 0.0, -1.0)
        else:
            target_direction = (-mx / miss, -my / miss, 0.0)

        nose = np.asarray(s.direction, dtype=float)
        td = np.asarray(target_direction, dtype=float)
        cos_ang = float(np.dot(_normalize(nose), _normalize(td)))
        angle = float(np.arccos(np.clip(cos_ang, -1.0, 1.0)))
        if angle <= math.radians(CORRECTION_LATCH_ANGLE_DEG):
            latched = True

        throttle = 1.0 if latched else 0.0
        b.controls.apply(
            target_direction=target_direction,
            reference_frame=frame,
            up=(0.0, 0.0, 1.0),
            roll_angle=0.0,
            throttle=throttle,
        )


def _run_aero_and_landing(
    b: Any,
    *,
    frame: Any,
    body: Any,
    flight: Any,
    body_spec: Any,
    drag_spec: Any,
    density_fn: Any,
    density_alts: np.ndarray,
    density_vals: np.ndarray,
    half_length: float,
    entry_state: Any,
) -> None:
    """Phase 3: aero glide + landing-burn phases A and B."""
    alpha_pts, cl_table, cd_table, clamp_aoa = sample_lift_table(
        body=body,
        flight=flight,
        target_frame=frame,
        position=tuple(entry_state.position),
        velocity=tuple(entry_state.velocity),
        alpha_max_deg=ALPHA_MAX_DEG,
        n_alpha=16,
    )

    aero = LiftTableModel(
        alpha_pts=alpha_pts,
        cl_table=cl_table,
        cd_table=cd_table,
        clamp_aoa=clamp_aoa,
        density_fn=density_fn,
        body_center=drag_spec.body_center,
        sea_level_radius=drag_spec.sea_level_radius,
        density_alts=density_alts,
        density_vals=density_vals,
    )
    predictor = ControlledPredictor.from_body_spec(body_spec, aero=aero)

    alpha_max = math.radians(ALPHA_MAX_DEG)
    cl_at_max = float(np.interp(alpha_max, alpha_pts, cl_table))

    pacer = FramePacer(hz=LOOP_HZ)
    t_start = time.monotonic()
    t_last_log = t_start
    log = open("zem_aero_to_landing.log", "w", encoding="utf-8")

    header = (
        f"{'t':>6s}  {'phase':>7s}  {'alt':>10s}  {'|v|':>10s}  "
        f"{'throttle':>8s}  {'end_x':>10s}  {'end_y':>10s}  {'end_z':>10s}"
    )
    print(header)
    print(header, file=log)
    log.flush()

    intro = (
        f"Aero+landing burn active at t=0; glide throttle=0.0, "
        f"predicted landing-burn throttle={TARGET_THROTTLE}, "
        f"ignite margin={IGNITE_MARGIN:.0f} m, "
        f"PEG velocity={PEG_VELOCITY:.1f} m/s, "
        f"touchdown height={half_length:.1f} m, "
        f"max AoA={ALPHA_MAX_DEG} deg, cl_sign={'+' if cl_at_max >= 0 else '-'}"
    )
    print(intro)
    print(intro, file=log)
    log.flush()

    phase = 0  # 0=aero, 1=landing burn A, 2=landing burn B
    target_pos_b = np.array([0.0, 0.0, half_length])
    p_end_prev: np.ndarray | None = None
    t_prev: float | None = None
    nose_prev: np.ndarray | None = None

    while True:
        pacer.tick()
        s = b.snapshot()
        if s is None:
            continue

        if phase == 2:
            # Landing-burn phase B: polynomial guidance to touchdown.
            pos = np.array(s.position, dtype=float)
            vel = np.array(s.velocity, dtype=float)
            v_mag = float(np.linalg.norm(vel))
            alt = s.surface_altitude

            if s.landed:
                msg = f"Touchdown detected at alt={alt:.1f} m, |v|={v_mag:.1f} m/s"
                print(msg)
                print(msg, file=log)
                log.flush()
                b.controls.cut_thrust()
                break

            r = pos - target_pos_b
            v_z = abs(vel[2])
            T = max(T_MIN, T_GAIN * alt / max(v_z, 1e-3))
            a_cmd = -6.0 * r / (T * T) - 4.0 * vel / T
            g_vec = np.array([0.0, 0.0, -body_spec.surface_gravity])
            a_thrust = a_cmd - g_vec
            a_thrust_mag = float(np.linalg.norm(a_thrust))
            throttle = float(np.clip(s.mass * a_thrust_mag / s.max_thrust, 0.0, 1.0))
            thrust_dir = a_thrust / a_thrust_mag if a_thrust_mag > 1e-6 else -_normalize(vel)

            b.controls.apply(
                target_direction=tuple(thrust_dir),
                reference_frame=frame,
                throttle=throttle,
            )

            now = time.monotonic()
            if now - t_last_log >= 1.0:
                line = (
                    f"{now - t_start:6.1f}  {'burn_b':>7s}  {alt:10.1f}  "
                    f"{v_mag:10.1f}  {throttle:8.2f}  "
                    f"{target_pos_b[0]:10.1f}  {target_pos_b[1]:10.1f}  "
                    f"{target_pos_b[2]:10.1f}"
                )
                print(line)
                print(line, file=log)
                log.flush()
                t_last_log = now
            continue

        if phase == 1:
            # Landing-burn phase A: brake absolute speed to PEG_VELOCITY at
            # altitude IGNITE_MARGIN, thrust strictly retrograde.
            v = np.array(s.velocity, dtype=float)
            v_mag = float(np.linalg.norm(v))
            alt = s.surface_altitude

            if alt <= IGNITE_MARGIN and v_mag <= PEG_VELOCITY:
                msg = (
                    f"Landing burn phase A complete: "
                    f"alt={alt:.1f} m, |v|={v_mag:.1f} m/s; entering phase B"
                )
                print(msg)
                print(msg, file=log)
                log.flush()
                phase = 2
                b.controls.apply(legs=True)
                # Select the single central engine for touchdown.
                # b.controls.toggle_action_group(AG3_THREE_TO_ONE)
                deploy_msg = (
                    "Deployed landing legs/gear, toggled action group 3 "
                    "(3-engine -> 1-engine)."
                )
                print(deploy_msg)
                print(deploy_msg, file=log)
                log.flush()
                t_last_log = time.monotonic()
                continue

            acc = (
                (v_mag * v_mag - PEG_VELOCITY * PEG_VELOCITY)
                / (2.0 * max(alt - IGNITE_MARGIN, 1.0))
            )
            g_local = body_spec.surface_gravity
            throttle = float(
                np.clip(s.mass * (acc + g_local) / s.max_thrust, 0.0, 1.0)
            )

            b.controls.apply(
                target_direction=tuple(-_normalize(v)),
                reference_frame=frame,
                throttle=throttle,
            )

            now = time.monotonic()
            if now - t_last_log >= 1.0:
                line = (
                    f"{now - t_start:6.1f}  {'burn_a':>7s}  {alt:10.1f}  "
                    f"{v_mag:10.1f}  {throttle:8.2f}  "
                    f"{s.position.x:10.1f}  {s.position.y:10.1f}  {s.position.z:10.1f}"
                )
                print(line)
                print(line, file=log)
                log.flush()
                t_last_log = now
            continue

        ref_control = VirtualControl((
            ControlSegment(
                throttle=ConstantThrottle(TARGET_THROTTLE),
                max_thrust=s.max_thrust,
                isp=s.specific_impulse,
                nose=RetrogradeNose(),
            ),
        ), dry_mass=s.dry_mass)

        traj = predictor.predict(
            s.position, s.velocity, s.mass, ref_control, stop_on_endpoint=True,
        )

        if traj.endpoint is not None:
            p_end = np.asarray(traj.endpoint.position, dtype=float)
        elif traj.impact is not None:
            p_end = np.asarray(traj.impact.position, dtype=float)
        else:
            print("Prediction did not reach endpoint or impact — skipping frame.")
            continue

        # Ignition: predicted endpoint dropped through the ignition plane.
        # Regardless of horizontal miss, start landing burn phase A.
        if p_end[2] <= IGNITE_MARGIN:
            now = time.monotonic()
            msg = (
                f"Predicted endpoint below ignition plane "
                f"(z={p_end[2]:.1f} m <= {IGNITE_MARGIN:.0f} m); "
                f"entering landing burn phase A."
            )
            print(msg)
            print(msg, file=log)
            log.flush()
            phase = 1
            t_last_log = now
            continue

        # PD lateral-acceleration command from predicted endpoint miss.
        # Damping blends the predicted-endpoint drift rate with the rocket's
        # current horizontal velocity to balance responsiveness and drag.
        r = p_end[:2]
        r_mag = float(np.linalg.norm(r))
        now = time.monotonic()
        if p_end_prev is not None and t_prev is not None:
            dt = max(now - t_prev, 1.0 / LOOP_HZ)
            v_endpoint = (p_end[:2] - p_end_prev) / dt
        else:
            v_endpoint = np.zeros(2)
        v_current = np.array(s.velocity, dtype=float)[:2]
        v_h = DAMP_BLEND_ENDPOINT * v_endpoint + (1.0 - DAMP_BLEND_ENDPOINT) * v_current

        if r_mag <= R_DEADBAND:
            # Close enough to the target: stop lateral steering and fly
            # nose-on-velocity (zero lift) to avoid high-frequency chatter.
            a_cmd = np.zeros(2)
        else:
            a_cmd = -KP * r - KD * v_h
        a_mag = float(np.linalg.norm(a_cmd))

        # Save endpoint history for the next frame's damping term.
        p_end_prev = p_end[:2].copy()
        t_prev = now

        # Available lateral acceleration at current dynamic pressure.
        v = np.array(s.velocity, dtype=float)
        v_mag = float(np.linalg.norm(v))
        v_hat = _normalize(v)
        q = 0.5 * s.atmosphere_density * v_mag * v_mag
        a_lat_max = q * abs(cl_at_max) / max(s.mass, 1e-3)

        # Desired AoA and tilt direction.
        if a_mag > 1e-3 and a_lat_max > 1e-6:
            alpha_cmd = min(a_mag / a_lat_max, alpha_max)
            a_cmd_3d = np.array([a_cmd[0], a_cmd[1], 0.0])
            a_perp = a_cmd_3d - np.dot(a_cmd_3d, v_hat) * v_hat
            perp_mag = float(np.linalg.norm(a_perp))
            if perp_mag > 1e-6:
                lift_dir = a_perp / perp_mag
            else:
                alpha_cmd = 0.0
                lift_dir = _perpendicular_to(v_hat)
        else:
            alpha_cmd = 0.0
            lift_dir = _perpendicular_to(v_hat)

        # The lift-table sign tells us which side the lift actually acts on.
        tilt_dir = lift_dir if cl_at_max >= 0.0 else -lift_dir
        nose = -v_hat * math.cos(alpha_cmd) + tilt_dir * math.sin(alpha_cmd)
        nose = _normalize(nose)

        # Low-pass filter the nose direction to suppress high-frequency
        # attitude jitter when the predicted endpoint jitters near target.
        if nose_prev is not None:
            nose = NOSE_SMOOTHING * nose + (1.0 - NOSE_SMOOTHING) * nose_prev
            nose = _normalize(nose)
        nose_prev = nose.copy()
        nose = tuple(nose)

        # Aero glide: zero throttle, attitude-only control.
        b.controls.apply(
            target_direction=nose,
            reference_frame=frame,
            up=(0.0, 0.0, 1.0),
            throttle=0.0,
        )

        now = time.monotonic()
        if now - t_last_log >= 1.0:
            line = (
                f"{now - t_start:6.1f}  {'aero':>7s}  {s.surface_altitude:10.1f}  "
                f"{v_mag:10.1f}  {0.0:8.2f}  "
                f"{p_end[0]:10.1f}  {p_end[1]:10.1f}  {p_end[2]:10.1f}"
            )
            print(line)
            print(line, file=log)
            log.flush()
            t_last_log = now

    log.close()


def main() -> None:
    with ConnectionManager(address="127.0.0.1") as km:
        b = km.add_booster("landing", VESSEL)
        b.register_target(lon=TARGET_LON, lat=TARGET_LAT)
        b.start()

        frame = b.frame("target")
        body = b.raw.orbit.body
        flight = b.raw.flight(frame)
        half_length = get_half_rocket_length(b.raw)

        b.physics_range = 200000.0
        b.controls.target_smoothing_time = 0.3
        b.controls.rcs = True

        body_spec, drag_spec = b.sample_predictor_specs()
        density_fn, density_alts, density_vals = _build_density_fn(drag_spec)

        # Boosterback uses the simple drag-only impact predictor.
        zb_predictor = LandingPredictor.from_body_spec(
            body_spec, aero=DragModel.from_spec(drag_spec)
        )

        # Phase 1: boosterback.
        _run_boosterback(b, frame=frame, predictor=zb_predictor)

        # Phase 2: default +z coast until aero entry.
        entry_state = _wait_for_aero_entry(b, frame=frame)

        # Phase 2.5: aero-entry ballistic check + correction burn.
        _run_correction_burn(b, frame=frame, predictor=zb_predictor)

        # Phase 3: aero glide + landing burn.
        _run_aero_and_landing(
            b,
            frame=frame,
            body=body,
            flight=flight,
            body_spec=body_spec,
            drag_spec=drag_spec,
            density_fn=density_fn,
            density_alts=density_alts,
            density_vals=density_vals,
            half_length=half_length,
            entry_state=entry_state,
        )


if __name__ == "__main__":
    main()
