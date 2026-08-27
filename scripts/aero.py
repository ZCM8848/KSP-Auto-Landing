"""Aero-guidance phase for booster recovery.

The aero segment is an unpowered glide: throttle is zero and the vehicle is
tilted to generate aerodynamic lift.  The reference prediction, however,
assumes immediate ignition at ``TARGET_THROTTLE`` with the nose pointing
opposite the velocity vector (zero lift).  Its endpoint is the hypothetical
landing-burn start point.  When that predicted endpoint drops to or below ground
level, the script hands off to landing burn by aligning retrograde and applying
``TARGET_THROTTLE``.

Debug output:
    * Two long vertical lines in the target frame: red at the predicted
      landing-burn start point, green at the target origin.
    * Console log of the predicted endpoint coordinates each second.

Usage::

    D:\\miniconda3\\envs\\KRPC\\python.exe scripts/aero.py
"""

from __future__ import annotations

import math
import time
from typing import Any

import numpy as np

from recovery import ConnectionManager, FramePacer
from recovery.data.targets import LAUNCHPAD_JNSQ
from recovery.guidance import (
    ConstantThrottle,
    ControlledPredictor,
    ControlSegment,
    LiftTableModel,
    RetrogradeNose,
    VirtualControl,
)
from recovery.ksp.sampling import sample_lift_table

# ---------------------------------------------------------------------------
# User-tunable parameters
# ---------------------------------------------------------------------------

VESSEL = "Booster 2"
TARGET_LON = LAUNCHPAD_JNSQ.lon
TARGET_LAT = LAUNCHPAD_JNSQ.lat

TARGET_THROTTLE = 0.9          # landing-burn throttle used in the endpoint prediction
IGNITE_MARGIN = 500.0          # ignite when predicted endpoint drops below this height (m)
IGNITE_HOLD_S = 2.0            # keep full-throttle retrograde after ignition before exiting (s)
ALPHA_MAX_DEG = 15.0           # maximum angle of attack (deg)
KP = 0.1                       # position gain on predicted endpoint miss (1/s^2)
KD = 0.2                       # velocity-damping gain (1/s); increase to suppress overshoot
ENTRY_VZ = 10.0                # |vertical speed| threshold to enter aero (m/s)
LOOP_HZ = 50.0                 # control-loop rate
LINE_LEN = 50000.0             # length of debug vertical markers (m)


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


def main() -> None:
    with ConnectionManager(address="127.0.0.1") as km:
        km.enable_debug()
        b = km.add_booster("aero", VESSEL)
        b.register_target(lon=TARGET_LON, lat=TARGET_LAT)
        b.start()

        frame = b.frame("target")
        body = b.raw.orbit.body
        flight = b.raw.flight(frame)

        b.physics_range = 200000.0
        b.controls.target_smoothing_time = 0.3
        b.controls.rcs = True

        body_spec, drag_spec = b.sample_predictor_specs()
        density_fn, density_alts, density_vals = _build_density_fn(drag_spec)

        print("Waiting for aero entry: descending and inside atmosphere...")
        entry_state = None
        while entry_state is None:
            s = b.snapshot()
            if s is None:
                time.sleep(0.05)
                continue
            if s.velocity.z < -ENTRY_VZ and s.surface_altitude < s.atmosphere_depth:
                entry_state = s
            time.sleep(0.05)

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

        # Debug markers: long vertical lines pointing to zenith.
        pred_line = b.debug.line(
            (0.0, 0.0, 0.0), (0.0, 0.0, LINE_LEN),
            frame_name="target", color=(1.0, 0.0, 0.0), thickness=0.3,
        )
        b.debug.line(
            (0.0, 0.0, 0.0), (0.0, 0.0, LINE_LEN),
            frame_name="target", color=(0.0, 1.0, 0.0), thickness=0.3,
        )

        pacer = FramePacer(hz=LOOP_HZ)
        t_start = time.monotonic()
        t_last_log = t_start
        log = open("aero_guidance.log", "w", encoding="utf-8")

        header = (
            f"{'t':>6s}  {'end_x':>10s}  {'end_y':>10s}  {'end_z':>10s}  "
            f"{'label':>8s}  {'miss':>10s}  {'alpha':>6s}  {'throttle':>8s}"
        )
        print(header)
        print(header, file=log)
        log.flush()

        intro = (
            f"Aero phase active at t=0; glide throttle=0.0, "
            f"predicted landing-burn throttle={TARGET_THROTTLE}, "
            f"ignite margin={IGNITE_MARGIN:.0f} m, "
            f"max AoA={ALPHA_MAX_DEG} deg, cl_sign={'+' if cl_at_max >= 0 else '-'}"
        )
        print(intro)
        print(intro, file=log)
        log.flush()

        ignite_until: float | None = None

        while True:
            pacer.tick()
            s = b.snapshot()
            if s is None:
                continue

            if ignite_until is not None:
                # Landing-burn hold: full throttle retrograde.
                v = np.array(s.velocity, dtype=float)
                b.controls.apply(
                    target_direction=tuple(-_normalize(v)),
                    reference_frame=frame,
                    throttle=1.0,
                )
                now = time.monotonic()
                if now - t_last_log >= 1.0:
                    line = (
                        f"{now - t_start:6.1f}  {'---':>10s}  {'---':>10s}  "
                        f"{'---':>10s}  {'burn':>8s}  {'---':>10s}  {'---':>6s}  "
                        f"{1.0:8.2f}"
                    )
                    print(line)
                    print(line, file=log)
                    log.flush()
                    t_last_log = now
                if now >= ignite_until:
                    print("Landing burn hold complete; exiting.")
                    break
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
                v_end = np.asarray(traj.final_velocity, dtype=float)
                end_label = "endpoint"
            elif traj.impact is not None:
                p_end = np.asarray(traj.impact.position, dtype=float)
                v_end = np.asarray(traj.final_velocity, dtype=float)
                end_label = "impact"
            else:
                print("Prediction did not reach endpoint or impact — skipping frame.")
                continue

            # Ignition: predicted endpoint dropped through the ignition plane.
            # Regardless of horizontal miss, start landing burn at full throttle.
            if p_end[2] <= IGNITE_MARGIN:
                now = time.monotonic()
                msg = (
                    f"Predicted endpoint below ignition plane "
                    f"(z={p_end[2]:.1f} m <= {IGNITE_MARGIN:.0f} m); "
                    f"starting landing burn."
                )
                print(msg)
                print(msg, file=log)
                log.flush()
                v = np.array(s.velocity, dtype=float)
                b.controls.apply(
                    target_direction=tuple(-_normalize(v)),
                    reference_frame=frame,
                    throttle=1.0,
                )
                ignite_until = now + IGNITE_HOLD_S
                t_last_log = now
                continue

            # PD lateral-acceleration command from predicted endpoint miss.
            r = p_end[:2]
            v_h = v_end[:2]
            a_cmd = -KP * r - KD * v_h
            a_mag = float(np.linalg.norm(a_cmd))

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
            nose = tuple(_normalize(nose))

            # Aero glide: zero throttle, attitude-only control.
            b.controls.apply(
                target_direction=nose,
                reference_frame=frame,
                up=(0.0, 0.0, 1.0),
                throttle=0.0,
            )

            # Update debug markers.  The red line is anchored at the ignition
            # plane (endpoint shifted up by IGNITE_MARGIN) to visualise when
            # the rocket must start landing burn.
            ignition_point = (
                float(p_end[0]), float(p_end[1]),
                float(p_end[2]) + IGNITE_MARGIN,
            )
            pred_line.set_points(
                ignition_point,
                (ignition_point[0], ignition_point[1], ignition_point[2] + LINE_LEN),
            )

            now = time.monotonic()
            if now - t_last_log >= 1.0:
                line = (
                    f"{now - t_start:6.1f}  "
                    f"{p_end[0]:10.1f}  {p_end[1]:10.1f}  {p_end[2]:10.1f}  "
                    f"{end_label:>8s}  "
                    f"{np.hypot(p_end[0], p_end[1]):10.1f}  "
                    f"{math.degrees(alpha_cmd):6.2f}  "
                    f"{0.0:8.2f}"
                )
                print(line)
                print(line, file=log)
                log.flush()
                t_last_log = now

        log.close()


if __name__ == "__main__":
    main()
