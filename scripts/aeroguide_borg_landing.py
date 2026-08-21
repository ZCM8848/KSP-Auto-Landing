"""Aero glide + BORG quadratic powered landing (Booster 2 / Zhuque-3 style).

Fuses ``scripts/demo_aeroguide.py`` (ballistic impact predictor + body-lift
steering) with BORG's fixed-time quadratic landing guidance -- the PEG
explicit solution from "An explicit solution to the exoatmospheric powered
flight guidance and trajectory optimization problem" (McHenry et al., AIAA
1977) -- using the user's energy-based ignition height and a 3->1 engine
cutdown.

Phases
------
ENTRY_BURN engine-on high-altitude brake (方案1) in the thin upper
           atmosphere (from ~45 km) using the NEW RK4 impact predictor
           (thrust + attitude-dependent aero) to refine the aim, slowing
           the fast re-entry so the aero glide and landing burn run at low
           dynamic pressure.  3 engines; exits once speed <= ENTRY_BURN_EXIT_SPEED.
GLIDE      engine off; steer body lift to null the predicted impact miss.
           Ignite at the energy-based height (``energy_ignition``).
QUAD       BORG phase-1: fixed-time quadratic guidance to the touchdown
           target.  Engine set = centre + a symmetric pair; cut to the
           centre engine once thrust demand drops.
TERMINAL   BORG phase-2: vertical constant-deceleration braking.

Bottom altitude is found by projecting each part's position (target frame)
onto the +z up axis and taking the minimum (kRPC ``Vessel.bounds`` is not
available in kRPC 0.6 / this craft).

Reference frames: the target frame has its origin at the landing-site surface
point and +z is local up; the powered phases aim at that origin.

NOTE: this script commands the real vessel.  Do not run it while another
autopilot is actively flying the same booster.
"""

from __future__ import annotations

import math
import time

import numpy as np

from recovery import ConnectionManager, FramePacer
from recovery.data.targets import LAUNCHPAD_JNSQ
from recovery.guidance import LandingPredictor
from binary_burn_landing import AeroParams, BinaryBurnPredictor

VESSEL = "Booster 2"

# -- glide (demo_aeroguide) ----------------------------------------------
MAX_AOA = 20.0        # deg — max angle of attack for body lift
AOA_GAIN = 0.02       # deg per metre of horizontal miss distance
# Landing-burn shortfall margin (BORG approach A). The aero phase sees the
# predicted impact displaced BACK toward the booster by delta, so it steers
# the ballistic trajectory to overshoot the target; the powered descent's
# horizontal shortfall then brings the touchdown back onto the target.
#   delta = GLIDE_MARGIN_FACTOR * GLIDE_MARGIN_BURN_ALT / |tan(FPA)|
# FPA is clamped to >= 10 deg shallow (BORG clamps FPA <= -10 deg) so the
# offset cannot blow up on near-horizontal glides.
GLIDE_MARGIN_FACTOR = 0.33     # BORG's empirical shortfall fraction (~1/3)
GLIDE_MARGIN_BURN_ALT = 5000.0  # assumed powered-descent start height (m). The
                                # powered descent starts at energy ignition (~km),
                                # so its lateral shortfall is much larger than
                                # BORG's fixed 2300 m model. Bumped to 5000 m to
                                # match the ~485 m shortfall observed in flight.

# -- entry burn (方案1: brake in the thin upper atmosphere before the aero glide)
# Slows the fast re-entry first.  At ~800 m/s a small AOA command produces huge
# lift that throws the booster sideways (no lateral manoeuvre authority), so we
# burn high, in thin air, to bring the speed down before the aero glide runs at
# low dynamic pressure (where it can steer) and the landing burn is in envelope.
ENTRY_BURN_START_ALT = 45000.0   # m — begin braking at/below this altitude
ENTRY_BURN_EXIT_SPEED = 400.0    # m/s — hand to the aero glide once slowed to this
ENTRY_MAX_TILT = 30.0            # deg — cone clamp around retrograde during entry burn
ENTRY_STEER_GAIN = 0.3           # strength of the horizontal-error correction

# -- ignition (user's energy method, binary_burn_landing.py) -------------
GFOLD_START_VELOCITY = 140.0   # m/s — target speed for the old burn law
CONE_RATIO = 5.0               # h_gfold = max(CONE_RATIO * d, CONE_FLOOR)
CONE_FLOOR = 3000.0            # m
IGNITION_FLOOR = 5000.0        # m — min ignition altitude
MAX_IGNITE_ALT = 10000.0       # m — hard ceiling: never ignite above this. The
                               # energy law's ign_h scales with altitude and can
                               # reach ~28 km from a high start (too high for the
                               # quadratic terminal guidance). Cap it here.
ENERGY_THRUST_FRAC = 0.9       # reference thrust fraction for the 3-engine burn

# -- powered landing (BORG phase 1 + 2) ----------------------------------
TOUCHDOWN_SPEED = 0.1          # m/s — commanded downward terminal speed
QUAD_AOA_BASE = 25.0           # deg — AOA cone at low dynamic pressure
QUAD_AOA_DOT = 0.5             # deg per second of time-to-go (tightens near ground)
PHASE2_TIME = 2.0              # s — hand quadratic guidance to terminal braking
CUTOFF_HEIGHT = 0.2            # m — bottom altitude engine cutoff
CUTDOWN_FRAC = 0.9             # x max-thrust(centre) below which to cut outer engines
MIN_THROTTLE = 0.0             # min commanded throttle
BOUNDS_UPDATE_PERIOD = 1.0     # s — refresh bottom altitude (legs deploy changes it)
BOUNDS_UPDATE_NEAR = 0.1        # s — near-ground bottom refresh (avoid staleness
                                #       that blinds the guidance in the last metres)
BOUNDS_UPDATE_NEAR_ALT = 200.0  # m — below this bottom altitude, refresh fast
# Time-to-go factor for the quadratic guidance: tgo = TGO_K * 2h/(v_td+|v_z|).
# K=1.0 is the no-braking marginal case (crashes); K>=1.2 gives braking.
# K=1.3 is the validated sweet spot: soft landing (v_z~-1 m/s), miss<=7 m.
# K=1.2 lands closer laterally (~3 m) but touches down hard (v_z~-29 m/s). Try
# values in the script sim before flying.
TGO_K = 1.3

UP = np.array([0.0, 0.0, 1.0])  # target-frame up (z is up)


# ---------------------------------------------------------------------------
# small attitude helpers (from demo_aeroguide.py)
# ---------------------------------------------------------------------------
def wind_up(nose, vel):
    """Return ``up`` so the belly faces the airflow (into *vel*)."""
    d = np.asarray(nose, dtype=float)
    v = np.asarray(vel, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-6:
        return (1.0, 0.0, 0.0)
    belly = v / n
    perp = belly - np.dot(belly, d) * d
    n2 = np.dot(perp, perp)
    if n2 < 1e-12:
        return (1.0, 0.0, 0.0)
    roof = -perp / np.sqrt(n2)
    return (float(roof[0]), float(roof[1]), float(roof[2]))


def nose_for_lift(velocity, miss_xy, max_aoa_deg, gain_deg_per_m):
    """Tilt the engine-end-down (nose-up) direction toward the desired lift."""
    v = np.asarray(velocity, dtype=float)
    speed = np.linalg.norm(v)
    if speed < 1e-6:
        return (0.0, 0.0, 1.0)
    v_dir = v / speed
    base = -v_dir                     # engine-end down = nose opposite velocity
    miss = np.array([miss_xy[0], miss_xy[1], 0.0])
    miss_dist = np.linalg.norm(miss)
    if miss_dist < 1e-3:
        return tuple(base)
    lift_dir = miss / miss_dist
    aoa_deg = min(miss_dist * gain_deg_per_m, max_aoa_deg)
    aoa = np.radians(aoa_deg)
    axis = np.cross(base, lift_dir)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-12:
        return tuple(base)
    axis = axis / axis_norm
    nose = base * np.cos(aoa) + np.cross(axis, base) * np.sin(aoa)
    return (float(nose[0]), float(nose[1]), float(nose[2]))


def clamp_dir(axis, desired, max_deg):
    """Rotate ``axis`` toward ``desired``, keeping within ``max_deg`` of it.

    Conic clamp (BORG / landing.py ``conic_clamp``): if ``desired`` lies
    outside the cone of half-angle ``max_deg`` around ``axis``, snap it onto
    the cone surface in the plane of the two vectors; otherwise return it.
    """
    a = np.asarray(axis, dtype=float)
    d = np.asarray(desired, dtype=float)
    an = np.linalg.norm(a)
    if an < 1e-9:
        return d
    a = a / an
    dn = np.linalg.norm(d)
    if dn < 1e-9:
        return a
    d = d / dn
    if np.dot(a, d) >= np.cos(np.radians(max_deg)):
        return d
    k = np.cross(a, d)
    kn = np.linalg.norm(k)
    if kn < 1e-12:
        return d
    k = k / kn
    ang = np.radians(max_deg)
    r = (a * np.cos(ang) + np.cross(k, a) * np.sin(ang)
         + k * np.dot(k, a) * (1.0 - np.cos(ang)))
    n = np.linalg.norm(r)
    return r / n if n > 1e-9 else a


# ---------------------------------------------------------------------------
# BORG phase-1: fixed-time quadratic guidance (PEG explicit solution)
# ---------------------------------------------------------------------------
class QuadraticGuidance:
    """BORG ``f9_quadratic_fixed_time`` port.

    Solves, at each call, the 4th-order position polynomial that drives the
    state to the touchdown target in ``tgo`` seconds, then returns the
    required thrust acceleration (gravity-compensated).  An angle-of-attack
    limit relative to the retrograde direction is enforced by clipping the
    command into a cone and rescaling to preserve the vertical thrust
    component (BORG's low-cost, height-preserving constraint).
    """

    def __init__(self, up: np.ndarray, g: float) -> None:
        self.up = np.asarray(up, dtype=float)
        self.g = float(g)

    @staticmethod
    def _cone_direction(vel: np.ndarray, cai: np.ndarray,
                        aoa_limit_deg: float) -> np.ndarray:
        """Rotate ``-vel`` toward ``cai`` so it lies on the AOA cone."""
        nv = -np.asarray(vel, dtype=float)
        nv_n = np.linalg.norm(nv)
        if nv_n < 1e-9:
            return np.array(UP)
        nv = nv / nv_n
        axis = np.cross(nv, np.asarray(cai, dtype=float))
        an = np.linalg.norm(axis)
        if an < 1e-12:
            return nv
        axis = axis / an
        ang = np.radians(aoa_limit_deg)
        k = axis
        v = nv
        rot = (v * np.cos(ang) + np.cross(k, v) * np.sin(ang)
               + k * np.dot(k, v) * (1.0 - np.cos(ang)))
        n = np.linalg.norm(rot)
        return rot / n if n > 1e-9 else nv

    def solve(self, r_i, v_i, r_t, v_t, a_t, tgo, aoa_limit_deg):
        """Return ``(cmd_acc, r_t_new)``.

        ``cmd_acc`` is the commanded thrust acceleration (m/s^2), including
        gravity compensation, in the working (target) frame.
        """
        qT = -float(tgo)
        if abs(qT) < 1e-6:
            return np.array(a_t) + self.g * self.up, np.array(r_t)

        R_I = np.asarray(r_i, dtype=float)
        V_I = np.asarray(v_i, dtype=float)
        R_T = np.asarray(r_t, dtype=float)
        V_T = np.asarray(v_t, dtype=float)
        A_T = np.asarray(a_t, dtype=float)

        J = (24 / qT ** 3) * (R_I - R_T) - (6 / qT ** 2) * (V_I + 3 * V_T) - (6 / qT) * A_T
        S = (-72 / qT ** 4) * (R_I - R_T) + (24 / qT ** 3) * (V_I + 2 * V_T) + (12 / qT ** 2) * A_T
        A_I = A_T + J * qT + 0.5 * S * qT ** 2
        C_A_I = A_I + self.g * self.up

        v_norm = np.linalg.norm(V_I)
        aoa = float(np.degrees(np.arccos(
            np.clip(np.dot(C_A_I, -V_I) / (np.linalg.norm(C_A_I) * v_norm + 1e-9),
                    -1.0, 1.0)))) if v_norm > 1e-6 else 0.0

        if aoa < aoa_limit_deg:
            return C_A_I, R_T

        # clip direction onto the cone, preserve vertical thrust component
        C_A_I_dir = self._cone_direction(V_I, C_A_I, aoa_limit_deg)
        up = self.up
        Rz = (np.dot(R_I, up), np.dot(R_T, up))
        Vz = (np.dot(V_I, up), np.dot(V_T, up))
        ATz = np.dot(A_T, up)
        Jz = 24 / qT ** 3 * (Rz[0] - Rz[1]) - 6 / qT ** 2 * (Vz[0] + 3 * Vz[1]) - 6 / qT * ATz
        Sz = -72 / qT ** 4 * (Rz[0] - Rz[1]) + 24 / qT ** 3 * (Vz[0] + 2 * Vz[1]) + 12 / qT ** 2 * ATz
        C_A_Iz = ATz + Jz * qT + 0.5 * Sz * qT ** 2 + self.g
        denom = np.dot(C_A_I_dir, up)
        if abs(denom) < 1e-9:
            C_A_I = C_A_I_dir * (abs(C_A_Iz) + 1e-3)
        else:
            C_A_I = (C_A_Iz / denom) * C_A_I_dir
        A_I = C_A_I - self.g * up
        J = 6 / qT ** 2 * (V_I - V_T) - 2 / qT * (A_I + 2 * A_T)
        S = -12 / qT ** 3 * (V_I - V_T) + 6 / qT ** 2 * (A_I + A_T)
        R_T_new = (R_I - V_T * qT - 0.5 * A_T * qT ** 2
                   - J * qT ** 3 / 6 - S * qT ** 4 / 24)
        return C_A_I, R_T_new


# ---------------------------------------------------------------------------
# vehicle helpers
# ---------------------------------------------------------------------------
def select_engine_triple(vessel):
    """Return ``(centre_idx, pair_idx, engine_objs)`` for the 3->1 cutdown.

    The centre engine is the one with the smallest radial offset from the
    vessel thrust axis (vessel-frame y).  The pair is any two outer engines
    whose x/z positions are near-opposite (symmetric thrust through CoM).
    """
    engines = vessel.parts.engines
    vframe = vessel.reference_frame
    pos = [np.array(e.part.position(vframe), dtype=float) for e in engines]
    radial = [np.hypot(p[0], p[2]) for p in pos]
    centre = int(np.argmin(radial))
    ring = [i for i in range(len(engines)) if i != centre]
    pair = None
    for i in ring:
        for j in ring:
            if j <= i:
                continue
            if np.hypot(pos[i][0] + pos[j][0], pos[i][2] + pos[j][2]) < 0.3:
                pair = (i, j)
                break
        if pair:
            break
    if pair is None:
        pair = (ring[0], ring[1])
    return centre, pair, engines


def compute_bottom_altitude(vessel, target_frame) -> float:
    """Height above the target surface plane of the vessel's lowest part."""
    best = float("inf")
    for p in vessel.parts.all:
        z = p.position(target_frame)[2]
        if z < best:
            best = z
    return best if best != float("inf") else 0.0


# ---------------------------------------------------------------------------
def main() -> None:
    with ConnectionManager(address="127.0.0.1") as km:
        b = km.add_booster("borg", VESSEL)
        km.register_target("borg", lon=LAUNCHPAD_JNSQ.lon, lat=LAUNCHPAD_JNSQ.lat)
        km.enable_debug()
        km.start()

        deadline = time.monotonic() + 5.0
        while km.snapshot("borg") is None:
            if time.monotonic() > deadline:
                raise RuntimeError("telemetry not ready")
            time.sleep(0.02)

        raw = b.raw
        body = raw.orbit.body
        frame = km.frame("borg", "target")
        body_spec, drag_spec = b.sample_predictor_specs()
        predictor = LandingPredictor.from_body_spec(body_spec)
        # entry-burn impact predictor (RK4: gravity + Coriolis + centrifugal +
        # attitude-dependent aero + thrust) — the NEW predictor, from
        # binary_burn_landing.py, used to refine the aim during ENTRY_BURN.
        m_sample = float(b.snapshot().mass) if b.snapshot() is not None else 20000.0
        eb_aero = AeroParams(
            cd0_area=m_sample / max(float(drag_spec.ballistic_coefficient), 1.0),
            cl_area=4.8, k_ind=16.8, clamp_aoa=np.radians(25.0),
        )
        eb_predictor = BinaryBurnPredictor(
            body_spec, aero=eb_aero,
            density_alts=drag_spec.density_alts,
            density_vals=drag_spec.density_vals,
            sea_r=drag_spec.sea_level_radius,
        )

        # engine set: centre + symmetric pair
        centre_idx, pair_idx, engines = select_engine_triple(raw)
        three = [engines[centre_idx], engines[pair_idx[0]], engines[pair_idx[1]]]
        burn_thrust = sum(e.max_thrust for e in three)
        centre_thrust = engines[centre_idx].max_thrust
        print(f"engines: centre={centre_idx} pair={pair_idx} "
              f"burn_thrust={burn_thrust:.0f}N centre_thrust={centre_thrust:.0f}N")

        b.physics_range = 250000.0
        b.controls.target_smoothing_time = 0.3
        b.controls.apply(throttle=0.0, roll_angle=0.0)

        pacer = FramePacer(hz=20)
        phase = "ENTRY_BURN"
        ignited = False
        cutdown = False
        active_max_thrust = burn_thrust
        next_bounds_update = time.monotonic()
        bottom_alt = compute_bottom_altitude(raw, frame)
        start = time.monotonic()

        print("phase  t(s)   miss(m) aoa  alt(m)  v(m/s)  tgo(s)  throttle")
        while True:
            pacer.tick()
            s = b.snapshot()
            if s is None:
                continue
            elapsed = time.monotonic() - start
            vel = np.array([s.velocity[0], s.velocity[1], s.velocity[2]])
            pos = np.array([s.position[0], s.position[1], s.position[2]])

            # bottom altitude (target-plane), refreshed adaptively: fast near the
            # ground so the guidance never goes blind in the last metres.
            if time.monotonic() >= next_bounds_update:
                bottom_alt = compute_bottom_altitude(raw, frame)
                period = (BOUNDS_UPDATE_NEAR if bottom_alt < BOUNDS_UPDATE_NEAR_ALT
                          else BOUNDS_UPDATE_PERIOD)
                next_bounds_update = time.monotonic() + period

            g = body_spec.mu / np.linalg.norm(pos + np.array(body_spec.body_center)) ** 2
            up = UP

            # ---- ENTRY_BURN (方案1: brake in thin air before the glide) ----
            if phase == "ENTRY_BURN":
                speed = float(np.linalg.norm(vel))
                retro = -vel / speed if speed > 1e-6 else up
                if bottom_alt > ENTRY_BURN_START_ALT:
                    # still above the braking altitude: coast down, hold retrograde
                    nose = retro
                    throttle = 0.0
                else:
                    # brake with 3 engines; steer to null the predicted impact
                    # (the NEW RK4 predictor, thrust + attitude-dependent aero)
                    pred = eb_predictor.predict(
                        pos, vel, s.mass, nose=retro, throttle=1.0,
                        available_thrust=float(s.available_thrust),
                        isp=float(s.specific_impulse), dry_mass=float(s.dry_mass))
                    if pred is not None:
                        ep = pred.endpoint
                        he = float(np.hypot(ep[0], ep[1]))
                    else:
                        ep, he = pos, float(np.hypot(pos[0], pos[1]))
                    # nudge the retrograde thrust to push the impact toward origin
                    desired = np.array(retro)
                    if he > 1e-3:
                        push = np.array([-ep[0] / he, -ep[1] / he, 0.0])
                        desired = desired + ENTRY_STEER_GAIN * push
                        dn = np.linalg.norm(desired)
                        if dn > 1e-9:
                            desired = desired / dn
                    nose = clamp_dir(retro, desired, ENTRY_MAX_TILT)
                    throttle = 1.0
                    if speed <= ENTRY_BURN_EXIT_SPEED:
                        print(f"== ENTRY_BURN done: speed={speed:.0f} <= "
                              f"{ENTRY_BURN_EXIT_SPEED:.0f} at alt={bottom_alt:.0f}, "
                              f"he={he:.0f}m ==")
                        phase = "GLIDE"
                b.controls.apply(target_direction=tuple(nose), reference_frame=frame,
                                 up=wind_up(nose, vel), throttle=throttle)
                he = float(np.hypot(pos[0], pos[1]))
                print(f"ENTRY {elapsed:6.1f} {he:8.0f} {0:5.1f} {bottom_alt:8.0f} "
                      f"{speed:6.1f} 0.00 {throttle:6.2f}")
                continue

            # ---- GLIDE ----------------------------------------------------
            if phase == "GLIDE":
                result = predictor.predict_from(s)
                if result is None:
                    nose = tuple(-vel / np.linalg.norm(vel)) if np.linalg.norm(vel) > 1e-6 else (0, 0, 1)
                    b.controls.apply(target_direction=nose, reference_frame=frame,
                                     up=wind_up(nose, vel), throttle=0.0)
                    continue
                bmx, bmy = float(result.position[0]), float(result.position[1])
                d = float(np.hypot(bmx, bmy))          # real ballistic miss (for energy ignition)
                # landing-burn shortfall margin (BORG approach A): the aero
                # steers against a SEEN impact displaced BACK toward the booster,
                # so the ballistic path aims to overshoot by ~delta.
                vxy = np.array([vel[0], vel[1]])
                vxy_n = float(np.linalg.norm(vxy))
                vz_down = max(0.0, -float(vel[2]))
                if vxy_n > 1e-3 and vz_down > 1e-3:
                    slope = min(vxy_n / vz_down, 1.0 / np.tan(np.radians(10.0)))
                    delta = GLIDE_MARGIN_FACTOR * GLIDE_MARGIN_BURN_ALT * slope
                    seen_x = bmx - delta * (vxy[0] / vxy_n)
                    seen_y = bmy - delta * (vxy[1] / vxy_n)
                else:
                    delta = 0.0
                    seen_x, seen_y = bmx, bmy
                miss = (seen_x, seen_y)                # seen miss (steering)
                vrad = float(vel[2])
                # energy-based ignition height
                h_gfold = max(CONE_RATIO * d, CONE_FLOOR)
                de = (0.5 * s.mass * (GFOLD_START_VELOCITY ** 2 - vrad ** 2)
                      + s.mass * g * (h_gfold - s.altitude))
                ign_h = abs(de / max(ENERGY_THRUST_FRAC * burn_thrust - s.mass * g, 1e-6))
                ign_alt = max(ign_h, IGNITION_FLOOR)
                aoa = min(float(np.hypot(*miss)) * AOA_GAIN, MAX_AOA)
                nose = nose_for_lift(vel, miss, MAX_AOA, AOA_GAIN)
                b.controls.apply(target_direction=nose, reference_frame=frame,
                                 up=wind_up(nose, vel), throttle=0.0)
                print(f"{phase} {elapsed:6.1f} {d:8.0f} {float(np.hypot(*miss)):7.0f} "
                      f"{aoa:5.1f} {bottom_alt:8.0f} {np.linalg.norm(vel):6.1f} "
                      f"mgn={delta:5.0f} ign={min(ign_alt, MAX_IGNITE_ALT):6.0f}  0.00")
                if s.altitude <= ign_alt and s.altitude <= MAX_IGNITE_ALT:
                    print(f"== IGNITE at alt={s.altitude:.0f} < "
                          f"ign_alt={ign_alt:.0f} (cap {MAX_IGNITE_ALT}) ==")
                    # ensure exactly the landing triple is active (cut all others)
                    triple_idx = {centre_idx, pair_idx[0], pair_idx[1]}
                    for idx, e in enumerate(engines):
                        e.active = idx in triple_idx
                    ignited = True
                    phase = "QUAD"
                continue

            # ---- powered phases (QUAD + TERMINAL) -------------------------
            if not ignited:
                triple_idx = {centre_idx, pair_idx[0], pair_idx[1]}
                for idx, e in enumerate(engines):
                    e.active = idx in triple_idx
                ignited = True
                active_max_thrust = burn_thrust

            up = UP
            R_T = np.zeros(3)          # will be set: CoM touchdown height = bottom offset
            V_T = -TOUCHDOWN_SPEED * up
            A_T = np.zeros(3)          # target terminal acceleration ~ 0

            if phase == "QUAD":
                # Altitude-consistent time-to-go: descend bottom_alt while
                # braking to touchdown speed under a constant vertical decel.
                # (BORG's speed-only tgo (v-v_td)/a dives when ignition is high,
                # which happens with the energy-based ignition height.)
                h = max(bottom_alt, 0.01)
                downward = max(0.0, -float(vel[2]))
                tgo = TGO_K * 2.0 * h / (TOUCHDOWN_SPEED + downward)
                # target CoM touchdown point: legs (bottom) at ground => CoM
                # sits bottom_offset above the pad.
                R_T = np.array([0.0, 0.0, float(pos[2]) - bottom_alt])
                aoa_limit = min(QUAD_AOA_BASE, QUAD_AOA_DOT * tgo)
                q = QuadraticGuidance(up, g)
                cmd_acc, _ = q.solve(pos, vel, R_T, V_T, A_T, tgo, aoa_limit)

                # 3 -> 1 engine cutdown (BORG thrust-demand trigger)
                demand = float(np.linalg.norm(cmd_acc)) * s.mass
                if (not cutdown) and demand < CUTDOWN_FRAC * centre_thrust:
                    for e in [engines[pair_idx[0]], engines[pair_idx[1]]]:
                        e.active = False
                    cutdown = True
                    active_max_thrust = centre_thrust
                throttle = float(np.clip(demand / active_max_thrust, MIN_THROTTLE, 1.0))

                # Hand to terminal only when it can actually decelerate:
                # the constant-decel law needs h >= v^2/(2·a_terminal) to stop.
                # This prevents entering terminal too low and too fast (which
                # caused the hard landings).  bottom_alt <= CUTOFF_HEIGHT is the
                # unavoidable touchdown fallback.
                speed = float(np.linalg.norm(vel))
                a_term = max(active_max_thrust * 0.9 / s.mass - g, 0.5)
                h_need = (speed * speed - TOUCHDOWN_SPEED ** 2) / (2 * a_term)
                if (tgo <= PHASE2_TIME and bottom_alt >= h_need) or bottom_alt <= CUTOFF_HEIGHT:
                    phase = "TERMINAL"

            else:  # TERMINAL — BORG phase 2 vertical braking
                vrad = float(vel[2])
                downward = max(0.0, -vrad)
                req = (downward ** 2 - TOUCHDOWN_SPEED ** 2) / (2 * max(0.01, bottom_alt)) + g
                if np.linalg.norm(vel) > 0.1:
                    cmd_dir = -vel / np.linalg.norm(vel)      # surface retrograde
                else:
                    cmd_dir = up
                cmd_acc = max(0.0, req) * cmd_dir
                throttle = float(np.clip(np.linalg.norm(cmd_acc) * s.mass / active_max_thrust,
                                         MIN_THROTTLE, 1.0))
                if vrad >= 0.0 or bottom_alt <= CUTOFF_HEIGHT:
                    b.controls.apply(throttle=0.0)
                    print("== TOUCHDOWN/CUTOFF ==")
                    break

            nose = cmd_acc / np.linalg.norm(cmd_acc) if np.linalg.norm(cmd_acc) > 1e-6 else tuple(up)
            b.controls.apply(target_direction=tuple(nose), reference_frame=frame,
                             up=wind_up(nose, vel), throttle=throttle)
            tgo = tgo if phase == "QUAD" else float("nan")
            print(f"{phase} {elapsed:6.1f} {0:8.0f} {0:5.1f} "
                  f"{bottom_alt:8.0f} {np.linalg.norm(vel):6.1f} "
                  f"{tgo if not np.isnan(tgo) else 0.0:6.2f} {throttle:6.2f}")


if __name__ == "__main__":
    main()
