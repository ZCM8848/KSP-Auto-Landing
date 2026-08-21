"""Binary-sign predictive guidance: unpowered air guidance + landing burn with
online sign selection.

Control flow (based on scripts/demo_aeroguide_and_burn.py; ignition height and
throttle laws from the original landing.py in KSP-Auto-Landing-main/):

  Phase A  unpowered air guidance — demo steering law, FIXED sign (nose tilts
           toward the predicted miss), throttle 0.  The impact prediction uses
           the tail-first attitude (AoA = 0), i.e. exactly the demo's
           LandingPredictor behaviour.
           Ignition when altitude <= max(energy_height, 5000)  [old landing.py]
           with prediction-based fallbacks (IGNITE_MISS / stopped-decreasing /
           altitude floor).
  Phase B  ONE powered-descent phase (burn and terminal are merged): the
           steering law is the same as the air phase, driven by a ZEM
           baseline — the ballistic (no-thrust) impact prediction always
           yields a landing point, and the target direction points from it
           toward the origin.  Every tick the predictor simulates ONLINE
           (a pure query — the vessel does not move) the horizontal force
           that WOULD act at each candidate attitude with the CURRENT
           throttle (thrust lateral component + aero drag/lift lateral
           component); the candidate whose force pushes the ZEM impact point
           hardest toward the target wins; retrograde when neither does.
           Throttle: energy law above the cone (target 140 m/s at
           gfold_start_altitude), old descent_throttle law (vt = -2 m/s)
           below it.  Cutoff on touchdown or hover near the ground.

Pure core (no kRPC): BinaryBurnPredictor, BinaryGuidance, the throttle laws and
steering helpers are importable for local closed-loop validation
(D:/hermes_ws/validate_binary_burn.py runs the exact same code without KSP).

Usage (repo root, KRPC env):
    python scripts/binary_burn_landing.py
"""

from __future__ import annotations

import csv
import pathlib
import time
from dataclasses import dataclass
from math import acos, cos, pi, sin, sqrt

import numpy as np

try:  # optional acceleration — pure-Python fallback below
    from numba import njit
except ImportError:  # pragma: no cover
    njit = None

from recovery import ConnectionManager, FramePacer
from recovery.data.targets import LAUNCHPAD_JNSQ

G0 = 9.80665

# ---------------------------------------------------------------------------
# Guidance settings
# ---------------------------------------------------------------------------
VESSEL = "Booster 2"
TARGET = LAUNCHPAD_JNSQ

# -- steering (demo_aeroguide_and_burn.py) ----------------------------------
MAX_AOA = 20.0               # degrees — max commanded angle of attack
AOA_GAIN = 0.02              # deg per metre of predicted impact miss
# DEMO_SIGN: the air-phase steering law tilts the nose TOWARD the miss
# (the demo convention, lift_dir = miss_hat).  The burn/terminal phase
# evaluates both signs every tick; this constant only seeds the initial sign.
DEMO_SIGN = 1.0

# -- ignition (old landing.py) ----------------------------------------------
GFOLD_START_VELOCITY = 140.0  # m/s — vertical speed target for the burn law
# Throttle clamp.  The OLD landing.py used [0.4, 1] — the 0.4 floor existed
# because G-FOLD took over right after the landing burn.  Without G-FOLD the
# floor DEADLOCKS high-TWR boosters: descent_throttle's demand drops below
# 0.4*T/m while 0.4*T/m still exceeds gravity, so the rocket hovers forever
# (observed: frozen at 1.6 km, thr=0.40).  Floor 0 lets the vertical law
# throttle down and keep descending.
THROTTLE_LIMIT = (0.0, 1.0)
IGNITION_FLOOR = 5000.0       # m — min ignition altitude (old landing.py)
CONE_RATIO = 5.0              # gfold_start_altitude = max(CONE_RATIO*d, ...)
CONE_FLOOR = 3000.0           # m — ... floor
BURN_CEILING_ALT = 20_000.0   # m — hard ceiling: never enter phase B above this
# -- prediction fallback triggers (zem_aeroguide_landing.py) -----------------
IGNITE_MISS = 300.0           # m — ignite if the predicted impact is this close
MISS_HYSTERESIS = 150.0       # m — ignite when d rises this far above d_min
MIN_IGNITE_ALT = 4000.0       # m — safety floor: ignite no matter what below
PROGRESS_FRACTION = 0.7       # d must drop below this x d_init before the
                              #   "stopped decreasing" trigger may fire

# -- binary sign evaluation -------------------------------------------------
BINARY_HOLD_TICKS = 5         # direction-consistency decision must favour a
                              #   candidate for this many consecutive ticks
                              #   before switching (anti-chatter); switching
                              #   INTO retrograde needs twice as long
# Direction score threshold: a candidate only counts as "consistent" if its
# simulated horizontal force projection onto the target direction exceeds
# this fraction of the vessel's weight.  Filters the low-speed noise that
# made the sign oscillate between -1 and 0 near hover (observed in flight).
SCORE_THRESHOLD_FRAC = 0.03

# -- terminal (old landing.py) ----------------------------------------------
VT_TOUCHDOWN = -2.0           # m/s — vertical speed target at touchdown
LEGS_ALT = 100.0              # m — deploy legs below this altitude
LEGS_TIME_RATIO = 4.0         # ... or when |r|/|v| <= this
CUTOFF_SPEED = 0.2            # m/s — end the hold-attitude tail after cutoff
CUTOFF_ALT = 50.0             # m — vrad>=0 only cuts off near the ground

# -- loops / prediction -----------------------------------------------------
PREDICT_HZ = 10.0
CONTROL_HZ = 20.0
PREDICT_DT = 0.1              # s — impact-prediction integration step
PREDICT_TMAX = 300.0          # s — max predicted trajectory duration
SMOOTHING_TIME = 0.3          # s — autopilot attitude smoothing

LOG_FILE = "binary_burn_debug.log"
CSV_FILE = "binary_burn_telemetry.csv"


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ---------------------------------------------------------------------------
# Attitude-dependent aero parameters (pure data)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class AeroParams:
    """Attitude-dependent aerodynamic model for the impact predictor.

    cd0_area:  zero-lift drag area C_D(0)*A           (m^2)
    cl_area:   lift slope C_L_alpha*A                (m^2/rad)
    k_ind:     induced-drag factor: C_D(a) = C_D0*(1 + k_ind*a^2)   (1/rad^2)
    clamp_aoa: stall clamp for the force model       (rad)
    """

    cd0_area: float
    cl_area: float
    k_ind: float
    clamp_aoa: float


@dataclass(frozen=True)
class ImpactPred:
    """Predicted impact point of a candidate attitude policy."""

    endpoint: np.ndarray     # impact position in the target frame (m)
    time: float              # time to impact (s)
    speed: float             # speed at impact (m/s)
    vrad: float              # radial (vertical) speed at impact (m/s)
    vh: float                # horizontal speed at impact (m/s)


# ---------------------------------------------------------------------------
# Numba kernels (fixed body-axis + attitude-dependent aero)
# ---------------------------------------------------------------------------
if njit is not None:

    @njit(cache=True)
    def _interp_jit(x, xp, fp):
        n = len(xp)
        if x <= xp[0]:
            return float(fp[0])
        if x > xp[n - 1]:
            return 0.0
        for i in range(n - 1):
            if xp[i] <= x <= xp[i + 1]:
                t = (x - xp[i]) / (xp[i + 1] - xp[i])
                return float(fp[i] + t * (fp[i + 1] - fp[i]))
        return 0.0

    @njit(cache=True)
    def _acc_jit(r, v, m, t_eng, mu, omega, center, radius,
                 nose, cd0a, cla, kind, clamp_aoa, alts, densities, sea_r):
        """Gravity + Coriolis + centrifugal + attitude-dependent aero +
        thrust along the FIXED body axis *nose*."""
        a = np.empty(3, dtype=np.float64)
        dx = r[0] - center[0]
        dy = r[1] - center[1]
        dz = r[2] - center[2]
        dist_sq = dx * dx + dy * dy + dz * dz
        dist = sqrt(dist_sq)
        inv_cube = 1.0 / (dist * dist * dist)
        g = -mu * inv_cube
        a[0] = g * dx
        a[1] = g * dy
        a[2] = g * dz
        # Coriolis: -2 * (omega x v)
        a[0] += -2.0 * (omega[1] * v[2] - omega[2] * v[1])
        a[1] += -2.0 * (omega[2] * v[0] - omega[0] * v[2])
        a[2] += -2.0 * (omega[0] * v[1] - omega[1] * v[0])
        # centrifugal: -(omega x (omega x d))
        ox_dx = omega[1] * dz - omega[2] * dy
        ox_dy = omega[2] * dx - omega[0] * dz
        ox_dz = omega[0] * dy - omega[1] * dx
        a[0] += -(omega[1] * ox_dz - omega[2] * ox_dy)
        a[1] += -(omega[2] * ox_dx - omega[0] * ox_dz)
        a[2] += -(omega[0] * ox_dy - omega[1] * ox_dx)
        speed = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        alt = dist - sea_r
        rho = _interp_jit(alt, alts, densities)
        if rho > 0.0 and speed > 1e-6 and m > 0.0:
            vx, vy, vz = v[0] / speed, v[1] / speed, v[2] / speed
            # flow angle of attack relative to tail-first
            c = -(nose[0] * vx + nose[1] * vy + nose[2] * vz)
            if c > 1.0:
                c = 1.0
            elif c < -1.0:
                c = -1.0
            alpha = acos(c)
            if alpha > clamp_aoa:
                alpha = clamp_aoa
            q = 0.5 * rho * speed * speed
            # drag along -v, with induced drag
            drag = q * cd0a * (1.0 + kind * alpha * alpha) / m
            a[0] -= drag * vx
            a[1] -= drag * vy
            a[2] -= drag * vz
            if alpha > 1e-4:
                # lift toward the nose-deflection side: lat = n - (n.v)v
                nv = nose[0] * vx + nose[1] * vy + nose[2] * vz
                lat_x = nose[0] - nv * vx
                lat_y = nose[1] - nv * vy
                lat_z = nose[2] - nv * vz
                ln = sqrt(lat_x * lat_x + lat_y * lat_y + lat_z * lat_z)
                if ln > 1e-9:
                    lift = q * cla * alpha / m
                    a[0] += lift * lat_x / ln
                    a[1] += lift * lat_y / ln
                    a[2] += lift * lat_z / ln
        if t_eng > 0.0 and m > 0.0:
            t_acc = t_eng / m
            a[0] += t_acc * nose[0]
            a[1] += t_acc * nose[1]
            a[2] += t_acc * nose[2]
        return a

    @njit(cache=True)
    def _impact_rk4_jit(r0, v0, m0, mu, omega, center, radius, nose,
                        thr, isp, g0, dry, cd0a, cla, kind, clamp_aoa,
                        alts, densities, sea_r, dt, t_max):
        """Fixed-step RK4 with a FIXED body axis and constant throttle until
        the first surface crossing (alt <= 0), linearly interpolated.
        Returns (hit, x, y, z, t, speed, vrad, vh); hit=1 impact found,
        hit=-1 otherwise."""
        r = np.empty(3, dtype=np.float64)
        v = np.empty(3, dtype=np.float64)
        r[0], r[1], r[2] = r0[0], r0[1], r0[2]
        v[0], v[1], v[2] = v0[0], v0[1], v0[2]
        cx, cy, cz = center[0], center[1], center[2]
        c_mag = sqrt(cx * cx + cy * cy + cz * cz)
        upx, upy, upz = -cx / c_mag, -cy / c_mag, -cz / c_mag
        mdot = thr / (isp * g0) if (isp > 0.0 and thr > 0.0) else 0.0
        m = m0
        dt2 = dt / 2.0
        dt6 = dt / 6.0
        max_n = int(t_max / dt) + 1
        nx, ny, nz = nose[0], nose[1], nose[2]
        t = 0.0
        # previous-step state for the crossing interpolation
        px, py, pz = r0[0], r0[1], r0[2]
        pvx, pvy, pvz = v0[0], v0[1], v0[2]
        pdist = sqrt((r0[0] - cx) ** 2 + (r0[1] - cy) ** 2 + (r0[2] - cz) ** 2)
        palt = pdist - radius
        for _step in range(max_n):
            dx = r[0] - cx
            dy = r[1] - cy
            dz = r[2] - cz
            dist = sqrt(dx * dx + dy * dy + dz * dz)
            alt = dist - radius
            if alt <= 0.0:
                if palt > 0.0:
                    frac = palt / (palt - alt)
                    ex = px + frac * (r[0] - px)
                    ey = py + frac * (r[1] - py)
                    ez = pz + frac * (r[2] - pz)
                    vx = pvx + frac * (v[0] - pvx)
                    vy = pvy + frac * (v[1] - pvy)
                    vz = pvz + frac * (v[2] - pvz)
                    sp = sqrt(vx * vx + vy * vy + vz * vz)
                    vr = vx * upx + vy * upy + vz * upz
                    vh = sqrt((vx - vr * upx) ** 2 + (vy - vr * upy) ** 2
                              + (vz - vr * upz) ** 2)
                    return 1, ex, ey, ez, t + frac * dt, sp, vr, vh
                return -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            if dist <= 0.5 * radius:
                return -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            px, py, pz = r[0], r[1], r[2]
            pvx, pvy, pvz = v[0], v[1], v[2]
            palt = alt
            # stage masses (linear depletion within the step while burning)
            m_a = m
            m_b = max(dry, m - mdot * 0.5 * dt)
            m_d = max(dry, m - mdot * dt)
            t_eng = thr if m > dry else 0.0

            k1 = _acc_jit(r, v, m_a, t_eng, mu, omega, center, radius,
                          (nx, ny, nz), cd0a, cla, kind, clamp_aoa,
                          alts, densities, sea_r)
            r2 = np.empty(3, dtype=np.float64)
            v2 = np.empty(3, dtype=np.float64)
            r2[0] = r[0] + dt2 * v[0]
            r2[1] = r[1] + dt2 * v[1]
            r2[2] = r[2] + dt2 * v[2]
            v2[0] = v[0] + dt2 * k1[0]
            v2[1] = v[1] + dt2 * k1[1]
            v2[2] = v[2] + dt2 * k1[2]
            k2 = _acc_jit(r2, v2, m_b, t_eng, mu, omega, center, radius,
                          (nx, ny, nz), cd0a, cla, kind, clamp_aoa,
                          alts, densities, sea_r)
            r3 = np.empty(3, dtype=np.float64)
            v3 = np.empty(3, dtype=np.float64)
            r3[0] = r[0] + dt2 * v2[0]
            r3[1] = r[1] + dt2 * v2[1]
            r3[2] = r[2] + dt2 * v2[2]
            v3[0] = v[0] + dt2 * k2[0]
            v3[1] = v[1] + dt2 * k2[1]
            v3[2] = v[2] + dt2 * k2[2]
            k3 = _acc_jit(r3, v3, m_b, t_eng, mu, omega, center, radius,
                          (nx, ny, nz), cd0a, cla, kind, clamp_aoa,
                          alts, densities, sea_r)
            r4 = np.empty(3, dtype=np.float64)
            v4 = np.empty(3, dtype=np.float64)
            r4[0] = r[0] + dt * v3[0]
            r4[1] = r[1] + dt * v3[1]
            r4[2] = r[2] + dt * v3[2]
            v4[0] = v[0] + dt * k3[0]
            v4[1] = v[1] + dt * k3[1]
            v4[2] = v[2] + dt * k3[2]
            k4 = _acc_jit(r4, v4, m_d, t_eng, mu, omega, center, radius,
                          (nx, ny, nz), cd0a, cla, kind, clamp_aoa,
                          alts, densities, sea_r)
            r[0] += dt6 * (v[0] + 2.0 * v2[0] + 2.0 * v3[0] + v4[0])
            r[1] += dt6 * (v[1] + 2.0 * v2[1] + 2.0 * v3[1] + v4[1])
            r[2] += dt6 * (v[2] + 2.0 * v2[2] + 2.0 * v3[2] + v4[2])
            v[0] += dt6 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0])
            v[1] += dt6 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])
            v[2] += dt6 * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2])
            if thr > 0.0:
                m = max(dry, m - mdot * dt)
            t += dt
        return -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0


# ---------------------------------------------------------------------------
# Predictor (pure core)
# ---------------------------------------------------------------------------
class BinaryBurnPredictor:
    """Impact predictor with a FIXED body axis, constant throttle and
    attitude-dependent aero.  ``predict`` propagates to the first surface
    crossing (impact) — or returns ``None`` when no impact exists within
    *t_max* (e.g. the burn would hover/climb first)."""

    def __init__(self, body_spec, aero: AeroParams | None = None, *,
                 density_alts=None, density_vals=None, sea_r: float | None = None,
                 dt: float = PREDICT_DT, t_max: float = PREDICT_TMAX,
                 g0: float = G0) -> None:
        self._mu = float(body_spec.mu)
        self._omega = np.asarray(body_spec.omega, dtype=float)
        self._center = np.asarray(body_spec.body_center, dtype=float)
        self._radius = float(body_spec.body_radius)
        self._aero = aero
        self._dt = float(dt)
        self._t_max = float(t_max)
        self._g0 = float(g0)
        c = float(np.linalg.norm(self._center))
        self._up = -self._center / c if c > 1e-9 else np.array([0.0, 0.0, 1.0])
        # density table (for aero in the jit kernel and the throttle laws)
        if aero is not None and density_alts is not None and len(density_alts):
            self._density_alts = np.asarray(density_alts, dtype=float)
            self._density_vals = np.asarray(density_vals, dtype=float)
            self._sea_r = float(sea_r) if sea_r is not None else float(body_spec.body_radius)
            self._jit_ready = njit is not None
        else:
            self._density_alts = np.array([0.0])
            self._density_vals = np.array([0.0])
            self._sea_r = float(body_spec.body_radius)
            self._jit_ready = False

    @property
    def up(self) -> np.ndarray:
        """Radial (up) direction at the target, in the target frame."""
        return self._up

    def rho_at(self, alt: float) -> float:
        """Atmospheric density at altitude *alt* (kg/m^3), 0 above the table."""
        alts = self._density_alts
        vals = self._density_vals
        if len(alts) == 0:
            return 0.0
        if alt <= alts[0]:
            return float(vals[0])
        if alt > alts[-1]:
            return 0.0
        i = int(np.searchsorted(alts, alt)) - 1
        i = min(max(i, 0), len(alts) - 2)
        t = (alt - alts[i]) / (alts[i + 1] - alts[i])
        return float(vals[i] + t * (vals[i + 1] - vals[i]))

    def lateral_force(self, position, velocity, mass, nose, throttle,
                      available_thrust) -> np.ndarray:
        """HORIZONTAL component of the total force (aero drag + lift + thrust)
        that WOULD act at the current state if the body axis were *nose* at
        the given throttle — an ONLINE FORCE SIMULATION (a pure query of the
        aero model at the candidate attitude; the vessel does not move).

        Returns the force projected onto the target-frame horizontal plane
        (Newtons, radial component removed)."""
        r = np.asarray(position, dtype=float)
        v = np.asarray(velocity, dtype=float)
        n = np.asarray(nose, dtype=float)
        nn = float(np.linalg.norm(n))
        if nn < 1e-9:
            n = np.array([0.0, 0.0, 1.0])
        else:
            n = n / nn
        m = float(mass)
        speed = float(np.linalg.norm(v))
        F = np.zeros(3)
        if m > 0.0 and speed > 1e-6 and self._aero is not None:
            alt = float(np.linalg.norm(r - self._center)) - self._sea_r
            rho = self.rho_at(alt)
            if rho > 0.0:
                vh = v / speed
                c = float(np.clip(-float(np.dot(n, vh)), -1.0, 1.0))
                alpha = min(acos(c), self._aero.clamp_aoa)
                q = 0.5 * rho * speed * speed
                # drag along -v (with induced drag), lift toward the nose side
                F -= (q * self._aero.cd0_area
                      * (1.0 + self._aero.k_ind * alpha * alpha)) * vh
                if alpha > 1e-4:
                    lat = n - float(np.dot(n, vh)) * vh
                    ln = float(np.linalg.norm(lat))
                    if ln > 1e-9:
                        F += (q * self._aero.cl_area * alpha) * (lat / ln)
        if throttle > 0.0 and available_thrust > 0.0:
            F += throttle * float(available_thrust) * n
        up = self._up
        return F - up * float(np.dot(F, up))

    def predict(
        self,
        position,
        velocity,
        mass: float,
        *,
        nose,
        throttle: float,
        available_thrust: float,
        isp: float,
        dry_mass: float,
    ) -> ImpactPred | None:
        r = np.asarray(position, dtype=float)
        v = np.asarray(velocity, dtype=float)
        n = np.asarray(nose, dtype=float)
        nn = float(np.linalg.norm(n))
        if nn < 1e-9:
            return None
        n = n / nn
        m = float(mass)
        thr = float(throttle) * float(available_thrust)
        dry = float(dry_mass)
        speed = float(np.linalg.norm(v))
        if speed < 1e-9:
            return None

        if self._aero is not None:
            cd0a = self._aero.cd0_area
            cla = self._aero.cl_area
            kind = self._aero.k_ind
            clamp_aoa = self._aero.clamp_aoa
        else:
            cd0a = cla = kind = clamp_aoa = 0.0

        if self._jit_ready:
            hit, ex, ey, ez, tc, sp, vr, vh = _impact_rk4_jit(
                r, v, m, self._mu, self._omega, self._center, self._radius,
                n, thr, float(isp), self._g0, dry,
                cd0a, cla, kind, clamp_aoa,
                self._density_alts, self._density_vals,
                self._sea_r,
                self._dt, self._t_max,
            )
            if hit == 1:
                return ImpactPred(
                    endpoint=np.array([ex, ey, ez]), time=tc,
                    speed=sp, vrad=vr, vh=vh,
                )
            return None

        # pure-Python fallback (step-end crossing, no interpolation)
        return self._predict_py(r, v, m, n, thr, float(isp), dry, cd0a, cla,
                                kind, clamp_aoa)

    # -- internals ----------------------------------------------------------

    def _predict_py(self, r, v, m, n, thr, isp, dry, cd0a, cla, kind,
                    clamp_aoa) -> ImpactPred | None:
        mdot = thr / (isp * self._g0) if isp > 0.0 and thr > 0.0 else 0.0
        dt = self._dt
        t = 0.0
        while t < self._t_max:
            d = r - self._center
            alt = float(np.linalg.norm(d)) - self._radius
            if alt <= 0.0:
                vrad = float(np.dot(v, self._up))
                return ImpactPred(
                    endpoint=r.copy(), time=t,
                    speed=float(np.linalg.norm(v)), vrad=vrad,
                    vh=float(np.linalg.norm(v - vrad * self._up)),
                )
            t_eng = thr if m > dry else 0.0

            def acc(rr, vv, mm):
                dd = rr - self._center
                dist = float(np.linalg.norm(dd))
                aa = -self._mu * dd / (dist * dist * dist)
                aa -= 2.0 * np.cross(self._omega, vv)
                aa -= np.cross(self._omega, np.cross(self._omega, dd))
                sp = float(np.linalg.norm(vv))
                if sp > 1e-6 and mm > 0.0:
                    vh = vv / sp
                    c = _clamp(-float(np.dot(n, vh)), -1.0, 1.0)
                    alpha = min(acos(c), clamp_aoa)
                    rho = self._rho(alt)
                    q = 0.5 * rho * sp * sp
                    aa -= (q * cd0a * (1.0 + kind * alpha * alpha) / mm) * vh
                    if alpha > 1e-4:
                        lat = n - float(np.dot(n, vh)) * vh
                        ln = float(np.linalg.norm(lat))
                        if ln > 1e-9:
                            aa += (q * cla * alpha / mm) * (lat / ln)
                if t_eng > 0.0:
                    aa += (t_eng / mm) * n
                return aa

            m_a, m_b, m_d = m, max(dry, m - mdot * 0.5 * dt), max(dry, m - mdot * dt)
            k1 = acc(r, v, m_a)
            k2 = acc(r + 0.5 * dt * v, v + 0.5 * dt * k1, m_b)
            k3 = acc(r + 0.5 * dt * (v + 0.5 * dt * k1), v + 0.5 * dt * k2, m_b)
            k4 = acc(r + dt * (v + 0.5 * dt * k2), v + dt * k3, m_d)
            r = r + (dt / 6.0) * (v + 2.0 * (v + 0.5 * dt * k1)
                                  + 2.0 * (v + 0.5 * dt * k2) + (v + dt * k3))
            v = v + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            if thr > 0.0:
                m = max(dry, m - mdot * dt)
            t += dt
        return None

    def _rho(self, alt: float) -> float:
        return self.rho_at(alt)


# ---------------------------------------------------------------------------
# Steering helpers (demo_aeroguide_and_burn.py / zem_aeroguide_landing.py)
# ---------------------------------------------------------------------------
def wind_up(nose, vel):
    """Roll reference: belly faces the airflow (into *vel*)."""
    d = np.asarray(nose, dtype=float)
    v = np.asarray(vel, dtype=float)
    n = float(np.linalg.norm(v))
    if n < 1e-6:
        return (1.0, 0.0, 0.0)
    belly = v / n
    perp = belly - np.dot(belly, d) * d
    n2 = float(np.dot(perp, perp))
    if n2 < 1e-12:
        return (1.0, 0.0, 0.0)
    roof = -perp / np.sqrt(n2)
    return (float(roof[0]), float(roof[1]), float(roof[2]))


def nose_for_lift(velocity, miss_xy, max_aoa_deg, gain_deg_per_m,
                  lift_sign: float = DEMO_SIGN):
    """Commanded nose: rotate the tail-first base ``-v`` by an AoA
    proportional to the predicted miss toward ``lift_sign * miss_hat``.

    lift_sign = +1  -> nose tilts TOWARD the miss (demo convention)
    lift_sign = -1  -> nose tilts toward the target (the negated vector)
    """
    v = np.asarray(velocity, dtype=float)
    speed = float(np.linalg.norm(v))
    if speed < 1e-6:
        return (0.0, 0.0, 1.0)
    v_dir = v / speed
    base = -v_dir
    miss = np.array([miss_xy[0], miss_xy[1], 0.0])
    miss_dist = float(np.linalg.norm(miss))
    if miss_dist < 1e-3:
        return tuple(base)
    steer = lift_sign * miss / miss_dist
    aoa_deg = min(miss_dist * gain_deg_per_m, max_aoa_deg)
    aoa = np.radians(aoa_deg)
    axis = np.cross(base, steer)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-12:
        return tuple(base)
    axis = axis / axis_norm
    nose = base * np.cos(aoa) + np.cross(axis, base) * np.sin(aoa)
    return (float(nose[0]), float(nose[1]), float(nose[2]))


def nose_at_aoa(velocity, steer_xy, aoa_deg):
    """Rotate the tail-first base ``-v`` by an EXPLICIT AoA toward the
    (horizontal) steer direction — same rotation as :func:`nose_for_lift`
    but with the angle given directly (used to scale the burn/terminal
    steering by the winning candidate's own predicted miss)."""
    v = np.asarray(velocity, dtype=float)
    speed = float(np.linalg.norm(v))
    if speed < 1e-6:
        return (0.0, 0.0, 1.0)
    v_dir = v / speed
    base = -v_dir
    steer = np.array([steer_xy[0], steer_xy[1], 0.0])
    s = float(np.linalg.norm(steer))
    if s < 1e-3:
        return tuple(base)
    steer = steer / s
    aoa = np.radians(aoa_deg)
    axis = np.cross(base, steer)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-12:
        return tuple(base)
    axis = axis / axis_norm
    nose = base * np.cos(aoa) + np.cross(axis, base) * np.sin(aoa)
    return (float(nose[0]), float(nose[1]), float(nose[2]))


def aoa_of(nose, velocity) -> float:
    """Angle between the nose and the tail-first base ``-v`` (deg)."""
    v = np.asarray(velocity, dtype=float)
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return 0.0
    base = -v / n
    d = np.asarray(nose, dtype=float)
    c = float(np.clip(np.dot(d, base) / (np.linalg.norm(d) * np.linalg.norm(base) + 1e-12),
                      -1.0, 1.0))
    return float(np.degrees(np.arccos(c)))


def miss_of(pred, up):
    """Horizontal offset of a prediction from the target (m), or ``inf``."""
    if pred is None:
        return float("inf"), (0.0, 0.0)
    rh = pred.endpoint - up * float(np.dot(pred.endpoint, up))
    return float(np.linalg.norm(rh)), (float(rh[0]), float(rh[1]))


def direction_score(f_h, g_hat) -> float:
    """Signed projection of the simulated horizontal force onto the target
    direction (Newtons): positive = pushes the landing point toward the
    target, negative = pushes it away, magnitude = how hard.  Mass is common
    to both candidates, so the comparison is unaffected by omitting it."""
    return float(np.dot(f_h, g_hat))


def retrograde(velocity):
    v = np.asarray(velocity, dtype=float)
    sp = float(np.linalg.norm(v))
    if sp < 1e-9:
        return (0.0, 0.0, 1.0)
    return tuple(-v / sp)


# ---------------------------------------------------------------------------
# Throttle laws (old landing.py)
# ---------------------------------------------------------------------------
def gfold_start_altitude(d: float) -> float:
    return max(CONE_RATIO * d, CONE_FLOOR)


def burn_throttle(mass: float, thrust: float, alt: float, vrad: float,
                  d: float, g: float):
    """Old energy law: brake to GFOLD_START_VELOCITY at gfold_start_altitude.

    Returns ``(throttle, h_gfold)``.  Above h_gfold the required deceleration
    is (v^2 - 140^2) / 2(h - h_gfold); below it the descent law takes over
    (the powered descent is a single phase).
    """
    h_gfold = gfold_start_altitude(d)
    if alt >= h_gfold:
        acc = (vrad * vrad - GFOLD_START_VELOCITY ** 2) / (2.0 * max(alt - h_gfold, 10.0))
        thr = mass * acc / max(thrust, 1e-6)
    else:
        thr = THROTTLE_LIMIT[1]
    return _clamp(thr, *THROTTLE_LIMIT), h_gfold


def terminal_throttle(mass: float, thrust: float, alt: float, vrad: float,
                      g: float, a_drag_up: float, h_term: float,
                      vt: float = VT_TOUCHDOWN):
    """Old descent_throttle law: target *vt* at *h_term*; hover-hold once
    vrad >= vt (|vrad| <= 2 m/s).  ``a_drag_up`` = vertical drag deceleration
    (drag assists the brake, so it reduces the required throttle)."""
    if vrad < vt:
        acc = (vt * vt + vrad * vrad) / (2.0 * max(alt - h_term, 5.0)) + g - a_drag_up
        thr = mass * acc / max(thrust, 1e-6)
    else:
        thr = mass * g / max(thrust, 1e-6)
    return _clamp(thr, *THROTTLE_LIMIT)


# ---------------------------------------------------------------------------
# Guidance state machine (pure core)
# ---------------------------------------------------------------------------
class BinaryGuidance:
    """Stateful guidance: fixed-sign aero phase + binary burn/terminal."""

    def __init__(self, *, max_aoa_deg: float = MAX_AOA,
                 aoa_gain: float = AOA_GAIN,
                 ignite_miss: float = IGNITE_MISS,
                 miss_hysteresis: float = MISS_HYSTERESIS,
                 min_ignite_alt: float = MIN_IGNITE_ALT,
                 progress_fraction: float = PROGRESS_FRACTION,
                 hold_ticks: int = BINARY_HOLD_TICKS) -> None:
        self.max_aoa_deg = max_aoa_deg
        self.aoa_gain = aoa_gain
        self.ignite_miss = ignite_miss
        self.miss_hysteresis = miss_hysteresis
        self.min_ignite_alt = min_ignite_alt
        self.progress_fraction = progress_fraction
        self.hold_ticks = hold_ticks
        # aero-phase tracking
        self.d_min = float("inf")
        self.d_init: float | None = None
        self.progress = False
        # binary state: +1 = tilt toward miss, -1 = tilt toward target,
        # 0 = retrograde (no tilt)
        self.sign = DEMO_SIGN
        self.hold = 0
        self.switches = 0

    # -- Phase A ------------------------------------------------------------
    def aero_step(self, velocity, miss_xy, d: float, altitude: float):
        """Fixed-sign steering (demo: nose toward the miss)."""
        nose = nose_for_lift(velocity, miss_xy, self.max_aoa_deg,
                             self.aoa_gain, lift_sign=DEMO_SIGN)
        up = wind_up(nose, velocity)
        if self.d_init is None:
            self.d_init = d
        if d < self.d_min:
            self.d_min = d
        if self.d_init > 0.0 and self.d_min < self.progress_fraction * self.d_init:
            self.progress = True
        return nose, up

    def candidates(self, velocity, miss_xy, d: float, phase: str):
        """The two candidate attitudes for the burn/terminal phases — the
        SAME steering law as the air phase (same gain / max AoA), the only
        difference being the sign of the steering vector: tilt toward the
        miss (+1) or toward the target (-1).  Retrograde (0) is the
        fallback when neither candidate's simulated force points toward
        the target."""
        nose_a = nose_for_lift(velocity, miss_xy, self.max_aoa_deg,
                               self.aoa_gain, lift_sign=+1.0)
        nose_b = nose_for_lift(velocity, (-miss_xy[0], -miss_xy[1]),
                               self.max_aoa_deg, self.aoa_gain, lift_sign=+1.0)
        nose_0 = retrograde(velocity)
        return nose_a, nose_b, nose_0

    def energy_ignition(self, altitude: float, vrad: float, mass: float,
                        g: float, thrust: float, d: float):
        """Old landing.py energy-based ignition height."""
        h_gfold = gfold_start_altitude(d)
        de = (0.5 * mass * (GFOLD_START_VELOCITY ** 2 - vrad * vrad)
              + mass * g * (h_gfold - altitude))
        ign_h = abs(de / max(thrust - mass * g, 1e-6))
        ign_alt = max(ign_h, IGNITION_FLOOR)
        if altitude <= ign_alt:
            return True, f"energy({ign_alt:.0f}m)"
        return False, ""

    def fallback_ignition(self, d: float, altitude: float):
        """Prediction-based fallback triggers (zem_aeroguide_landing.py)."""
        if d < self.ignite_miss:
            return True, "IGNITE_MISS"
        if self.progress and d > self.d_min + self.miss_hysteresis:
            return True, "miss stopped decreasing"
        if altitude < self.min_ignite_alt:
            return True, "altitude floor"
        return False, ""

    # -- binary decision ----------------------------------------------------
    def decide_direction(self, score_a: float, score_b: float,
                         threshold: float = 0.0) -> float:
        """Direction-consistency decision (the user's scheme): simulate the
        horizontal force (thrust lateral component + aero-force lateral
        component) that would act if EACH candidate attitude were executed,
        and execute the candidate whose force pushes hardest toward the
        target (score = signed projection of the force onto the target
        direction; >= threshold = "consistent").  If neither candidate's
        force pushes toward the target, go retrograde (0).
        Anti-chatter: a candidate must be favoured for hold_ticks consecutive
        ticks before switching; switching INTO retrograde needs twice as long
        (retrograde is the no-action fallback, so it must win persistently)."""
        desired = 0.0
        if max(score_a, score_b) >= threshold:
            desired = +1.0 if score_a >= score_b else -1.0
        if desired == self.sign:
            self.hold = 0
            return self.sign
        need = self.hold_ticks if desired != 0.0 else self.hold_ticks * 2
        self.hold += 1
        if self.hold >= need:
            self.sign = desired
            self.hold = 0
            self.switches += 1
        return self.sign


# ---------------------------------------------------------------------------
# Flight entry point
# ---------------------------------------------------------------------------
def _half_rocket_length(b) -> float:
    """Weighted mean distance of parts below the root part (old landing.py)."""
    try:
        rf = b.raw.reference_frame
        ds = [float(np.linalg.norm(p.position(rf)))
              for p in b.raw.parts.all if p.position(rf)[1] < 0.0]
        return float(np.mean(ds)) if ds else 10.0
    except Exception:  # noqa: BLE001 — cosmetic, never fatal
        return 10.0


def main() -> None:
    log_path = pathlib.Path(LOG_FILE).resolve()
    csv_path = pathlib.Path(CSV_FILE).resolve()
    print(f"== BINARY-SIGN LANDING — VESSEL={VESSEL} ==", flush=True)
    print(f"  log: {log_path}", flush=True)
    print(f"  csv: {csv_path}", flush=True)
    print(f"  numba jit: {'READY' if njit is not None else 'MISSING (slow fallback)'}",
          flush=True)

    with ConnectionManager(address="127.0.0.1") as km:
        b = km.add_booster("bin", VESSEL)
        km.register_target("bin", lon=TARGET.lon, lat=TARGET.lat)
        km.enable_debug()
        km.start()

        deadline = time.monotonic() + 5.0
        while km.snapshot("bin") is None:
            if time.monotonic() > deadline:
                raise RuntimeError("telemetry not ready")
            time.sleep(0.02)

        frame = km.frame("bin", "target")
        body_spec, drag_spec = b.sample_predictor_specs()
        # AeroParams: C_D0*A implied by the sampled ballistic coefficient at
        # the sampling mass; lift slope / induced drag from the flight fit.
        m_sample = float(b.snapshot().mass) if b.snapshot() is not None else 20_000.0
        aero = AeroParams(
            cd0_area=m_sample / max(float(drag_spec.ballistic_coefficient), 1.0),
            cl_area=4.8,
            k_ind=16.8,
            clamp_aoa=np.radians(25.0),
        )
        predictor = BinaryBurnPredictor(
            body_spec, aero=aero,
            density_alts=drag_spec.density_alts,
            density_vals=drag_spec.density_vals,
            sea_r=drag_spec.sea_level_radius,
        )

        b.physics_range = 250000.0
        b.controls.target_smoothing_time = SMOOTHING_TIME
        b.controls.rcs = True
        b.controls.apply(throttle=0.0)

        guidance = BinaryGuidance()
        half_len = _half_rocket_length(b)

        log = open(LOG_FILE, "w", encoding="utf-8")
        csvf = open(CSV_FILE, "w", newline="", encoding="utf-8")
        csvw = csv.writer(csvf)
        csvw.writerow([
            "met", "phase", "alt", "speed", "vrad", "d_base", "d_win",
            "sign", "aoa", "throttle", "epx", "epy", "epz", "note",
        ])

        def logline(msg: str) -> None:
            print(msg, flush=True)
            print(msg, file=log, flush=True)

        phase = "aero"
        g = float(body_spec.surface_gravity)
        up = predictor.up
        pacer = FramePacer(hz=PREDICT_HZ)
        t_start = time.monotonic()
        nose_cmd = (0.0, 0.0, 1.0)
        thr_cmd = 0.0
        last_draw = 0.0
        ceiling_logged = False
        endpoint_line = None

        logline(f"== BINARY-SIGN LANDING (VESSEL={VESSEL}, phase=A) ==")
        logline(f"  aero: cd0_area={aero.cd0_area:.2f} m^2  cl_area={aero.cl_area:.2f} "
                f"m^2/rad  k_ind={aero.k_ind}")

        while True:
            pacer.tick()
            s = b.snapshot()
            if s is None:
                continue
            if s.landed:
                logline("landed during guidance")
                break

            pos = np.array([s.position.x, s.position.y, s.position.z])
            vel = np.array([s.velocity.x, s.velocity.y, s.velocity.z])
            alt = float(s.surface_altitude)
            speed = float(np.linalg.norm(vel))
            vrad = float(np.dot(vel, up))
            met = float(s.met)
            now = time.monotonic() - t_start

            if phase == "aero":
                # tail-first (AoA=0) impact prediction — identical to the
                # demo's LandingPredictor behaviour
                pred = predictor.predict(
                    pos, vel, s.mass, nose=retrograde(vel), throttle=0.0,
                    available_thrust=s.available_thrust,
                    isp=s.specific_impulse, dry_mass=s.dry_mass,
                )
                d_base, miss_xy = miss_of(pred, up)
                d_win = d_base
                nose, upv = guidance.aero_step(vel, miss_xy, d_base, alt)
                thr_cmd = 0.0

                ign, reason = guidance.energy_ignition(
                    alt, vrad, s.mass, g, s.available_thrust, d_base)
                if not ign:
                    ign, reason = guidance.fallback_ignition(d_base, alt)
                if ign and alt > BURN_CEILING_ALT:
                    # hard ceiling: the burn phase is not allowed up here —
                    # keep gliding (air guidance) until we drop below 20 km
                    ign = False
                    if not ceiling_logged:
                        ceiling_logged = True
                        logline(f"!! ignition triggers above BURN_CEILING_ALT "
                                f"({BURN_CEILING_ALT:.0f} m) suppressed @ "
                                f"alt={alt:8.1f} reason={reason}")
                if ign:
                    phase = "burn"
                    b.controls.rcs = False
                    guidance.sign = DEMO_SIGN
                    guidance.hold = 0
                    logline(f"== IGNITE @ t={now:6.1f} met={met:7.1f} reason={reason} "
                            f"alt={alt:8.1f} v={speed:6.1f} d={d_base:8.1f}")
            else:
                # ---- binary phases (burn / terminal) ----
                # ZEM baseline: BALLISTIC (no-thrust) impact prediction —
                # the classic zero-effort miss at the current state.  A
                # ballistic trajectory always reaches the ground at these
                # speeds/altitudes, so d is never inf here.  Thrust enters
                # only the direction-decision force simulation below.
                pred0 = predictor.predict(
                    pos, vel, s.mass, nose=retrograde(vel), throttle=0.0,
                    available_thrust=s.available_thrust,
                    isp=s.specific_impulse, dry_mass=s.dry_mass,
                )
                d_base, miss_xy = miss_of(pred0, up)
                thr_now = float(s.throttle)

                if d_base == float("inf") or d_base < 1e-3:
                    # no ballistic impact (should not happen down here) or
                    # already on target: no target direction — retrograde
                    nose = retrograde(vel)
                    d_win = d_base
                else:
                    # target direction: from the predicted ballistic impact
                    # point toward the origin, in the horizontal plane
                    g_hat = np.array([-miss_xy[0], -miss_xy[1], 0.0])
                    g_hat /= float(np.linalg.norm(g_hat))
                    nose_a, nose_b, nose_0 = guidance.candidates(
                        vel, miss_xy, d_base, phase)
                    # ONLINE force simulation at each candidate attitude —
                    # a pure query of the aero/thrust model; the vessel
                    # does NOT move.  The CURRENT throttle is used (the
                    # force that would act if the command were executed now).
                    # Execute the candidate whose simulated horizontal force
                    # (thrust lateral component + aero drag/lift lateral
                    # component) pushes the ZEM impact point hardest toward
                    # the target; retrograde when neither does.
                    f_a = predictor.lateral_force(
                        pos, vel, s.mass, nose_a, thr_now, s.available_thrust)
                    f_b = predictor.lateral_force(
                        pos, vel, s.mass, nose_b, thr_now, s.available_thrust)
                    score_a = direction_score(f_a, g_hat)
                    score_b = direction_score(f_b, g_hat)
                    # a candidate only counts as consistent if its push
                    # exceeds a small fraction of the vessel's weight
                    # (filters the low-speed sign oscillation)
                    thr_score = SCORE_THRESHOLD_FRAC * s.mass * g
                    sign = guidance.decide_direction(score_a, score_b,
                                                     threshold=thr_score)
                    nose = {+1.0: nose_a, -1.0: nose_b, 0.0: nose_0}[sign]
                    d_win = d_base
                upv = wind_up(nose, vel)

                # throttle: ONE powered-descent phase — energy law above the
                # cone (brake to 140 m/s at h_gfold), descent law below it
                h_gfold = gfold_start_altitude(d_base)
                if alt >= h_gfold:
                    thr_cmd, _ = burn_throttle(
                        s.mass, s.available_thrust, alt, vrad, d_base, g)
                else:
                    a_drag_up = _a_drag_vertical(predictor, pos, vel, s.mass, aero)
                    thr_cmd = terminal_throttle(
                        s.mass, s.available_thrust, alt, vrad, g,
                        a_drag_up, half_len)

                # cutoff: landed, or hovering near the ground (powered phase)
                if phase == "burn" and (s.landed or (vrad >= 0.0 and alt < CUTOFF_ALT)):
                    logline(f"== CUTOFF @ t={now:6.1f} met={met:7.1f} "
                            f"vrad={vrad:6.1f} alt={alt:8.1f} "
                            f"d={d_win:8.1f} sign_switches={guidance.switches}")
                    b.controls.apply(throttle=0.0)
                    b.controls.rcs = True
                    # hold attitude until the ship settles
                    settle_start = time.monotonic()
                    while True:
                        pacer.tick()
                        s2 = b.snapshot()
                        if s2 is None:
                            continue
                        if s2.landed:
                            break
                        v2 = np.array([s2.velocity.x, s2.velocity.y, s2.velocity.z])
                        if float(np.linalg.norm(v2)) <= CUTOFF_SPEED:
                            break
                        if time.monotonic() - settle_start > 30.0:
                            break
                        b.controls.apply(target_direction=retrograde(v2),
                                         reference_frame=frame, throttle=0.0)
                    break

            b.controls.apply(target_direction=nose, reference_frame=frame,
                             up=upv, throttle=thr_cmd)

            if now - last_draw >= 0.5 and phase != "aero":
                last_draw = now
                ep = None
                try:
                    if d_win != float("inf"):
                        ep = _ep_of(predictor, pos, vel, s.mass, nose,
                                    s.available_thrust, s.specific_impulse,
                                    s.dry_mass)
                except Exception:  # noqa: BLE001
                    ep = None
                if ep is not None:
                    if endpoint_line is None:
                        endpoint_line = b.debug.line(
                            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                            frame_name="target", color=(0.3, 1.0, 0.6),
                            thickness=0.5)
                    try:
                        endpoint_line.set_points(tuple(pos), tuple(ep))
                    except Exception as exc:  # noqa: BLE001
                        logline(f"draw error: {exc}")

            aoa = aoa_of(nose, vel)
            csvw.writerow([
                f"{met:.2f}", phase, f"{alt:.1f}", f"{speed:.1f}", f"{vrad:.1f}",
                f"{d_base:.1f}", f"{d_win:.1f}", f"{guidance.sign:+.0f}",
                f"{aoa:.1f}", f"{thr_cmd:.2f}", "", "", "", "",
            ])
            csvf.flush()
            logline(
                f"[{phase[0].upper()}] t={now:6.1f} alt={alt:8.1f} v={speed:6.1f} "
                f"vrad={vrad:6.1f} d={d_win:8.1f} sign={guidance.sign:+.0f} "
                f"aoa={aoa:5.1f} thr={thr_cmd:.2f}"
            )

        b.controls.cut_thrust()
        logline("== END ==")
        logline(f"  final: alt={alt:8.1f} v={speed:6.1f} vrad={vrad:6.1f} "
                f"d={d_win:8.1f} sign_switches={guidance.switches}")
        csvf.close()
        log.close()


def _ep_of(predictor, pos, vel, mass, nose, avail, isp, dry):
    """Ballistic (ZEM) impact point of the given attitude — for drawing.
    Consistent with the guidance: the whole script predicts the landing
    point ballistically; thrust enters only the direction-decision force
    simulation (lateral_force)."""
    pr = predictor.predict(pos, vel, mass, nose=nose, throttle=0.0,
                           available_thrust=avail, isp=isp, dry_mass=dry)
    if pr is None:
        return None
    return tuple(pr.endpoint)


def _a_drag_vertical(predictor, pos, vel, mass, aero: AeroParams) -> float:
    """Vertical drag deceleration at the current state (m/s^2, positive up)."""
    d = np.asarray(pos) - predictor._center
    alt = float(np.linalg.norm(d)) - predictor._radius
    sp = float(np.linalg.norm(vel))
    if sp < 1e-6 or mass <= 0.0:
        return 0.0
    rho = _table_rho(predictor, alt)
    if rho <= 0.0:
        return 0.0
    q = 0.5 * rho * sp * sp
    a_drag = q * aero.cd0_area / mass
    vrad = float(np.dot(vel, predictor.up))
    return a_drag * (-vrad / sp) if vrad < 0.0 else 0.0


def _table_rho(predictor, alt: float) -> float:
    alts = getattr(predictor, "_density_alts", None)
    vals = getattr(predictor, "_density_vals", None)
    if alts is None or vals is None or len(alts) == 0:
        return 0.0
    if alt <= alts[0]:
        return float(vals[0])
    if alt > alts[-1]:
        return 0.0
    i = int(np.searchsorted(alts, alt)) - 1
    i = min(max(i, 0), len(alts) - 2)
    t = (alt - alts[i]) / (alts[i + 1] - alts[i])
    return float(vals[i] + t * (vals[i + 1] - vals[i]))


if __name__ == "__main__":
    main()
