"""ZEM-predictive air guidance + landing burn (successor to landing.py).

Concept (as specified):
    Every tick the rocket asks: "If I ignite RIGHT NOW at TARGET_THROTTLE,
    thrust always anti-parallel to the target-frame velocity, then coast —
    at the moment the trajectory first crosses TARGET_ALTITUDE, where am I
    and what is my horizontal speed?"  That point is the ZEM handoff state.
    The guidance drives its horizontal position onto the target and its
    horizontal speed toward zero; vertical speed is free (the next segment
    owns the vertical channel — final touchdown is out of scope).

    Phase A (unpowered air guidance): the rocket banks its body (AoA) so
    aerodynamic lift bends the trajectory and the predicted crossing point
    slides toward the landing target.  The predicted miss distance d is
    assumed to decrease monotonically; the rocket ignites the moment d stops
    decreasing (or is already within IGNITE_MISS, or the altitude floor is
    reached).

    Phase B (landing burn): follow the prediction model exactly — nose on -v,
    engine at TARGET_THROTTLE — until speed < TERMINAL_SPEED, then exit
    guidance.

    Final touchdown is deliberately OUT OF SCOPE (next iteration).

Framework integration:
    ConnectionManager / FlightState / BodySpec / DragSpec / DragModel from
    the recovery package.  The ZEM burn+coast is integrated here in pure
    numpy (numba-accelerated RK4, pure-Python fallback) because the
    framework's LandingPredictor has no thrust term; the aerodynamics reuse
    DragModel (tail-first assumption, which matches the burn attitude).

    Attitude/steering law inherited from scripts/demo_aeroguide.py; the one
    new knob is LIFT_SIGN — see its comment below.

Local closed-loop validation: D:/hermes_ws/validate_zem_aeroguide.py
(parallel, runs the exact same pure core without KSP).

Usage (repo root, KRPC env):
    python scripts/zem_aeroguide_landing.py
"""

from __future__ import annotations

import csv
import time
from dataclasses import dataclass

import numpy as np

try:  # optional acceleration — pure-Python fallback below
    from numba import njit
except ImportError:  # pragma: no cover
    njit = None

from recovery import ConnectionManager, FramePacer
from recovery.data.targets import LAUNCHPAD_JNSQ
from recovery.guidance import DragModel

# ---------------------------------------------------------------------------
# Guidance settings
# ---------------------------------------------------------------------------
VESSEL = "Booster 2"
TARGET = LAUNCHPAD_JNSQ

TARGET_THROTTLE = 1.0        # landing-burn throttle (the prediction uses it too)
TERMINAL_SPEED = 50.0        # m/s — burn exits guidance below this speed
TARGET_ALTITUDE = 200.0      # m — handoff altitude: the predicted ZEM state is
                             #   the point where the burn+coast trajectory first
                             #   crosses this altitude (must be above terrain)

# -- air guidance (phase A) --------------------------------------------------
MAX_AOA = 20.0               # degrees — max commanded angle of attack
AOA_GAIN = 0.02              # deg per metre of predicted miss
# LIFT_SIGN: the steering law tilts the nose by LIFT_SIGN * miss_hat and
# relies on the lift pushing the predicted endpoint toward the target.
#   -1: nose tilts AWAY from the miss (toward the target) — the standard
#       aerodynamic convention (lift acts toward the nose-deflection side;
#       the user's flight-fit CL*A = 4.8/rad model uses the same sign).
#   +1: nose tilts TOWARD the miss (the demo_aeroguide comment's claim,
#       never flight-validated).
# If the predicted miss INCREASES from the start of phase A, flip this.
LIFT_SIGN = -1.0

IGNITE_MISS = 300.0          # m — ignite if the predicted endpoint is this close
MISS_HYSTERESIS = 150.0      # m — ignite when d rises this far above d_min
MIN_IGNITE_ALT = 4000.0      # m — safety floor: ignite no matter what below this
PROGRESS_FRACTION = 0.7      # d must drop below this × d_init before the
                             #   "stopped decreasing" trigger may fire (guards
                             #   against a wrong LIFT_SIGN igniting instantly)

PREDICT_HZ = 10.0            # ZEM prediction / steering rate
CONTROL_HZ = 20.0            # burn loop rate
PREDICT_DT = 0.1             # s — ZEM integration step
PREDICT_TMAX = 300.0         # s — max predicted burn+coast duration

LOG_FILE = "zem_aeroguide_debug.log"
CSV_FILE = "zem_aeroguide_telemetry.csv"

# ---------------------------------------------------------------------------
# Pure core (no kRPC — importable for local closed-loop testing)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class BurnPrediction:
    """Predicted ZEM state: the point where the burn+coast trajectory first
    crosses TARGET_ALTITUDE.  ``endpoint`` is its position (target frame),
    ``horizontal_speed`` its horizontal speed (the guidance drives this to
    zero over the target)."""

    endpoint: np.ndarray
    cutoff_time: float
    speed_at_cutoff: float
    vertical_at_cutoff: float
    horizontal_speed: float = 0.0


def _jit_available() -> bool:
    return njit is not None


if njit is not None:

    @njit(cache=True)
    def _burn_coast_rk4_jit(
        r0, v0, m0, mu, omega, center, radius,
        beta, alts, densities, sea_r,
        thr, isp, g0, dry, dt, t_max, min_speed, target_alt,
    ):
        """Fixed-step RK4 of the landing burn + unpowered coast, in the
        target frame.  Thrust (along -v) is ON while the rocket is still
        descending (vrad < 0) and faster than *min_speed* and has fuel;
        afterwards it coasts ballistically.  Stops at the FIRST crossing of
        *target_alt* (altitude measured against the surface sphere), with
        linear interpolation inside the crossing step.  Returns
        ``(hit, x, y, z, t, speed, vrad, v_h)``: hit=1 on the altitude
        crossing, hit=-1 on ground contact / timeout (no crossing)."""
        r = np.empty(3, dtype=np.float64)
        v = np.empty(3, dtype=np.float64)
        r[0] = r0[0]
        r[1] = r0[1]
        r[2] = r0[2]
        v[0] = v0[0]
        v[1] = v0[1]
        v[2] = v0[2]
        cx = center[0]
        cy = center[1]
        cz = center[2]
        c_mag = (cx * cx + cy * cy + cz * cz) ** 0.5
        upx = -cx / c_mag
        upy = -cy / c_mag
        upz = -cz / c_mag
        mdot = thr / (isp * g0)
        dt2 = dt / 2.0
        dt6 = dt / 6.0
        max_n = int(t_max / dt) + 1
        m = m0
        t = 0.0
        coasting = False
        # previous-step state (for crossing interpolation)
        px, py, pz = r0[0], r0[1], r0[2]
        pvx, pvy, pvz = v0[0], v0[1], v0[2]
        pdist = ((r0[0] - cx) ** 2 + (r0[1] - cy) ** 2 + (r0[2] - cz) ** 2) ** 0.5
        palt = pdist - radius
        for _step in range(max_n):
            dx = r[0] - cx
            dy = r[1] - cy
            dz = r[2] - cz
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5
            alt = dist - radius
            if alt <= target_alt:
                if palt > target_alt:
                    frac = (palt - target_alt) / (palt - alt)
                    rx = px + frac * (r[0] - px)
                    ry = py + frac * (r[1] - py)
                    rz = pz + frac * (r[2] - pz)
                    vx = pvx + frac * (v[0] - pvx)
                    vy = pvy + frac * (v[1] - pvy)
                    vz = pvz + frac * (v[2] - pvz)
                    speed = (vx * vx + vy * vy + vz * vz) ** 0.5
                    vrad = vx * upx + vy * upy + vz * upz
                    vh = ((vx - vrad * upx) ** 2 + (vy - vrad * upy) ** 2
                          + (vz - vrad * upz) ** 2) ** 0.5
                    return 1, rx, ry, rz, t + frac * dt, speed, vrad, vh
                return -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            if dist <= radius:
                return -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            px, py, pz = r[0], r[1], r[2]
            pvx, pvy, pvz = v[0], v[1], v[2]
            palt = alt
            if not coasting:
                vrad = v[0] * upx + v[1] * upy + v[2] * upz
                speed = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5
                burning = vrad < 0.0 and (min_speed <= 0.0 or speed > min_speed) and m > dry
                if not burning:
                    coasting = True  # engine cut off once; never re-ignites
            else:
                burning = False

            # stage masses (linear depletion within the step while burning)
            m_a = m
            m_b = max(dry, m - mdot * 0.5 * dt) if burning else m
            m_d = max(dry, m - mdot * dt) if burning else m

            k1v = _accel_burn_jit(r, v, m_a, mu, omega, center, radius,
                                  beta, alts, densities, sea_r, thr if burning else 0.0)
            r2 = np.empty(3, dtype=np.float64)
            v2 = np.empty(3, dtype=np.float64)
            r2[0] = r[0] + dt2 * v[0]
            r2[1] = r[1] + dt2 * v[1]
            r2[2] = r[2] + dt2 * v[2]
            v2[0] = v[0] + dt2 * k1v[0]
            v2[1] = v[1] + dt2 * k1v[1]
            v2[2] = v[2] + dt2 * k1v[2]
            k2v = _accel_burn_jit(r2, v2, m_b, mu, omega, center, radius,
                                  beta, alts, densities, sea_r, thr if burning else 0.0)
            r3 = np.empty(3, dtype=np.float64)
            v3 = np.empty(3, dtype=np.float64)
            r3[0] = r[0] + dt2 * v2[0]
            r3[1] = r[1] + dt2 * v2[1]
            r3[2] = r[2] + dt2 * v2[2]
            v3[0] = v[0] + dt2 * k2v[0]
            v3[1] = v[1] + dt2 * k2v[1]
            v3[2] = v[2] + dt2 * k2v[2]
            k3v = _accel_burn_jit(r3, v3, m_b, mu, omega, center, radius,
                                  beta, alts, densities, sea_r, thr if burning else 0.0)
            r4 = np.empty(3, dtype=np.float64)
            v4 = np.empty(3, dtype=np.float64)
            r4[0] = r[0] + dt * v3[0]
            r4[1] = r[1] + dt * v3[1]
            r4[2] = r[2] + dt * v3[2]
            v4[0] = v[0] + dt * k3v[0]
            v4[1] = v[1] + dt * k3v[1]
            v4[2] = v[2] + dt * k3v[2]
            k4v = _accel_burn_jit(r4, v4, m_d, mu, omega, center, radius,
                                  beta, alts, densities, sea_r, thr if burning else 0.0)
            r[0] += dt6 * (v[0] + 2.0 * v2[0] + 2.0 * v3[0] + v4[0])
            r[1] += dt6 * (v[1] + 2.0 * v2[1] + 2.0 * v3[1] + v4[1])
            r[2] += dt6 * (v[2] + 2.0 * v2[2] + 2.0 * v3[2] + v4[2])
            v[0] += dt6 * (k1v[0] + 2.0 * k2v[0] + 2.0 * k3v[0] + k4v[0])
            v[1] += dt6 * (k1v[1] + 2.0 * k2v[1] + 2.0 * k3v[1] + k4v[1])
            v[2] += dt6 * (k1v[2] + 2.0 * k2v[2] + 2.0 * k3v[2] + k4v[2])
            if burning:
                m = max(dry, m - mdot * dt)
            t += dt
        return -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    @njit(cache=True)
    def _accel_burn_jit(r, v, m, mu, omega, center, radius,
                        beta, alts, densities, sea_r, thr):
        """Gravity + Coriolis + centrifugal + drag + thrust (along -v)."""
        a = np.empty(3, dtype=np.float64)
        dx = r[0] - center[0]
        dy = r[1] - center[1]
        dz = r[2] - center[2]
        dist_sq = dx * dx + dy * dy + dz * dz
        dist = dist_sq**0.5
        inv_cube = 1.0 / (dist * dist * dist)
        g = -mu * inv_cube
        a[0] = g * dx
        a[1] = g * dy
        a[2] = g * dz
        a[0] += -2.0 * (omega[1] * v[2] - omega[2] * v[1])
        a[1] += -2.0 * (omega[2] * v[0] - omega[0] * v[2])
        a[2] += -2.0 * (omega[0] * v[1] - omega[1] * v[0])
        ox_dx = omega[1] * dz - omega[2] * dy
        ox_dy = omega[2] * dx - omega[0] * dz
        ox_dz = omega[0] * dy - omega[1] * dx
        a[0] += -(omega[1] * ox_dz - omega[2] * ox_dy)
        a[1] += -(omega[2] * ox_dx - omega[0] * ox_dz)
        a[2] += -(omega[0] * ox_dy - omega[1] * ox_dx)
        if beta > 0.0 and beta != float("inf"):
            alt = dist - sea_r
            rho = _interp_jit(alt, alts, densities)
            if rho > 0.0:
                v_mag = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5
                if v_mag > 1e-6:
                    drag = -0.5 * rho * v_mag / beta
                    a[0] += drag * v[0]
                    a[1] += drag * v[1]
                    a[2] += drag * v[2]
        if m > 0.0:
            v_mag = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5
            if v_mag > 1e-9:
                t_acc = thr / m
                a[0] += t_acc * (-v[0] / v_mag)
                a[1] += t_acc * (-v[1] / v_mag)
                a[2] += t_acc * (-v[2] / v_mag)
        return a

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


class ZemBurnPredictor:
    """Predict the ZEM handoff state: "If I ignite RIGHT NOW, thrust along -v
    while descending and faster than TERMINAL_SPEED, then coast — at the
    moment the trajectory first crosses TARGET_ALTITUDE, where am I and how
    fast am I moving horizontally?"

    Dynamics: inverse-square gravity + Coriolis + centrifugal (BodySpec),
    aero drag (AeroModel, tail-first), thrust = throttle * available_thrust
    always anti-parallel to the target-frame velocity.  The altitude
    crossing is linearly interpolated inside the integration step.  The
    guidance layer drives the crossing point's HORIZONTAL position onto the
    target and its horizontal speed toward zero; vertical speed is free (the
    next guidance segment owns the vertical channel).
    """

    def __init__(
        self,
        body_spec,
        aero=None,
        *,
        dt: float = PREDICT_DT,
        t_max: float = PREDICT_TMAX,
        g0: float = 9.80665,
    ) -> None:
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
        # numba fast path (mirrors the framework's DragModel.rk4 path): only
        # when numba is importable AND the aero model exposes a density table.
        self._jit_params = None
        if njit is not None and aero is not None:
            params = getattr(aero, "numba_params", None)
            if params is not None:
                self._jit_params = params

    @property
    def up(self) -> np.ndarray:
        """Radial (up) direction at the target, in the target frame."""
        return self._up

    def vertical(self, velocity) -> float:
        """Vertical (radial) velocity component, m/s.  Negative = descending."""
        return float(np.dot(np.asarray(velocity, dtype=float), self._up))

    def predict(
        self,
        position,
        velocity,
        mass: float,
        *,
        throttle: float,
        available_thrust: float,
        isp: float,
        dry_mass: float,
        min_speed: float | None = None,
        target_altitude: float = TARGET_ALTITUDE,
    ) -> BurnPrediction | None:
        r = np.asarray(position, dtype=float)
        v = np.asarray(velocity, dtype=float)
        m = float(mass)
        thr = float(throttle) * float(available_thrust)
        dry = float(dry_mass)
        speed = float(np.linalg.norm(v))
        # already at/below the handoff altitude: no crossing ahead
        if float(np.linalg.norm(r - self._center)) - self._radius <= float(target_altitude):
            return None
        if speed < 1e-9:
            return BurnPrediction(endpoint=r.copy(), cutoff_time=0.0, speed_at_cutoff=0.0, vertical_at_cutoff=0.0)
        if m <= dry or thr <= 0.0 or isp <= 0.0 or not np.isfinite(float(isp)):
            return None

        min_speed = float(min_speed) if min_speed is not None else 0.0

        # numba fast path
        if self._jit_params is not None:
            beta, alts, vals, sea_r = self._jit_params
            hit, ex, ey, ez, tc, spd, vr, vh = _burn_coast_rk4_jit(
                r, v, m, self._mu, self._omega, self._center, self._radius,
                beta, alts, vals, sea_r,
                thr, float(isp), self._g0, dry, self._dt, self._t_max,
                min_speed, float(target_altitude),
            )
            if hit == 1:
                return BurnPrediction(
                    endpoint=np.array([ex, ey, ez]), cutoff_time=tc,
                    speed_at_cutoff=spd, vertical_at_cutoff=vr,
                    horizontal_speed=vh,
                )
            return None

        # pure-Python fallback (same semantics, step-end state w/o interpolation)
        mdot = thr / (float(isp) * self._g0)
        dt = self._dt
        t = 0.0
        burning = True
        while t < self._t_max:
            d = r - self._center
            alt = float(np.linalg.norm(d)) - self._radius
            if alt <= float(target_altitude):
                vrad = float(np.dot(v, self._up))
                return BurnPrediction(
                    endpoint=r.copy(), cutoff_time=t,
                    speed_at_cutoff=speed, vertical_at_cutoff=vrad,
                    horizontal_speed=float(np.linalg.norm(v - vrad * self._up)),
                )
            if float(np.linalg.norm(d)) <= self._radius:
                return None
            vrad = float(np.dot(v, self._up))
            if burning:
                burning = vrad < 0.0 and (min_speed <= 0.0 or speed > min_speed) and m > dry
            r, v, t = self._rk4_step(r, v, m, t, dt, thr if burning else 0.0, isp, dry)
            speed = float(np.linalg.norm(v))
            if burning:
                m = max(dry, m - mdot * dt)
        return None

    # -- internals ----------------------------------------------------------

    def _acc(self, r, v, m, thr, isp, dry):
        d = r - self._center
        dist = float(np.linalg.norm(d))
        a = -self._mu * d / (dist * dist * dist)
        a -= 2.0 * np.cross(self._omega, v)
        a -= np.cross(self._omega, np.cross(self._omega, d))
        if self._aero is not None:
            a += np.asarray(self._aero.acceleration(r, v), dtype=float)
        speed = float(np.linalg.norm(v))
        if m > dry and speed > 1e-9:
            a += (thr / m) * (-v / speed)
        return a

    def _rk4_step(self, r, v, m, t, dt, thr, isp, dry):
        mdot = thr / (isp * self._g0)

        def mass_at(tau: float) -> float:
            return max(dry, m - mdot * (tau - t))

        a1 = self._acc(r, v, mass_at(t), thr, isp, dry)
        k1v, k1a = v, a1
        a2 = self._acc(r + 0.5 * dt * k1v, v + 0.5 * dt * k1a, mass_at(t + 0.5 * dt), thr, isp, dry)
        k2v, k2a = v + 0.5 * dt * k1a, a2
        a3 = self._acc(r + 0.5 * dt * k2v, v + 0.5 * dt * k2a, mass_at(t + 0.5 * dt), thr, isp, dry)
        k3v, k3a = v + 0.5 * dt * k2a, a3
        a4 = self._acc(r + dt * k3v, v + dt * k3a, mass_at(t + dt), thr, isp, dry)
        k4v, k4a = v + dt * k3a, a4
        r_new = r + (dt / 6.0) * (k1v + 2.0 * k2v + 2.0 * k3v + k4v)
        v_new = v + (dt / 6.0) * (k1a + 2.0 * k2a + 2.0 * k3a + k4a)
        return r_new, v_new, t + dt


def _wind_up(nose, vel):
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


def nose_for_lift(velocity, miss_xy, max_aoa_deg, gain_deg_per_m, lift_sign: float = -1.0):
    """Commanded nose for unpowered air guidance.

    Base attitude is tail-first (nose opposite velocity, engine into the
    airflow).  The nose is tilted by an AoA proportional to the predicted
    miss toward ``lift_sign * miss_hat`` so the body lift pushes the
    predicted endpoint toward the target (see the LIFT_SIGN comment).
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


@dataclass(frozen=True)
class AirGuidanceCommand:
    nose: tuple[float, float, float]
    up: tuple[float, float, float]
    ignite: bool
    reason: str
    miss: float
    miss_min: float


class ZemAirGuidance:
    """Stateful unpowered air-guidance law + ignition decision."""

    def __init__(
        self,
        *,
        max_aoa_deg: float = MAX_AOA,
        aoa_gain: float = AOA_GAIN,
        lift_sign: float = LIFT_SIGN,
        ignite_miss: float = IGNITE_MISS,
        miss_hysteresis: float = MISS_HYSTERESIS,
        min_ignite_alt: float = MIN_IGNITE_ALT,
        progress_fraction: float = PROGRESS_FRACTION,
    ) -> None:
        self.max_aoa_deg = max_aoa_deg
        self.aoa_gain = aoa_gain
        self.lift_sign = lift_sign
        self.ignite_miss = ignite_miss
        self.miss_hysteresis = miss_hysteresis
        self.min_ignite_alt = min_ignite_alt
        self.progress_fraction = progress_fraction
        self._d_min = float("inf")
        self._d_init: float | None = None
        self._progress = False

    def step(self, velocity, miss_xy, miss: float, altitude: float) -> AirGuidanceCommand:
        """One air-guidance tick.  *miss* is the predicted-endpoint distance
        (``inf`` when no prediction exists)."""
        nose = nose_for_lift(velocity, miss_xy, self.max_aoa_deg, self.aoa_gain, self.lift_sign)
        up = _wind_up(nose, velocity)

        if self._d_init is None:
            self._d_init = miss
        if miss < self._d_min:
            self._d_min = miss
        if self._d_init > 0.0 and self._d_min < self.progress_fraction * self._d_init:
            self._progress = True

        ignite = False
        reason = ""
        if miss < self.ignite_miss:
            ignite, reason = True, "miss < IGNITE_MISS"
        elif altitude < self.min_ignite_alt:
            ignite, reason = True, "altitude floor"
        elif self._progress and miss > self._d_min + self.miss_hysteresis:
            ignite, reason = True, "miss stopped decreasing"

        return AirGuidanceCommand(nose, up, ignite, reason, miss, self._d_min)


def burn_command(velocity, throttle: float):
    """Landing-burn attitude: nose exactly anti-parallel to the velocity."""
    v = np.asarray(velocity, dtype=float)
    speed = float(np.linalg.norm(v))
    if speed < 1e-9:
        return (1.0, 0.0, 0.0)
    return tuple(-v / speed)


def _aoa_deg(nose, velocity) -> float:
    v = np.asarray(velocity, dtype=float)
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return 0.0
    base = -v / n
    d = np.asarray(nose, dtype=float)
    c = float(np.clip(np.dot(d, base) / (np.linalg.norm(d) * np.linalg.norm(base) + 1e-12), -1.0, 1.0))
    return float(np.degrees(np.arccos(c)))


# ---------------------------------------------------------------------------
# Flight entry point
# ---------------------------------------------------------------------------


def main() -> None:
    with ConnectionManager(address="127.0.0.1") as km:
        b = km.add_booster("zem", VESSEL)
        km.register_target("zem", lon=TARGET.lon, lat=TARGET.lat)
        km.enable_debug()
        km.start()

        deadline = time.monotonic() + 5.0
        while km.snapshot("zem") is None:
            if time.monotonic() > deadline:
                raise RuntimeError("telemetry not ready")
            time.sleep(0.02)

        frame = km.frame("zem", "target")
        body_spec, drag_spec = b.sample_predictor_specs()
        aero = DragModel.from_spec(drag_spec)
        predictor = ZemBurnPredictor(body_spec, aero=aero)

        b.physics_range = 200000.0
        b.controls.target_smoothing_time = 0.3
        b.controls.rcs = True
        b.controls.apply(throttle=0.0)

        guidance = ZemAirGuidance(
            max_aoa_deg=MAX_AOA,
            aoa_gain=AOA_GAIN,
            lift_sign=LIFT_SIGN,
            ignite_miss=IGNITE_MISS,
            miss_hysteresis=MISS_HYSTERESIS,
            min_ignite_alt=MIN_IGNITE_ALT,
            progress_fraction=PROGRESS_FRACTION,
        )

        log = open(LOG_FILE, "w", encoding="utf-8")
        csvf = open(CSV_FILE, "w", newline="", encoding="utf-8")
        csvw = csv.writer(csvf)
        csvw.writerow([
            "met", "phase", "alt", "speed", "d_pred", "d_min",
            "aoa_deg", "vh", "epx", "epy", "epz", "cutoff_t", "vrad", "throttle",
        ])

        def logline(msg: str) -> None:
            print(msg, flush=True)
            print(msg, file=log, flush=True)

        # -- Phase A: unpowered air guidance ---------------------------------
        logline(f"== PHASE A: air guidance (LIFT_SIGN={LIFT_SIGN}) ==")
        endpoint_line = b.debug.line(
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
            frame_name="target", color=(0.3, 1.0, 0.6), thickness=0.5,
        )
        pacer = FramePacer(hz=PREDICT_HZ)
        t_start = time.monotonic()
        last_draw = 0.0
        last_pred = None
        s = None

        while True:
            pacer.tick()
            s = b.snapshot()
            if s is None:
                continue
            if s.landed:
                logline("landed during air guidance")
                break

            pos = np.array([s.position.x, s.position.y, s.position.z])
            vel = np.array([s.velocity.x, s.velocity.y, s.velocity.z])

            pred = predictor.predict(
                pos, vel, s.mass,
                throttle=TARGET_THROTTLE,
                available_thrust=s.available_thrust,
                isp=s.specific_impulse,
                dry_mass=s.dry_mass,
                min_speed=TERMINAL_SPEED,
                target_altitude=TARGET_ALTITUDE,
            )
            last_pred = pred
            if pred is None:
                d = float("inf")
                miss_xy = (0.0, 0.0)
            else:
                # ZEM miss = horizontal offset of the TARGET_ALTITUDE crossing
                # point from the target (up-projection, frame-agnostic)
                up = predictor.up
                rh = pred.endpoint - up * float(np.dot(pred.endpoint, up))
                d = float(np.linalg.norm(rh))
                miss_xy = (float(rh[0]), float(rh[1]))

            cmd = guidance.step(vel, miss_xy, d, s.surface_altitude)
            b.controls.apply(
                target_direction=cmd.nose, reference_frame=frame,
                up=cmd.up, throttle=0.0,
            )

            now = time.monotonic()
            if now - last_draw >= 0.5 and pred is not None:
                last_draw = now
                try:
                    endpoint_line.set_points(tuple(pos), tuple(pred.endpoint))
                except Exception as exc:  # noqa: BLE001 — drawing must not kill guidance
                    logline(f"draw error: {exc}")

            speed = float(np.linalg.norm(vel))
            aoa = _aoa_deg(cmd.nose, vel)
            csvw.writerow([
                f"{s.met:.2f}", "aero", f"{s.surface_altitude:.1f}", f"{speed:.1f}",
                f"{d:.1f}", f"{cmd.miss_min:.1f}", f"{aoa:.1f}",
                f"{pred.horizontal_speed:.1f}" if pred is not None else "",
                f"{pred.endpoint[0]:.1f}" if pred is not None else "",
                f"{pred.endpoint[1]:.1f}" if pred is not None else "",
                f"{pred.endpoint[2]:.1f}" if pred is not None else "",
                f"{pred.cutoff_time:.1f}" if pred is not None else "",
                f"{predictor.vertical(vel):.1f}", "0.0",
            ])
            csvf.flush()

            logline(
                f"[A] t={now - t_start:6.1f} alt={s.surface_altitude:8.1f} "
                f"v={speed:6.1f} d={d:8.1f} dmin={cmd.miss_min:8.1f} "
                f"aoa={aoa:5.1f} vh={pred.horizontal_speed:5.1f} "
                f"cross={pred.cutoff_time:5.1f}s"
                if pred is not None else
                f"[A] t={now - t_start:6.1f} alt={s.surface_altitude:8.1f} "
                f"v={speed:6.1f} d=   inf (no crossing ahead)"
            )

            if cmd.ignite:
                logline(f"== IGNITE: {cmd.reason} (d={d:.1f} m, d_min={cmd.miss_min:.1f} m) ==")
                break

        # -- Phase B: landing burn (follow the prediction model) -------------
        if s is None or s.landed:
            logline("no burn — vessel landed / no telemetry")
        else:
            if last_pred is not None:
                logline(
                    f"predicted handoff (TARGET_ALTITUDE={TARGET_ALTITUDE:.0f} m): "
                    f"({last_pred.endpoint[0]:.1f}, {last_pred.endpoint[1]:.1f}, "
                    f"{last_pred.endpoint[2]:.1f}) m, horizontal speed "
                    f"{last_pred.horizontal_speed:.1f} m/s, crossing in "
                    f"{last_pred.cutoff_time:.1f} s"
                )
            logline("== PHASE B: landing burn ==")
            pacer2 = FramePacer(hz=CONTROL_HZ)
            while True:
                pacer2.tick()
                s = b.snapshot()
                if s is None:
                    continue
                if s.landed:
                    logline("landed during burn")
                    break

                vel = np.array([s.velocity.x, s.velocity.y, s.velocity.z])
                speed = float(np.linalg.norm(vel))
                vrad = predictor.vertical(vel)

                if speed < TERMINAL_SPEED or vrad >= 0.0:
                    logline(
                        f"== GUIDANCE EXIT: speed={speed:.1f} m/s (< {TERMINAL_SPEED}) "
                        f"or vrad={vrad:.1f} m/s =="
                    )
                    break

                nose = burn_command(vel, TARGET_THROTTLE)
                b.controls.apply(
                    target_direction=nose, reference_frame=frame,
                    throttle=TARGET_THROTTLE,
                )

                csvw.writerow([
                    f"{s.met:.2f}", "burn", f"{s.surface_altitude:.1f}", f"{speed:.1f}",
                    "", "", "", "", "", "", "", "", f"{vrad:.1f}", f"{TARGET_THROTTLE:.2f}",
                ])
                csvf.flush()
                logline(
                    f"[B] alt={s.surface_altitude:8.1f} v={speed:6.1f} "
                    f"vrad={vrad:6.1f} thr={TARGET_THROTTLE:.2f}"
                )

            b.controls.apply(throttle=0.0)
            pos = np.array([s.position.x, s.position.y, s.position.z])
            logline(
                f"== SUMMARY ==\n"
                f"  exit position : ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) m  "
                f"(target frame)\n"
                f"  exit speed    : {speed:.1f} m/s  alt={s.surface_altitude:.1f} m\n"
                f"  mass          : {s.mass:.1f} kg  (dry {s.dry_mass:.1f})\n"
                f"  final touchdown is not implemented yet — guidance has exited."
            )

        b.controls.cut_thrust()
        csvf.close()
        log.close()


if __name__ == "__main__":
    main()
