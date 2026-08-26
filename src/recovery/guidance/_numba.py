"""Numba-accelerated fixed-step RK4 integrator and helper kernels.

These functions are pure numerical kernels with zero kRPC dependency.
They accept scalars / 1-D numpy arrays and return numeric values only.
"""

import numpy as np
from numba import njit
from math import acos, sqrt


@njit(cache=True)
def rk4_fixed(
    r0: np.ndarray,
    v0: np.ndarray,
    mu: float,
    omega: np.ndarray,
    center: np.ndarray,
    radius: float,
    beta: float,
    alts: np.ndarray,
    densities: np.ndarray,
    sea_r: float,
    dt: float,
    max_n: int,
) -> tuple[float, float, float, float] | None:
    """Fixed-step RK4 integration with optional drag (inline).

    Returns ``(x, y, z, t_hit)`` or ``None`` if the surface is not reached.
    """
    # The explicit per-component ``r[0] / r[1] / r[2]`` indexing below (rather
    # than a vectorised ``r += dt/6 * (k1 + 2*k2 + 2*k3 + k4)``) is
    # deliberate: numba compiles scalar loads/stores far more efficiently than
    # temporary array allocations inside the hot loop, and this kernel runs
    # ~6k+ steps per prediction.  Keep the expansion unless a benchmark proves
    # a vectorised form faster.
    r = np.empty(3, dtype=np.float64)
    v = np.empty(3, dtype=np.float64)
    r[0] = r0[0]
    r[1] = r0[1]
    r[2] = r0[2]
    v[0] = v0[0]
    v[1] = v0[1]
    v[2] = v0[2]
    dt2 = dt / 2.0

    for _step in range(max_n):
        rpx = r[0]
        rpy = r[1]
        rpz = r[2]
        k1r = v
        k1v = _accel_jit(r, v, mu, omega, center, beta, alts, densities, sea_r)

        r2 = np.empty(3, dtype=np.float64)
        v2 = np.empty(3, dtype=np.float64)
        r2[0] = r[0] + dt2 * k1r[0]
        r2[1] = r[1] + dt2 * k1r[1]
        r2[2] = r[2] + dt2 * k1r[2]
        v2[0] = v[0] + dt2 * k1v[0]
        v2[1] = v[1] + dt2 * k1v[1]
        v2[2] = v[2] + dt2 * k1v[2]
        k2r = v2
        k2v = _accel_jit(r2, v2, mu, omega, center, beta, alts, densities, sea_r)

        r3 = np.empty(3, dtype=np.float64)
        v3 = np.empty(3, dtype=np.float64)
        r3[0] = r[0] + dt2 * k2r[0]
        r3[1] = r[1] + dt2 * k2r[1]
        r3[2] = r[2] + dt2 * k2r[2]
        v3[0] = v[0] + dt2 * k2v[0]
        v3[1] = v[1] + dt2 * k2v[1]
        v3[2] = v[2] + dt2 * k2v[2]
        k3r = v3
        k3v = _accel_jit(r3, v3, mu, omega, center, beta, alts, densities, sea_r)

        r4 = np.empty(3, dtype=np.float64)
        v4 = np.empty(3, dtype=np.float64)
        r4[0] = r[0] + dt * k3r[0]
        r4[1] = r[1] + dt * k3r[1]
        r4[2] = r[2] + dt * k3r[2]
        v4[0] = v[0] + dt * k3v[0]
        v4[1] = v[1] + dt * k3v[1]
        v4[2] = v[2] + dt * k3v[2]
        k4r = v4
        k4v = _accel_jit(r4, v4, mu, omega, center, beta, alts, densities, sea_r)

        r[0] += (dt / 6.0) * (k1r[0] + 2.0 * k2r[0] + 2.0 * k3r[0] + k4r[0])
        r[1] += (dt / 6.0) * (k1r[1] + 2.0 * k2r[1] + 2.0 * k3r[1] + k4r[1])
        r[2] += (dt / 6.0) * (k1r[2] + 2.0 * k2r[2] + 2.0 * k3r[2] + k4r[2])
        v[0] += (dt / 6.0) * (k1v[0] + 2.0 * k2v[0] + 2.0 * k3v[0] + k4v[0])
        v[1] += (dt / 6.0) * (k1v[1] + 2.0 * k2v[1] + 2.0 * k3v[1] + k4v[1])
        v[2] += (dt / 6.0) * (k1v[2] + 2.0 * k2v[2] + 2.0 * k3v[2] + k4v[2])

        dx = r[0] - center[0]
        dy = r[1] - center[1]
        dz = r[2] - center[2]
        dist_post = (dx * dx + dy * dy + dz * dz) ** 0.5
        if dist_post < radius:
            # Interpolate the surface crossing between the pre-step (rpx/rpy/
            # rpz) and post-step positions, so the reported impact lies on the
            # surface (alt ~ 0) rather than one step below it.
            pdx = rpx - center[0]
            pdy = rpy - center[1]
            pdz = rpz - center[2]
            dist_pre = (pdx * pdx + pdy * pdy + pdz * pdz) ** 0.5
            lam = (dist_pre - radius) / (dist_pre - dist_post)
            ix = rpx + lam * (r[0] - rpx)
            iy = rpy + lam * (r[1] - rpy)
            iz = rpz + lam * (r[2] - rpz)
            t_hit = float(_step) * dt + lam * dt
            return (float(ix), float(iy), float(iz), t_hit)

    return None


@njit(cache=True)
def _accel_jit(
    r: np.ndarray,
    v: np.ndarray,
    mu: float,
    omega: np.ndarray,
    center: np.ndarray,
    beta: float,
    alts: np.ndarray,
    densities: np.ndarray,
    sea_r: float,
) -> np.ndarray:
    """Gravity + Coriolis + centrifugal + drag, inline for numba."""
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

    dx_c = r[0] - center[0]
    dy_c = r[1] - center[1]
    dz_c = r[2] - center[2]
    ox_dx = omega[1] * dz_c - omega[2] * dy_c
    ox_dy = omega[2] * dx_c - omega[0] * dz_c
    ox_dz = omega[0] * dy_c - omega[1] * dx_c
    a[0] += -(omega[1] * ox_dz - omega[2] * ox_dy)
    a[1] += -(omega[2] * ox_dx - omega[0] * ox_dz)
    a[2] += -(omega[0] * ox_dy - omega[1] * ox_dx)

    use_drag = False
    if beta > 0.0:
        if beta == float("inf"):
            use_drag = False
        else:
            use_drag = True
    if use_drag:
        alt = dist - sea_r
        rho = _interp_jit(alt, alts, densities)
        if rho > 0.0:
            v_mag = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5
            if v_mag > 1e-6:
                drag = -0.5 * rho * v_mag / beta
                a[0] += drag * v[0]
                a[1] += drag * v[1]
                a[2] += drag * v[2]

    return a


@njit(cache=True)
def _interp_jit(x: float, xp: np.ndarray, fp: np.ndarray) -> float:
    """Atmospheric density interpolation.

    Mirrors the Python-side :class:`~recovery.guidance.DragModel` boundary
    policy used by the fast RK4 path:

    * at or below the lowest sample -> clamp to the lowest value;
    * strictly above the highest sample -> zero (above the atmosphere);
    * otherwise linearly interpolate between the bracketing samples.
    """
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
def _interp_lin(x: float, xp: np.ndarray, fp: np.ndarray) -> float:
    """Linear interpolation with clamped boundaries (lift/drag coefficient
    tables).  Unlike :func:`_interp_jit` (density, returns 0 above the top),
    this clamps to the endpoint values so a table evaluated at its extremes
    stays defined.
    """
    n = len(xp)
    if n == 0:
        return 0.0
    if x <= xp[0]:
        return float(fp[0])
    if x >= xp[n - 1]:
        return float(fp[n - 1])
    for i in range(n - 1):
        if xp[i] <= x <= xp[i + 1]:
            t = (x - xp[i]) / (xp[i + 1] - xp[i])
            return float(fp[i] + t * (fp[i + 1] - fp[i]))
    return float(fp[n - 1])


# ---------------------------------------------------------------------------
# Controlled propagation (virtual control + engine events)
# ---------------------------------------------------------------------------

@njit(cache=True)
def _nose_jit(kind: int, fx: float, fy: float, fz: float,
              rx: float, ry: float, rz: float,
              vx: float, vy: float, vz: float,
              upx: float, upy: float, upz: float) -> tuple[float, float, float]:
    """Steering rule -> unit nose direction ``(nx, ny, nz)``.

    ``kind``: 0 = retrograde, 1 = up, 2 = fixed, 3 = toward target (origin).
    Degenerate cases (zero velocity / zero vector / at origin) fall back to up.
    Returns a scalar tuple (no array allocation in the hot loop).
    """
    if kind == 0:  # retrograde
        vm = sqrt(vx * vx + vy * vy + vz * vz)
        if vm < 1e-9:
            return upx, upy, upz
        return -vx / vm, -vy / vm, -vz / vm
    if kind == 1:  # up
        return upx, upy, upz
    if kind == 2:  # fixed
        fm = sqrt(fx * fx + fy * fy + fz * fz)
        if fm < 1e-9:
            return upx, upy, upz
        return fx / fm, fy / fm, fz / fm
    # toward target (origin): -r_hat
    rm = sqrt(rx * rx + ry * ry + rz * rz)
    if rm < 1e-9:
        return upx, upy, upz
    return -rx / rm, -ry / rm, -rz / rm


@njit(cache=True)
def _accel_ctrl_jit(rx: float, ry: float, rz: float,
                    vx: float, vy: float, vz: float, m: float,
                    thr: float, thrust: float, isp: float,
                    nx: float, ny: float, nz: float,
                    mu: float, omega: np.ndarray, center: np.ndarray,
                    clamp_aoa: float, alpha_pts: np.ndarray,
                    cl_table: np.ndarray, cd_table: np.ndarray,
                    alts: np.ndarray, densities: np.ndarray,
                    sea_r: float, use_aero: bool) -> tuple[float, float, float]:
    """Gravity + Coriolis + centrifugal + attitude-dependent aero + thrust.

    Thrust is ``throttle * max_thrust / mass`` along the (unit) nose direction.
    The aero term is attitude-dependent: the body axis ``nose`` sets the angle
    of attack (against the velocity), which produces zero-lift + induced drag
    along ``-v`` and body lift perpendicular to ``v`` toward the nose side.
    Returns ``(ax, ay, az)`` as a scalar tuple (no hot-loop allocation).
    """
    dx = rx - center[0]
    dy = ry - center[1]
    dz = rz - center[2]
    dist = sqrt(dx * dx + dy * dy + dz * dz)
    inv_cube = 1.0 / (dist * dist * dist)
    g = -mu * inv_cube
    a0 = g * dx
    a1 = g * dy
    a2 = g * dz
    # Coriolis
    a0 += -2.0 * (omega[1] * vz - omega[2] * vy)
    a1 += -2.0 * (omega[2] * vx - omega[0] * vz)
    a2 += -2.0 * (omega[0] * vy - omega[1] * vx)
    # centrifugal
    ox_dx = omega[1] * dz - omega[2] * dy
    ox_dy = omega[2] * dx - omega[0] * dz
    ox_dz = omega[0] * dy - omega[1] * dx
    a0 += -(omega[1] * ox_dz - omega[2] * ox_dy)
    a1 += -(omega[2] * ox_dx - omega[0] * ox_dz)
    a2 += -(omega[0] * ox_dy - omega[1] * ox_dx)
    # attitude-dependent aero: drag (zero-lift + induced) + body lift
    if use_aero:
        alt = dist - sea_r
        rho = _interp_jit(alt, alts, densities)
        if rho > 0.0 and m > 0.0:
            v_mag = sqrt(vx * vx + vy * vy + vz * vz)
            if v_mag > 1e-6:
                vhx = vx / v_mag
                vhy = vy / v_mag
                vhz = vz / v_mag
                # angle of attack: between -nose (tail-first) and velocity
                c = -(nx * vhx + ny * vhy + nz * vhz)
                if c > 1.0:
                    c = 1.0
                elif c < -1.0:
                    c = -1.0
                alpha = acos(c)
                if alpha > clamp_aoa:
                    alpha = clamp_aoa
                q = 0.5 * rho * v_mag * v_mag
                # interpolate lift/drag coefficients (m^2) at this alpha
                cl = _interp_lin(alpha, alpha_pts, cl_table)
                cd = _interp_lin(alpha, alpha_pts, cd_table)
                drag = q * cd / m
                a0 -= drag * vhx
                a1 -= drag * vhy
                a2 -= drag * vhz
                # lift toward the nose-deflection side (cl_table carries the sign)
                if alpha > 1e-4:
                    nv = nx * vhx + ny * vhy + nz * vhz
                    latx = nx - nv * vhx
                    laty = ny - nv * vhy
                    latz = nz - nv * vhz
                    ln = sqrt(latx * latx + laty * laty + latz * latz)
                    if ln > 1e-9:
                        lift = q * cl / m
                        a0 += lift * latx / ln
                        a1 += lift * laty / ln
                        a2 += lift * latz / ln
    # thrust
    if thrust > 0.0 and thr > 0.0 and m > 0.0:
        ta = thr * thrust / m
        a0 += ta * nx
        a1 += ta * ny
        a2 += ta * nz
    return a0, a1, a2


@njit(cache=True)
def _throttle_jit(kind: int, p1: float, p2: float,
                  m: float, thrust: float, g_local: float,
                  alt: float, vrad: float) -> float:
    """Throttle rule -> throttle (0..1).

    ``kind``: 0 = constant (``p1`` = throttle), 1 = brake_to (``p1`` =
    v_terminal, ``p2`` = h_terminal; full throttle below h_terminal).
    ``vrad`` is vertical velocity (up positive); the law uses ``vrad^2`` so it
    is sign-agnostic (it only ever brakes a descent).  ``h_terminal == 0`` is
    the constant-deceleration (suicide-burn) case: brake to v_terminal at the
    surface.
    """
    if kind == 0:
        return p1
    if thrust <= 0.0 or m <= 0.0:
        return 0.0
    # kind == 1: brake_to (v_terminal, h_terminal)
    v_terminal = p1
    h_terminal = p2
    if alt >= h_terminal:
        acc = (vrad * vrad - v_terminal * v_terminal) / (2.0 * max(alt - h_terminal, 1e-3))
        thr = m * (acc + g_local) / thrust
    else:
        thr = 1.0
    if thr < 0.0:
        thr = 0.0
    elif thr > 1.0:
        thr = 1.0
    return thr


@njit(cache=True)
def rk4_controlled(
    r0: np.ndarray,
    v0: np.ndarray,
    m0: float,
    mu: float,
    omega: np.ndarray,
    center: np.ndarray,
    radius: float,
    clamp_aoa: float,
    alpha_pts: np.ndarray,
    cl_table: np.ndarray,
    cd_table: np.ndarray,
    alts: np.ndarray,
    densities: np.ndarray,
    sea_r: float,
    seg_trigger_val: np.ndarray,
    seg_trigger_kind: np.ndarray,
    seg_throttle_kind: np.ndarray,
    seg_throttle_p1: np.ndarray,
    seg_throttle_p2: np.ndarray,
    seg_thrust: np.ndarray,
    seg_isp: np.ndarray,
    seg_nose_kind: np.ndarray,
    seg_nose_fixed: np.ndarray,
    up: np.ndarray,
    dry_mass: float,
    g0: float,
    dt: float,
    max_n: int,
    out: np.ndarray,
) -> tuple[int, int]:
    """Fixed-step RK4 with piecewise-constant virtual control and full output.

    Segments are passed as parallel arrays; ``segment[0]`` is active from t=0.
    Each later segment fires (in order) when its trigger is met — a time
    trigger when ``t >= value``, or an altitude trigger when the vessel
    descends through ``alt <= value`` — and its throttle / thrust / isp / nose
    rule take over immediately.

    Every step is written to ``out[step, :] = [t, rx, ry, rz, vx, vy, vz, m]``.

    Returns ``(n_steps, hit)`` where ``hit=1`` if the surface sphere was
    crossed and ``n_steps`` is the number of rows written.
    """
    n_seg = seg_throttle_kind.shape[0]
    thr_kind = seg_throttle_kind[0]
    thr_p1 = seg_throttle_p1[0]
    thr_p2 = seg_throttle_p2[0]
    thrust = seg_thrust[0]
    isp = seg_isp[0]
    nk = seg_nose_kind[0]
    fx = seg_nose_fixed[0, 0]
    fy = seg_nose_fixed[0, 1]
    fz = seg_nose_fixed[0, 2]
    fired = np.zeros(n_seg, dtype=np.int8)
    fired[0] = 1

    rx = r0[0]; ry = r0[1]; rz = r0[2]
    vx = v0[0]; vy = v0[1]; vz = v0[2]
    m = m0
    upx = up[0]; upy = up[1]; upz = up[2]
    cx = center[0]; cy = center[1]; cz = center[2]
    dt2 = dt / 2.0
    dt6 = dt / 6.0
    t = 0.0
    use_aero = alpha_pts.size > 0

    for step in range(max_n):
        dx = rx - cx
        dy = ry - cy
        dz = rz - cz
        dist = sqrt(dx * dx + dy * dy + dz * dz)
        alt = dist - sea_r

        # ---- engine events ------------------------------------------------
        for j in range(1, n_seg):
            if fired[j] == 0:
                if seg_trigger_kind[j] == 0:
                    met = t >= seg_trigger_val[j]
                else:
                    met = alt <= seg_trigger_val[j]
                if met:
                    thr_kind = seg_throttle_kind[j]
                    thr_p1 = seg_throttle_p1[j]
                    thr_p2 = seg_throttle_p2[j]
                    thrust = seg_thrust[j]
                    isp = seg_isp[j]
                    nk = seg_nose_kind[j]
                    fx = seg_nose_fixed[j, 0]
                    fy = seg_nose_fixed[j, 1]
                    fz = seg_nose_fixed[j, 2]
                    fired[j] = 1

        # ---- throttle rule + mass flow ------------------------------------
        vrad = vx * upx + vy * upy + vz * upz
        g_local = mu / (dist * dist)
        thr = _throttle_jit(thr_kind, thr_p1, thr_p2, m, thrust, g_local, alt, vrad)
        mdot = thr * thrust / (isp * g0) if (thrust > 0.0 and isp > 0.0) else 0.0

        # ---- record state --------------------------------------------------
        out[step, 0] = t
        out[step, 1] = rx; out[step, 2] = ry; out[step, 3] = rz
        out[step, 4] = vx; out[step, 5] = vy; out[step, 6] = vz
        out[step, 7] = m

        # ---- nose direction ------------------------------------------------
        nx, ny, nz = _nose_jit(nk, fx, fy, fz, rx, ry, rz, vx, vy, vz, upx, upy, upz)

        # ---- RK4 (mass held constant within the step) ----------------------
        a1x, a1y, a1z = _accel_ctrl_jit(rx, ry, rz, vx, vy, vz, m, thr, thrust, isp,
                                        nx, ny, nz, mu, omega, center, clamp_aoa,
                                        alpha_pts, cl_table, cd_table, alts, densities, sea_r, use_aero)
        r2x = rx + dt2 * vx; r2y = ry + dt2 * vy; r2z = rz + dt2 * vz
        v2x = vx + dt2 * a1x; v2y = vy + dt2 * a1y; v2z = vz + dt2 * a1z
        a2x, a2y, a2z = _accel_ctrl_jit(r2x, r2y, r2z, v2x, v2y, v2z, m, thr, thrust, isp,
                                        nx, ny, nz, mu, omega, center, clamp_aoa,
                                        alpha_pts, cl_table, cd_table, alts, densities, sea_r, use_aero)
        r3x = rx + dt2 * v2x; r3y = ry + dt2 * v2y; r3z = rz + dt2 * v2z
        v3x = vx + dt2 * a2x; v3y = vy + dt2 * a2y; v3z = vz + dt2 * a2z
        a3x, a3y, a3z = _accel_ctrl_jit(r3x, r3y, r3z, v3x, v3y, v3z, m, thr, thrust, isp,
                                        nx, ny, nz, mu, omega, center, clamp_aoa,
                                        alpha_pts, cl_table, cd_table, alts, densities, sea_r, use_aero)
        r4x = rx + dt * v3x; r4y = ry + dt * v3y; r4z = rz + dt * v3z
        v4x = vx + dt * a3x; v4y = vy + dt * a3y; v4z = vz + dt * a3z
        a4x, a4y, a4z = _accel_ctrl_jit(r4x, r4y, r4z, v4x, v4y, v4z, m, thr, thrust, isp,
                                        nx, ny, nz, mu, omega, center, clamp_aoa,
                                        alpha_pts, cl_table, cd_table, alts, densities, sea_r, use_aero)

        rx += dt6 * (vx + 2.0 * v2x + 2.0 * v3x + v4x)
        ry += dt6 * (vy + 2.0 * v2y + 2.0 * v3y + v4y)
        rz += dt6 * (vz + 2.0 * v2z + 2.0 * v3z + v4z)
        vx += dt6 * (a1x + 2.0 * a2x + 2.0 * a3x + a4x)
        vy += dt6 * (a1y + 2.0 * a2y + 2.0 * a3y + a4y)
        vz += dt6 * (a1z + 2.0 * a2z + 2.0 * a3z + a4z)

        # ---- mass flow (Euler) ---------------------------------------------
        m -= mdot * dt
        if m < dry_mass:
            m = dry_mass
        t += dt

        # ---- impact (interpolate the exact surface crossing) ---------------
        dx = rx - cx
        dy = ry - cy
        dz = rz - cz
        dist_post = sqrt(dx * dx + dy * dy + dz * dz)
        if dist_post < radius:
            # ``dist`` (computed at the top of the step) is the pre-step
            # distance; ``dist_post`` is the post-step distance.  Linearly
            # interpolate the altitude crossing so the reported impact state
            # sits on the surface (alt ~ 0) rather than one step early.
            lam = (dist - radius) / (dist - dist_post)
            out[step, 0] += lam * dt
            out[step, 1] += lam * (rx - out[step, 1])
            out[step, 2] += lam * (ry - out[step, 2])
            out[step, 3] += lam * (rz - out[step, 3])
            out[step, 4] += lam * (vx - out[step, 4])
            out[step, 5] += lam * (vy - out[step, 5])
            out[step, 6] += lam * (vz - out[step, 6])
            return step + 1, 1

    return max_n, 0
