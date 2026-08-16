"""Numba-accelerated fixed-step RK4 integrator and helper kernels.

These functions are pure numerical kernels with zero kRPC dependency.
They accept scalars / 1-D numpy arrays and return numeric values only.
"""

import numpy as np
from numba import njit


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
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5
        if dist < radius:
            t_hit = float(_step + 1) * dt
            return (float(r[0]), float(r[1]), float(r[2]), t_hit)

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
