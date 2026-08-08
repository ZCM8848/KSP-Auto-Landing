"""Landing point predictor via RK45 numerical integration.

Integrates the equations of motion in the rotating (planet-fixed) target
reference frame from a given position and velocity until the vessel
crosses the surface sphere.  The dynamics include:

* altitude-decaying inverse-square gravity  (mu / r^2)
* Coriolis acceleration  (-2 * omega x v)
* centrifugal acceleration  (-omega x (omega x r))
* optional aerodynamic acceleration from an :class:`AeroModel` (e.g.
  :class:`KrpcAeroModel` for per-step kRPC force simulation)

The recommended constructor is :meth:`LandingPredictor.from_body`, which
derives all planetary constants from a kRPC ``CelestialBody``; callers can
then use :meth:`predict` with :class:`Vector3` values or
:meth:`predict_from` with a :class:`FlightState` snapshot.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from math import sqrt
from typing import Any

import numpy as np
from numba import njit
from numpy.typing import NDArray
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation

from ..ksp.types import FlightState, Vector3

Vec3 = tuple[float, float, float]


def _retrograde_quaternion(velocity: NDArray[np.float64]) -> Vec4:
    """Return a quaternion ``(x, y, z, w)`` that rotates the vessel so its
    nose (+Y) points opposite to *velocity* (AoA 180° / tail-first).

    The quaternion is expressed in the same reference frame as *velocity*,
    suitable for passing to kRPC
    ``Flight.simulate_aerodynamic_force_at``'s *rotation* parameter.
    """
    v_norm = float(sqrt(velocity[0] ** 2 + velocity[1] ** 2 + velocity[2] ** 2))
    if v_norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    nose = -velocity / v_norm
    world_up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(nose, world_up))) > 0.999:
        world_up = np.array([1.0, 0.0, 0.0])
    roof = np.cross(world_up, nose)
    roof /= float(sqrt(roof[0] ** 2 + roof[1] ** 2 + roof[2] ** 2))
    rot, _ = Rotation.align_vectors([nose, roof], [[0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
    q = rot.as_quat()
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


Vec4 = tuple[float, float, float, float]


@njit(cache=True)
def _rk4_fixed(
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
    All arguments are numba‑compatible scalars / 1‑D arrays.
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
        # --- RK4 slope-1 ----------------------------------------------------
        k1r = v
        k1v = _accel_jit(r, v, mu, omega, center, beta, alts, densities, sea_r)

        # --- RK4 slope-2 ----------------------------------------------------
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

        # --- RK4 slope-3 ----------------------------------------------------
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

        # --- RK4 slope-4 ----------------------------------------------------
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

        # --- update ---------------------------------------------------------
        r[0] += (dt / 6.0) * (k1r[0] + 2.0 * k2r[0] + 2.0 * k3r[0] + k4r[0])
        r[1] += (dt / 6.0) * (k1r[1] + 2.0 * k2r[1] + 2.0 * k3r[1] + k4r[1])
        r[2] += (dt / 6.0) * (k1r[2] + 2.0 * k2r[2] + 2.0 * k3r[2] + k4r[2])
        v[0] += (dt / 6.0) * (k1v[0] + 2.0 * k2v[0] + 2.0 * k3v[0] + k4v[0])
        v[1] += (dt / 6.0) * (k1v[1] + 2.0 * k2v[1] + 2.0 * k3v[1] + k4v[1])
        v[2] += (dt / 6.0) * (k1v[2] + 2.0 * k2v[2] + 2.0 * k3v[2] + k4v[2])

        # --- surface event --------------------------------------------------
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

    # coriolis: -2 * omega x v
    a[0] += -2.0 * (omega[1] * v[2] - omega[2] * v[1])
    a[1] += -2.0 * (omega[2] * v[0] - omega[0] * v[2])
    a[2] += -2.0 * (omega[0] * v[1] - omega[1] * v[0])

    # centrifugal: -omega x (omega x d)
    dx_c = r[0] - center[0]
    dy_c = r[1] - center[1]
    dz_c = r[2] - center[2]
    # omega x d
    ox_dx = omega[1] * dz_c - omega[2] * dy_c
    ox_dy = omega[2] * dx_c - omega[0] * dz_c
    ox_dz = omega[0] * dy_c - omega[1] * dx_c
    # - omega x (omega x d)
    a[0] += -(omega[1] * ox_dz - omega[2] * ox_dy)
    a[1] += -(omega[2] * ox_dx - omega[0] * ox_dz)
    a[2] += -(omega[0] * ox_dy - omega[1] * ox_dx)

    # --- drag (inline interpolation) ---------------------------------------
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
    """Linear interpolation — numba‑safe for a single query."""
    n = len(xp)
    if x <= xp[0]:
        return float(fp[0])
    if x >= xp[n - 1]:
        return float(fp[n - 1])
    for i in range(n - 1):
        if xp[i] <= x <= xp[i + 1]:
            t = (x - xp[i]) / (xp[i + 1] - xp[i])
            return float(fp[i] + t * (fp[i + 1] - fp[i]))
    return 0.0


class AeroModel(ABC):
    """Aerodynamic acceleration contribution for use inside
    :class:`LandingPredictor` integration.
    """

    @abstractmethod
    def acceleration(
        self,
        position: NDArray[np.float64],
        velocity: NDArray[np.float64],
    ) -> Vec3:
        """Return aerodynamic acceleration (m/s²) at the given state,
        expressed in the predictor's working reference frame.
        """
        ...


class KrpcAeroModel(AeroModel):
    """Per-step aerodynamic model via kRPC
    ``Flight.simulate_aerodynamic_force_at``.

    At each integration step the vessel orientation is set to **retrograde**
    (AoA 180° / tail-first), and the returned force is divided by *mass* to
    obtain the acceleration contribution.

    The kRPC ``Flight`` instance must have been created with the **same**
    reference frame that the surrounding :class:`LandingPredictor` uses.
    """

    def __init__(self, flight: Any, body: Any, mass: float) -> None:
        self._flight = flight
        self._body = body
        self._mass = float(mass)

    def acceleration(
        self,
        position: NDArray[np.float64],
        velocity: NDArray[np.float64],
    ) -> Vec3:
        v_norm = float(sqrt(velocity[0] ** 2 + velocity[1] ** 2 + velocity[2] ** 2))
        if v_norm < 1e-6 or self._mass <= 0:
            return (0.0, 0.0, 0.0)
        quat = _retrograde_quaternion(velocity)
        pos = (float(position[0]), float(position[1]), float(position[2]))
        vel = (float(velocity[0]), float(velocity[1]), float(velocity[2]))
        force = self._flight.simulate_aerodynamic_force_at(self._body, pos, vel, quat)
        return (force[0] / self._mass, force[1] / self._mass, force[2] / self._mass)


class DragModel(AeroModel):
    """Offline aerodynamic drag model using a sampled density profile.

    Acceleration at each step is computed from the standard drag formula
    without any further kRPC calls:

        ``a_drag = -½·ρ(h)·|v|²/β · v̂``

    where *β* is the ballistic coefficient (``mass / (C_d·A)``, kg/m²)
    and *ρ(h)* is the atmospheric density interpolated from a pre-sampled
    altitude‑density table.

    Use :meth:`from_krpc` to build a model by sampling a kRPC
    ``CelestialBody`` and ``Flight`` once; after construction the model is
    fully offline and safe to evaluate from any thread.
    """

    def __init__(
        self,
        *,
        ballistic_coefficient: float,
        density_fn: Callable[[float], float],
        body_center: Sequence[float],
        sea_level_radius: float,
        density_alts: np.ndarray | None = None,
        density_vals: np.ndarray | None = None,
    ) -> None:
        self._beta = float(ballistic_coefficient)
        self._density = density_fn
        self._center = np.asarray(body_center, dtype=float)
        self._sea_r = float(sea_level_radius)
        self._density_alts = density_alts
        self._density_vals = density_vals

    @classmethod
    def from_krpc(
        cls,
        body: Any,
        flight: Any,
        target_frame: Any,
        *,
        mass: float | None = None,
        manual_beta: float | None = None,
        altitude_samples: int = 64,
    ) -> DragModel:
        """Sample density profile and ballistic coefficient from kRPC.

        **One-time RPC cost:** ~25 ms for 64 samples (measured on Kerbin).
        After this call the returned model performs zero network I/O.

        *altitude_samples* is treated as a minimum; the actual count is
        never less than ``max(32, atmosphere_depth / 500)`` so very deep
        atmospheres (RSS Earth ~140 km) automatically get more points.
        Sample spacing follows a cosine distribution — dense near sea level,
        coarser at high altitude.
        """
        center = tuple(np.array(body.position(target_frame)))
        sea_r = float(body.equatorial_radius)
        depth = float(body.atmosphere_depth)

        min_s = max(32, int(depth / 500.0))
        n = max(altitude_samples, min_s)
        raw = np.linspace(0.0, np.pi / 2.0, n)
        alts = depth * (1.0 - np.cos(raw))
        alts[0] = 0.0
        alts[-1] = depth
        densities = np.array(
            [float(body.density_at(float(h))) for h in alts], dtype=float
        )

        def _interp(h: float) -> float:
            if h < 0.0:
                return float(densities[0])
            if h > depth:
                return 0.0
            return float(np.interp(h, alts, densities))

        # --- ballistic coefficient ------------------------------------------
        beta: float
        if manual_beta is not None:
            beta = manual_beta
        elif getattr(getattr(body, "space_center", None), "far_available", False):
            beta = float(getattr(flight, "ballistic_coefficient", 0.0))
        elif mass is not None:
            rho = float(flight.atmosphere_density)
            d = flight.drag
            drag_mag = float(sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]))
            spd = float(flight.speed)
            if drag_mag > 1e-6 and spd > 1e-6 and rho > 1e-12:
                beta = mass * rho * spd * spd / (2.0 * drag_mag)
            else:
                beta = float("inf")
        else:
            raise ValueError(
                "Cannot determine ballistic coefficient: "
                "FAR not installed, no manual_beta, and no mass for estimation"
            )

        return cls(
            ballistic_coefficient=beta,
            density_fn=_interp,
            body_center=center,
            sea_level_radius=sea_r,
            density_alts=alts,
            density_vals=densities,
        )

    @property
    def _numba_supported(self) -> bool:
        """True when this model can be used with the fixed-step RK4 path."""
        return (
            self._beta > 0.0
            and self._beta != float("inf")
            and self._density_alts is not None
            and self._density_vals is not None
        )

    def acceleration(
        self,
        position: NDArray[np.float64],
        velocity: NDArray[np.float64],
    ) -> Vec3:
        v_mag2 = (
            float(velocity[0] * velocity[0])
            + float(velocity[1] * velocity[1])
            + float(velocity[2] * velocity[2])
        )
        v_mag = sqrt(v_mag2)
        if v_mag < 1e-6 or self._beta <= 0.0 or self._beta == float("inf"):
            return (0.0, 0.0, 0.0)

        d = position - self._center
        dist = float(sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]))
        altitude = dist - self._sea_r
        rho = self._density(altitude)
        if rho <= 0.0:
            return (0.0, 0.0, 0.0)

        drag = -0.5 * rho * v_mag / self._beta
        return (
            float(drag * velocity[0]),
            float(drag * velocity[1]),
            float(drag * velocity[2]),
        )


@dataclass(frozen=True)
class ImpactResult:
    """Ballistic impact prediction.  When an :class:`AeroModel` is attached,
    the trajectory includes aerodynamic drag as well."""

    position: Vec3
    """Surface impact point in the target reference frame (metres)."""

    time: float
    """Time to impact (seconds)."""


class LandingPredictor:
    """RK45 ballistic trajectory integrator in a rotating frame.

    Keyword Args:
        mu: Standard gravitational parameter (m^3/s^2).
        omega: Planet rotation vector in the target frame (rad/s).
        body_center: Position of the body centre in the target frame (m).
        body_radius: Surface radius at the target (m).
        aero: Optional aerodynamic model whose
            :meth:`AeroModel.acceleration` is evaluated at every RK45 step.
    """

    def __init__(
        self,
        *,
        mu: float,
        omega: Sequence[float],
        body_center: Sequence[float],
        body_radius: float,
        aero: AeroModel | None = None,
    ) -> None:
        self._mu = float(mu)
        self._omega = np.asarray(omega, dtype=float)
        self._center = np.asarray(body_center, dtype=float)
        self._radius = float(body_radius)
        self._aero = aero

    @classmethod
    def from_body(
        cls,
        body: Any,
        target_frame: Any,
        lat: float,
        lon: float,
        aero: AeroModel | None = None,
    ) -> LandingPredictor:
        """Construct a predictor from a kRPC ``CelestialBody``.

        Queries the body for gravitational parameter, rotation vector,
        surface radius, and body-centre position — all expressed in
        *target_frame*.
        """
        omega = tuple(
            np.array(body.direction(target_frame)) * body.rotational_speed
        )
        center = tuple(np.array(body.position(target_frame)))
        radius = float(
            body.equatorial_radius + body.surface_height(lat, lon)
        )
        return cls(
            mu=float(body.gravitational_parameter),
            omega=omega,
            body_center=center,
            body_radius=radius,
            aero=aero,
        )

    def predict(
        self,
        *,
        position: Vector3 | Sequence[float],
        velocity: Vector3 | Sequence[float],
        t_max: float = 600.0,
        rtol: float = 1e-9,
        atol: float = 1e-9,
    ) -> ImpactResult | None:
        """Integrate until the surface sphere is reached.

        Args:
            position: Current position in the target frame (m).
            velocity: Current velocity in the target frame (m/s).
            t_max: Maximum integration time (s).
            rtol, atol: scipy RK45 tolerances (ignored by numba path).

        Returns:
            ``ImpactResult``, or ``None`` if the surface is never reached
            within *t_max*.
        """
        r0 = np.asarray(position, dtype=float)
        v0 = np.asarray(velocity, dtype=float)

        result = self._predict_numba(r0, v0, t_max)
        if result is not None:
            return result

        y0 = np.concatenate([r0, v0])
        solution = solve_ivp(
            fun=self._dynamics,
            t_span=(0.0, t_max),
            y0=y0,
            method="RK45",
            events=self._surface_event,
            rtol=rtol,
            atol=atol,
            dense_output=False,
        )
        if solution.t_events is None or len(solution.t_events[0]) == 0:
            return None
        t_impact = float(solution.t_events[0][-1])
        r_impact = solution.y_events[0][-1, 0:3]
        return ImpactResult(
            position=(float(r_impact[0]), float(r_impact[1]), float(r_impact[2])),
            time=t_impact,
        )

    def _predict_numba(
        self,
        r0: np.ndarray,
        v0: np.ndarray,
        t_max: float,
    ) -> ImpactResult | None:
        """Fixed-step RK4 fast path, used when a :class:`DragModel` with
        pre-sampled density tables is attached."""
        aero = self._aero
        if not isinstance(aero, DragModel) or not aero._numba_supported:
            return None

        dt = 0.04
        max_n = int(t_max / dt)
        hit = _rk4_fixed(
            r0,
            v0,
            self._mu,
            self._omega,
            self._center,
            self._radius,
            aero._beta,
            aero._density_alts,  # type: ignore[arg-type]  # guarded by _numba_supported
            aero._density_vals,  # type: ignore[arg-type]
            aero._sea_r,
            dt,
            max_n,
        )
        if hit is None:
            return None
        return ImpactResult(
            position=(float(hit[0]), float(hit[1]), float(hit[2])),
            time=float(hit[3]),
        )

    def predict_from(
        self,
        state: FlightState,
        *,
        t_max: float = 600.0,
        rtol: float = 1e-9,
        atol: float = 1e-9,
    ) -> ImpactResult | None:
        """Equivalent of :meth:`predict` reading position and velocity
        directly from a :class:`FlightState` snapshot.
        """
        return self.predict(
            position=state.position,
            velocity=state.velocity,
            t_max=t_max,
            rtol=rtol,
            atol=atol,
        )

    # -- internal -----------------------------------------------------------

    def _dynamics(self, _t: float, y: NDArray[np.float64]) -> list[float]:
        r = y[0:3]
        v = y[3:6]
        d = r - self._center
        dist = float(sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]))
        inv_cube = 1.0 / (dist * dist * dist)
        g_vec = -self._mu * inv_cube * d
        coriolis = -2.0 * np.cross(self._omega, v)
        d_center = r - self._center
        centrifugal = -np.cross(self._omega, np.cross(self._omega, d_center))
        a = g_vec + coriolis + centrifugal
        if self._aero is not None:
            a += np.array(self._aero.acceleration(r, v), dtype=float)
        return [v[0], v[1], v[2], float(a[0]), float(a[1]), float(a[2])]

    def _surface_event(self, _t: float, y: NDArray[np.float64]) -> float:
        r = y[0:3]
        d = r - self._center
        return float(sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])) - self._radius
    _surface_event.terminal = True  # type: ignore[attr-defined]
    _surface_event.direction = -1.0  # type: ignore[attr-defined]
