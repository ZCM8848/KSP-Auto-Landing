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
    ) -> None:
        self._beta = float(ballistic_coefficient)
        self._density = density_fn
        self._center = np.asarray(body_center, dtype=float)
        self._sea_r = float(sea_level_radius)

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
            rtol, atol: scipy RK45 tolerances.

        Returns:
            ``ImpactResult``, or ``None`` if the surface is never reached
            within *t_max*.
        """
        r0 = np.asarray(position, dtype=float)
        v0 = np.asarray(velocity, dtype=float)
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
