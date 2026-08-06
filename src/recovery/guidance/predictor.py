"""Landing point predictor via RK45 numerical integration.

Integrates the equations of motion in the rotating (planet-fixed) target
reference frame from a given position and velocity until the vessel
crosses the surface sphere.  The dynamics include:

* altitude-decaying inverse-square gravity  (mu / r^2)
* Coriolis acceleration  (-2 * omega x v)
* centrifugal acceleration  (-omega x (omega x r))

The recommended constructor is :meth:`LandingPredictor.from_body`, which
derives all planetary constants from a kRPC ``CelestialBody``; callers can
then use :meth:`predict` with :class:`Vector3` values or
:meth:`predict_from` with a :class:`FlightState` snapshot.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from math import sqrt
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.integrate import solve_ivp

from ..ksp.types import FlightState, Vector3

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class ImpactResult:
    """Ballistic impact prediction (no atmosphere)."""

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
    """

    def __init__(
        self,
        *,
        mu: float,
        omega: Sequence[float],
        body_center: Sequence[float],
        body_radius: float,
    ) -> None:
        self._mu = float(mu)
        self._omega = np.asarray(omega, dtype=float)
        self._center = np.asarray(body_center, dtype=float)
        self._radius = float(body_radius)

    @classmethod
    def from_body(
        cls,
        body: Any,
        target_frame: Any,
        lat: float,
        lon: float,
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
        return [v[0], v[1], v[2], float(a[0]), float(a[1]), float(a[2])]

    def _surface_event(self, _t: float, y: NDArray[np.float64]) -> float:
        r = y[0:3]
        d = r - self._center
        return float(sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])) - self._radius
    _surface_event.terminal = True  # type: ignore[attr-defined]
    _surface_event.direction = -1.0  # type: ignore[attr-defined]
