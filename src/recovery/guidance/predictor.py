"""Landing point predictor via RK45 numerical integration.

Integrates the equations of motion in the rotating (planet-fixed) target
reference frame from a given position and velocity until the vessel
crosses the surface sphere.  The dynamics include:

* altitude-decaying inverse-square gravity  (mu / r^2)
* Coriolis acceleration  (-2 * omega x v)
* centrifugal acceleration  (-omega x (omega x r))
* optional aerodynamic acceleration from an :class:`AeroModel` (e.g.
  :class:`DragModel` for per-step kRPC-based drag simulation)

The recommended constructor is :meth:`LandingPredictor.from_body_spec`, which
takes a pure-data :class:`~recovery.specs.BodySpec` (produced by the KSP
isolation layer via :func:`recovery.ksp.sampling.sample_body_spec`); callers
can then use :meth:`predict` with :class:`Vector3` values or
:meth:`predict_from` with a :class:`FlightState` snapshot.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from math import sqrt

import numpy as np
from numpy.typing import NDArray
from scipy.integrate import solve_ivp

from ..specs import BodySpec
from ..types import FlightState, Vector3
from ._numba import rk4_fixed
from .aerodynamics import AeroModel, DragModel

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class ImpactResult:
    """Ballistic impact prediction.  When an :class:`AeroModel` is attached,
    the trajectory includes aerodynamic drag as well."""

    position: Vec3
    """Surface impact point in the target reference frame (metres)."""

    time: float
    """Time to impact (seconds)."""


class LandingPredictor:
    """Ballistic trajectory integrator in a rotating frame.

    Two integration paths are available:

    * scipy RK45 adaptive integrator (default fallback, respects *rtol/atol*);
    * numba fixed-step RK4 fast path (used when a :class:`DragModel` with
      pre-sampled density tables is attached).

    The fixed-step path is configured by *dt*; smaller values increase accuracy
    at the cost of more steps.

    Keyword Args:
        mu: Standard gravitational parameter (m^3/s^2).
        omega: Planet rotation vector in the target frame (rad/s).
        body_center: Position of the body centre in the target frame (m).
        body_radius: Surface radius at the target (m).
        aero: Optional aerodynamic model whose
            :meth:`AeroModel.acceleration` is evaluated at every RK45 step.
        dt: Default fixed-step size (s) for the numba RK4 path.
    """

    def __init__(
        self,
        *,
        mu: float,
        omega: Sequence[float],
        body_center: Sequence[float],
        body_radius: float,
        aero: AeroModel | None = None,
        dt: float = 0.04,
    ) -> None:
        if dt <= 0.0:
            raise ValueError(f"dt must be positive, got {dt}")
        self._mu = float(mu)
        self._omega = np.asarray(omega, dtype=float)
        self._center = np.asarray(body_center, dtype=float)
        self._radius = float(body_radius)
        self._aero = aero
        self._dt = float(dt)

    @classmethod
    def from_body_spec(
        cls,
        spec: BodySpec,
        aero: AeroModel | None = None,
    ) -> LandingPredictor:
        """Construct a predictor from a pure-data :class:`BodySpec`.

        The spec is produced by :func:`recovery.ksp.sampling.sample_body_spec`
        (which performs the one-shot kRPC sampling); this constructor is pure
        and makes no network calls.
        """
        return cls(
            mu=spec.mu,
            omega=spec.omega,
            body_center=spec.body_center,
            body_radius=spec.body_radius,
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
        dt: float | None = None,
    ) -> ImpactResult | None:
        """Integrate until the surface sphere is reached.

        Args:
            position: Current position in the target frame (m).
            velocity: Current velocity in the target frame (m/s).
            t_max: Maximum integration time (s).
            rtol: Relative tolerance for the scipy RK45 path (ignored by
                the numba path).
            atol: Absolute tolerance for the scipy RK45 path (ignored by
                the numba path).
            dt: Fixed-step size (s) for the numba RK4 path.  Overrides the
                constructor default.  Ignored by the scipy fallback.

        Returns:
            ``ImpactResult``, or ``None`` if the surface is never reached
            within *t_max*.
        """
        if dt is not None and dt <= 0.0:
            raise ValueError(f"dt must be positive, got {dt}")

        r0 = np.asarray(position, dtype=float)
        v0 = np.asarray(velocity, dtype=float)

        result = self._predict_numba(r0, v0, t_max, dt=dt)
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
        *,
        dt: float | None = None,
    ) -> ImpactResult | None:
        """Fixed-step RK4 fast path, used when a :class:`DragModel` with
        pre-sampled density tables is attached."""
        aero = self._aero
        if not isinstance(aero, DragModel):
            return None
        params = aero.numba_params
        if params is None:
            return None
        beta, alts, vals, sea_r = params

        use_dt = self._dt if dt is None else float(dt)
        if use_dt <= 0.0:
            raise ValueError(f"dt must be positive, got {use_dt}")
        max_n = int(t_max / use_dt) + 1
        hit = rk4_fixed(
            r0,
            v0,
            self._mu,
            self._omega,
            self._center,
            self._radius,
            beta,
            alts,
            vals,
            sea_r,
            use_dt,
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
        dt: float | None = None,
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
            dt=dt,
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
        centrifugal = -np.cross(self._omega, np.cross(self._omega, d))
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
