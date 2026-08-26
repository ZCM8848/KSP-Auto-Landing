"""Aerodynamic models for impact prediction.

Defines the pluggable :class:`AeroModel` interface and two concrete
implementations:

* :class:`KrpcAeroModel` — per-step kRPC force simulation (online).
* :class:`DragModel` — pre-sampled density profile + ballistic coefficient
  (offline, thread-safe).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable, Sequence
from math import acos, sqrt
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from ..specs import DragSpec

Vec3 = tuple[float, float, float]
Vec4 = tuple[float, float, float, float]


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


class AeroModel(ABC):
    """Aerodynamic acceleration contribution for use inside
    :class:`~recovery.guidance.LandingPredictor` integration.
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
    reference frame that the surrounding :class:`~recovery.guidance.LandingPredictor` uses.
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

    Use :meth:`from_spec` to build a model from a pre-sampled
    :class:`~recovery.specs.DragSpec`; after construction the model is fully
    offline and safe to evaluate from any thread.
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
    def from_spec(cls, spec: DragSpec) -> DragModel:
        """Construct a model from a pure-data :class:`DragSpec`.

        The spec is produced by
        :func:`recovery.ksp.sampling.sample_drag_spec` (which performs the
        one-shot kRPC density/coefficient sampling); this constructor is pure
        and makes no network calls.
        """
        alts = spec.density_alts
        vals = spec.density_vals
        depth = float(alts[-1])

        def _interp(h: float) -> float:
            if h < 0.0:
                return float(vals[0])
            if h > depth:
                return 0.0
            return float(np.interp(h, alts, vals))

        return cls(
            ballistic_coefficient=spec.ballistic_coefficient,
            density_fn=_interp,
            body_center=spec.body_center,
            sea_level_radius=spec.sea_level_radius,
            density_alts=alts,
            density_vals=vals,
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

    @property
    def numba_params(
        self,
    ) -> tuple[float, np.ndarray, np.ndarray, float] | None:
        """Return ``(beta, density_alts, density_vals, sea_level_radius)``
        for the fixed-step RK4 fast path, or ``None`` when the model cannot
        be evaluated by the numba kernel.

        This is the only supported way for :class:`~recovery.guidance.LandingPredictor`
        to feed a :class:`DragModel` into its numba fast path — callers must
        not reach into the private fields directly.
        """
        if not self._numba_supported:
            return None
        alts = self._density_alts
        vals = self._density_vals
        if alts is None or vals is None:
            return None
        return (self._beta, alts, vals, self._sea_r)

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


class LiftDragModel:
    """Attitude-dependent aerodynamic model (drag + induced drag + body lift).

    Unlike :class:`DragModel` (zero-lift, attitude-independent), this model
    produces a force that depends on the body axis ``nose``: the angle of
    attack against the velocity gives zero-lift + induced drag along ``-v``
    and body lift perpendicular to ``v`` toward the nose side.  It is what
    makes aerodynamic steering (the air-guidance strategy) simulatable: tilting
    the nose changes the lift, which changes the trajectory.

    Parameters (SI):
        cd0_area: zero-lift drag area ``C_D0·A`` (m²).
        cl_area: lift slope ``C_Lα·A`` (m²/rad).
        k_ind: induced-drag factor ``C_D(α) = C_D0(1 + k_ind·α²)`` (1/rad²).
        clamp_aoa: stall clamp for the force model (rad).
        density_fn: ``ρ(altitude)`` callable, or supply ``density_alts``/
            ``density_vals`` for the numba fast path.
        body_center / sea_level_radius: geometry for altitude.
    """

    def __init__(
        self,
        *,
        cd0_area: float,
        cl_area: float = 0.0,
        k_ind: float = 0.0,
        clamp_aoa: float = 1.0,
        density_fn: Callable[[float], float],
        body_center: Sequence[float],
        sea_level_radius: float,
        density_alts: np.ndarray | None = None,
        density_vals: np.ndarray | None = None,
    ) -> None:
        self._cd0a = float(cd0_area)
        self._cla = float(cl_area)
        self._kind = float(k_ind)
        self._clamp = float(clamp_aoa)
        self._density = density_fn
        self._center = np.asarray(body_center, dtype=float)
        self._sea_r = float(sea_level_radius)
        self._alts = density_alts
        self._vals = density_vals
        # Linear coefficient tables for the numba kernel (equivalent to the
        # closed-form lift/drag below; interpolated at runtime).
        self._alpha_pts = np.linspace(0.0, self._clamp, 33)
        self._cl_table = self._cla * self._alpha_pts
        self._cd_table = self._cd0a * (1.0 + self._kind * self._alpha_pts ** 2)

    @classmethod
    def from_drag_spec(
        cls,
        spec: DragSpec,
        *,
        mass_ref: float,
        cl_area: float = 0.0,
        k_ind: float = 0.0,
        clamp_aoa: float = 1.0,
    ) -> "LiftDragModel":
        """Build from a :class:`DragSpec`; ``cd0_area = mass_ref / β``."""
        alts = spec.density_alts
        vals = spec.density_vals
        depth = float(alts[-1])

        def _interp(h: float) -> float:
            if h < 0.0:
                return float(vals[0])
            if h > depth:
                return 0.0
            return float(np.interp(h, alts, vals))

        return cls(
            cd0_area=mass_ref / float(spec.ballistic_coefficient),
            cl_area=cl_area,
            k_ind=k_ind,
            clamp_aoa=clamp_aoa,
            density_fn=_interp,
            body_center=spec.body_center,
            sea_level_radius=spec.sea_level_radius,
            density_alts=alts,
            density_vals=vals,
        )

    def acceleration(
        self,
        position: NDArray[np.float64],
        velocity: NDArray[np.float64],
        nose: NDArray[np.float64],
        mass: float,
    ) -> Vec3:
        """Aerodynamic acceleration (m/s²) at the given state and body axis."""
        v = np.asarray(velocity, dtype=float)
        n = np.asarray(nose, dtype=float)
        v_mag = float(np.linalg.norm(v))
        n_mag = float(np.linalg.norm(n))
        if v_mag < 1e-6 or n_mag < 1e-9 or mass <= 0.0:
            return (0.0, 0.0, 0.0)
        vh = v / v_mag
        n = n / n_mag
        d = np.asarray(position, dtype=float) - self._center
        alt = float(np.linalg.norm(d)) - self._sea_r
        rho = self._density(alt)
        if rho <= 0.0:
            return (0.0, 0.0, 0.0)
        c = float(np.clip(-np.dot(n, vh), -1.0, 1.0))
        alpha = min(acos(c), self._clamp)
        q = 0.5 * rho * v_mag * v_mag
        drag = q * self._cd0a * (1.0 + self._kind * alpha * alpha) / mass
        a = -drag * vh
        if alpha > 1e-4:
            nv = float(np.dot(n, vh))
            lat = n - nv * vh
            ln = float(np.linalg.norm(lat))
            if ln > 1e-9:
                lift = q * self._cla * alpha / mass
                a = a + lift * (lat / ln)
        return (float(a[0]), float(a[1]), float(a[2]))

    @property
    def numba_params(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, np.ndarray, np.ndarray, float] | None:
        """``(alpha_pts, cl_table, cd_table, clamp_aoa, alts, vals, sea_r)`` for
        the numba fast path, or ``None`` when the density table is unavailable."""
        if self._alts is None or self._vals is None:
            return None
        return (
            self._alpha_pts, self._cl_table, self._cd_table, self._clamp,
            self._alts, self._vals, self._sea_r,
        )


class LiftTableModel:
    """Nonlinear attitude-dependent aero model via coefficient tables.

    Stores lift / drag coefficients as discrete tables vs angle-of-attack and
    interpolates at runtime, reproducing the real (nonlinear, stalling)
    behaviour measured by :func:`recovery.ksp.sampling.sample_lift_table`.
    The lift table carries its own sign, so the lift direction is whatever the
    sampled data implies (e.g. tail-mounted fins -> lift opposite the
    nose-deflection side -> negative ``cl_table``).

    Parameters (SI):
        alpha_pts: Angle-of-attack samples (rad, ascending, from 0).
        cl_table: Lift coefficient ``C_L(α)·A`` at each α (m², signed).
        cd_table: Drag coefficient ``C_D(α)·A`` at each α (m²).
        clamp_aoa: Stall/table clamp for the force model (rad).
        density_fn / body_center / sea_level_radius: geometry, as
            :class:`LiftDragModel`.
        density_alts / density_vals: optional numba fast-path tables.
    """

    def __init__(
        self,
        *,
        alpha_pts: Sequence[float],
        cl_table: Sequence[float],
        cd_table: Sequence[float],
        clamp_aoa: float,
        density_fn: Callable[[float], float],
        body_center: Sequence[float],
        sea_level_radius: float,
        density_alts: np.ndarray | None = None,
        density_vals: np.ndarray | None = None,
    ) -> None:
        self._alpha_pts = np.asarray(alpha_pts, dtype=float)
        self._cl_table = np.asarray(cl_table, dtype=float)
        self._cd_table = np.asarray(cd_table, dtype=float)
        if not (self._alpha_pts.shape == self._cl_table.shape == self._cd_table.shape):
            raise ValueError("alpha_pts / cl_table / cd_table must be the same length")
        self._clamp = float(clamp_aoa)
        self._density = density_fn
        self._center = np.asarray(body_center, dtype=float)
        self._sea_r = float(sea_level_radius)
        self._alts = density_alts
        self._vals = density_vals

    def acceleration(
        self,
        position: NDArray[np.float64],
        velocity: NDArray[np.float64],
        nose: NDArray[np.float64],
        mass: float,
    ) -> Vec3:
        """Aerodynamic acceleration (m/s²) via table interpolation in α."""
        v = np.asarray(velocity, dtype=float)
        n = np.asarray(nose, dtype=float)
        v_mag = float(np.linalg.norm(v))
        n_mag = float(np.linalg.norm(n))
        if v_mag < 1e-6 or n_mag < 1e-9 or mass <= 0.0:
            return (0.0, 0.0, 0.0)
        vh = v / v_mag
        n = n / n_mag
        d = np.asarray(position, dtype=float) - self._center
        alt = float(np.linalg.norm(d)) - self._sea_r
        rho = self._density(alt)
        if rho <= 0.0:
            return (0.0, 0.0, 0.0)
        c = float(np.clip(-np.dot(n, vh), -1.0, 1.0))
        alpha = min(acos(c), self._clamp)
        q = 0.5 * rho * v_mag * v_mag
        cl = float(np.interp(alpha, self._alpha_pts, self._cl_table))
        cd = float(np.interp(alpha, self._alpha_pts, self._cd_table))
        a = -q * cd / mass * vh
        if alpha > 1e-4:
            nv = float(np.dot(n, vh))
            lat = n - nv * vh
            ln = float(np.linalg.norm(lat))
            if ln > 1e-9:
                a = a + q * cl / mass * (lat / ln)
        return (float(a[0]), float(a[1]), float(a[2]))

    @property
    def numba_params(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, np.ndarray, np.ndarray, float] | None:
        """``(alpha_pts, cl_table, cd_table, clamp_aoa, alts, vals, sea_r)`` for
        the numba fast path, or ``None`` when the density table is unavailable."""
        if self._alts is None or self._vals is None:
            return None
        return (
            self._alpha_pts, self._cl_table, self._cd_table, self._clamp,
            self._alts, self._vals, self._sea_r,
        )
