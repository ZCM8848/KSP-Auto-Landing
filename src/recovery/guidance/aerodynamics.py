"""Attitude-dependent aerodynamic models for trajectory prediction.

Two concrete models, both mass-aware (``a = F / m`` with *mass* passed
explicitly so variable-mass flight is correct):

* :class:`LiftDragModel` — linearised drag + induced drag + body lift
  (``C_D(α) = C_D0·(1 + k·α²)``, ``C_L(α) = C_Lα·α``); also the drag-only
  model used for ballistic / boosterback coast prediction
  (``cl_area = 0``, ``k_ind = 0``).
* :class:`LiftTableModel` — nonlinear lift / drag via discrete coefficient
  tables (preserves stalling), the production aero-glide / landing model.

Both expose ``acceleration(position, velocity, nose, mass)`` and a
``numba_params`` tuple for the fixed-step RK4 kernel.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from math import acos

import numpy as np
from numpy.typing import NDArray

from ..specs import DragSpec

Vec3 = tuple[float, float, float]


class LiftDragModel:
    """Attitude-dependent aerodynamic model (drag + induced drag + body lift).

    The body axis ``nose`` sets the angle of attack against the velocity,
    producing zero-lift + induced drag along ``-v`` and body lift perpendicular
    to ``v`` toward the nose side.  It is what makes aerodynamic steering (the
    air-guidance strategy) simulatable: tilting the nose changes the lift, which
    changes the trajectory.

    With ``cl_area = 0`` and ``k_ind = 0`` the model reduces to drag-only
    (AoA = 0 under a retrograde nose) — the ballistic / boosterback coast
    case.

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
    ) -> LiftDragModel:
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
