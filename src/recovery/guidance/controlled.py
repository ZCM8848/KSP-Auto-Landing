"""Controlled trajectory predictor (virtual control -> full trajectory).

:class:`ControlledPredictor` propagates a vessel under a piecewise-constant
:class:`~recovery.guidance.control.VirtualControl` — engine events (throttle /
engine-set thrust / Isp changes at time or altitude triggers) plus a rule-based
steering law — and returns the *entire* state trajectory as feedback, not just
the impact point.

This is an independent implementation that supersedes the ballistic-only
:class:`~recovery.guidance.LandingPredictor` for anything that involves thrust
or attitude.  The heavy lifting is the numba kernel
:func:`~recovery.guidance._numba.rk4_controlled`.
"""

from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Sequence

import numpy as np

from ..specs import BodySpec
from ..types import Vector3
from ._numba import rk4_controlled
from .aerodynamics import LiftDragModel, LiftTableModel
from .control import (
    BrakeToThrottle,
    ConstantThrottle,
    FixedNose,
    RetrogradeNose,
    TowardTargetNose,
    UpNose,
    VirtualControl,
)
from .predictor import ImpactResult


@dataclass
class Trajectory:
    r"""The full predicted state history returned as feedback.

    All arrays are aligned: row ``i`` is the state at ``times[i]``.  ``impact``
    is populated (with the exact crossing state) only when the surface was
    reached within ``t_max``; ``endpoint`` is populated when the propagation was
    stopped at the ``v \cdot up = 0`` crossing requested by ``stop_on_endpoint``.
    """

    times: np.ndarray
    """Time at each step (s)."""

    positions: np.ndarray
    """Positions (m) in the target frame, shape ``(N, 3)``."""

    velocities: np.ndarray
    """Velocities (m/s) in the target frame, shape ``(N, 3)``."""

    masses: np.ndarray
    """Masses (kg), shape ``(N,)``."""

    impact: ImpactResult | None
    """Surface-crossing state, or ``None`` if no impact within ``t_max``."""

    endpoint: ImpactResult | None = None
    r"""Endpoint state (``v \cdot up = 0``), or ``None`` if not requested/reached."""

    hit: bool = False
    """Whether the surface sphere was reached."""

    @property
    def n(self) -> int:
        """Number of recorded steps."""
        return int(self.times.shape[0])

    @property
    def final_position(self) -> np.ndarray:
        return self.positions[-1]

    @property
    def final_velocity(self) -> np.ndarray:
        return self.velocities[-1]

    @property
    def final_mass(self) -> float:
        return float(self.masses[-1])


class ControlledPredictor:
    """Propagate a :class:`VirtualControl` to a full trajectory.

    Keyword Args:
        mu: Standard gravitational parameter (m^3/s^2).
        omega: Planet rotation vector in the target frame (rad/s).
        body_center: Body-centre position in the target frame (m).
        body_radius: Surface radius at the landing target (m).
        drag: Optional :class:`DragModel` supplying the ballistic drag term.
        dt: Fixed integration step (s).
        t_max: Maximum propagation time (s).
    """

    def __init__(
        self,
        *,
        mu: float,
        omega: Sequence[float],
        body_center: Sequence[float],
        body_radius: float,
        aero: LiftDragModel | LiftTableModel | None = None,
        dt: float = 0.1,
        t_max: float = 600.0,
    ) -> None:
        if dt <= 0.0:
            raise ValueError(f"dt must be positive, got {dt}")
        self._mu = float(mu)
        self._omega = np.asarray(omega, dtype=float)
        self._center = np.asarray(body_center, dtype=float)
        self._radius = float(body_radius)
        self._aero = aero
        self._dt = float(dt)
        self._t_max = float(t_max)

        c = float(np.linalg.norm(self._center))
        self._up = (-self._center / c) if c > 1e-9 else np.array([0.0, 0.0, 1.0])

        # attitude-dependent aero params for the kernel (empty -> no aero)
        params = aero.numba_params if aero is not None else None
        if params is not None:
            alpha_pts, cl_table, cd_table, clamp_aoa, alts, vals, sea_r = params
            self._alpha_pts = np.asarray(alpha_pts, dtype=float)
            self._cl_table = np.asarray(cl_table, dtype=float)
            self._cd_table = np.asarray(cd_table, dtype=float)
            self._clamp = float(clamp_aoa)
            self._alts = np.asarray(alts, dtype=float)
            self._vals = np.asarray(vals, dtype=float)
            self._sea_r = float(sea_r)
        else:
            self._alpha_pts = np.zeros(0, dtype=float)
            self._cl_table = np.zeros(0, dtype=float)
            self._cd_table = np.zeros(0, dtype=float)
            self._clamp = 1.0
            self._alts = np.zeros(1, dtype=float)
            self._vals = np.zeros(1, dtype=float)
            self._sea_r = float(self._radius)

    @classmethod
    def from_body_spec(
        cls,
        spec: BodySpec,
        aero: LiftDragModel | LiftTableModel | None = None,
        *,
        dt: float = 0.1,
        t_max: float = 600.0,
    ) -> "ControlledPredictor":
        """Construct from a pure-data :class:`BodySpec` (no network calls)."""
        return cls(
            mu=spec.mu,
            omega=spec.omega,
            body_center=spec.body_center,
            body_radius=spec.body_radius,
            aero=aero,
            dt=dt,
            t_max=t_max,
        )

    @property
    def up(self) -> np.ndarray:
        """Local up direction at the target, in the target frame."""
        return self._up

    def predict(
        self,
        position: Vector3 | Sequence[float],
        velocity: Vector3 | Sequence[float],
        mass: float,
        control: VirtualControl,
        *,
        dt: float | None = None,
        t_max: float | None = None,
        stop_on_endpoint: bool = False,
    ) -> Trajectory:
        r"""Propagate *control* from the given state and return the trajectory.

        Args:
            position: Initial position in the target frame (m).
            velocity: Initial velocity in the target frame (m/s).
            mass: Initial total mass (kg).
            control: The virtual control to follow.
            dt: Override the fixed integration step (s).
            t_max: Override the maximum propagation time (s).
            stop_on_endpoint: If ``True``, stop at the first ``v \cdot up = 0``
                crossing from below instead of continuing to the surface.
        """
        if mass <= 0.0:
            raise ValueError(f"mass must be positive, got {mass}")

        r0 = np.asarray(position, dtype=float)
        v0 = np.asarray(velocity, dtype=float)
        use_dt = self._dt if dt is None else float(dt)
        use_tmax = self._t_max if t_max is None else float(t_max)
        if use_dt <= 0.0:
            raise ValueError(f"dt must be positive, got {use_dt}")
        max_n = int(use_tmax / use_dt) + 1

        segs = control.segments
        n_seg = len(segs)
        seg_trigger_val = np.zeros(n_seg, dtype=float)
        seg_trigger_kind = np.zeros(n_seg, dtype=np.int8)
        seg_throttle_kind = np.zeros(n_seg, dtype=np.int8)
        seg_throttle_p1 = np.zeros(n_seg, dtype=float)
        seg_throttle_p2 = np.zeros(n_seg, dtype=float)
        seg_thrust = np.zeros(n_seg, dtype=float)
        seg_isp = np.zeros(n_seg, dtype=float)
        seg_nose_kind = np.zeros(n_seg, dtype=np.int8)
        seg_nose_fixed = np.zeros((n_seg, 3), dtype=float)

        for i, seg in enumerate(segs):
            match seg.throttle:
                case ConstantThrottle(throttle=t):
                    seg_throttle_kind[i] = 0
                    seg_throttle_p1[i] = t
                    seg_throttle_p2[i] = 0.0
                case BrakeToThrottle(v_terminal=v, h_terminal=h):
                    seg_throttle_kind[i] = 1
                    seg_throttle_p1[i] = v
                    seg_throttle_p2[i] = h
            seg_thrust[i] = seg.max_thrust
            seg_isp[i] = seg.isp
            match seg.nose:
                case RetrogradeNose():
                    seg_nose_kind[i] = 0
                case UpNose():
                    seg_nose_kind[i] = 1
                case FixedNose(direction=d):
                    seg_nose_kind[i] = 2
                    seg_nose_fixed[i, 0] = d[0]
                    seg_nose_fixed[i, 1] = d[1]
                    seg_nose_fixed[i, 2] = d[2]
                case TowardTargetNose():
                    seg_nose_kind[i] = 3
            if i == 0:
                # initial segment: active from t=0, trigger unused
                seg_trigger_val[i] = 0.0
                seg_trigger_kind[i] = 0
            else:
                trig = seg.trigger
                if trig is None:
                    raise ValueError("non-initial segments require a trigger")
                seg_trigger_val[i] = trig.value
                seg_trigger_kind[i] = int(trig.kind)

        out = np.empty((max_n, 8), dtype=float)
        n_steps, hit = rk4_controlled(
            r0, v0, float(mass),
            self._mu, self._omega, self._center, self._radius,
            self._clamp, self._alpha_pts, self._cl_table, self._cd_table,
            self._alts, self._vals, self._sea_r,
            seg_trigger_val, seg_trigger_kind, seg_throttle_kind,
            seg_throttle_p1, seg_throttle_p2, seg_thrust,
            seg_isp, seg_nose_kind, seg_nose_fixed, self._up,
            float(control.dry_mass), float(control.g0),
            use_dt, max_n, out,
            bool(stop_on_endpoint),
        )

        times = out[:n_steps, 0]
        positions = out[:n_steps, 1:4]
        velocities = out[:n_steps, 4:7]
        masses = out[:n_steps, 7]

        impact = None
        endpoint = None
        if hit == 1:
            impact = ImpactResult(
                position=(float(positions[-1, 0]), float(positions[-1, 1]),
                          float(positions[-1, 2])),
                time=float(times[-1]),
            )
        elif hit == 2:
            endpoint = ImpactResult(
                position=(float(positions[-1, 0]), float(positions[-1, 1]),
                          float(positions[-1, 2])),
                time=float(times[-1]),
            )

        return Trajectory(
            times=times,
            positions=positions,
            velocities=velocities,
            masses=masses,
            impact=impact,
            endpoint=endpoint,
            hit=bool(hit == 1),
        )
