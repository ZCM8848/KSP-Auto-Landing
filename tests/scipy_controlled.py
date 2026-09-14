"""Scipy reference integrator for :class:`ControlledPredictor`.

This is a *variable-mass* continuous-state reference used only by the test
suite to quantify the operator-splitting error of the numba frozen-mass RK4
kernel (:func:`recovery.guidance._numba.rk4_controlled`).

It integrates the full ``[r, v, m]`` state with adaptive
:func:`scipy.integrate.solve_ivp` (RK45) at tight tolerances, treating mass
as a *continuous* state variable (thrust / aero accelerations use the
instantaneous mass).  The production kernel, by contrast, holds mass constant
within each fixed RK4 step and applies a forward-Euler mass-flow update
afterwards — a scheme that is **1st-order (linear) in ``dt``** in the powered
regime (not 4th-order): the step-top mass is systematically higher than the
step-average, biasing the thrust acceleration low by an amount proportional to
``dt``; integrated over the burn that constant bias yields a position error
proportional to ``dt``.

The dynamics, throttle rule, steering rule and segment-firing logic mirror
the numba kernel bit-for-bit (``_accel_ctrl_jit`` / ``_throttle_jit`` /
``_nose_jit`` / the segment-event loop in :func:`rk4_controlled`); only the
mass treatment and the adaptive integration differ by design.  Segment firing
is re-evaluated idempotently on every RHS call (the active segment is the
highest-index segment whose trigger is satisfied), which is the correct
formulation for an adaptive integrator and matches the numba latch for
monotonic-trigger schedules.

It is **not** wired into the production guidance stack — it lives under
``tests/`` purely as a numerical ground truth.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.integrate import solve_ivp

from recovery.guidance.aerodynamics import LiftDragModel, LiftTableModel
from recovery.guidance.control import (
    ConstantThrottle,
    ControlSegment,
    FixedNose,
    RetrogradeNose,
    TowardTargetNose,
    TriggerKind,
    VirtualControl,
)
from recovery.guidance.controlled import ImpactResult
from recovery.specs import BodySpec
from recovery.types import Vector3


@dataclass
class ScipyResult:
    """Reference trajectory summary (crossing state + full history)."""

    impact: ImpactResult | None
    """Surface-crossing state, or ``None`` if not reached."""

    endpoint: ImpactResult | None
    """``v·up = 0`` crossing state (only when ``stop_on_endpoint``)."""

    final_position: NDArray[np.float64]
    final_velocity: NDArray[np.float64]
    final_mass: float

    times: NDArray[np.float64]
    positions: NDArray[np.float64]
    velocities: NDArray[np.float64]
    masses: NDArray[np.float64]


class ScipyControlledPredictor:
    """Adaptive RK45 reference for the controlled-trajectory dynamics.

    Mirrors :class:`recovery.guidance.ControlledPredictor` but integrates
    ``[position, velocity, mass]`` as a continuous state with
    :func:`scipy.integrate.solve_ivp`, so the thrust and aerodynamic
    accelerations use the *instantaneous* mass — unlike the numba kernel,
    which freezes mass within each fixed RK4 step.
    """

    def __init__(
        self,
        *,
        mu: float,
        omega: Sequence[float],
        body_center: Sequence[float],
        body_radius: float,
        aero: LiftDragModel | LiftTableModel | None = None,
        rtol: float = 1e-9,
        atol: float = 1e-9,
        t_max: float = 600.0,
    ) -> None:
        self._mu = float(mu)
        self._omega = np.asarray(omega, dtype=float)
        self._center = np.asarray(body_center, dtype=float)
        self._radius = float(body_radius)
        self._aero = aero
        self._rtol = float(rtol)
        self._atol = float(atol)
        self._t_max = float(t_max)

        c = float(np.linalg.norm(self._center))
        self._up = (-self._center / c) if c > 1e-9 else np.array([0.0, 0.0, 1.0])

        # sea-level radius used for the altitude-trigger convention.  When an
        # aero model is attached, mirror ControlledPredictor and read it from
        # the model's numba_params (the same value the numba kernel uses);
        # otherwise fall back to the body radius.
        if aero is not None:
            params = aero.numba_params
            self._sea_r = float(params[-1]) if params is not None else float(body_radius)
        else:
            self._sea_r = float(body_radius)

    @classmethod
    def from_body_spec(
        cls,
        spec: BodySpec,
        aero: LiftDragModel | LiftTableModel | None = None,
        *,
        rtol: float = 1e-9,
        atol: float = 1e-9,
        t_max: float = 600.0,
    ) -> ScipyControlledPredictor:
        """Construct from a pure-data :class:`BodySpec` (no network calls)."""
        return cls(
            mu=spec.mu,
            omega=spec.omega,
            body_center=spec.body_center,
            body_radius=spec.body_radius,
            aero=aero,
            rtol=rtol,
            atol=atol,
            t_max=t_max,
        )

    def predict(
        self,
        position: Vector3 | Sequence[float],
        velocity: Vector3 | Sequence[float],
        mass: float,
        control: VirtualControl,
        *,
        stop_on_endpoint: bool = False,
        rtol: float | None = None,
        atol: float | None = None,
        t_max: float | None = None,
    ) -> ScipyResult:
        """Propagate *control* with adaptive RK45 and return the trajectory."""
        if mass <= 0.0:
            raise ValueError(f"mass must be positive, got {mass}")

        segments = control.segments
        dry_mass = float(control.dry_mass)
        g0 = float(control.g0)
        center = self._center
        omega = self._omega
        mu = self._mu
        up = self._up
        sea_r = self._sea_r
        radius = self._radius
        aero = self._aero

        def active_seg(t: float, alt: float) -> ControlSegment:
            active = segments[0]
            for j in range(1, len(segments)):
                trig = segments[j].trigger
                if trig is None:
                    break
                if trig.kind == TriggerKind.TIME:
                    met = t >= trig.value
                else:
                    met = alt <= trig.value
                if met:
                    active = segments[j]
            return active

        def throttle(seg: ControlSegment, m: float, dist: float,
                      alt: float, v: NDArray[np.float64]) -> float:
            rule = seg.throttle
            if isinstance(rule, ConstantThrottle):
                return rule.throttle
            if seg.max_thrust <= 0.0 or m <= 0.0:
                return 0.0
            vrad = float(np.dot(v, up))
            if alt >= rule.h_terminal:
                denom = max(alt - rule.h_terminal, 1e-3)
                acc = (vrad * vrad - rule.v_terminal * rule.v_terminal) / (2.0 * denom)
                g_local = mu / (dist * dist)
                thr = m * (acc + g_local) / seg.max_thrust
            else:
                thr = 1.0
            return float(min(1.0, max(0.0, thr)))

        def nose_dir(seg: ControlSegment, r: NDArray[np.float64],
                     v: NDArray[np.float64]) -> NDArray[np.float64]:
            rule = seg.nose
            if isinstance(rule, RetrogradeNose):
                vm = float(np.linalg.norm(v))
                return up if vm < 1e-9 else -v / vm
            if isinstance(rule, FixedNose):
                d = np.asarray(rule.direction, dtype=float)
                dm = float(np.linalg.norm(d))
                return up if dm < 1e-9 else d / dm
            if isinstance(rule, TowardTargetNose):
                rm = float(np.linalg.norm(r))
                return up if rm < 1e-9 else -r / rm
            return up  # UpNose

        def rhs(_t: float, y: NDArray[np.float64]) -> NDArray[np.float64]:
            r = y[0:3]
            v = y[3:6]
            m = float(y[6])
            d = r - center
            dist = float(np.linalg.norm(d))
            alt = dist - sea_r
            seg = active_seg(_t, alt)
            thr = throttle(seg, m, dist, alt, v)
            n = nose_dir(seg, r, v)

            inv_cube = 1.0 / (dist * dist * dist)
            a = -mu * inv_cube * d
            a = a - 2.0 * np.cross(omega, v)
            a = a - np.cross(omega, np.cross(omega, d))

            m_eff = m if m > dry_mass else dry_mass
            if aero is not None and m_eff > 0.0:
                a = a + np.asarray(aero.acceleration(r, v, n, m_eff), dtype=float)
            if seg.max_thrust > 0.0 and thr > 0.0 and m_eff > 0.0:
                a = a + (thr * seg.max_thrust / m_eff) * n

            if seg.max_thrust > 0.0 and seg.isp > 0.0 and m > dry_mass:
                mdot = thr * seg.max_thrust / (seg.isp * g0)
            else:
                mdot = 0.0

            out = np.empty(7, dtype=float)
            out[0:3] = v
            out[3:6] = a
            out[6] = -mdot
            return out

        def surface_event(_t: float, y: NDArray[np.float64]) -> float:
            return float(np.linalg.norm(y[0:3] - center) - radius)

        surface_event.terminal = True  # type: ignore[attr-defined]
        surface_event.direction = -1.0  # type: ignore[attr-defined]

        def endpoint_event(_t: float, y: NDArray[np.float64]) -> float:
            return float(np.dot(y[3:6], up))

        endpoint_event.terminal = True  # type: ignore[attr-defined]
        endpoint_event.direction = 1.0  # type: ignore[attr-defined]

        events: list[Any] = [surface_event]
        if stop_on_endpoint:
            events.append(endpoint_event)

        r0 = np.asarray(position, dtype=float)
        v0 = np.asarray(velocity, dtype=float)
        y0 = np.concatenate([r0, v0, [float(mass)]])
        use_rtol = self._rtol if rtol is None else float(rtol)
        use_atol = self._atol if atol is None else float(atol)
        use_tmax = self._t_max if t_max is None else float(t_max)

        sol = solve_ivp(
            rhs,
            (0.0, use_tmax),
            y0,
            method="RK45",
            rtol=use_rtol,
            atol=use_atol,
            events=events,
            dense_output=False,
        )

        impact: ImpactResult | None = None
        endpoint: ImpactResult | None = None
        if sol.t_events is not None:
            se = sol.t_events[0]
            if se is not None and len(se) > 0:
                ye = sol.y_events[0][-1]
                impact = ImpactResult(
                    position=(float(ye[0]), float(ye[1]), float(ye[2])),
                    time=float(se[-1]),
                )
            if stop_on_endpoint and len(sol.t_events) > 1:
                ee = sol.t_events[1]
                if ee is not None and len(ee) > 0:
                    ye = sol.y_events[1][-1]
                    endpoint = ImpactResult(
                        position=(float(ye[0]), float(ye[1]), float(ye[2])),
                        time=float(ee[-1]),
                    )

        return ScipyResult(
            impact=impact,
            endpoint=endpoint,
            final_position=sol.y[0:3, -1].copy(),
            final_velocity=sol.y[3:6, -1].copy(),
            final_mass=float(sol.y[6, -1]),
            times=sol.t.copy(),
            positions=sol.y[0:3, :].T.copy(),
            velocities=sol.y[3:6, :].T.copy(),
            masses=sol.y[6, :].copy(),
        )
