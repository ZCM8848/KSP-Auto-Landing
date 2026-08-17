"""G-FOLD fuel-optimal powered-descent planner (pure, KSP-free).

Wraps the ``gfold`` SOCP solver behind a small pure-data API so the guidance
layer never touches kRPC.  All solver inputs are either derived automatically
from a :class:`~recovery.types.FlightState` snapshot (mass, fuel, thrust,
specific impulse, initial position/velocity), supplied as the constant surface
gravity ``g0`` (constant because G-FOLD ignites close to the ground), or
exposed explicitly through :class:`GfoldParams` (target state, glide slope,
pointing limit, throttle bounds, discretisation, time-of-flight).

The reference-frame convention matches the target frame built by
:func:`recovery.ksp.reference_frames.create_target_reference_frame` and the
``gfold`` library itself: **+z is zenith (up)**, gravity is ``[0, 0, -g]``,
and the thrust acceleration points along +z.  A
:class:`~recovery.types.FlightState` can therefore be fed in directly with no
axis permutation.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, replace
from typing import Protocol, cast

import gfold
import numpy as np

from ..types import FlightState

G0 = 9.80665
"""Standard gravity (m/s^2), used to convert specific impulse to the
``fuel_consumption`` mass-flow coefficient ``1 / (Isp * G0)``."""


def _require_positive_isp(isp: float) -> float:
    """Return *isp* as a float, raising if it is not a positive finite value.

    A non-positive (or non-finite, e.g. NaN) specific impulse means the
    active engines are not firing (or the telemetry stream is stale).
    ``1 / (Isp * G0)`` would then either divide by zero or silently fabricate
    a nonsensical mass-flow rate, so we fail loudly instead of guessing a
    fallback.
    """
    isp = float(isp)
    if not math.isfinite(isp) or isp <= 0.0:
        raise ValueError(f"specific_impulse must be a positive finite value, got {isp}")
    return isp


Vec3 = tuple[float, float, float]

class Trajectory(Protocol):
    """Structural type for the ``gfold`` trajectory result.

    The ``gfold`` extension ships no stubs, so the fields we read are declared
    structurally: ``u_values`` (thrust-acceleration profiles) and
    ``time_points`` (sample times).  See ``gfold_Python_API文档.md``.
    """

    u_values: Sequence[Sequence[float]]
    time_points: Sequence[float]


@dataclass(frozen=True)
class GfoldParams:
    """Every tunable G-FOLD quantity, with sensible landing defaults.

    Fixed quantities (mass, fuel, thrust, Isp, gravity, initial state) are
    *not* exposed here — they are derived automatically from the snapshot and
    surface gravity on every solve.
    """

    target_position: Vec3 = (0.0, 0.0, 0.0)
    """Landing target position in the target frame (m).  Default: ground at
    the landing-site origin."""

    target_velocity: Vec3 = (0.0, 0.0, 0.0)
    """Landing target velocity in the target frame (m/s)."""

    glide_slope_angle_deg: float = 30.0
    """Glide-slope cone half-angle about +z (deg).  0 = only height >= 0."""

    max_angle_deg: float = 25.0
    """Maximum angle between the thrust vector and +z (deg).  >= 180 disables
    the pointing constraint."""

    min_throttle: float = 0.2
    """Minimum thrust fraction (``real_max_thrust * min_throttle``)."""

    max_throttle: float = 1.0
    """Maximum thrust fraction (``real_max_thrust * max_throttle``)."""

    max_velocity: float = 1000.0
    """Speed upper bound (m/s) imposed as an SOC constraint."""

    n: int = 50
    """Number of discrete nodes (solve cost scales ~linearly)."""

    tof: float | None = None
    """Fixed time-of-flight (s).  ``None`` = fuel-optimal TOF search (slow —
    use once offline / at ignition, not in the replan loop)."""

    tof_min: float | None = None
    """Lower bound for the TOF search.  ``None`` = auto-bracket."""

    tof_max: float | None = None
    """Upper bound for the TOF search.  ``None`` = auto-bracket."""


def features_of(state: FlightState, params: GfoldParams) -> list[float]:
    """Return the 14-dim TOF-Net feature vector for *state* and *params*.

    The order is the contract with the offline TOF-Net training pipeline
    (``GFOLD-solver``'s ``common/config.py`` ``FEATURES``): position
    (x, y, z), velocity (vx, vy, vz), dry mass, fuel, max thrust, min/max
    throttle fraction, fuel consumption, glide-slope angle, max thrust-pointing
    angle.

    ``fuel_consumption`` uses the standard-gravity constant ``G0`` (Isp unit
    conversion), *not* the surface gravity ``g0`` — this must match the value
    the network was trained with.
    """
    p = state.position
    v = state.velocity
    isp = _require_positive_isp(state.specific_impulse)
    thrust = state.available_thrust if state.available_thrust > 0.0 else state.max_thrust
    return [
        p.x,
        p.y,
        p.z,
        v.x,
        v.y,
        v.z,
        state.dry_mass,
        state.mass - state.dry_mass,
        thrust,
        params.min_throttle,
        params.max_throttle,
        1.0 / (float(isp) * G0),
        params.glide_slope_angle_deg,
        params.max_angle_deg,
    ]


def build_config(
    state: FlightState,
    g0: float,
    params: GfoldParams,
    *,
    tof: float | None = None,
    n: int | None = None,
) -> gfold.Config:
    """Assemble a :class:`gfold.Config` from a snapshot, surface gravity and
    params.

    Mass/fuel/thrust/Isp are taken from *state*; *g0* is the constant surface
    gravity magnitude (m/s^2).  *tof* and *n* override
    :attr:`GfoldParams.tof` / `.n` when given (used by the replan loop to
    shrink the time-of-flight).
    """
    g = float(g0)
    available = state.available_thrust if state.available_thrust > 0.0 else state.max_thrust
    isp = _require_positive_isp(state.specific_impulse)

    spacecraft = gfold.Spacecraft(
        wet_mass=float(state.mass),
        fuel=float(state.mass - state.dry_mass),
        real_max_thrust=float(available),
        min_thrust_pct=float(params.min_throttle),
        max_thrust_pct=float(params.max_throttle),
        max_velocity=float(params.max_velocity),
        initial_position=list(state.position),
        initial_velocity=list(state.velocity),
        target_position=list(params.target_position),
        target_velocity=list(params.target_velocity),
        fuel_consumption=1.0 / (float(isp) * G0),
    )
    environment = gfold.Environment(
        gravity=[0.0, 0.0, -g],
        glide_slope_angle_deg=float(params.glide_slope_angle_deg),
        max_angle_deg=float(params.max_angle_deg),
    )
    solver = gfold.Solver(
        n=int(n) if n is not None else int(params.n),
        time_of_flight=float(tof) if tof is not None else (
            float(params.tof) if params.tof is not None else None
        ),
        tof_min=float(params.tof_min) if params.tof_min is not None else None,
        tof_max=float(params.tof_max) if params.tof_max is not None else None,
    )
    return gfold.Config(spacecraft, environment, solver)


def solve(config: gfold.Config) -> Trajectory | None:
    """Solve *config*, returning ``None`` on infeasibility / solver failure.

    ``gfold.solve`` raises :class:`ValueError` (e.g. ``"solver status:
    PrimalInfeasible"``) when no trajectory exists — callers rely on the
    ``None`` return as the "no solution yet" signal for the landing-burn
    trigger.  A :class:`ValueError` carrying any *other* message is a genuine
    programming error (e.g. a malformed config) and is re-raised rather than
    masked as "no solution".
    """
    try:
        return cast(Trajectory | None, gfold.solve(config))
    except ValueError as exc:
        if "infeasible" in str(exc).lower():
            return None
        raise
    except RuntimeError:
        return None


def replan(
    state: FlightState,
    g0: float,
    params: GfoldParams,
    *,
    tof: float | None = None,
) -> Trajectory | None:
    """One fixed-time-of-flight re-solve from the current snapshot.

    This is the hot-loop entry point (single solve, ~10 ms at ``n=50``);
    pass a shrinking *tof* each cycle for MPC-style replanning.
    """
    return solve(build_config(state, g0, params, tof=tof))


def solve_optimal(
    state: FlightState,
    g0: float,
    params: GfoldParams,
) -> Trajectory | None:
    """Fuel-optimal solve (``time_of_flight=None``, ~0.5 s).

    Use once at ignition to fix the initial time-of-flight and to act as the
    "is a landing feasible from this state?" trigger.
    """
    return solve(build_config(state, g0, replace(params, tof=None)))


def tof_of(traj: Trajectory) -> float:
    """Recover the solved time-of-flight (s) from a trajectory.

    ``gfold`` reports ``time_points`` sampled at ``dt = tof / n``, so the last
    point is ``tof * (n - 1) / n``; undo that scaling here.
    """
    t = np.asarray(traj.time_points, dtype=float)
    n = len(t)
    if n <= 1:
        return 0.0
    return float(t[-1]) * n / (n - 1)


def _u_to_command(
    u: np.ndarray,
    *,
    mass: float,
    available_thrust: float,
    min_throttle: float,
    max_throttle: float,
    max_angle_deg: float,
) -> tuple[float, Vec3]:
    """Convert a single thrust-acceleration vector into ``(throttle, nose)``."""
    mag = float(np.linalg.norm(u))
    if mag < 1e-12:
        return 0.0, (0.0, 0.0, 1.0)
    if available_thrust > 0.0:
        throttle = mag * float(mass) / float(available_thrust)
    else:
        throttle = 0.0
    lo = min(min_throttle, max_throttle)
    hi = max(min_throttle, max_throttle)
    throttle = min(max(throttle, lo), hi)

    nose = np.array([u[0] / mag, u[1] / mag, u[2] / mag])
    if max_angle_deg < 180.0:
        cos_lim = float(np.cos(np.radians(max_angle_deg)))
        if nose[2] < cos_lim:
            horiz = float(np.hypot(nose[0], nose[1]))
            if horiz > 1e-12:
                sin_lim = float(np.sin(np.radians(max_angle_deg)))
                nose[0] *= sin_lim / horiz
                nose[1] *= sin_lim / horiz
                nose[2] = cos_lim
            else:
                nose = np.array([0.0, 0.0, 1.0])
    return throttle, (float(nose[0]), float(nose[1]), float(nose[2]))


def command(
    traj: Trajectory,
    *,
    mass: float,
    available_thrust: float,
    min_throttle: float = 0.2,
    max_throttle: float = 1.0,
    max_angle_deg: float = 180.0,
) -> tuple[float, Vec3]:
    """Convert the first node of a trajectory into a control command.

    Returns ``(throttle, nose)`` where *nose* is the nose direction (equal to
    the thrust direction) and *throttle* is ``|u0| * mass / available_thrust``
    clamped to the throttle bounds — the dynamic thrust reconstruction, which
    is equivalent to the solver's ``normalized_thrusts[0]`` when the lossless
    convexification has no slack.

    *nose* is additionally clamped to within *max_angle_deg* of +z, because
    gfold 0.3.2 does not enforce its own pointing cone during the dive phase.

    Prefer :func:`command_at_time` in the control loop: it accounts for the
    solver latency by interpolating along the trajectory instead of always
    re-applying node 0.
    """
    return _u_to_command(
        np.asarray(traj.u_values[0], dtype=float),
        mass=mass,
        available_thrust=available_thrust,
        min_throttle=min_throttle,
        max_throttle=max_throttle,
        max_angle_deg=max_angle_deg,
    )


def command_at_time(
    traj: Trajectory,
    t: float,
    *,
    mass: float,
    available_thrust: float,
    min_throttle: float = 0.2,
    max_throttle: float = 1.0,
    max_angle_deg: float = 180.0,
) -> tuple[float, Vec3]:
    """Interpolate *traj* at elapsed time *t* and return ``(throttle, nose)``.

    *t* is measured in seconds since the trajectory's ``t=0`` (the snapshot
    time the trajectory was solved from).  Values before the first node and
    after the last node are clamped to the endpoint controls, so a trajectory
    can be followed all the way to touchdown.
    """
    tp = np.asarray(traj.time_points, dtype=float)
    u = np.asarray(traj.u_values, dtype=float)
    n = len(tp)
    if n == 0:
        return 0.0, (0.0, 0.0, 1.0)
    if t <= tp[0] or n == 1:
        u_t = u[0]
    elif t >= tp[-1]:
        u_t = u[-1]
    else:
        idx = int(np.searchsorted(tp, t, side="right") - 1)
        idx = max(0, min(idx, n - 2))
        span = float(tp[idx + 1] - tp[idx])
        frac = 0.0 if span <= 0.0 else float(t - tp[idx]) / span
        u_t = u[idx] + frac * (u[idx + 1] - u[idx])
    return _u_to_command(
        np.asarray(u_t, dtype=float),
        mass=mass,
        available_thrust=available_thrust,
        min_throttle=min_throttle,
        max_throttle=max_throttle,
        max_angle_deg=max_angle_deg,
    )
