"""RPC sampling of planetary and atmospheric data into pure-data specs.

This module lives inside the KSP isolation layer because it performs kRPC
calls (through the passed-in ``CelestialBody`` / ``Flight`` remote objects).
It converts those results into the frozen :mod:`recovery.specs` dataclasses
that the pure guidance layer consumes — after this point no kRPC object ever
crosses into guidance.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from ..specs import BodySpec, DragSpec


def sample_body_spec(
    body: Any,
    target_frame: Any,
    lat: float,
    lon: float,
) -> BodySpec:
    """Sample planetary constants for a :class:`ControlledPredictor`.

    Reads gravitational parameter, rotation vector, body-centre position and
    surface radius at (*lat*, *lon*) — all expressed in *target_frame*.
    One-shot RPC cost.
    """
    omega = tuple(
        np.array(body.direction(target_frame)) * body.rotational_speed
    )
    center = tuple(np.array(body.position(target_frame)))
    radius = float(body.equatorial_radius + body.surface_height(lat, lon))
    return BodySpec(
        mu=float(body.gravitational_parameter),
        omega=omega,
        body_center=center,
        body_radius=radius,
        surface_gravity=float(body.surface_gravity),
    )


def _local_sea_level_radius(body: Any, lat: float, lon: float) -> float:
    """Return the sea-level radius of *body* at (*lat*, *lon*).

    KSP's physics treats celestial bodies as spheres, so this currently
    returns ``body.equatorial_radius`` for every latitude/longitude.  If a
    future mod or KRPC version exposes oblate-body geometry, this helper is
    the single place to switch to a latitude-dependent sea-level radius.
    """
    del lat, lon  # reserved for future oblate-body support
    return float(body.equatorial_radius)


def sample_drag_spec(
    body: Any,
    flight: Any,
    target_frame: Any,
    *,
    space_center: Any | None = None,
    lat: float | None = None,
    lon: float | None = None,
    mass: float | None = None,
    manual_beta: float | None = None,
    altitude_samples: int = 64,
    beta_ref_altitude: float = 5000.0,
    beta_ref_speed: float = 300.0,
) -> DragSpec:
    """Sample atmosphere density profile and ballistic coefficient.

    *space_center* is the kRPC ``SpaceCenter`` service root; its
    ``far_available`` flag selects the FAR ballistic-coefficient path.

    Without FAR and without *manual_beta*, the ballistic coefficient is
    estimated from a simulated aerodynamic force at a reference dense-air
    state (*beta_ref_altitude* / *beta_ref_speed*), not the instantaneous
    ``flight.drag`` (which is numerically unstable at boosterback altitude).

    **One-time RPC cost:** ~25 ms for 64 samples (measured on Kerbin).

    *altitude_samples* is treated as a minimum; the actual count is never
    less than ``max(32, atmosphere_depth / 500)`` so very deep atmospheres
    (RSS Earth ~140 km) automatically get more points.  Sample spacing follows
    a cosine distribution — dense near sea level, coarser at high altitude.
    """
    center = tuple(np.array(body.position(target_frame)))
    if lat is not None and lon is not None:
        sea_r = _local_sea_level_radius(body, lat, lon)
    else:
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

    beta = _resolve_beta(
        body,
        flight,
        target_frame,
        mass,
        manual_beta,
        space_center=space_center,
        ref_altitude=beta_ref_altitude,
        ref_speed=beta_ref_speed,
    )

    return DragSpec(
        ballistic_coefficient=beta,
        density_alts=alts,
        density_vals=densities,
        body_center=center,
        sea_level_radius=sea_r,
    )


def _resolve_beta(
    body: Any,
    flight: Any,
    target_frame: Any,
    mass: float | None,
    manual_beta: float | None,
    space_center: Any | None = None,
    ref_altitude: float = 5000.0,
    ref_speed: float = 300.0,
) -> float:
    """Resolve the ballistic coefficient (kg/m²).

    Priority: *manual_beta* override, then the FAR-reported coefficient when
    FAR is installed, then an estimate from a simulated aerodynamic force at a
    reference dense-air state.  The reference-state path avoids the numerically
    unstable instantaneous ``flight.drag`` at boosterback altitude.
    """
    if manual_beta is not None:
        return float(manual_beta)
    if space_center is not None and getattr(space_center, "far_available", False):
        return float(getattr(flight, "ballistic_coefficient", 0.0))
    if mass is None:
        raise ValueError(
            "Cannot determine ballistic coefficient: "
            "FAR not installed, no manual_beta, and no mass for estimation"
        )
    return _beta_from_reference_force(
        body, flight, target_frame, float(mass), ref_altitude, ref_speed
    )


def _beta_from_reference_force(
    body: Any,
    flight: Any,
    target_frame: Any,
    mass: float,
    ref_altitude: float,
    ref_speed: float,
) -> float:
    """Estimate ``mass / (C_d·A)`` from a simulated force at a reference state.

    The reference state is a vertical, tail-first (retrograde) descent at
    *ref_altitude* metres and *ref_speed* m/s above the target — a dense-air
    regime where the aerodynamic force is well-conditioned.  ``β = mass·ρ·v² /
    (2·D)`` where *D* is the drag component of the simulated force along the
    (upward) direction opposing the descent velocity.
    """
    center = np.asarray(body.position(target_frame), dtype=float)
    cnorm = float(np.linalg.norm(center))
    if cnorm < 1e-9:
        return float("inf")
    up = -center / cnorm
    rho = float(body.density_at(ref_altitude))
    if rho <= 1e-12:
        return float("inf")
    sea_r = float(body.equatorial_radius)
    position = center + (sea_r + ref_altitude) * up
    velocity = -up * ref_speed
    quat = _nose_quat(up, velocity, up)
    force = np.asarray(
        flight.simulate_aerodynamic_force_at(
            body,
            tuple(float(x) for x in position),
            tuple(float(x) for x in velocity),
            quat,
        ),
        dtype=float,
    )
    drag_mag = float(np.dot(force, up))
    if drag_mag <= 1e-6:
        return float("inf")
    return mass * rho * ref_speed * ref_speed / (2.0 * drag_mag)


def _nose_quat(
    nose: np.ndarray,
    velocity: np.ndarray,
    world_up: np.ndarray,
) -> tuple[float, float, float, float]:
    """Return a quaternion rotating the vessel nose (+Y) onto *nose*.

    The roof reference (roll about the nose) is chosen perpendicular to both
    the nose and ``world_up`` — the same roll convention used by the
    aerodynamic force sampling, expressed in the frame of *velocity*.
    """
    n = nose / float(np.linalg.norm(nose))
    up = world_up / float(np.linalg.norm(world_up))
    if abs(float(np.dot(n, up))) > 0.999:
        up = np.array([1.0, 0.0, 0.0])
    roof = np.cross(up, n)
    roof /= float(np.linalg.norm(roof))
    rot, _ = Rotation.align_vectors([n, roof], [[0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
    q = rot.as_quat()
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def sample_lift_model(
    body: Any,
    flight: Any,
    target_frame: Any,
    *,
    position: tuple[float, float, float],
    velocity: tuple[float, float, float],
    alpha_max_deg: float = 15.0,
    n_alpha: int = 16,
) -> tuple[float, float, float]:
    """Fit lift/drag coefficients by sweeping angle-of-attack.

    At a caller-supplied *reference state* (``position`` / ``velocity`` in
    *target_frame*, representing the flight regime the aero segment will fly
    in), the AoA is swept from 0 to *alpha_max_deg*.  At each AoA the kRPC
    ``Flight.simulate_aerodynamic_force_at`` measures the aerodynamic force,
    which is decomposed into drag (along ``-velocity``) and lift (along the
    tilt side).  The three coefficients of :class:`LiftDragModel` are then
    least-squares fitted against

        D(α) = q · cd0_area · (1 + k·α²)        (q = ½ρv²)
        L(α) = q · cl_area · α

    Returns ``(cd0_area, cl_area, k_ind)`` in m², m²/rad and 1/rad².

    **One-time RPC cost:** ``n_alpha`` force simulations (~10--20 ms).
    Call once at startup, never inside a hot loop.
    """
    center = np.asarray(body.position(target_frame), dtype=float)
    r = np.asarray(position, dtype=float)
    v = np.asarray(velocity, dtype=float)
    speed = float(np.linalg.norm(v))
    if speed < 1e-9:
        raise ValueError("sample_lift_model requires non-zero reference velocity")

    height = float(np.linalg.norm(r - center)) - float(body.equatorial_radius)
    rho = float(body.density_at(max(height, 0.0)))
    q = 0.5 * rho * speed * speed
    if q < 1e-9:
        raise ValueError(
            "reference state is outside the atmosphere (q≈0); "
            "pick a lower-altitude reference to fit lift"
        )

    vhat = v / speed
    # tilt direction: local up projected onto the plane perpendicular to v
    up = -center / float(np.linalg.norm(center))
    side = up - np.dot(up, vhat) * vhat
    side_n = float(np.linalg.norm(side))
    if side_n < 1e-6:  # velocity vertical -> pick any perpendicular direction
        perp = np.array([0.0, 1.0, 0.0]) if abs(vhat[0]) < 0.9 else np.array([0.0, 0.0, 1.0])
        side = perp - np.dot(perp, vhat) * vhat
        side_n = float(np.linalg.norm(side))
    side /= side_n

    alphas = np.radians(np.linspace(0.0, alpha_max_deg, n_alpha))
    drag = np.empty_like(alphas)
    lift = np.empty_like(alphas)
    pos = tuple(float(x) for x in r)
    vel = tuple(float(x) for x in v)
    for i, a in enumerate(alphas):
        nose = -vhat * float(np.cos(a)) + side * float(np.sin(a))
        quat = _nose_quat(nose, v, up)
        F = np.asarray(flight.simulate_aerodynamic_force_at(body, pos, vel, quat), dtype=float)
        drag[i] = -float(np.dot(F, vhat))
        lift[i] = float(np.dot(F, side))

    cd0_area = float(drag[0]) / q if q > 0 else 0.0
    # cl_area:  L = q·cl_area·α  ->  normal equations (L·α)/(q·α²)
    cl_area = float(np.sum(lift * alphas)) / float(q * np.sum(alphas * alphas))
    # k_ind:  D = q·cd0_area·(1 + k·α²)  ->  k = mean( (D/q/cd0_area - 1)/α² ) for α>0
    idx = (alphas > 1e-9) & (cd0_area > 1e-12)
    if np.any(idx):
        k_ind = float(np.mean((drag[idx] / (q * cd0_area) - 1.0) / (alphas[idx] ** 2)))
    else:
        k_ind = 0.0
    k_ind = max(k_ind, 0.0)

    return (cd0_area, cl_area, k_ind)


def sample_lift_table(
    body: Any,
    flight: Any,
    target_frame: Any,
    *,
    position: tuple[float, float, float],
    velocity: tuple[float, float, float],
    alpha_max_deg: float = 25.0,
    n_alpha: int = 26,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Sample full lift/drag coefficient tables vs angle-of-attack.

    Sweeps AoA from 0 to *alpha_max_deg* at a caller-supplied reference state
    and returns the discrete tables consumed by
    :class:`~recovery.guidance.aerodynamics.LiftTableModel`:

        ``(alpha_pts, cl_table, cd_table, clamp_aoa)``

    where ``alpha_pts`` (rad), ``cl_table`` = ``C_L(α)·A`` (m², signed — carries
    the lift direction), ``cd_table`` = ``C_D(α)·A`` (m²), and ``clamp_aoa`` is
    the table's maximum angle (rad).  Unlike :func:`sample_lift_model` (linear
    slope fit), this preserves the real nonlinear / stalling behaviour.

    **One-time RPC cost:** ``n_alpha`` force simulations.  Call once at startup.
    """
    center = np.asarray(body.position(target_frame), dtype=float)
    r = np.asarray(position, dtype=float)
    v = np.asarray(velocity, dtype=float)
    speed = float(np.linalg.norm(v))
    if speed < 1e-9:
        raise ValueError("sample_lift_table requires non-zero reference velocity")

    height = float(np.linalg.norm(r - center)) - float(body.equatorial_radius)
    rho = float(body.density_at(max(height, 0.0)))
    q = 0.5 * rho * speed * speed
    if q < 1e-9:
        raise ValueError(
            "reference state is outside the atmosphere (q≈0); "
            "pick a lower-altitude reference to sample the table"
        )

    vhat = v / speed
    up = -center / float(np.linalg.norm(center))
    side = up - np.dot(up, vhat) * vhat
    side_n = float(np.linalg.norm(side))
    if side_n < 1e-6:  # velocity vertical -> pick any perpendicular direction
        perp = np.array([0.0, 1.0, 0.0]) if abs(vhat[0]) < 0.9 else np.array([0.0, 0.0, 1.0])
        side = perp - np.dot(perp, vhat) * vhat
        side_n = float(np.linalg.norm(side))
    side /= side_n

    alphas = np.radians(np.linspace(0.0, alpha_max_deg, n_alpha))
    cl_table = np.empty_like(alphas)
    cd_table = np.empty_like(alphas)
    pos = tuple(float(x) for x in r)
    vel = tuple(float(x) for x in v)
    for i, a in enumerate(alphas):
        nose = -vhat * float(np.cos(a)) + side * float(np.sin(a))
        quat = _nose_quat(nose, v, up)
        F = np.asarray(flight.simulate_aerodynamic_force_at(body, pos, vel, quat), dtype=float)
        cl_table[i] = float(np.dot(F, side)) / q
        cd_table[i] = float(np.dot(F, -vhat)) / q

    return alphas, cl_table, cd_table, float(np.radians(alpha_max_deg))
