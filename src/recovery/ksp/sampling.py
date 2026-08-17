"""RPC sampling of planetary and atmospheric data into pure-data specs.

This module lives inside the KSP isolation layer because it performs kRPC
calls (through the passed-in ``CelestialBody`` / ``Flight`` remote objects).
It converts those results into the frozen :mod:`recovery.specs` dataclasses
that the pure guidance layer consumes — after this point no kRPC object ever
crosses into guidance.
"""

from __future__ import annotations

from math import sqrt
from typing import Any

import numpy as np

from ..specs import BodySpec, DragSpec


def sample_body_spec(
    body: Any,
    target_frame: Any,
    lat: float,
    lon: float,
) -> BodySpec:
    """Sample planetary constants for a :class:`LandingPredictor`.

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
) -> DragSpec:
    """Sample atmosphere density profile and ballistic coefficient.

    *space_center* is the kRPC ``SpaceCenter`` service root; its
    ``far_available`` flag selects the FAR ballistic-coefficient path.

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

    beta = _resolve_beta(space_center, flight, mass, manual_beta)

    return DragSpec(
        ballistic_coefficient=beta,
        density_alts=alts,
        density_vals=densities,
        body_center=center,
        sea_level_radius=sea_r,
    )


def _resolve_beta(
    space_center: Any | None,
    flight: Any,
    mass: float | None,
    manual_beta: float | None,
) -> float:
    """Ballistic coefficient via manual override, FAR, or drag-force estimate."""
    if manual_beta is not None:
        return float(manual_beta)
    if space_center is not None and getattr(space_center, "far_available", False):
        return float(getattr(flight, "ballistic_coefficient", 0.0))
    if mass is not None:
        rho = float(flight.atmosphere_density)
        d = flight.drag
        drag_mag = float(sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]))
        spd = float(flight.speed)
        if drag_mag > 1e-6 and spd > 1e-6 and rho > 1e-12:
            return mass * rho * spd * spd / (2.0 * drag_mag)
        return float("inf")
    raise ValueError(
        "Cannot determine ballistic coefficient: "
        "FAR not installed, no manual_beta, and no mass for estimation"
    )
