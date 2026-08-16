"""Pure-data specifications crossing the KSP / guidance boundary.

These frozen dataclasses carry sampled planetary and atmospheric constants
from the KSP isolation layer (:mod:`recovery.ksp.sampling`) into the pure
guidance layer (:mod:`recovery.guidance`). They contain no kRPC references
and no behaviour — just self-describing data.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BodySpec:
    """Planetary constants for a :class:`~recovery.guidance.LandingPredictor`.

    All spatial vectors are expressed in a single target reference frame.
    """

    mu: float
    """Standard gravitational parameter (m^3/s^2)."""

    omega: tuple[float, float, float]
    """Planet rotation vector in the target frame (rad/s)."""

    body_center: tuple[float, float, float]
    """Position of the body centre in the target frame (m)."""

    body_radius: float
    """Surface radius at the landing target (m)."""

    surface_gravity: float
    """Gravitational acceleration at the body's surface (m/s²)."""


@dataclass(frozen=True)
class DragSpec:
    """Atmosphere drag parameters for a :class:`~recovery.guidance.DragModel`.

    All spatial vectors are expressed in the same target reference frame used
    by the matching :class:`BodySpec`.
    """

    ballistic_coefficient: float
    """Ballistic coefficient (``mass / (C_d·A)``) in kg/m²."""

    density_alts: np.ndarray
    """Sampled altitudes above sea level (m), ascending."""

    density_vals: np.ndarray
    """Atmospheric densities at ``density_alts`` (kg/m³)."""

    body_center: tuple[float, float, float]
    """Position of the body centre in the target frame (m)."""

    sea_level_radius: float
    """Radius at sea level (m), used to convert position to altitude."""
