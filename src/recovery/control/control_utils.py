"""Vector math helpers shared by the local control algorithms.

Pure NumPy/math utilities with no kRPC dependency.  Ported from the legacy
control layer; behaviour is preserved bit-for-bit.
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike


def normalize(v: ArrayLike) -> np.ndarray:
    """Return the unit vector in the direction of *v* (or *v* if it is zero)."""
    v = np.asarray(v)
    n = np.linalg.norm(v)
    if n == 0:
        return v
    return np.asarray(v / n)


def rotate(k: np.ndarray, v: np.ndarray, ang: float) -> np.ndarray:
    """Rotate vector *v* about axis *k* by *ang* radians (Rodrigues' formula)."""
    return np.asarray(
        math.cos(ang) * v
        + (1 - math.cos(ang)) * np.dot(v, k) * k
        + math.sin(ang) * np.cross(k, v)
    )


def angle_between(vec1: ArrayLike, vec2: ArrayLike) -> float:
    """Return the unsigned angle (radians) between two vectors."""
    return float(np.arccos(np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))))
