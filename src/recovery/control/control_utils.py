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
    """Rotate vector *v* about axis *k* by *ang* radians (Rodrigues' formula).

    Retained as the bit-for-bit legacy reference — the production attitude
    path uses :class:`scipy.spatial.transform.Rotation` instead (see
    ``local_attitude.roll_from_axes``); this helper is still exercised by the
    regression tests as an independent baseline.
    """
    return np.asarray(
        math.cos(ang) * v
        + (1 - math.cos(ang)) * np.dot(v, k) * k
        + math.sin(ang) * np.cross(k, v)
    )


def angle_between(vec1: ArrayLike, vec2: ArrayLike) -> float:
    """Return the unsigned angle (radians) between two vectors.

    The cosine is clipped to ``[-1, 1]`` so that floating-point round-off on
    near-parallel (or near-antiparallel) vectors cannot push ``arccos`` out of
    its domain and produce ``NaN``.
    """
    n1 = np.linalg.norm(vec1)
    n2 = np.linalg.norm(vec2)
    if n1 == 0.0 or n2 == 0.0:
        return 0.0
    cosine = float(np.dot(vec1, vec2) / (n1 * n2))
    return float(np.arccos(np.clip(cosine, -1.0, 1.0)))
