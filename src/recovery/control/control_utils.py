"""Scalar and vector math helpers shared by the local control algorithms.

Pure NumPy/math utilities with no kRPC dependency.  Ported from the legacy
control layer; behaviour is preserved bit-for-bit.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from numpy.typing import ArrayLike


def lerp(vec1: Any, vec2: Any, t: float) -> Any:
    """Linear interpolation between two scalars or arrays."""
    return t * vec2 + (1 - t) * vec1


def clamp(num: float, limit1: float, limit2: float) -> float:
    """Clamp *num* to the inclusive interval spanned by *limit1* and *limit2*."""
    return max(min(num, max(limit1, limit2)), min(limit1, limit2))


def sgn(f: float) -> int:
    """Return the sign of *f* as ``-1``, ``0`` or ``1``."""
    if f > 0:
        return 1
    if f < 0:
        return -1
    return 0


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


def q(axis: Any, angle: float) -> tuple[float, float, float, float]:
    """Build a unit quaternion ``(x, y, z, w)`` from an axis and an angle."""
    (x, y, z) = axis
    s = math.sin(angle / 2)
    c = math.cos(angle / 2)
    axis = np.array([x + 0.0, y + 0.0, z + 0.0])
    axis /= np.linalg.norm(axis)
    return float(s * axis[0]), float(s * axis[1]), float(s * axis[2]), float(c)


def rotation_mat(q: Any) -> np.ndarray:
    """Return the 3x3 rotation matrix for a quaternion ``(x, y, z, w)``."""
    x, y, z, w = q[0], q[1], q[2], q[3]
    return np.asmatrix(
        [
            [1 - 2 * y**2 - 2 * z**2, 2 * x * y + 2 * w * z, 2 * x * z - 2 * w * y],
            [2 * x * y - 2 * w * z, 1 - 2 * x**2 - 2 * z**2, 2 * y * z + 2 * w * x],
            [2 * x * z + 2 * w * y, 2 * y * z - 2 * w * x, 1 - 2 * x**2 - 2 * y**2],
        ]
    )


def transform(vec: Any, matrix: Any) -> np.ndarray:
    """Apply a rotation matrix to a vector and return the rotated vector."""
    res = np.asmatrix(vec) * matrix
    return np.array([res[0, 0], res[0, 1], res[0, 2]])


def angle_around_axis(v1: Any, v2: Any, axis: Any) -> float:
    """Return the signed angle (radians) from *v1* to *v2* around *axis*."""
    axis = normalize(axis)
    v1 = normalize(np.cross(v1, axis))
    v2 = normalize(np.cross(v2, axis))
    direction = sgn(np.dot(np.cross(v1, v2), axis))
    return float(direction * np.arccos(np.dot(v1, v2)))


def angle_between(vec1: ArrayLike, vec2: ArrayLike) -> float:
    """Return the unsigned angle (radians) between two vectors."""
    return float(np.arccos(np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))))


def conic_clamp(vec1: ArrayLike, vec2: ArrayLike, angle: float) -> np.ndarray:
    """Clamp *vec2* to a cone of half-angle *angle* (degrees) around *vec1*.

    Both inputs are normalised.  If *vec2* already lies inside the cone it is
    returned unchanged; otherwise it is projected onto the cone boundary.
    """
    vec1 = normalize(vec1)
    vec2 = normalize(vec2)
    angle = math.radians(angle)

    cos_theta = np.dot(vec1, vec2)

    if cos_theta >= math.cos(angle):
        return vec2

    projection_length = math.cos(angle)
    projection_vertical = vec1 * projection_length

    horizontal_component = vec2 - projection_vertical
    horizontal_length = np.linalg.norm(horizontal_component)

    if horizontal_length > 0:
        horizontal_unit = normalize(horizontal_component)
        projection_horizontal = horizontal_unit * math.sin(angle)
    else:
        projection_horizontal = np.array([0, 0, 0])

    return projection_vertical + projection_horizontal
