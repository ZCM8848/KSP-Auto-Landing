from math import sin, cos, tan

from numpy import array, cross, dot, deg2rad, mat, arccos
from numpy.linalg import norm


# math functions
def lerp(vec1, vec2, t):
    return t * vec2 + (1 - t) * vec1


def clamp(num, limit1, limit2):
    return max(min(num, max(limit1, limit2)), min(limit1, limit2))


def sgn(f):
    if f > 0:
        return 1
    elif f < 0:
        return -1
    return 0


# vector-related
def normalize(v):
    v = array(v)
    n = norm(v)
    if n == 0:
        return v
    return v / n


def q(axis, angle):
    (x, y, z) = axis
    s = sin(angle / 2)
    c = cos(angle / 2)
    axis = array([x + .0, y + .0, z + .0])
    axis /= norm(axis)
    return s * axis[0], s * axis[1], s * axis[2], c


def rotation_mat(q):
    x, y, z, w = q[0], q[1], q[2], q[3]
    return mat([
        [1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y + 2 * w * z, 2 * x * z - 2 * w * y],
        [2 * x * y - 2 * w * z, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z + 2 * w * x],
        [2 * x * z + 2 * w * y, 2 * y * z - 2 * w * x, 1 - 2 * x ** 2 - 2 * y ** 2]
    ])


def transform(vec, matrix):
    res = mat(vec) * matrix
    return array([res[0, 0], res[0, 1], res[0, 2]])


def angle_around_axis(v1, v2, axis):
    """The angle between <v1> and <v2>, return in radians"""
    axis = normalize(axis)
    v1 = normalize(cross(v1, axis))
    v2 = normalize(cross(v2, axis))
    direction = sgn(dot(cross(v1, v2), axis))
    return direction * arccos(dot(v1, v2))


def conic_clamp(vec1, vec2, angle):
    """
    <vec1> is the standard vector\n
    <vec2> is the constrained vector\n
    <angle> represents the half-cone angle of the cone (angle system)
    """
    angle = deg2rad(angle)
    angle_between_vectors = arccos(dot(vec1, vec2) / (norm(vec1) * norm(vec2)))
    if angle_between_vectors <= angle:
        return vec2
    else:
        projection_vertical = normalize(vec1) * norm(vec2) * cos(angle_between_vectors)
        projection_horizontal = normalize(vec2 - projection_vertical) * tan(angle) * norm(projection_vertical)
        return projection_vertical + projection_horizontal
