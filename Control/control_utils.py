from math import sin, cos, tan, pi, radians, degrees, sqrt
from numpy import array, cross, dot, arccos, asmatrix, abs, clip
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

def rotate(k, v, ang):
    return cos(ang) * v + (1 - cos(ang)) * dot(v, k) * k + sin(ang) * cross(k, v)

def q(axis, angle):
    (x, y, z) = axis
    s = sin(angle / 2)
    c = cos(angle / 2)
    axis = array([x + .0, y + .0, z + .0])
    axis /= norm(axis)
    return s * axis[0], s * axis[1], s * axis[2], c


def rotation_mat(q):
    x, y, z, w = q[0], q[1], q[2], q[3]
    return asmatrix([
        [1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y + 2 * w * z, 2 * x * z - 2 * w * y],
        [2 * x * y - 2 * w * z, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z + 2 * w * x],
        [2 * x * z + 2 * w * y, 2 * y * z - 2 * w * x, 1 - 2 * x ** 2 - 2 * y ** 2]
    ])


def transform(vec, matrix):
    res = asmatrix(vec) * matrix
    return array([res[0, 0], res[0, 1], res[0, 2]])


def angle_around_axis(v1, v2, axis):
    """The angle between v1 and v2, return in radians"""
    axis = normalize(axis)
    v1 = normalize(cross(v1, axis))
    v2 = normalize(cross(v2, axis))
    direction = sgn(dot(cross(v1, v2), axis))
    return direction * arccos(dot(v1, v2))

def angle_between(vec1, vec2):
    """in radians"""
    return arccos(dot(vec1, vec2) / (norm(vec1) * norm(vec2)))

def conic_clamp(vec1, vec2, angle):
    """
    vec1 is the standard vector

    vec2 is the constrained vector

    angle represents the half-cone angle of the cone (angle system)
    """
    # Normalize vec1 and vec2
    vec1 = normalize(vec1)
    vec2 = normalize(vec2)
    angle = radians(angle)

    # Calculate the cosine of the angle between vec1 and vec2
    cos_theta = dot(vec1, vec2)

    # If the angle is within the cone, return vec2 as is
    if cos_theta >= cos(angle):
        return vec2
    else:
        # Calculate the projection of vec2 onto vec1
        projection_length = cos(angle)
        projection_vertical = vec1 * projection_length
        
        # Calculate the horizontal component
        horizontal_component = vec2 - projection_vertical
        horizontal_length = norm(horizontal_component)
        
        # Normalize the horizontal component and scale it to the correct length
        if horizontal_length > 0:
            horizontal_unit = normalize(horizontal_component)
            projection_horizontal = horizontal_unit * sin(angle)
        else:
            projection_horizontal = array([0, 0, 0])
        
        # Combine the vertical and horizontal components
        return projection_vertical + projection_horizontal