from math import sin, cos, tan, exp, sqrt, e, pi, inf
from numpy import array, cross, dot, deg2rad, rad2deg, mat, arccos
from numpy.linalg import norm

#math functions
def lerp(vec1, vec2, t):
    return t * vec2 + (1-t) * vec1

def clamp(num, limit1, limit2):
    return max(min(num, max(limit1, limit2)), min(limit1, limit2))

def sgn(f):
    if f > 0:
        return 1
    elif f < 0:
        return -1
    return 0

def improved_sigmoid(x):
    return -1/(1+e**(-x*x)) + 1

def normal_distribution(x):
    return 1/(sqrt(2*pi)) * e**(-(x**2)/2*0.4**2)

#vector-related
def conic_clamp(vector, angle:float):
    '''
    Clamp a vector inside a cone
    '''
    vector = array(vector).copy()
    angle = deg2rad(angle)

    vector_vertical = vector[0]
    vector_horizontal = vector[1:3]
    norm_horizontal = norm(vector_horizontal)
    max_n = abs(vector_vertical/tan(angle))
    if max_n < norm_horizontal:
        vector_horizontal = vector_horizontal / norm_horizontal * max_n
    return vector

def normalize(v):
    v = array(v)
    n = norm(v)
    if n == 0.: return v
    return v / n

def q(axis, angle):
    (x, y, z) = axis
    s = sin(angle / 2)
    c = cos(angle / 2)
    axis = array([x+.0, y+.0, z+.0])
    axis /= norm(axis)
    return (s * axis[0], s * axis[1], s * axis[2], c)

def rotation_mat(q):
    x, y, z, w = q[0], q[1], q[2], q[3]
    return mat([
    [1-2*y**2-2*z**2, 2*x*y+2*w*z, 2*x*z-2*w*y],
    [2*x*y-2*w*z, 1-2*x**2-2*z**2, 2*y*z+2*w*x],
    [2*x*z+2*w*y, 2*y*z-2*w*x, 1-2*x**2-2*y**2]
    ])

def transform(vec, matrix):
    res = mat(vec) * matrix
    return array([res[0,0], res[0,1], res[0,2]])

def angle_around_axis(v1, v2, axis):
    '''The angle between <v1> and <v2>, return in radians'''
    axis = normalize(axis)
    v1 = normalize(cross(v1, axis))
    v2 = normalize(cross(v2, axis))
    direction = sgn(dot(cross(v1, v2), axis))
    return direction * arccos(dot(v1, v2))
