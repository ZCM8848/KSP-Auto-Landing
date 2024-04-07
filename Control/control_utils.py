from math import pi, sin, cos, tan, sqrt
from numpy import array, cross, dot, deg2rad, rad2deg, mat, arccos
from numpy.linalg import norm, inv

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

#vector-related
def conic_clamp(target_a, min_mag, max_mag, max_tilt):
    a_mag = norm(target_a)
    hor_dir = array([0, target_a[1], target_a[2]])
    hor_dir /= norm(hor_dir)
    #target_direction = target_a / a_mag
    a_hor = norm(target_a[1:3])
    a_ver = target_a[0]
    
    if (a_hor < min_mag * sin(max_tilt)):
        a_ver_min = sqrt(min_mag**2 - a_hor**2)
    else:
        a_ver_min = cos(max_tilt) * min_mag
    
    if (a_hor < max_mag * sin(max_tilt)):
        a_ver_max = sqrt(max_mag**2 - a_hor**2)
    else:
        a_ver_max = cos(max_tilt) * max_mag
    
    a_ver = clamp(a_ver, a_ver_max, a_ver_min)
    
    a_hor = min(a_hor, a_ver * tan(max_tilt))
    
    return hor_dir * a_hor + array([a_ver, 0, 0])

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