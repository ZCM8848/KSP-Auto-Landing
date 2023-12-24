import numpy as np
from numpy import array, cross, dot, ndarray, rad2deg, radians, cos, tan, sin
from numpy.linalg.linalg import norm
from PID import clamp

def vec_clamp(v, max_norm):
    n = norm(v)
    if n == 0.: return v
    return clamp(n, 0, max_norm) * v / n

def vec_ang(a, b):
    na = norm(a)
    nb = norm(b)
    if na == 0. or nb == 0.: return 0.
    cos_ang = np.dot(a, b) / na / nb
    return rad2deg(np.arccos(clamp(cos_ang, -1., 1.))) # 可能存在计算误差使得cos值不在正确范围内，角度制

def vec_clamp_yz(vector, ang: float):
    '''
    vector: 需要限制指向的向量
    ang:    离yz平面最小角度，单位是角度
    '''
    vector = array(vector)
    vector = vector.copy()
    ang = radians(ang)
    x = vector[0]
    yz = vector[1:3] # yz分量
    n_yz = norm(yz)
    max_n = abs(x / np.tan(ang))
    if max_n < n_yz:
        yz = yz / n_yz * max_n
    vector[1:3] = yz
    return vector

def vec_around(a, b, ang) -> ndarray:
    target_ang = vec_ang(a, b)
    rot_ang = clamp(target_ang, -ang, ang)
    rot_axis = normalize(cross(a, b))
    return rotate(rot_axis, a, rot_ang)

def rotate(k, v, ang):
    return np.cos(ang) * v + (1 - np.cos(ang)) * np.dot(v, k) * k + np.sin(ang) * cross(k, v)

def normalize(v):
    v = array(v)
    n = norm(v)
    if n == 0.: return v
    return v / n

def cone(vector1, vector2, angle):#quite odd
    vector1 = normalize(vector1) # normalize vector1
    vector2 = normalize(vector2) # normalize vector2
    angle = clamp(angle, 90, 0) # clamp angle between 0 and 90 degrees
    if vec_ang(vector1, vector2) <= angle:
        pass
    else:
        angle = radians(angle)
        projection_y = cos(angle) * vector2 * vector1 # use normalized vector2
        projection_x = norm(projection_y)*tan(angle) * normalize(vector2-projection_y)
        vector2 = projection_x + projection_y
        if dot(vector1, vector2) < 0: # check the direction
            vector2 = -vector2 # reverse the direction
    return vector2


a = array([35,25,-80])
b = array([2.4,46,99])
print(vec_ang(a,b))
c = cone(a,b,45)
print(c)
print(vec_ang(a,c))