import krpc
from solver.RTLS.solver_simple import ImapctPointSolver
from typing import List, Tuple
import numpy as np
from tqdm import trange
from math import sqrt, sin, radians
from control import PID
from typing import Optional

from internal.utils import ignition_height, descent_throttle, get_half_rocket_length, landed, transform_to_target_frame, transform_to_body_frame
from control import normalize, norm, array, conic_clamp, angle_between
from internal.behaviors import create_target_reference_frame
from internal.targets import *

class _VesselAdapter:
    def __init__(self, vessel, ref):
        self.vessel = vessel
        self._ref = ref

    def __getattr__(self, name):
        return getattr(self.vessel, name)

    def position(self):
        return array(self.vessel.position(self._ref))

    def velocity(self):
        return array(self.vessel.velocity(self._ref))

STT = array([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0]])
conn = krpc.connect("RTLS")
# tgt = (conn.space_center.target_vessel.flight().longitude, conn.space_center.target_vessel.flight().latitude)
tgt = Targets_JNSQ.landing_zone_1
trf = create_target_reference_frame(conn, tgt)
space_center = conn.space_center
vessel = space_center.active_vessel
vessel.auto_pilot.reference_frame = trf
vessel.auto_pilot.engage()
# vessel.control.input_mode = space_center.ControlInputMode.override

vessel_adapt = _VesselAdapter(vessel, trf)
body = vessel.orbit.body
brf = body.reference_frame
bnrf = body.non_rotating_reference_frame
g0 = body.surface_gravity
target_origion = array(space_center.transform_position((0,0,0), trf, brf))

def blend(a, b, t):
    """线性混合两个向量，t在[0,1]之间"""
    t = max(0.0, min(1.0, t))  # 安全钳制
    return a * (1 - t) + b * t

def smoothstep(edge0, edge1, x):
    """三次平滑插值，导数在边界处为0"""
    if edge0 == edge1:
        return 0.0 if x < edge0 else 1.0
    
    t = (x - edge0) / (edge1 - edge0)
    t = max(0.0, min(1.0, t))  # clamp to [0,1]
    return t * t * (3.0 - 2.0 * t)

def calculate_horizontal_lift(
    position: np.ndarray,           # 位置矢量 [x, y, z]，从行星中心出发 (m)
    velocity: np.ndarray,           # 速度矢量 [vx, vy, vz] (m/s)
    target_direction: np.ndarray,   # 体轴方向（指向头部），会自动归一化
    diameter: float,                # 圆柱直径 (m)
    length: float,                  # 圆柱长度 (m)
    density: float,                 # 大气密度 (kg/m³)
    cl_value: float,                # 【实时升力系数】已根据当前流场条件计算
    reference_area: Optional[float] = None  # 参考面积 (m²)，默认使用侧面积 D*L
) -> Tuple[np.ndarray, float, float]:
    """
    计算姿态倾斜造成的水平方向升力
    
    水平方向定义：垂直于当地径向（位置矢量）的平面
    
    Returns:
        F_horizontal: 水平升力矢量 [Fx, Fy, Fz] (N)
        lift_magnitude: 总升力大小（标量，含符号，负值表示方向与默认相反）(N)
        alpha_deg: 几何攻角 (度)，用于验证
    """
    # 输入验证与单位化
    v_mag = np.linalg.norm(velocity)
    r_mag = np.linalg.norm(position)
    
    # 边界条件检查
    if v_mag < 0.1 or density < 1e-10 or r_mag < 1.0:
        return np.zeros(3), 0.0, 0.0
    
    v_hat = velocity / v_mag
    r_hat = position / r_mag
    b_hat = target_direction / np.linalg.norm(target_direction)
    
    # 1. 计算动压
    q = 0.5 * density * v_mag ** 2
    
    # 2. 参考面积（若未指定，使用侧面积 D*L）
    if reference_area is None:
        A_ref = diameter * length
    else:
        A_ref = reference_area
    
    # 3. 计算总升力大小（使用传入的实时 Cl）
    # 注意：Cl 的符号将影响方向
    lift_magnitude = q * A_ref * cl_value
    
    # 4. 计算攻角（用于返回信息，验证实时 Cl 的合理性）
    # 定义：体轴与速度矢量的夹角，0°表示头部迎风（无升力），90°表示横风（最大升力）
    cos_alpha = -np.dot(b_hat, v_hat)  # 负号因为体轴指向头部，与速度反向为0攻角
    cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
    alpha = np.arccos(cos_alpha)
    alpha_deg = np.degrees(alpha)
    
    # 5. 确定升力方向（垂直于速度，位于体轴-速度平面内）
    # 计算体轴垂直于速度的分量
    b_parallel = np.dot(b_hat, v_hat) * v_hat
    b_perp = b_hat - b_parallel
    
    perp_mag = np.linalg.norm(b_perp)
    
    # 当攻角接近0°或180°时，垂直分量为0，升力方向不确定
    # 此时根据物理意义，升力应为0（但保留传入的 Cl 值用于调试）
    if perp_mag < 1e-6:
        # 攻角为0或180，理论上无升力方向，返回零矢量
        return np.zeros(3), lift_magnitude, alpha_deg
    
    # 升力方向单位矢量（垂直于速度，指向体轴一侧）
    lift_direction = b_perp / perp_mag
    
    # 6. 计算总升力矢量
    F_lift = lift_magnitude * lift_direction
    
    # 7. 投影到水平面（移除径向分量）
    # 水平面定义为垂直于位置矢量的平面
    radial_component = np.dot(F_lift, r_hat) * r_hat
    F_horizontal = F_lift - radial_component
    
    return F_horizontal, abs(lift_magnitude), alpha_deg

def get_rocket_diameter(vessel):
    parts = vessel.parts.all
    max_dims = np.zeros(3)
    min_dims = np.zeros(3)

    for i, part in enumerate(parts):
        pos = part.position(vessel.reference_frame)
        if i == 0:
            max_dims = np.array(pos)
            min_dims = np.array(pos)
        else:
            max_dims = np.maximum(max_dims, pos)
            min_dims = np.minimum(min_dims, pos)

    size_visual = max_dims - min_dims
    return size_visual[-1]

def get_density_data(vessel, trf, atd: float) -> List[Tuple[float, float]]:
    density = []
    altitude = []

    for alt in trange(0, int(atd), 100, desc='sampling atmosphere'):
        pos = (alt, 0, 0)
        altitude.append(alt)
        density.append(vessel.orbit.body.atmospheric_density_at_position(pos, trf))

    altitude = np.array(altitude)
    density = np.array(density)

    return list(zip(altitude.tolist(), density.tolist()))

IPS = ImapctPointSolver(
    vessel.mass,
    vessel.position(brf),
    vessel.velocity(brf),
    body.gravitational_parameter,
    body.angular_velocity(bnrf),
    (body.equatorial_radius + get_half_rocket_length(vessel_adapt) + body.bedrock_height(tgt[1], tgt[0]), 0, 0),
)

error = [np.inf]
t_e = space_center.ut
while True:
    if space_center.ut - t_e <= 0.019:
        continue
    t_e = space_center.ut
    position = array(vessel.position(trf))
    velocity = array(vessel.velocity(trf))
    body_ip = IPS.predict_impact_point()['event_X'][0][0:3]
    targ_ip = transform_to_target_frame(body_ip, target_origion, tgt[0], tgt[1])
    # targ_ip = space_center.transform_position(tuple(body_ip), brf, trf)
    estimated_landing_point = targ_ip
    horizontal_error = norm(estimated_landing_point[1:3])
    # t_c = max((velocity[0] - sqrt(velocity[0]**2 + 2 * g0 * position[0])) / g0, (velocity[0] + sqrt(velocity[0]**2 + 2 * g0 * position[0])) / g0)

    # target_direction = (0, -(position + t_c * velocity)[1], -(position + t_c * velocity)[2])
    target_direction = - normalize(estimated_landing_point)
    vessel.auto_pilot.target_direction = tuple(target_direction)
    vessel.control.throttle = 1
    IPS.mass = vessel.mass
    IPS.position = transform_to_body_frame(position, target_origion, tgt[0], tgt[1])
    IPS.velocity = transform_to_body_frame(velocity, target_origion, tgt[0], tgt[1], is_velocity=True)
    # IPS.k = norm(vessel.flight(brf).aerodynamic_force) / norm(vessel.velocity(brf))**2
    print("ERROR: %.3f" % (horizontal_error))
    if norm(estimated_landing_point[1:3]) <= 5000 and norm(estimated_landing_point[1:3]) > min(error):
        vessel.control.throttle = 0
        break
    else:
        error.append(norm(estimated_landing_point[1:3]))

while array(vessel.velocity(trf))[0] > 0:
    vessel.auto_pilot.target_direction = (1, 0, 0)
t_e = space_center.ut
# while vessel.velocity()[0] < 0:
#     if space_center.ut - t_e <= 0.019:
#         continue
#     t_e = space_center.ut
#     position = vessel.position()
#     velocity = vessel.velocity()
#     IPS.mass = vessel.mass
#     IPS.position = transform_to_body_frame(position, target_origion, tgt[0], tgt[1])
#     IPS.velocity = transform_to_body_frame(velocity, target_origion, tgt[0], tgt[1], is_velocity=True)
#     body_ip = IPS.predict_impact_point()['event_X'][0, :3]
#     targ_ip = transform_to_target_frame(body_ip, target_origion, tgt[0], tgt[1])
#     if vessel.position()[0] <= 70000 and norm(targ_ip[1:3]) >= 1000:
#         target_direction = - normalize(targ_ip)
#         target_direction = conic_clamp(-velocity, target_direction, 5)
#         vessel.auto_pilot.target_direction = tuple(target_direction)
#         vessel.control.throttle = 1
#         print(targ_ip[1:3])
#         if norm(targ_ip[1:3]) < 1000:
#             break

print('AERODYNAMIC GUIDANCE:')
atd = body.atmosphere_depth
last_ip = estimated_landing_point
t_e = space_center.ut
intergral_error = np.zeros(2)
while True: 
    if space_center.ut - t_e <= 0.019:
        continue
    t_e = space_center.ut
    t0 = space_center.ut
    position = array(vessel.position(trf))
    velocity = array(vessel.velocity(trf))
    body_ip = IPS.predict_impact_point()['event_X'][0, :3]
    last_ip = targ_ip
    targ_ip = transform_to_target_frame(body_ip, target_origion, tgt[0], tgt[1])
    dt = space_center.ut - t0
    ip_vel = (targ_ip-last_ip) / dt
    estimated_landing_point = targ_ip
    horizontal_error = norm(estimated_landing_point[1:3])

    intergral_error +=  estimated_landing_point[1:3] * dt
    intergral_error = np.clip(intergral_error, -500, 500)
    target_direction = (-velocity + 
                        array([0, estimated_landing_point[1], estimated_landing_point[2]]) + 
                        ip_vel + 
                        array([0, intergral_error[0], intergral_error[1]]) * 0.02)
    target_direction = normalize(target_direction) + normalize(position)
    target_direction = conic_clamp(-velocity, target_direction, 10)
    vessel.auto_pilot.target_direction = tuple(target_direction)
    vessel.control.throttle = 0
    print(estimated_landing_point, ip_vel, horizontal_error)
    IPS.mass = vessel.mass
    IPS.position = transform_to_body_frame(position, target_origion, tgt[0], tgt[1])
    IPS.velocity = transform_to_body_frame(velocity, target_origion, tgt[0], tgt[1], is_velocity=True)
    if position[0] <= min(ignition_height(vessel_adapt, trf, 0, 0), 10000):
        vessel.control.throttle = 1
        break

pid_throttle = PID()
pid_throttle.kp = 0.05
pid_throttle.ki = 0.01
pid_throttle.kd = 0.02
hrl = get_half_rocket_length(vessel_adapt)
t_e = space_center.ut
length = 2*get_half_rocket_length(vessel_adapt)
diameter = get_rocket_diameter(vessel)
C_l = vessel.flight(trf).lift_coefficient
AoA = 5

while True:
    if space_center.ut - t_e <= 0.019:
        continue
    t_e = space_center.ut
    # t0 = space_center.ut
    position = array(vessel.position(trf))
    velocity = array(vessel.velocity(trf))
    mass = vessel.mass
    thrust = vessel.thrust
    a_eff = (thrust / mass) - g0
    if a_eff > 0.1:
        # 解析解考虑速度
        time_to_land = (velocity[0] + sqrt(velocity[0]**2 + 2*a_eff*position[0])) / a_eff
    else:
        time_to_land = sqrt(2*position[0]/g0) if position[0] > 0 else 0
    estimated_landing_point = position + velocity*time_to_land
    damping_factor = 2.0
    target_direction = -estimated_landing_point - velocity * damping_factor
    target_direction = normalize(target_direction)
    lift = calculate_horizontal_lift(position,
                                     velocity,
                                     target_direction,
                                     diameter,
                                     length,
                                     vessel.flight(trf).atmosphere_density,
                                     vessel.flight(trf).lift_coefficient)
    lift_to_thrust_ratio = lift[1] / (thrust)
    target_direction = -velocity
    vessel.auto_pilot.target_direction = tuple(target_direction)
    vessel.control.throttle = descent_throttle(vessel_adapt, hrl, 0)
    print("%.2f\t%.2f" % (thrust*sin(angle_between(-velocity, target_direction)), lift_to_thrust_ratio))
    if velocity[0] >= 0:
        vessel.control.throttle = 0
        break
    if position[0] <= 1000:
        vessel.control.gear = True