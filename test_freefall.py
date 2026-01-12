import krpc
import time
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq, bisect
from typing import Tuple

from math import log, nan, sin
from control.extension import Rocket
from internal.utils import *
from internal.targets import *
from compiled_solvers.normal_landing.cpg_solver import cpg_solve
from solver import GFoldSolver, GFoldConfig
from control import conic_clamp, angle_between

throttle_limits = [0.4, 1]

conn = krpc.connect(name='test_FF')
space_center = conn.space_center
vessel = space_center.active_vessel
body = vessel.orbit.body
g = body.surface_gravity

trf = create_target_reference_frame(conn, Targets_JNSQ.launchpad)
trf = create_solver_reference_frame(conn, trf)
draw_reference_frame(conn, trf)
vessel = Rocket(space_center, vessel, trf)

def get_k(aerodynamic_force: ndarray, velocity: ndarray) -> float:
    return norm(aerodynamic_force) / (norm(velocity)**2)

def simulate_error(ignition_altitude: float, 
                   vessel: 'Rocket', 
                   target_altitude: float, 
                   target_velocity: float,
                   # 将初始状态作为参数传入，而不是从vessel实时读取
                   initial_velocity: Tuple[float, float, float],
                   initial_mass: float,
                   dt: float = 0.1) -> float:
    """
    核心模拟函数：给定点火高度，返回轨迹误差
    这个函数将被scipy求解器调用多次
    """
    # 在指定高度重置初始状态
    position = np.array([0, 0, ignition_altitude])  # 简化，只关注高度轴
    velocity = np.array(initial_velocity)
    mass = initial_mass
    
    thrust = vessel.vessel.available_thrust
    isp = vessel.vessel.specific_impulse
    mass_flow = thrust / (isp * g)
    
    flight = vessel.vessel.flight(vessel.vessel.reference_frame)
    k = get_k(flight.aerodynamic_force, velocity)
    
    # 主模拟循环
    while True:
        # 计算合外力（气动阻力+推力）
        speed = np.linalg.norm(velocity)
        drag_force = k * speed**2
        
        # 力矢量（假设阻力方向与速度相反）
        force_vector = -drag_force * (velocity / speed) if speed > 0 else np.array([0,0,0])
        force_vector[2] += thrust - g  # 推力沿z轴正向
        
        # 更新状态
        acceleration = force_vector / (mass - mass_flow * velocity[2])  # 修正质量流影响
        position += velocity * dt
        velocity += acceleration * dt
        
        # 终止条件
        if position[2] <= target_altitude or velocity[2] <= target_velocity:
            break
            
        # 防止无限循环
        if position[2] > 1e6 or velocity[2] < -1e4:
            return 1e6  # 返回大误差表示无效解
    
    # 计算综合误差（高度误差 + 速度误差）
    h_error = position[2] - target_altitude
    v_error = velocity[2] - target_velocity
    return h_error + v_error


def find_ignition_altitude(vessel: 'Rocket', 
                           target_altitude: float, 
                           target_velocity: float,
                           search_range: Tuple[float, float] = (1000, 80000)) -> float:
    """
    主函数：直接返回点火高度
    search_range: 搜索区间（米），默认5km到80km
    """
    # 记录当前状态作为初始条件
    initial_vel = vessel.velocity()
    initial_mass = vessel.mass
    
    # 使用偏函数包装，保持其他参数固定
    from functools import partial
    error_func = partial(simulate_error, 
                        vessel=vessel,
                        target_altitude=target_altitude,
                        target_velocity=target_velocity,
                        initial_velocity=initial_vel,
                        initial_mass=initial_mass)
    
    try:
        # 方法1：brentq（更稳健，需要符号变化）
        # 需要确保error_func在区间两端符号相反
        ignition_alt = brentq(error_func, search_range[0], search_range[1], xtol=10)
        
        # 方法2：fsolve（更快但可能收敛到局部解）
        # ignition_alt = fsolve(error_func, x0=search_range[1])[0]
        
        return float(ignition_alt)
    except ValueError as e:
        # 处理无解情况
        print(f"警告：在{search_range}范围内未找到解，错误：{e}")
        return None

while True:
    ignition_altitude = find_ignition_altitude(vessel, 1000, -100)
    print(vessel.position(), ignition_altitude)
    if ignition_altitude is not None and ignition_altitude > vessel.position()[2]:
        vessel.vessel.control.throttle = 1
        break
