import krpc
import time
import numpy as np

from math import log, nan, sin
from control.extension import Rocket
from internal.utils import *
from internal.targets import *
from solver.GFOLD.compiled_solvers.normal_landing.cpg_solver import cpg_solve
from solver.GFOLD import GFoldSolver, GFoldConfig
from control import conic_clamp, angle_between
from internal.utils import generate_cubic_with_vertical_end, estimate_duration
from cubic import CubicSplineTrajectory

throttle_limits = [0.4, 1]

conn = krpc.connect(name='test_gfold')
space_center = conn.space_center
vessel = space_center.active_vessel
body = vessel.orbit.body
g = body.surface_gravity

solver = GFoldSolver()
problem = solver.problem
problem.register_solve('CPG', cpg_solve)

trf = create_target_reference_frame(conn, Targets_JNSQ.launchpad)
trf = create_solver_reference_frame(conn, trf)
draw_reference_frame(conn, trf)
vessel = Rocket(space_center, vessel, trf)

def generate_spline_trajectory(initial_state, target_pos, tf, N=100, target_vel=None):
    """
    主接口函数：生成三次样条轨迹
    
    参数:
        initial_state: [6,] 数组 [px,py,pz,vx,vy,vz]
        target_pos: [3,] 数组 [px,py,pz]
        tf: 总飞行时间（由你的估算函数提供）
        N: 采样点数（默认100）
        target_vel: [3,] 目标速度，默认为零向量
    
    返回:
        solution: 字典，可直接索引 solution['x'][:, :3] 等
    """
    if target_vel is None:
        target_vel = np.zeros(3)
    
    # 提取初始状态
    p0 = initial_state[:3]
    v0 = initial_state[3:]
    pf = target_pos
    vf = target_vel
    
    # 生成轨迹
    planner = CubicSplineTrajectory(N=N)
    solution = planner.solve(p0, v0, pf, vf, tf)
    
    return solution

terminal = False
need_retry = True
draw = True
use_upsample = True
last_retry_time = space_center.ut
target_direction = array([1,0,0])
while True:
    mass = vessel.vessel.mass
    available_thrust = vessel.vessel.available_thrust
    position = vessel.position()
    velocity = vessel.velocity()
    vessel.update_ap(target_direction)
    half_rocket_length = get_half_rocket_length(vessel.vessel)

    if not terminal:
        solution = generate_spline_trajectory(
            initial_state=np.concatenate((position, velocity)),
            target_pos=np.array([0, 0, half_rocket_length]),
            tf=estimate_duration(position, velocity, np.zeros(3), np.zeros(3)),
            N=100
        )


    if use_upsample:
        trajectory_position = upsample_traj(solution['x'][:, :3])
        trajectory_velocity = upsample_traj(solution['x'][:, 3:6])
        trajectory_acceleration = upsample_traj(solution['u'])
    else:
        trajectory_position = solution['x'][:, :3]
        trajectory_velocity = solution['x'][:, 3:6]
        trajectory_acceleration = solution['u']

    if 0: 
        conn.krpc.paused = True
        draw_trajectory(conn, trajectory_position, trajectory_acceleration, trf)
        conn.krpc.paused = False
        draw = False

    results_position = []
    for point in trajectory_position:
        results_position.append(norm(point - position))
    min_index = results_position.index(min(results_position))

    # define waypoints
    position_waypoint = array(trajectory_position[min_index])
    velocity_waypoint = array(trajectory_velocity[min_index])
    acceleration_waypoint = array(trajectory_acceleration[min_index])

    # define errors
    velocity_error = velocity_waypoint - velocity
    position_error = position_waypoint - position

    # main control
    target_direction = acceleration_waypoint + velocity_error * 0.3 + position_error * 0.1
    throttle = (norm(target_direction) * mass / available_thrust)
    # throttle = clamp(throttle, THROTTLE_LIMIT[0], THROTTLE_LIMIT[1])
    landing_time = norm(position) / norm(velocity)
    estimated_impact_speed = velocity[2] + vessel.vessel.thrust / mass * landing_time
    vessel.vessel.control.throttle = throttle if velocity[2] < -2 else mass * g / available_thrust
    # target_direction = conic_clamp(array([0,0,1]), target_direction, 10)
    vessel.update_ap(target_direction)
    print(f"throttle: {throttle}, landing time: {landing_time}, estimated impact speed: {estimated_impact_speed}")

    if not terminal and angle_between(target_direction, array([0,0,1])) > radians(10) and space_center.ut - last_retry_time > 5:
        print('retry')
        need_retry = True
    else:
        vessel.vessel.control.throttle = throttle if velocity[2] < -2 else mass * g / available_thrust
    
    if landing_time < 5:
        terminal = True

    if velocity[2] >= 0:
        vessel.vessel.control.throttle = 0
        break