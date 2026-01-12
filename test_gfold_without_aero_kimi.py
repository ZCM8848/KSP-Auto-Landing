import krpc
import time
import numpy as np
import multiprocessing as mp
from math import log, nan, sin, radians, degrees
from control.extension import Rocket
from internal.utils import *
from internal.targets import *
from compiled_solvers.normal_landing.cpg_solver import cpg_solve
from solver import GFoldSolver, GFoldConfig
from control import conic_clamp, angle_between
from control.PID import PID
from numpy.linalg import norm

# ==================== 全局配置 ====================
throttle_limits = [0.4, 1.0]
array = np.array  # 确保array可用

# ==================== 求解器进程 ====================
def solver_worker(result_queue, shared_state, conn_info):
    """独立进程：持续求解GFOLD，保持队列中总是最新结果"""
    # 每个进程必须独立建立krpc连接
    conn = krpc.connect(**conn_info)
    space_center = conn.space_center
    vessel_obj = space_center.active_vessel
    body = vessel_obj.orbit.body
    g = body.surface_gravity
    
    # 创建局部参考系
    trf = create_target_reference_frame(conn, Targets_JNSQ.launchpad)
    trf = create_solver_reference_frame(conn, trf)
    vessel = Rocket(space_center, vessel_obj, trf)
    
    print("Solver process started, waiting for initial state...")
    
    while shared_state.solver_running:
        try:
            # 求解（包含状态检查）
            result = solve_gfold_local(vessel, g)
            
            # 将最新结果放入队列（覆盖旧值）
            try:
                result_queue.put_nowait(result)
            except:  # 队列满时先清空再写入
                try:
                    result_queue.get_nowait()
                except:
                    pass
                result_queue.put_nowait(result)
                
        except Exception as e:
            print(f"Solver error: {e}")
            time.sleep(0.05)

def solve_gfold_local(vessel, g):
    """在求解器进程内执行的求解函数"""
    params = GFoldConfig()
    params.spacecraft.initial_position = vessel.position()
    params.spacecraft.initial_velocity = vessel.velocity()
    params.spacecraft.target_velocity = array([0, 0, 0])
    params.spacecraft.target_position = array([0, 0, 0])
    params.spacecraft.wet_mass = vessel.vessel.mass
    params.spacecraft.fuel = vessel.vessel.mass - vessel.vessel.dry_mass
    params.spacecraft.real_max_thrust = vessel.vessel.available_thrust
    params.spacecraft.fuel_consumption = 1 / (g * vessel.vessel.specific_impulse)
    params.spacecraft.min_thrust_pct = throttle_limits[0]
    params.spacecraft.max_thrust_pct = throttle_limits[1]
    params.environment.max_angle = 10
    params.environment.gravity = array([0, 0, -g])
    params.environment.glide_slope_angle = 45
    params.solver.n = 100
    
    solver = GFoldSolver(params)
    solver.problem.register_solve('CPG', cpg_solve)
    solver.problem.solve(method='CPG')
    
    result = {
        'status': solver.problem.status,
        'x': solver.variables['x'].value,
        'u': solver.variables['u'].value,
        'solve_time': time.time()
    }
    
    # 验证结果有效性
    if has_nan(result['u']) or has_nan(result['x']):
        result['status'] = "4"
    else:
        for u in result['u']:
            if u[2] < 0:
                result['status'] = "4"
                break
    
    return result

def has_nan(a):
    return bool(np.isnan(a).any())

# ==================== 主控制进程 ====================
def main_control():
    """主进程：实时控制火箭"""
    # 主进程krpc连接
    conn = krpc.connect(name='main_control')
    space_center = conn.space_center
    vessel_obj = space_center.active_vessel
    body = vessel_obj.orbit.body
    g = body.surface_gravity
    
    # 创建参考系
    trf = create_target_reference_frame(conn, Targets_JNSQ.launchpad)
    trf = create_solver_reference_frame(conn, trf)
    draw_reference_frame(conn, trf)
    vessel = Rocket(space_center, vessel_obj, trf)
    
    # 创建共享状态和队列
    manager = mp.Manager()
    shared_state = manager.Namespace()
    shared_state.solver_running = True
    shared_state.latest_result = None
    
    result_queue = mp.Queue(maxsize=1)  # 只保留最新解
    
    # 启动求解器进程
    conn_info = {'name': 'gfold_solver'}
    solver_process = mp.Process(
        target=solver_worker,
        args=(result_queue, shared_state, conn_info)
    )
    solver_process.daemon = True
    solver_process.start()
    
    print("Main control process started...")
    
    # 等待第一个有效解
    print("=== Waiting for first valid solution ===")
    current_solution = None
    while current_solution is None:
        if not result_queue.empty():
            result = result_queue.get()
            if is_solution_valid(result):
                current_solution = result
                last_valid_time = space_center.ut
                print(f"✓ First valid solution obtained at t={last_valid_time:.1f}")
        time.sleep(0.01)
    
    # 初始化控制参数
    throttle_pid = PID()
    throttle_pid.kp = 0.1
    throttle_pid.ki = 0
    throttle_pid.kd = 0.5
    
    terminal = False
    need_fresh_solution = False  # 标记是否需要新解
    target_direction = array([0, 0, 1])
    use_upsample = True
    
    print("=== Starting control loop ===")
    
    while True:
        t_s = space_center.ut
        mass = vessel.vessel.mass
        available_thrust = vessel.vessel.available_thrust
        position = vessel.position()
        velocity = vessel.velocity()
        half_rocket_length = get_half_rocket_length(vessel.vessel)
        
        # ========== 1. 尝试获取最新求解结果（非阻塞） ==========
        got_new_result = False
        while not result_queue.empty():
            try:
                new_result = result_queue.get_nowait()
                got_new_result = True
            except:
                break
        
        if got_new_result:
            if is_solution_valid(new_result):
                current_solution = new_result
                last_valid_time = t_s
                need_fresh_solution = False  # 获得新解，清除标志
                print(f"↑ Updated solution at t={t_s:.1f}, status={new_result['status']}")
            else:
                print(f"↓ Invalid solution (status={new_result['status']}), keeping previous")
        
        # ========== 2. 检查是否需要新解（角度偏差过大） ==========
        # 注意：target_direction是上周期计算的值
        if not terminal and angle_between(target_direction, array([0,0,1])) > radians(10):
            if t_s - last_valid_time > 2.0:  # 距离上次有效解超过2秒
                need_fresh_solution = True
                # print("⚠ Deviation too large, waiting for fresh solution...")
        
        # ========== 3. 确保始终有可用的解 ==========
        if current_solution is None:
            print("✗ No valid solution available! Emergency stop.")
            vessel.vessel.control.throttle = 0
            break
        
        # ========== 4. 轨迹跟踪控制 ==========
        solution = current_solution
        
        # 轨迹上采样
        if use_upsample:
            trajectory_position = upsample_traj(solution['x'][:, :3])
            trajectory_velocity = upsample_traj(solution['x'][:, 3:6])
            trajectory_acceleration = upsample_traj(solution['u'])
        else:
            trajectory_position = solution['x'][:, :3]
            trajectory_velocity = solution['x'][:, 3:6]
            trajectory_acceleration = solution['u']
        
        # 查找最近轨迹点
        results_position = [norm(p - position) for p in trajectory_position]
        min_index = results_position.index(min(results_position))
        
        position_waypoint = array(trajectory_position[min_index])
        velocity_waypoint = array(trajectory_velocity[min_index])
        acceleration_waypoint = array(trajectory_acceleration[min_index])
        
        # 计算误差
        velocity_error = velocity_waypoint - velocity
        position_error = position_waypoint - position
        
        # 计算目标推力和方向
        target_direction = acceleration_waypoint + velocity_error * 0.3 + position_error * 0.1
        
        # 计算油门
        landing_time = norm(position) / (norm(velocity) + 1e-6)
        estimated_impact_speed = velocity[2] + vessel.vessel.thrust / mass * landing_time
        compensation = throttle_pid.update(-0.01*estimated_impact_speed, space_center.ut-t_s)
        compensation = max(compensation, 0)
        throttle_raw = norm(target_direction) * mass / available_thrust
        throttle = conic_clamp(throttle_limits[0], throttle_raw + compensation, throttle_limits[1])
        
        # 更新自动驾驶方向
        vessel.update_ap(target_direction)
        
        # ========== 5. 执行控制 ==========
        if terminal or velocity[2] >= 0:
            vessel.vessel.control.throttle = 0
            print("=== Landing complete ===")
            break
        else:
            # 下降阶段油门逻辑
            if velocity[2] < -2:
                vessel.vessel.control.throttle = throttle
            else:
                vessel.vessel.control.throttle = mass * g / available_thrust
        
        # ========== 6. 状态判断 ==========
        if landing_time < 5 and not terminal:
            terminal = True
            print("Entering terminal phase")
        
        # 打印状态
        status_flag = "✓" if not need_fresh_solution else "⏳"
        print(f"{status_flag} t={t_s:.1f}, throttle={throttle:.3f}, "
              f"dev={degrees(angle_between(target_direction, array([0,0,1]))):.1f}°, "
              f"landing_t={landing_time:.1f}s")
        
        time.sleep(0.02)  # 控制频率 ~50Hz
    
    # ========== 清理 ==========
    print("Shutting down solver...")
    shared_state.solver_running = False
    solver_process.terminate()
    solver_process.join(timeout=1.0)
    vessel.vessel.control.throttle = 0
    print("Done.")

def is_solution_valid(result):
    """检查求解结果是否有效"""
    if result is None:
        return False
    status = result['status']
    if any(ch in status for ch in ("0", "3", "4")):
        return False
    if has_nan(result['u']) or has_nan(result['x']):
        return False
    # 检查推力方向
    for u in result['u']:
        if u[2] < 0:
            return False
    return True

# ==================== 入口 ====================
if __name__ == '__main__':
    # Windows下多进程需要
    mp.freeze_support()
    main_control()