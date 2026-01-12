import krpc
import time
import numpy as np
import multiprocessing
from queue import Empty, Full

from math import log, nan, sin, radians
from control.extension import Rocket
from internal.utils import *
from internal.targets import *
from compiled_solvers.normal_landing.cpg_solver import cpg_solve
from solver import GFoldSolver, GFoldConfig
from control import conic_clamp, angle_between
from control.PID import PID

throttle_limits = [0.4, 1]


def has_nan(a):
    return bool(np.isnan(a).any())


def solve_gfold_process(params_queue, results_queue, stop_event, throttle_limits, g):
    """
    求解进程：持续从参数队列接收最新的航天器状态，求解轨迹，将结果放入结果队列
    """
    print("求解进程启动")
    
    while not stop_event.is_set():
        try:
            # 非阻塞获取最新参数（丢弃旧参数）
            params = None
            while True:
                try:
                    params = params_queue.get_nowait()
                except Empty:
                    break
            
            if params is None:
                time.sleep(0.01)  # 如果没有参数，短暂休眠
                continue
            
            # 使用最新参数求解
            print(f"求解进程：开始求解，位置={params['position']}, 速度={params['velocity']}")
            t0 = time.time()
            
            # 构建求解参数
            solver_params = GFoldConfig()
            solver_params.spacecraft.initial_position = params['position']
            solver_params.spacecraft.initial_velocity = params['velocity']
            solver_params.spacecraft.target_velocity = array([0, 0, 0])
            solver_params.spacecraft.target_position = array([0, 0, 9])
            solver_params.spacecraft.wet_mass = params['mass']
            solver_params.spacecraft.fuel = params['fuel']
            solver_params.spacecraft.real_max_thrust = params['available_thrust']
            solver_params.spacecraft.fuel_consumption = params['fuel_consumption']
            solver_params.spacecraft.min_thrust_pct = throttle_limits[0]
            solver_params.spacecraft.max_thrust_pct = throttle_limits[1]
            solver_params.environment.max_angle = 10
            solver_params.environment.gravity = array([0, 0, -g])
            solver_params.environment.glide_slope_angle = 45
            solver_params.solver.n = 100
            
            # 求解
            solver = GFoldSolver(solver_params)
            solver.problem.register_solve('CPG', cpg_solve)
            solver.problem.solve(method='CPG')
            
            result = dict()
            result['status'] = solver.problem.status
            result['x'] = solver.variables['x'].value
            result['u'] = solver.variables['u'].value
            result['timestamp'] = time.time()
            
            # 验证结果
            if has_nan(result['u']) or has_nan(result['x']):
                result['status'] = "4"
            for u in result['u']:
                if u[2] < 0:
                    result['status'] = "4"
                    break
            
            # 检查约束是否满足
            if all(ch not in result['status'] for ch in ("0", "3", "4")):
                print(f"求解进程：约束满足，耗时 {time.time() - t0:.2f}秒")
                # 将结果放入队列（清空旧结果，只保留最新的）
                try:
                    while True:
                        results_queue.get_nowait()
                except Empty:
                    pass
                results_queue.put(result)
            else:
                print(f"求解进程：约束不满足，状态={result['status']}，耗时 {time.time() - t0:.2f}秒")
                
        except Exception as e:
            print(f"求解进程错误: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(0.1)
    
    print("求解进程结束")


def control_process(params_queue, results_queue, stop_event, conn_name, throttle_limits):
    """
    控制进程：读取航天器状态，发送给求解进程，使用最新求解结果进行控制
    """
    print("控制进程启动")
    
    # 在控制进程中创建 krpc 连接
    conn = krpc.connect(name=conn_name)
    space_center = conn.space_center
    vessel = space_center.active_vessel
    body = vessel.orbit.body
    g = body.surface_gravity
    
    trf = create_target_reference_frame(conn, Targets_JNSQ.launchpad)
    trf = create_solver_reference_frame(conn, trf)
    draw_reference_frame(conn, trf)
    vessel = Rocket(space_center, vessel, trf)
    
    # PID 控制器
    throttle_pid = PID()
    throttle_pid.kp = 0.1
    throttle_pid.ki = 0.
    throttle_pid.kd = 0.5
    v_pid = PID()
    v_pid.kp = 0.1
    v_pid.ki = 0
    v_pid.kd = 0.5
    x_pid = PID()
    x_pid.kp = 0.1
    x_pid.ki = 0
    x_pid.kd = 0.5
    
    terminal = False
    use_upsample = True
    last_retry_time = space_center.ut
    target_direction = array([0, 0, 1])
    solution = None
    need_initial_solve = True
    
    # 发送初始求解请求
    position = vessel.position()
    velocity = vessel.velocity()
    mass = vessel.vessel.mass
    available_thrust = vessel.vessel.available_thrust
    
    initial_params = {
        'position': position,
        'velocity': velocity,
        'mass': mass,
        'fuel': mass - vessel.vessel.dry_mass,
        'available_thrust': available_thrust,
        'fuel_consumption': 1 / (g * vessel.vessel.specific_impulse)
    }
    params_queue.put(initial_params)
    print("控制进程：发送初始求解请求")
    
    while True:
        t_s = space_center.ut
        mass = vessel.vessel.mass
        available_thrust = vessel.vessel.available_thrust
        position = vessel.position()
        velocity = vessel.velocity()
        vessel.update_ap(target_direction)
        half_rocket_length = get_half_rocket_length(vessel.vessel)
        
        # 获取最新的求解结果（非阻塞，只取最新的）
        latest_solution = None
        while True:
            try:
                latest_solution = results_queue.get_nowait()
            except Empty:
                break
        
        if latest_solution is not None:
            solution = latest_solution
            print(f"控制进程：收到新的求解结果，状态={solution['status']}")
            need_initial_solve = False
        
        # 如果还没有初始解，等待
        if solution is None:
            time.sleep(0.1)
            continue
        
        # 定期发送新的求解请求（使用当前状态更新求解参数）
        if space_center.ut - last_retry_time > 5 and not terminal:
            current_params = {
                'position': position,
                'velocity': velocity,
                'mass': mass,
                'fuel': mass - vessel.vessel.dry_mass,
                'available_thrust': available_thrust,
                'fuel_consumption': 1 / (g * vessel.vessel.specific_impulse)
            }
            # 清空旧参数，只保留最新的
            try:
                while True:
                    params_queue.get_nowait()
            except Empty:
                pass
            params_queue.put(current_params)
            last_retry_time = space_center.ut
            print("控制进程：发送新的求解请求")
        
        # 使用最新的求解结果进行控制
        if use_upsample:
            trajectory_position = upsample_traj(solution['x'][:, :3])
            trajectory_velocity = upsample_traj(solution['x'][:, 3:6])
            trajectory_acceleration = upsample_traj(solution['u'])
        else:
            trajectory_position = solution['x'][:, :3]
            trajectory_velocity = solution['x'][:, 3:6]
            trajectory_acceleration = solution['u']
        
        if 0:  # 绘图功能（可选）
            conn.krpc.paused = True
            draw_trajectory(conn, trajectory_position, trajectory_acceleration, trf)
            conn.krpc.paused = False
        
        # 找到最近的轨迹点
        results_position = []
        for point in trajectory_position:
            results_position.append(norm(point - position))
        min_index = results_position.index(min(results_position))
        
        # 定义航点
        position_waypoint = array(trajectory_position[min_index])
        velocity_waypoint = array(trajectory_velocity[min_index])
        acceleration_waypoint = array(trajectory_acceleration[min_index])
        
        # 定义误差
        velocity_error = velocity_waypoint - velocity
        position_error = position_waypoint - position
        
        # 主控制
        target_direction = acceleration_waypoint + velocity_error * 0.3 + position_error * 0.1
        dt = space_center.ut - t_s
        
        landing_time = norm(position) / norm(velocity) if norm(velocity) > 0 else 0
        estimated_impact_speed = velocity[2] + vessel.vessel.thrust / mass * landing_time if landing_time > 0 else 0
        
        throttle = (norm(target_direction) * mass / available_thrust) if available_thrust > 0 else 0
        if terminal:
            throttle = max(descent_throttle(vessel, half_rocket_length), throttle)
        
        vessel.update_ap(target_direction)
        print(f"throttle: {throttle:.3f}, landing time: {landing_time:.2f}, estimated impact speed: {estimated_impact_speed:.3f}")
        
        # 检查是否需要重新求解
        if not terminal and angle_between(target_direction, array([0, 0, 1])) > radians(10) and space_center.ut - last_retry_time > 5:
            print('控制进程：触发重新求解')
            current_params = {
                'position': position,
                'velocity': velocity,
                'mass': mass,
                'fuel': mass - vessel.vessel.dry_mass,
                'available_thrust': available_thrust,
                'fuel_consumption': 1 / (g * vessel.vessel.specific_impulse)
            }
            try:
                while True:
                    params_queue.get_nowait()
            except Empty:
                pass
            params_queue.put(current_params)
            last_retry_time = space_center.ut
        else:
            vessel.vessel.control.throttle = throttle if velocity[2] < -2 else mass * g / available_thrust
        
        if landing_time < 5:
            terminal = True
        
        if velocity[2] >= 0:
            vessel.vessel.control.throttle = 0
            print("控制进程：垂直速度为正，结束控制")
            stop_event.set()
            break
        
        time.sleep(0.01)  # 控制循环频率
    
    print("控制进程结束")


def main():
    """
    主函数：创建并启动求解进程和控制进程
    """
    # 创建进程间通信队列
    params_queue = multiprocessing.Queue(maxsize=1)  # 只保留最新的参数
    results_queue = multiprocessing.Queue(maxsize=1)  # 只保留最新的结果
    stop_event = multiprocessing.Event()
    
    # 获取初始参数（用于求解进程）
    conn = krpc.connect(name='test_gfold_init')
    space_center = conn.space_center
    body = space_center.active_vessel.orbit.body
    g = body.surface_gravity
    conn.close()
    
    # 创建并启动求解进程
    solver_process = multiprocessing.Process(
        target=solve_gfold_process,
        args=(params_queue, results_queue, stop_event, throttle_limits, g)
    )
    solver_process.start()
    
    # 创建并启动控制进程
    control_proc = multiprocessing.Process(
        target=control_process,
        args=(params_queue, results_queue, stop_event, 'test_gfold_control', throttle_limits)
    )
    control_proc.start()
    
    try:
        # 等待控制进程结束
        control_proc.join()
    except KeyboardInterrupt:
        print("收到中断信号，停止进程...")
        stop_event.set()
    
    # 等待进程结束
    solver_process.join(timeout=5)
    control_proc.join(timeout=5)
    
    if solver_process.is_alive():
        solver_process.terminate()
        solver_process.join()
    if control_proc.is_alive():
        control_proc.terminate()
        control_proc.join()
    
    print("所有进程已结束")


if __name__ == '__main__':
    main()
