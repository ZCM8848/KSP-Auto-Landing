import krpc
import time
import numpy as np
import multiprocessing
from multiprocessing import Process, Queue, Manager

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


def solve_gfold_from_params(params_dict, g):
    """从参数字典求解 GFold，不依赖 krpc 对象"""
    params = GFoldConfig()
    params.spacecraft.initial_position = array(params_dict['initial_position'])
    params.spacecraft.initial_velocity = array(params_dict['initial_velocity'])
    params.spacecraft.target_velocity = array([0, 0, 0])
    params.spacecraft.target_position = array([0, 0, 9])
    params.spacecraft.wet_mass = params_dict['wet_mass']
    params.spacecraft.fuel = params_dict['fuel']
    params.spacecraft.real_max_thrust = params_dict['real_max_thrust']
    params.spacecraft.fuel_consumption = params_dict['fuel_consumption']
    params.spacecraft.min_thrust_pct = throttle_limits[0]
    params.spacecraft.max_thrust_pct = throttle_limits[1]
    params.environment.max_angle = 10
    params.environment.gravity = array([0, 0, -g])
    params.environment.glide_slope_angle = 45
    params.solver.n = 100
    
    solver = GFoldSolver(params)
    solver.problem.register_solve('CPG', cpg_solve)
    solver.problem.solve(method='CPG')
    
    result = dict()
    result['status'] = solver.problem.status
    result['x'] = solver.variables['x'].value
    result['u'] = solver.variables['u'].value
    
    if has_nan(result['u']) or has_nan(result['x']):
        result['status'] = "4"
    for u in result['u']:
        if u[2] < 0:
            result['status'] = "4"
            break
    
    # 将 numpy 数组转换为列表以便序列化
    result['x'] = result['x'].tolist()
    result['u'] = result['u'].tolist()
    
    return result


def solver_process(params_queue, solution_dict, g, stop_event):
    """求解进程：持续接收参数并求解"""
    print("求解进程启动")
    current_params = None
    solve_count = 0
    
    while not stop_event.is_set():
        try:
            # 非阻塞获取新参数
            if not params_queue.empty():
                current_params = params_queue.get_nowait()
                print(f"求解进程收到新参数: position={current_params['initial_position']}")
            
            # 如果有参数，进行求解
            if current_params is not None:
                solve_count += 1
                print(f"求解进程开始求解 (第 {solve_count} 次)")
                t0 = time.time()
                solution = solve_gfold_from_params(current_params, g)
                solve_time = time.time() - t0
                
                status = solution['status']
                if all(ch not in status for ch in ("0", "3", "4")):
                    print(f"求解成功，耗时: {solve_time:.2f}s")
                    # 更新共享字典中的解
                    solution_dict['solution'] = solution
                    solution_dict['timestamp'] = time.time()
                    solution_dict['valid'] = True
                    print(f"求解进程已将解写入共享字典 (timestamp={solution_dict['timestamp']:.3f})")
                else:
                    print(f"求解约束未满足 (status: {status})，耗时: {solve_time:.2f}s")
                    # 即使约束未满足也保存，控制进程可以决定是否使用
                    solution_dict['solution'] = solution
                    solution_dict['timestamp'] = time.time()
                    solution_dict['valid'] = False
                    print(f"求解进程已将解写入共享字典 (timestamp={solution_dict['timestamp']:.3f}, valid=False)")
                
                current_params = None  # 求解完成后清空，等待新参数
                print(f"求解进程等待新参数...")
            else:
                # 没有参数时，短暂休眠避免 CPU 占用过高
                time.sleep(0.01)
            
        except Exception as e:
            print(f"求解进程错误: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(0.1)
    
    print("求解进程退出")


def control_process(params_queue, solution_dict, stop_event):
    """控制进程：持续获取飞船状态，发送给求解进程，并使用最新解进行控制"""
    print("控制进程启动")
    
    # 初始化 krpc 连接
    conn = krpc.connect(name='test_gfold_multiprocess')
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
    current_solution = None
    solution_received = False
    last_solution_timestamp = 0
    
    # 发送初始参数给求解进程
    position = vessel.position()
    velocity = vessel.velocity()
    params = {
        'initial_position': position.tolist(),
        'initial_velocity': velocity.tolist(),
        'wet_mass': vessel.vessel.mass,
        'fuel': vessel.vessel.mass - vessel.vessel.dry_mass,
        'real_max_thrust': vessel.vessel.available_thrust,
        'fuel_consumption': 1 / (g * vessel.vessel.specific_impulse),
    }
    params_queue.put(params)
    print("控制进程发送初始参数给求解进程")
    
    while not stop_event.is_set():
        try:
            t_s = space_center.ut
            mass = vessel.vessel.mass
            available_thrust = vessel.vessel.available_thrust
            position = vessel.position()
            velocity = vessel.velocity()
            half_rocket_length = get_half_rocket_length(vessel.vessel)
            
            # 如果没有有效解，先设置竖直向上方向（在检查新解之前）
            if not solution_received:
                target_direction = array([0, 0, 1])  # 竖直向上
                vessel.update_ap(target_direction)
                vessel.vessel.control.throttle = 0  # 不开发动机
            
            # 检查是否有新的解
            if solution_dict.get('solution') is not None:
                new_timestamp = solution_dict.get('timestamp', 0)
                is_valid = solution_dict.get('valid', False)
                
                # 检查是否有新解（通过时间戳判断）
                if new_timestamp > last_solution_timestamp:
                    # 只有满足条件的解才用于控制
                    if is_valid:
                        new_solution = solution_dict['solution'].copy()
                        # 将列表转换回 numpy 数组
                        new_solution['x'] = np.array(new_solution['x'])
                        new_solution['u'] = np.array(new_solution['u'])
                        
                        current_solution = new_solution
                        last_solution_timestamp = new_timestamp
                        solution_received = True
                        print(f"控制进程收到有效解 (valid=True)")
                    else:
                        # 无效解不用于控制，但更新时间戳并发送新参数继续求解
                        last_solution_timestamp = new_timestamp
                        print(f"控制进程收到无效解 (valid=False)，忽略此解")
                    
                    # 收到解后（无论有效与否）立即发送最新参数给求解进程进行下一次求解
                    params = {
                        'initial_position': position.tolist(),
                        'initial_velocity': velocity.tolist(),
                        'wet_mass': mass,
                        'fuel': mass - vessel.vessel.dry_mass,
                        'real_max_thrust': available_thrust,
                        'fuel_consumption': 1 / (g * vessel.vessel.specific_impulse),
                    }
                    # 清空队列中的旧参数，只保留最新的
                    while not params_queue.empty():
                        try:
                            params_queue.get_nowait()
                        except:
                            pass
                    params_queue.put(params)
                    print("控制进程发送新参数给求解进程")
            
            # 如果有解，使用解进行控制
            if current_solution is not None and solution_received:
                if use_upsample:
                    trajectory_position = upsample_traj(current_solution['x'][:, :3])
                    trajectory_velocity = upsample_traj(current_solution['x'][:, 3:6])
                    trajectory_acceleration = upsample_traj(current_solution['u'])
                else:
                    trajectory_position = current_solution['x'][:, :3]
                    trajectory_velocity = current_solution['x'][:, 3:6]
                    trajectory_acceleration = current_solution['u']
                
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
                
                landing_time = norm(position) / norm(velocity) if norm(velocity) > 0.1 else 1000
                estimated_impact_speed = velocity[2] + vessel.vessel.thrust / mass * landing_time
                throttle = (norm(target_direction) * mass / available_thrust)
                if terminal:
                    throttle = max(descent_throttle(vessel, half_rocket_length), throttle)
                
                vessel.update_ap(target_direction)
                print(f"throttle: {throttle:.3f}, landing time: {landing_time:.2f}, estimated impact speed: {estimated_impact_speed:.2f}")
                
                # 检查是否需要重新求解
                if not terminal and angle_between(target_direction, array([0, 0, 1])) > radians(10) and space_center.ut - last_retry_time > 5:
                    print('触发重新求解')
                    last_retry_time = space_center.ut
                    # 发送新参数
                    params = {
                        'initial_position': position.tolist(),
                        'initial_velocity': velocity.tolist(),
                        'wet_mass': mass,
                        'fuel': mass - vessel.vessel.dry_mass,
                        'real_max_thrust': available_thrust,
                        'fuel_consumption': 1 / (g * vessel.vessel.specific_impulse),
                    }
                    while not params_queue.empty():
                        try:
                            params_queue.get_nowait()
                        except:
                            pass
                    params_queue.put(params)
                else:
                    vessel.vessel.control.throttle = throttle if velocity[2] < -2 else mass * g / available_thrust
                
                if landing_time < 5:
                    terminal = True
                
                if velocity[2] >= 0:
                    vessel.vessel.control.throttle = 0
                    print("速度向上，停止控制")
                    stop_event.set()
                    break
            else:
                # 没有有效解时，保持竖直向上且不开发动机
                target_direction = array([0, 0, 1])  # 竖直向上
                vessel.update_ap(target_direction)
                vessel.vessel.control.throttle = 0  # 不开发动机
            
            # 控制循环频率，避免 CPU 占用过高
            time.sleep(0.05)
            
        except KeyboardInterrupt:
            print("控制进程收到中断信号")
            stop_event.set()
            break
        except Exception as e:
            print(f"控制进程错误: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(0.1)
    
    print("控制进程退出")


def main():
    """主函数：启动两个进程"""
    # 创建进程间通信对象
    manager = Manager()
    params_queue = Queue()
    solution_dict = manager.dict()
    solution_dict['solution'] = None
    solution_dict['timestamp'] = 0
    solution_dict['valid'] = False
    stop_event = manager.Event()
    
    # 获取重力加速度（需要在主进程中获取，因为 krpc 连接可能无法序列化）
    conn = krpc.connect(name='test_gfold_init')
    g = conn.space_center.active_vessel.orbit.body.surface_gravity
    conn.close()
    
    # 创建并启动求解进程
    solver_proc = Process(target=solver_process, args=(params_queue, solution_dict, g, stop_event))
    solver_proc.start()
    
    # 创建并启动控制进程
    control_proc = Process(target=control_process, args=(params_queue, solution_dict, stop_event))
    control_proc.start()
    
    try:
        # 等待进程结束
        solver_proc.join()
        control_proc.join()
    except KeyboardInterrupt:
        print("主进程收到中断信号，停止所有进程")
        stop_event.set()
        solver_proc.terminate()
        control_proc.terminate()
        solver_proc.join()
        control_proc.join()
    
    print("所有进程已退出")


if __name__ == '__main__':
    main()

