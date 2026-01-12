import krpc
import time
import numpy as np

from math import log, nan, sin
from control.extension import Rocket
from internal.utils import *
from internal.targets import *
from compiled_solvers.normal_landing.cpg_solver import cpg_solve
from solver import GFoldSolver, GFoldConfig
from control import conic_clamp, angle_between
from control.PID import PID

throttle_limits = [0.4, 1]

conn = krpc.connect(name='test_gfold')
space_center = conn.space_center
vessel = space_center.active_vessel
body = vessel.orbit.body
g = body.surface_gravity

trf = create_target_reference_frame(conn, Targets_JNSQ.launchpad)
trf = create_solver_reference_frame(conn, trf)
draw_reference_frame(conn, trf)
vessel = Rocket(space_center, vessel, trf)


def has_nan(a):
    return bool(np.isnan(a).any())

def solve_gfold(vessel):
    params = GFoldConfig()
    params.spacecraft.initial_position = vessel.position()
    params.spacecraft.initial_velocity = vessel.velocity()
    params.spacecraft.target_velocity = array([0, 0, 0])
    params.spacecraft.target_position = array([0, 0, 0])
    params.spacecraft.wet_mass = vessel.vessel.mass
    params.spacecraft.fuel = vessel.vessel.mass - vessel.vessel.dry_mass
    params.spacecraft.real_max_thrust = vessel.vessel.available_thrust
    params.spacecraft.fuel_consumption = 1 / (g*vessel.vessel.specific_impulse)
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
    return result


while True:
    position = vessel.position()
    velocity = vessel.velocity()
    heading = vessel.vessel.flight(trf).heading
    estimated_landing_point = impact_point(vessel, trf)
    compensation = normalize(position) * estimated_landing_point[2]
    estimated_landing_point = estimated_landing_point + array([compensation[0], compensation[1], 0])
    altitude = vessel.vessel.flight(trf).surface_altitude
    horizontal_error = norm(estimated_landing_point[1:3])
    target_direction = - velocity + array([estimated_landing_point[0], estimated_landing_point[1], 0])
    target_direction = normalize(target_direction) + normalize(position)
    target_direction = conic_clamp(-velocity, target_direction, 10)
    target_roll = roll_controller(heading)
    vessel.update_ap(target_direction, target_roll)
    vessel.vessel.control.throttle = 0
    print(vessel.position(), vessel.velocity())
    t0 = time.time()
    status = solve_gfold(vessel)['status']
    print(f"solver status: {status}")
    # print(f"Time cost: {time.time() - t0}")
    # breakx
    if all(ch not in status for ch in ("0", "3", "4")):
        break

throttle_pid = PID()
throttle_pid.kp = 0.1
throttle_pid.ki = 0
throttle_pid.kd = 0.5
terminal = False
need_retry = True
draw = True
use_upsample = True
last_retry_time = space_center.ut
target_direction = array([0,0,1])
while True:
    t_s = space_center.ut
    mass = vessel.vessel.mass
    available_thrust = vessel.vessel.available_thrust
    position = vessel.position()
    velocity = vessel.velocity()
    vessel.update_ap(target_direction)
    half_rocket_length = get_half_rocket_length(vessel.vessel)

    if need_retry:
        while True:
            print(position, velocity)
            t0 = time.time()
            last_retry_time = space_center.ut
            solution = solve_gfold(vessel)
            status = solution['status']
            if all(ch not in status for ch in ("0", "3", "4")):
                print("All constraints satisfied")
                print(f"Time cost: {time.time() - t0}")
                break
            else:
                print("Some constraints not satisfied, ignored")
                print(f"Time cost: {time.time() - t0}")
        need_retry = False
        # draw = True


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
    # throttle = clamp(throttle, THROTTLE_LIMIT[0], THROTTLE_LIMIT[1])
    landing_time = norm(position) / norm(velocity)
    estimated_impact_speed = velocity[2] + vessel.vessel.thrust / mass * landing_time
    compensation = throttle_pid.update(-0.01*estimated_impact_speed, space_center.ut-t_s)
    compensation = compensation if compensation > 0 else 0
    throttle = (norm(target_direction) * mass / available_thrust) 
    # if terminal: throttle = max(descent_throttle(vessel, half_rocket_length), throttle)
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