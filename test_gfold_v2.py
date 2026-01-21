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

def has_nan(a):
    return bool(np.isnan(a).any())


def solve_gfold(vessel):
    available_thrust = vessel.vessel.available_thrust
    solver.update_parameter("initial_position", vessel.position())
    solver.update_parameter("initial_velocity", vessel.velocity())
    solver.update_parameter('target_velocity', array([0, 0, 0]))
    solver.update_parameter("log_mass", log(vessel.vessel.mass))
    solver.update_parameter("log_dry_mass", log(vessel.vessel.dry_mass))
    solver.update_parameter("max_vel", 1000)
    solver.update_parameter("sin_glide_slope", sin(45))
    solver.update_parameter("min_thrust", available_thrust * throttle_limits[0])
    solver.update_parameter("max_thrust", available_thrust * throttle_limits[1])
    solver.update_parameter('fuel_consumption', 1 / (g*vessel.vessel.specific_impulse))
    solver.update_parameter('max_angle', 10)
    solver.update_parameter('gravity', array([0, 0, -g]))
    result = dict()
    problem.solve(method='CPG')
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
    vessel.update_ap(array([0,0,1]))
    vessel.vessel.control.throttle = 0
    #print(vessel.position(), vessel.velocity())
    t0 = time.time()
    status = solve_gfold(vessel)['status']
    print(f"solver status: {status}")
    # print(f"Time cost: {time.time() - t0}")
    # break
    if all(ch not in status for ch in ("0", "4")):
        break

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

    if need_retry:
        # print(position, velocity)
        t0 = time.time()
        last_retry_time = space_center.ut
        solution = solve_gfold(vessel)
        status = solution['status']
        if all(ch not in status for ch in ("0", "4")):
            print("All constraints satisfied")
        else:
            print("Some constraints not satisfied")
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