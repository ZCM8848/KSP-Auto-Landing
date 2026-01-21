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

trf = create_target_reference_frame(conn, Targets_JNSQ.launchpad)
draw_reference_frame(conn, trf)
vessel = Rocket(space_center, vessel, trf)

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
    solution = generate_cubic_with_vertical_end(
        start_pos=position,
        start_vel=velocity,
        target_pos=array([0,0,0]),
        target_vel=array([0,0,0]),
        duration=estimate_duration(position, velocity, (0, 0, 0), (0, 0, 0))
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
    estimated_impact_speed = velocity[0] + vessel.vessel.thrust / mass * landing_time
    throttle = max(descent_throttle(vessel, half_rocket_length), throttle)
    vessel.vessel.control.throttle = throttle if velocity[0] < -2 else mass * g / available_thrust
    # target_direction = conic_clamp(array([0,0,1]), target_direction, 10)
    vessel.update_ap(target_direction)
    print(f"throttle: {throttle}, landing time: {landing_time}, estimated impact speed: {estimated_impact_speed}, duration:{estimate_duration(position, velocity, (0, 0, 0), (0, 0, 0))}")
    
    if landing_time < 5:
        terminal = True

    if velocity[0] >= 0:
        vessel.vessel.control.throttle = 0
        break