import krpc
import scipy as sp
import numpy as np
import math
from scipy.integrate import solve_ivp
from typing import Callable
from math import sqrt
from numba import jit
import pyautogui

from internal.utils import roll_controller
from control import normalize, norm, array, conic_clamp
from control import Rocket
from internal.behaviors import create_target_reference_frame
from internal.targets import *

conn = krpc.connect("RTLS")
# tgt = (conn.space_center.target_vessel.flight().longitude, conn.space_center.target_vessel.flight().latitude)
tgt = Targets_JNSQ.launchpad
trf = create_target_reference_frame(conn, tgt)
space_center = conn.space_center
vessel = Rocket(space_center, space_center.active_vessel, trf)
body = vessel.vessel.orbit.body
brf = body.reference_frame
bnrf = body.non_rotating_reference_frame
g0 = body.surface_gravity

def get_rocket_diameter(vessel: Rocket):
    parts = vessel.vessel.parts.all
    max_dims = np.zeros(3)
    min_dims = np.zeros(3)

    for i, part in enumerate(parts):
        pos = part.position(vessel.vessel.reference_frame)
        if i == 0:
            max_dims = np.array(pos)
            min_dims = np.array(pos)
        else:
            max_dims = np.maximum(max_dims, pos)
            min_dims = np.minimum(min_dims, pos)

    size_visual = max_dims - min_dims
    return size_visual[-1]

def factory(vessel: Rocket, target: Targets_JNSQ|Targets) -> tuple[Callable, Callable]:
    target_lon = target[0]
    target_lat = target[1]
    m = vessel.vessel.mass
    body_angular_velocity = array(body.angular_velocity(bnrf))
    gravitational_parameter = body.gravitational_parameter
    if body.bedrock_height(target_lat, target_lon) < 0:
        target_reference_frame_height = body.equatorial_radius
    else:
        target_reference_frame_height = body.equatorial_radius + body.surface_height(target_lat, target_lon)
    
    def dynamics(t, X):
        r = X[0:3]
        v = X[3:6]
        g = gravitational_parameter / norm(r)**2
        F = -2*m*np.cross(body_angular_velocity, array(v))
        a = -normalize(r)*g + F/m
        return np.concatenate([v, a])
    
    def impact(t, X):
        return norm(X[0:3]) - target_reference_frame_height
    impact.terminal = True
    impact.direction = -1
    return dynamics, impact

def impact_point(vessel: Rocket, d: Callable, e: Callable):
    position = array(vessel.vessel.position(brf))
    velocity = array(vessel.vessel.velocity(brf))
    solution = solve_ivp(
        fun=d,
        t_span=[0, 600],
        events=e,
        y0=np.concatenate([position, velocity]),
    )
    return solution.y_events

def solver_worker(vessel, target):
    d, e = factory(vessel, target)
    body_ip = impact_point(vessel, d, e)[0][0, :3]
    targ_ip = space_center.transform_position(tuple(body_ip), brf, trf)
    return targ_ip

error = [np.inf]
while True:
    d, e = factory(vessel, Targets_JNSQ.launchpad)
    position = array(vessel.position()) 
    velocity = array(vessel.velocity()) 
    body_ip = impact_point(vessel, d, e)[0][0, :3]
    targ_ip = space_center.transform_position(tuple(body_ip), brf, trf)
    estimated_landing_point = targ_ip
    horizontal_error = norm(estimated_landing_point[1:3])
    # t_c = max((velocity[0] - sqrt(velocity[0]**2 + 2 * g0 * position[0])) / g0, (velocity[0] + sqrt(velocity[0]**2 + 2 * g0 * position[0])) / g0)

    # target_direction = (0, -(position + t_c * velocity)[1], -(position + t_c * velocity)[2])
    target_direction = - normalize(estimated_landing_point)
    vessel.update_ap(target_direction)
    vessel.vessel.control.throttle = 1
    print("ERROR: %.3f" % (horizontal_error))
    if norm(estimated_landing_point[1:3]) <= 5000 and norm(estimated_landing_point[1:3]) > min(error):
        vessel.vessel.control.throttle = 0
        pyautogui.press('x')
        break
    else:
        error.append(norm(estimated_landing_point[1:3]))

print('AERODYNAMIC GUIDANCE:')
while True: 
    d, e = factory(vessel, tgt)
    position = array(vessel.position())
    velocity = array(vessel.velocity())
    body_ip = impact_point(vessel, d, e)[0][0, :3]
    targ_ip = space_center.transform_position(tuple(body_ip), brf, trf)
    estimated_landing_point = targ_ip
    altitude = vessel.flight(trf).surface_altitude
    horizontal_error = norm(estimated_landing_point[1:3])
    gfold_start_altitude = max(5 * horizontal_error, 3000)

    target_direction = -velocity + array([0, estimated_landing_point[1], estimated_landing_point[2]])
    target_direction = normalize(target_direction) + normalize(position)
    target_direction = conic_clamp(-velocity, target_direction, 10)
    vessel.update_ap(target_direction, roll_controller(vessel.vessel.flight(trf).heading))
    vessel.vessel.control.throttle = 0
    print(horizontal_error)