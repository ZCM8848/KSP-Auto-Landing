import krpc
from solver.RTLS.solver import ImapctPointSolver
from typing import Callable
import numpy as np

from internal.utils import roll_controller, ignition_height, descent_throttle, get_half_rocket_length, landed
from control import normalize, norm, array, conic_clamp
from control import Rocket
from internal.behaviors import create_target_reference_frame
from internal.targets import *

conn = krpc.connect("RTLS")
# tgt = (conn.space_center.target_vessel.flight().longitude, conn.space_center.target_vessel.flight().latitude)
tgt = Targets_JNSQ.landing_zone_2
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

IPS = ImapctPointSolver(
    vessel.vessel.mass,
    vessel.vessel.position(brf),
    vessel.vessel.velocity(brf),
    body.gravitational_parameter,
    body.angular_velocity(bnrf),
    (body.equatorial_radius, 0, 0)
)

print('AERODYNAMIC GUIDANCE:')
atd = body.atmosphere_depth
while True: 
    position = array(vessel.position())
    velocity = array(vessel.velocity())
    body_ip = IPS.predict_impact_point()[0][0, :3]
    ratio = position[0] / atd
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
    IPS.mass = vessel.vessel.mass
    IPS.position = np.array(vessel.vessel.position(brf))
    IPS.velocity = np.array(vessel.vessel.velocity(brf))
    if position[0] <= min(ignition_height(vessel, trf, 0, 0), 5000):
        vessel.vessel.control.throttle = 1
        break

while True:
    vessel.update_ap(-vessel.velocity())
    vessel.vessel.control.throttle = descent_throttle(vessel, get_half_rocket_length(vessel), 0)
    if landed(vessel) or vessel.velocity()[0] >= 0:
        vessel.vessel.control.throttle = 0
        break
    if vessel.position()[0] <= 500:
        vessel.vessel.control.gear = True