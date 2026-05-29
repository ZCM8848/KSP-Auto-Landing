import krpc
from solver.RTLS.solver import ImapctPointSolver
from solver.landing.solver import SuicideBurnSolver
from typing import List, Tuple
from math import sin, cos, radians
from tqdm import trange
import numpy as np

from internal.utils import roll_controller, ignition_height, descent_throttle, get_half_rocket_length, landed, transform_to_target_frame, transform_to_body_frame
from control import normalize, norm, array, conic_clamp, angle_between
from control import Rocket
from internal.behaviors import create_target_reference_frame
from internal.targets import *

def get_density_data(vessel, trf, atd: float) -> List[Tuple[float, float]]:
    density = []
    altitude = []

    for alt in trange(0, int(atd), 100, desc='sampling atmosphere'):
        pos = (alt, 0, 0)
        altitude.append(alt)
        density.append(vessel.orbit.body.atmospheric_density_at_position(pos, trf))

    altitude = np.array(altitude)
    density = np.array(density)

    return list(zip(altitude.tolist(), density.tolist()))

STT = array([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0]])
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
target_origion = array(space_center.transform_position((0,0,0), trf, brf))

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
    (body.equatorial_radius, 0, 0),
    0,
    0,
    get_rocket_diameter(vessel),
    2*get_half_rocket_length(vessel),
    body.equatorial_radius,
    radians(5),
    0,
    (1, 0, 0),
    get_density_data(vessel, trf, body.atmosphere_depth),
    body.bedrock_height(tgt[1], tgt[0])
)
SSS = SuicideBurnSolver(
    vessel.vessel.mass,
    vessel.vessel.dry_mass,
    vessel.vessel.position(brf),
    vessel.vessel.velocity(brf),
    body.gravitational_parameter,
    body.angular_velocity(bnrf),
    (body.equatorial_radius, 0, 0),
    vessel.vessel.available_thrust,
    vessel.vessel.specific_impulse,
    get_rocket_diameter(vessel),
    2*get_half_rocket_length(vessel),
    get_density_data(vessel, trf, body.atmosphere_depth),
    vessel.vessel.flight(brf).drag_coefficient,
    body.equatorial_radius,
)


error = [np.inf]
while True:
    position = vessel.position()
    velocity = vessel.velocity()
    print(position, velocity)
    body_ip = IPS.predict_impact_point()['event_X'][0, :3]
    targ_ip = transform_to_target_frame(body_ip, target_origion, tgt[0], tgt[1])
    # targ_ip = space_center.transform_position(tuple(body_ip), brf, trf)
    estimated_landing_point = targ_ip
    horizontal_error = norm(estimated_landing_point[1:3])
    # t_c = max((velocity[0] - sqrt(velocity[0]**2 + 2 * g0 * position[0])) / g0, (velocity[0] + sqrt(velocity[0]**2 + 2 * g0 * position[0])) / g0)

    # target_direction = (0, -(position + t_c * velocity)[1], -(position + t_c * velocity)[2])
    target_direction = - normalize(estimated_landing_point)
    vessel.update_ap(target_direction)
    vessel.vessel.control.throttle = 1
    IPS.mass = vessel.vessel.mass
    IPS.position = transform_to_body_frame(position, target_origion, tgt[0], tgt[1])
    IPS.velocity = transform_to_body_frame(velocity, target_origion, tgt[0], tgt[1], is_velocity=True)
    print("ERROR: %.3f" % (horizontal_error))
    if norm(estimated_landing_point[1:3]) <= 3000:
        vessel.vessel.control.throttle = 0.4
    if norm(estimated_landing_point[1:3]) <= 5000 and norm(estimated_landing_point[1:3]) > min(error):
        vessel.vessel.control.throttle = 0
        break
    else:
        error.append(norm(estimated_landing_point[1:3]))

while vessel.velocity()[0] > -150:
    vessel.update_ap((1, 0, 0))
    pass
print('AERODYNAMIC GUIDANCE:')
atd = body.atmosphere_depth
igh = vessel.position()[0]
while True: 
    position = vessel.position()
    velocity = vessel.velocity()
    body_ip = IPS.predict_impact_point()['event_X'][0, :3]
    # ratio = position[0] / atd
    targ_ip = transform_to_target_frame(body_ip, target_origion, tgt[0], tgt[1])
    estimated_landing_point = targ_ip
    # estimated_landing_point = (1-ratio)*array([0, position[1], position[2]]) + ratio*array(estimated_landing_point)
    altitude = vessel.vessel.flight(trf).surface_altitude
    horizontal_error = norm(estimated_landing_point[1:3])
    gfold_start_altitude = max(5 * horizontal_error, 3000)

    target_direction = -velocity + array([0, estimated_landing_point[1], estimated_landing_point[2]])
    target_direction = normalize(target_direction) + normalize(position)
    target_direction = conic_clamp(-velocity, target_direction, 10)
    vessel.update_ap(target_direction, roll_controller(vessel.vessel.flight(trf).heading))
    vessel.vessel.control.throttle = 0
    igh = targ_ip[0]
    print(horizontal_error, igh)
    IPS.mass = SSS.mass = vessel.vessel.mass
    IPS.position = SSS.position = transform_to_body_frame(position, target_origion, tgt[0], tgt[1])
    IPS.velocity = SSS.velocity = transform_to_body_frame(velocity, target_origion, tgt[0], tgt[1], is_velocity=True)
    IPS.drag_coefficient = SSS.drag_coefficient = vessel.vessel.flight(brf).drag_coefficient
    IPS.lift_coefficient = vessel.vessel.flight(brf).lift_coefficient
    IPS.AoA = angle_between(-velocity, target_direction)
    if position[0] <= min(ignition_height(vessel, trf, 0, 0), 10000):
        vessel.vessel.control.throttle = 1
        break

while True:
    position = vessel.position()
    velocity = vessel.velocity()
    body_ip = IPS.predict_impact_point()['event_X'][0, :3]
    # ratio = position[0] / atd
    targ_ip = transform_to_target_frame(body_ip, target_origion, tgt[0], tgt[1])
    estimated_landing_point = targ_ip
    target_direction = - normalize(estimated_landing_point)
    target_direction = conic_clamp(-velocity, target_direction, 3)
    vessel.update_ap(target_direction)
    vessel.vessel.control.throttle = descent_throttle(vessel, get_half_rocket_length(vessel), 0)
    if landed(vessel) or vessel.velocity()[0] >= 0:
        vessel.vessel.control.throttle = 0
        break
    if vessel.position()[0] <= 1000:
        vessel.vessel.control.gear = True