import krpc
from solver.RTLS.solver import ImapctPointSolver
from typing import Callable
from math import sin, cos, radians
import numpy as np

from internal.utils import roll_controller, ignition_height, descent_throttle, get_half_rocket_length, landed, transform_to_target_frame, transform_to_body_frame
from control import normalize, norm, array, conic_clamp
from control import Rocket
from internal.behaviors import create_target_reference_frame
from internal.targets import *

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

print(get_rocket_diameter(vessel))
print(get_half_rocket_length(vessel))
print(vessel.vessel.mass)
print(vessel.vessel.dry_mass)
print(vessel.vessel.available_thrust)
print(vessel.vessel.specific_impulse)
print(vessel.vessel.flight(trf).drag_coefficient)
print("="*50)
print(body.gravitational_parameter)
print(body.equatorial_radius)
print(body.rotational_speed)
print(Targets_JNSQ.launchpad)
print(body.surface_height(Targets_JNSQ.launchpad[1], Targets_JNSQ.launchpad[0]))
print("="*50)
print(vessel.position())
print(vessel.velocity())