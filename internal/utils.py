import krpc
from tqdm import trange
from numpy.linalg import norm
from collections import Counter

from math import sin, cos, radians, sqrt
from .targets import Targets, Targets_JNSQ

# define target reference frame
def create_target_reference_frame(conn:krpc.client.Client, target):
    space_center = conn.space_center
    body = space_center.active_vessel.orbit.body
    body_reference_frame = body.reference_frame
    target_lon = target[0]
    target_lat = target[1]
    # spin around y-axis by -target_lon degrees
    temp_reference_frame = space_center.ReferenceFrame.create_relative(body_reference_frame, rotation=(0., sin(-radians(target_lon / 2)), 0., cos(-radians(target_lon / 2))))
    # spin around z axis by target_lat degrees
    temp_reference_frame = space_center.ReferenceFrame.create_relative(temp_reference_frame, rotation=(0., 0., sin(radians(target_lat / 2)), cos(radians(target_lat / 2))))
    if body.bedrock_height(target_lat, target_lon) < 0:
        target_reference_frame_height = body.equatorial_radius
    else:
        target_reference_frame_height = body.equatorial_radius + body.surface_height(target_lat, target_lon)
    reference_frame = space_center.ReferenceFrame.create_relative(temp_reference_frame, position=(target_reference_frame_height, 0., 0.))
    # spin around y-axis by 90 degrees
    temp_reference_frame = space_center.ReferenceFrame.create_relative(reference_frame, rotation=(0., sin(radians(45)), 0., cos(radians(45))))
    # spin around z-axis by 90 degrees
    reference_frame = space_center.ReferenceFrame.create_relative(temp_reference_frame, rotation=(0., 0., sin(radians(45)), cos(radians(45))))
    return reference_frame


# debug
def draw_reference_frame(conn:krpc.client.Client, reference_frame):
    x_axis = conn.drawing.add_line((10, 0, 0), (0, 0, 0), reference_frame)
    x_axis.color = (1, 0, 0)  # red
    x_axis.thickness = 0.5
    y_axis = conn.drawing.add_line((0, 10, 0), (0, 0, 0), reference_frame)
    y_axis.color = (0, 1, 0)  # green
    y_axis.thickness = 0.5
    z_axis = conn.drawing.add_line((0, 0, 10), (0, 0, 0), reference_frame)
    z_axis.color = (0, 0, 1)  # blue
    z_axis.thickness = 0.5

def draw_direction(conn:krpc.client.Client, direction, reference_frame):
    direction = conn.drawing.add_direction(direction, reference_frame)
    direction.thickness = 0.1

def draw_line(conn:krpc.client.Client, origin, terminal, colour, reference_frame):
    line = conn.drawing.add_line(reference_frame=reference_frame, start=origin, end=terminal)
    line.thickness = 0.1
    line.color = colour
    return line

def draw_trajectory(conn, x, u, reference_frame):
    lines = []
    for i in trange(len(x), desc='drawing trajectory'):
        if i >= 1:
            line_x = draw_line(conn=conn, colour=(255, 255, 255), origin=(x[i - 1, 0], x[i - 1, 1], x[i - 1, 2]),
                      terminal=(x[i, 0], x[i, 1], x[i, 2]),
                      reference_frame=reference_frame)
            line_u = draw_line(conn=conn, colour=(0, 0, 255), origin=(x[i - 1, 0], x[i - 1, 1], x[i - 1, 2]),
                  terminal=(x[i - 1, 0] + u[i - 1, 0], x[i - 1, 1] + u[i - 1, 1], x[i - 1, 2] + u[i - 1, 2]),
                  reference_frame=reference_frame)
            lines.append(line_x)
            lines.append(line_u)
    return lines

def clear_lines(lines):
    for line in lines:
        line.remove()


# other utilities
def get_all_available_vessels(conn:krpc.client.Client):
    return conn.space_center.vessels

def use_JNSQ(conn:krpc.client.Client):
    return conn.space_center.bodies['Kerbin'].atmosphere_depth > 70000

def find_vessel_by_name(conn:krpc.client.Client, name): #This function shouldn't be here, I will move it to another file
    space_center = conn.space_center
    last_matching_vessel = None
    for vessel in space_center.vessels:
        if vessel.name == name:
            last_matching_vessel = vessel
    return last_matching_vessel

def define_targets():
    if use_JNSQ():
        return Targets_JNSQ
    else:
        return Targets

# control utilities
def get_half_rocket_length(rocket):
    vessel_reference_frame = rocket.reference_frame
    part_distance = [norm(part.position(vessel_reference_frame)) for part in rocket.parts.all if part.position(vessel_reference_frame)[1] < 0]
    value_weight_dict = dict(Counter(part_distance))
    total_weight = len(part_distance)
    weighted_sum = sum(value * weight for value, weight in value_weight_dict.items())
    return weighted_sum / total_weight

def landed(rocket):
    legs = rocket.parts.legs
    return all(leg.is_grounded for leg in legs)

def has_legs(rocket):
    return len(rocket.parts.legs) > 0