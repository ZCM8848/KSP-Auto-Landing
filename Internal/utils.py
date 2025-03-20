from tqdm import trange
from collections import Counter

from Control import sin, cos, radians, norm, sqrt, clamp, degrees

# define target reference frame
def create_target_reference_frame(conn, target):
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
    return reference_frame


# debug
def draw_reference_frame(conn, reference_frame):
    x_axis = conn.drawing.add_line((10, 0, 0), (0, 0, 0), reference_frame)
    x_axis.color = (1, 0, 0)  # red
    x_axis.thickness = 0.5
    y_axis = conn.drawing.add_line((0, 10, 0), (0, 0, 0), reference_frame)
    y_axis.color = (0, 1, 0)  # green
    y_axis.thickness = 0.5
    z_axis = conn.drawing.add_line((0, 0, 10), (0, 0, 0), reference_frame)
    z_axis.color = (0, 0, 1)  # blue
    z_axis.thickness = 0.5

def draw_direction(conn, direction, reference_frame):
    direction = conn.drawing.add_direction(direction, reference_frame)
    direction.thickness = 0.1

def draw_line(conn, origin, terminal, colour, reference_frame):
    line = conn.drawing.add_line(reference_frame=reference_frame, start=origin, end=terminal)
    line.thickness = 0.1
    line.color = colour

def draw_trajectory(x, u, reference_frame):
    for i in trange(len(x[0]), desc='drawing trajectory'):
        if i >= 1:
            draw_line(colour=(255, 255, 255), origin=(x[0, i - 1], x[1, i - 1], x[2, i - 1]),
                      terminal=(x[0, i], x[1, i], x[2, i]), reference_frame=reference_frame)
            draw_line(colour=(0, 0, 255), origin=(x[0, i - 1], x[1, i - 1], x[2, i - 1]),
                      terminal=(x[0, i - 1] + u[0, i], x[1, i - 1] + u[1, i], x[2, i - 1] + u[2, i]),
                      reference_frame=reference_frame)


# control utilities
def find_vessel_by_name(conn, name): #This function shouldn't be here, I will move it to another file
    space_center = conn.space_center
    for vessel in space_center.vessels:
        if vessel.name == name:
            return vessel
    return None

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

def ignition_height(rocket, reference_frame, altitude, velocity):
    body = rocket.orbit.body
    g = body.surface_gravity
    current_height = rocket.flight(reference_frame).mean_altitude
    current_velocity = rocket.flight(reference_frame).vertical_speed
    mass = rocket.mass
    available_thrust = rocket.available_thrust
    kinetic_energy_change = 0.5 * mass * (velocity ** 2 - current_velocity ** 2)
    potential_energy_change = mass * g * (altitude - current_height)
    total_energy_change = kinetic_energy_change + potential_energy_change
    ignition_height = total_energy_change / (available_thrust - mass * g)

    return abs(ignition_height)

def descent_throttle(rocket, target_height=0, vt=-2):
    body = rocket.orbit.body
    vessel_reference_frame = rocket.reference_frame
    position = rocket.position()[0]
    velocity = rocket.velocity()[0]
    g = body.surface_gravity
    aero_force = rocket.flight(vessel_reference_frame).aerodynamic_force[2]
    mass = rocket.mass
    available_thrust = rocket.available_thrust

    acc = (vt**2 + velocity**2) / (2 * (position - target_height)) + g + aero_force / (mass * g)
    return mass * acc / available_thrust

def impact_point(rocket, reference_frame):
    body = rocket.orbit.body
    g = body.surface_gravity
    position = rocket.position()
    velocity = rocket.velocity()
    terminal_velocity = rocket.flight(reference_frame).terminal_velocity
    estimated_landing_time1 = norm(position) / norm(velocity)
    velocity = rocket.velocity()
    terminal_velocity = rocket.flight(reference_frame).terminal_velocity
    estimated_landing_time1 = norm(position) / norm(velocity)
    estimated_landing_time2 = max((velocity[0] - sqrt(velocity[0]**2 + 2 * g * position[0])) / g, (velocity[0] + sqrt(velocity[0]**2 + 2 * g * position[0])) / g)
    ratio = clamp(norm(velocity) / terminal_velocity, 0, 1)
    estimated_landing_time = (ratio) * estimated_landing_time1 + (1 - ratio) * estimated_landing_time2
    estimated_landing_time = estimated_landing_time2
    estimated_landing_point = position + estimated_landing_time * velocity
    return estimated_landing_point

def roll_controller(heading):
    if heading >= 0 and heading <= 90:
        return radians(90 + heading)
    elif heading > 90 and heading <= 180:
        return radians(90 - heading + 180)
    elif heading > 180 and heading <= 270:
        return radians(90 + heading - 180)
    else:
        return radians(90 - heading + 360)

# solver utilities
def bundle_data(rocket, target_reference_frame, params):
    body = rocket.orbit.body
    g = body.surface_gravity
    vessel_surface_reference_frame = rocket.surface_reference_frame
    min_tf = int(sqrt(2 * rocket.position()[0] / g))
    N = 200
    data = {'vessel': rocket, 'trf': target_reference_frame,
            'gravity': g, 'dry_mass': rocket.dry_mass, 'fuel_mass': rocket.mass - rocket.dry_mass,
            'max_thrust': rocket.available_thrust,
            'min_throttle': params['throttle_limit'][0], 'max_throttle': params['throttle_limit'][1],
            'max_structural_Gs': 3,
            'specific_impulse': rocket.specific_impulse,
            'max_velocity': rocket.flight(target_reference_frame).speed, 'glide_slope_cone': 20,
            'thrust_pointing_constraint': degrees(params['max_tilt']),
            'planetary_angular_velocity': body.angular_velocity(vessel_surface_reference_frame),
            'initial_position': rocket.position(),
            'initial_velocity': rocket.velocity(),
            'target_position': (600, 0, 0), 'target_velocity': (-50, 0, 0),
            'prog_flag': 'p4', 'solver': 'ECOS', 'N_tf': N, 'plot': False, #160
            'min_tf': min_tf, 'max_tf': int(min_tf * sqrt(3))}
    return data
