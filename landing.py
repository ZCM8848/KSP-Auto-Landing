import krpc
from collections import Counter
from tqdm import trange
from math import sqrt

from Control import *
from Solver import GFOLD

# params
throttle_limit = [0.2, 0.9]
max_tilt = 10

# define basic KRPC things
conn = krpc.connect(name='KAL')
space_center = conn.space_center
vessel = space_center.active_vessel
body = vessel.orbit.body
g = body.surface_gravity

# define reference frames
vessel_reference_frame = vessel.reference_frame
vessel_orbital_reference_frame = vessel.orbital_reference_frame
vessel_surface_reference_frame = vessel.surface_reference_frame
vessel_surface_velocity_reference_frame = vessel.surface_velocity_reference_frame
body_reference_frame = body.reference_frame


# define targets
class Targets:
    launchpad = (-74.557673364044, -0.0972078545440865)
    landing_zone_1 = (-74.4730633292066, -0.185355657540052)
    landing_zone_2 = (-74.4729967713462, -0.20551670559373)
    landing_zone_3 = (-74.4853049576317, -0.195548763275873)


# noinspection PyPep8Naming
class Targets_JNSQ:
    launchpad = (-91.7839786112259, 5.1753303155099E-06)
    VAB_A = (-91.8063860071064, -4.23555000546582E-06)


# define target reference frame
def create_target_reference_frame(target):
    target_vessel = space_center.target_vessel
    if target_vessel:
        target_lon = target_vessel.flight(body_reference_frame).longitude
        target_lat = target_vessel.flight(body_reference_frame).latitude
    else:
        target_lon = target[0]
        target_lat = target[1]
    # spin around y-axis by -target_lon degrees
    temp_reference_frame = space_center.ReferenceFrame.create_relative(body_reference_frame,
                                                                       rotation=(0., sin(-deg2rad(target_lon / 2)), 0.,
                                                                                 cos(-deg2rad(target_lon / 2))))
    # spin around z axis by target_lat degrees
    temp_reference_frame = space_center.ReferenceFrame.create_relative(temp_reference_frame,
                                                                       rotation=(0., 0., sin(deg2rad(target_lat / 2)),
                                                                                 cos(deg2rad(target_lat / 2))))
    if body.bedrock_height(target_lat, target_lon) < 0:
        target_reference_frame_height = body.equatorial_radius
    else:
        target_reference_frame_height = body.equatorial_radius + body.surface_height(target_lat, target_lon)
    reference_frame = space_center.ReferenceFrame.create_relative(temp_reference_frame,
                                                                  position=(target_reference_frame_height, 0., 0.))
    return reference_frame


# debug
def draw_reference_frame(reference_frame):
    x_axis = conn.drawing.add_line((10, 0, 0), (0, 0, 0), reference_frame)
    x_axis.color = (1, 0, 0)  # red
    x_axis.thickness = 0.5
    y_axis = conn.drawing.add_line((0, 10, 0), (0, 0, 0), reference_frame)
    y_axis.color = (0, 1, 0)  # green
    y_axis.thickness = 0.5
    z_axis = conn.drawing.add_line((0, 0, 10), (0, 0, 0), reference_frame)
    z_axis.color = (0, 0, 1)  # blue
    z_axis.thickness = 0.5


def draw_direction(direction, reference_frame):
    direction = conn.drawing.add_direction(direction, reference_frame)
    direction.thickness = 0.1


def draw_line(origin, terminal, colour, reference_frame):
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
def get_half_rocket_length(rocket):
    part_distance = [norm(part.position(vessel_reference_frame)) for part in rocket.parts.all if
                     part.position(vessel_reference_frame)[1] < 0]
    value_weight_dict = dict(Counter(part_distance))
    total_weight = len(part_distance)
    weighted_sum = sum(value * weight for value, weight in value_weight_dict.items())
    return weighted_sum / total_weight


def landed(rocket):
    legs = rocket.parts.legs
    return all(leg.is_grounded for leg in legs)


def ignition_height(reference_frame):
    return abs(vessel.flight(reference_frame).vertical_speed ** 2 / (
            2 * (throttle_limit[1] * vessel.available_thrust / vessel.mass - g)))


# solver utilities
def bundle_data(rocket):
    data = {'gravity': g, 'dry_mass': rocket.dry_mass, 'fuel_mass': rocket.mass - rocket.dry_mass,
            'max_thrust': rocket.available_thrust, 'min_throttle': throttle_limit[0],
            'max_throttle': throttle_limit[1], 'max_structural_Gs': 9,
            'specific_impulse': rocket.specific_impulse,
            'max_velocity': rocket.flight(target_reference_frame).speed, 'glide_slope_cone': 20,
            'thrust_pointing_constraint': max_tilt,
            'planetary_angular_velocity': body.angular_velocity(vessel_surface_reference_frame),
            'initial_position': rocket.position(target_reference_frame),
            'initial_velocity': rocket.velocity(target_reference_frame),
            'target_position': (get_half_rocket_length(rocket), 0, 0), 'target_velocity': (0, 0, 0),
            'prog_flag': 'p4', 'solver': 'ECOS', 'N_tf': 160, 'plot': False, 'min_tf': min_tf,
            'max_tf': int(min_tf * sqrt(3))}
    return data


target_reference_frame = create_target_reference_frame(target=Targets_JNSQ.launchpad)
half_rocket_length = get_half_rocket_length(vessel)
draw_reference_frame(target_reference_frame)
draw_reference_frame(vessel_reference_frame)

# activate autopilot
vessel.auto_pilot.reference_frame = vessel_surface_reference_frame
vessel.auto_pilot.engage()

# aerodynamic guidance
print('AERODYNAMIC GUIDANCE:')
while vessel.position(target_reference_frame)[0] >= min(ignition_height(target_reference_frame), 10000):
    position = array(vessel.position(target_reference_frame))
    velocity = array(vessel.velocity(target_reference_frame))
    estimated_landing_time = norm(position) / norm(velocity)
    estimated_landing_point = position + estimated_landing_time * velocity

    target_direction = - velocity + estimated_landing_point
    target_direction = conic_clamp(-velocity, target_direction, 7)
    vessel.auto_pilot.target_direction = target_direction
    vessel.control.throttle = 0
    print('    HEIGHT:%.3f | IGNITION HEIGHT:%.3f' % (position[0], ignition_height(target_reference_frame)))

# initialize landing burn
print('LANDING BURN:')
while vessel.velocity(target_reference_frame)[0] <= -100 and vessel.position(target_reference_frame)[0] >= 2000:
    position = array(vessel.position(target_reference_frame))
    velocity = array(vessel.velocity(target_reference_frame))
    available_thrust = vessel.available_thrust
    mass = vessel.mass

    remaining_height = position[0] - 2000

    acc = (-100 ** 2 + velocity[0] ** 2) / (2 * remaining_height)

    target_direction = -array(
        space_center.transform_direction(velocity, from_=target_reference_frame, to=vessel_surface_reference_frame))
    vessel.auto_pilot.target_direction = target_direction
    vessel.control.throttle = clamp((acc * mass) / available_thrust, throttle_limit[0], throttle_limit[1])
    print('    THROTTLE:%.3f' % vessel.control.throttle)

while vessel.velocity(target_reference_frame)[0] <= -100:
    velocity = array(vessel.velocity(target_reference_frame))
    target_direction = -array(
        space_center.transform_direction(velocity, from_=target_reference_frame, to=vessel_surface_reference_frame))
    vessel.control.throttle = throttle_limit[1]
    print('    THROTTLE:%.3f' % vessel.control.throttle)

# final landing phase
conn.krpc.paused = True
conn.ui.message('GENERATING SOLUTION', duration=1)
min_tf = int(sqrt(2 * vessel.position(target_reference_frame)[0] / g))

bundled_data = bundle_data(vessel)
problem = GFOLD(bundled_data)
problem.find_optimal_solution()

result = problem.solution

draw_trajectory(result['x'], result['u'], target_reference_frame)
conn.ui.message('SOLUTION GENERATED', duration=1)
conn.krpc.paused = False

nav_mode = 'GFOLD'
end = False

trajectory = array(result['x'])
trajectory_position = [(trajectory[0, i], trajectory[1, i], trajectory[2, i]) for i in range(len(trajectory[0]))]
trajectory_velocity = [(trajectory[3, i], trajectory[4, i], trajectory[5, i]) for i in range(len(trajectory[0]))]
trajectory_acceleration = [(result['u'][0, i], result['u'][1, i], result['u'][2, i]) for i in
                           range(len(result['u'][0]))]

print('GFOLD PHASE:')

while not end:
    while nav_mode == 'GFOLD':
        # gather information
        velocity = array(vessel.velocity(target_reference_frame))
        position = array(vessel.position(target_reference_frame))
        thrust = vessel.thrust
        mass = vessel.mass
        available_thrust = vessel.available_thrust
        aerodynamic_force = array(vessel.flight(target_reference_frame).aerodynamic_force)

        # get the index of nearest waypoints
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
        target_direction_x = target_direction[0]
        while target_direction_x <= 0:
            target_direction_x = target_direction_x + g
        target_direction = (target_direction_x, target_direction[1], target_direction[2])
        target_direction = conic_clamp(-normalize((velocity)) + 5 * array([1, 0, 0]), target_direction, max_tilt)
        target_direction = space_center.transform_direction(target_direction, from_=target_reference_frame,
                                                            to=vessel_surface_reference_frame)
        compensation = norm(aerodynamic_force[1:3]) / available_thrust
        throttle = norm(target_direction) / (available_thrust / mass) + compensation
        throttle = clamp(throttle, throttle_limit[0], throttle_limit[1])
        vessel.control.throttle = throttle
        vessel.auto_pilot.target_direction = target_direction
        print('    THROTTLE:%3f | COMPENSATION:%3f | INDEX:%i' % (throttle, compensation, min_index))

        if norm(position) <= 4 * half_rocket_length or velocity[0] >= -2:
            nav_mode = 'PID'
            vessel.control.legs = True
            print('PID LANDING PHASE:')
            break

        if norm(position) / norm(velocity) <= 4:
            vessel.control.legs = True

    while nav_mode == 'PID':
        velocity = array(vessel.velocity(target_reference_frame))
        position = array(vessel.position(target_reference_frame))
        available_thrust = vessel.available_thrust
        mass = vessel.mass

        max_acc = throttle_limit[1] * (available_thrust / mass) - g
        max_acc_low = throttle_limit[1] * 0.8 * (available_thrust / mass) - g
        est_h = position[0] - velocity[0] ** 2 / (2 * max_acc)
        est_h_low = position[0] - velocity[0] ** 2 / (2 * max_acc_low)
        est_h_center = (est_h + est_h_low) / 2
        position_hor = array([0, position[1], position[2]])
        vel_hor = array([0, velocity[1], velocity[2]])
        ctrl_hor = -position_hor * 0.03 - vel_hor * 0.06
        target_direction = ctrl_hor + array([1, 0, 0])
        throttle = clamp(0.5 * (-2 - velocity[0]), throttle_limit[0], throttle_limit[1])
        vessel.control.throttle = throttle
        vessel.auto_pilot.target_direction = conic_clamp(array([1, 0, 0]), target_direction, 3)

        print('    THROTTLE:%.3f' % throttle)

        if landed(vessel):
            vessel.control.throttle = 0.
            vessel.control.sas = True
            print('END')
            end = True
            break
