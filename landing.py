import time

import krpc
from numpy import array, cos, sin, deg2rad, rad2deg, arccos, sqrt, inf, mean, average, exp
from numpy.linalg import norm
from collections import Counter
from tqdm import trange

from PID import PID, clamp
from GFOLD_SOLVER import GFOLD, generate_solution
from vector import vec_ang, vec_around, vec_clamp, vec_clamp_yz, normalize, cone

#define basic KRPC things
conn = krpc.connect(name='KAL')
space_center = conn.space_center
vessel = space_center.active_vessel
body = vessel.orbit.body
g = body.surface_gravity

#define reference frames
vessel_reference_frame = vessel.reference_frame
vessel_orbital_reference_frame = vessel.orbital_reference_frame
vessel_surface_reference_frame = vessel.surface_reference_frame
vessel_surface_velocity_reference_frame = vessel.surface_velocity_reference_frame
body_reference_frame = body.reference_frame

#define targets
class targets:
    launchpad = (-74.557673364044,-0.0972078545440865)
    landing_zone_1 = (-74.4730633292066,-0.185355657540052)
    landing_zone_2 = (-74.4729967713462,-0.20551670559373)
    landing_zone_3 = (-74.4853049576317,-0.195548763275873)

class targets_JNSQ:
    launchpad = (-91.7839786112259,5.1753303155099E-06)
    VAB_A = (-91.8063860071064,-4.23555000546582E-06)


#define target reference frame *(Search for "Quaternion vs. Three-Dimensional Rotation" on your own)*
def create_target_reference_frame(target):
    target_vessel = space_center.target_vessel
    if target_vessel:
        target_lon = target_vessel.flight(body_reference_frame).longitude
        target_lat = target_vessel.flight(body_reference_frame).latitude
    else:
        target_lon = target[0]
        target_lat = target[1]
    temp_reference_frame = space_center.ReferenceFrame.create_relative(body_reference_frame, 
                                                                       rotation=(0., sin(-deg2rad(target_lon/2)), 0., cos(-deg2rad(target_lon/2))))#spin around y axis by -target_lon degrees
    temp_reference_frame = space_center.ReferenceFrame.create_relative(temp_reference_frame, 
                                                                       rotation=(0., 0., sin(deg2rad(target_lat/2)), cos(deg2rad(target_lat/2))))  #spin around z axis by target_lat degrees
    if body.bedrock_height(target_lat, target_lon) < 0:
        target_reference_frame_height = body.equatorial_radius
    else:
        target_reference_frame_height = body.equatorial_radius + body.surface_height(target_lat, target_lon)
    target_reference_frame = space_center.ReferenceFrame.create_relative(temp_reference_frame, 
                                                                         position=(target_reference_frame_height, 0., 0.))
    return target_reference_frame

#define instant reference frame
def create_instant_reference_frame():
    vessel_lon = vessel.flight().longitude
    vessel_lat = vessel.flight().latitude
    temp_reference_frame = space_center.ReferenceFrame.create_relative(body_reference_frame, 
                                                                       rotation=(0., sin(-deg2rad(vessel_lon/2.)), 0., cos(-deg2rad(vessel_lon/2.))))#spin around y axis by -target_lon degrees
    temp_reference_frame = space_center.ReferenceFrame.create_relative(temp_reference_frame, 
                                                                       rotation=(0., 0., sin(deg2rad(vessel_lat/2.)), cos(deg2rad(vessel_lat/2.))))  #spin around z axis by target_lat degrees
    if body.bedrock_height(vessel_lat, vessel_lon) < 0:
        instant_reference_frame_height = body.equatorial_radius
    else:
        instant_reference_frame_height = body.equatorial_radius + body.surface_height(vessel_lat,vessel_lon)
    instant_reference_frame = space_center.ReferenceFrame.create_relative(temp_reference_frame, 
                                                                          position=(instant_reference_frame_height,0.,0.))
    return instant_reference_frame

#debug
def draw_reference_frame(reference_frame):
    x_axis = conn.drawing.add_line((10,0,0),(0,0,0), reference_frame)
    x_axis.color = (1, 0, 0)#red
    x_axis.thickness = 0.5
    y_axis = conn.drawing.add_line((0,10,0),(0,0,0), reference_frame)
    y_axis.color = (0, 1, 0)#green
    y_axis.thickness = 0.5
    z_axis = conn.drawing.add_line((0,0,10),(0,0,0), reference_frame)
    z_axis.color = (0, 0, 1)#blue
    z_axis.thickness = 0.5

def draw_direction(direction, reference_frame):
    direction = conn.drawing.add_direction(direction, reference_frame)
    direction.thickness = 0.1

def draw_line(start, end, colour,reference_frame):
    position = conn.drawing.add_line(reference_frame=reference_frame,start=start,end=end)
    position.thickness = 0.1
    position.color = colour

def draw_trajectory(x,u,reference_frame):
    for i in trange(len(x[0]), desc='drawing trajectory'):
        if i >=1:
            draw_line(colour=(255,255,255),start=(x[0,i-1],x[1,i-1],x[2,i-1]),end=(x[0,i],x[1,i],x[2,i]),reference_frame=reference_frame)
            draw_line(colour=(0,0,255),start=(x[0,i-1],x[1,i-1],x[2,i-1]),end=(x[0,i-1]+u[0,i],x[1,i-1]+u[1,i],x[2,i-1]+u[2,i]),reference_frame=reference_frame)

#control utilities
def find_nearest_waypoints(current_position, trajectory, last_index):
    results_position = []
    current_position = array(current_position)
    trajectory = array(trajectory)
    trajectory_position = [(trajectory[0,i],trajectory[1,i],trajectory[2,i]) for i in range(len(trajectory[0]))]
    trajectory_velocity = [(trajectory[3,i],trajectory[4,i],trajectory[5,i]) for i in range(len(trajectory[0]))]
    trajectory_acceleration = [(result['u'][0,i],result['u'][1,i],result['u'][2,i]) for i in range(len(result['u'][0]))]

    for point in trajectory_position:
        results_position.append(norm(point - current_position))
    min_index = max( results_position.index(min(results_position)), last_index )

    try:
        upper_position_waypoint = trajectory_position[min_index]
        lower_position_waypoint = trajectory_position[min_index+1]
        upper_velocity_waypoint = trajectory_velocity[min_index]
        lower_velocity_waypoint = trajectory_velocity[min_index+1]
        upper_acceleration_waypoint = trajectory_acceleration[min_index]
        lower_acceleration_waypoint = trajectory_acceleration[min_index+1]
    except:
        upper_position_waypoint = lower_position_waypoint = trajectory_position[min_index]
        upper_velocity_waypoint = lower_velocity_waypoint = trajectory_velocity[min_index]
        upper_acceleration_waypoint = lower_acceleration_waypoint = trajectory_acceleration[min_index]

    return upper_position_waypoint, lower_position_waypoint, upper_velocity_waypoint, lower_velocity_waypoint, upper_acceleration_waypoint, lower_acceleration_waypoint, min_index

def get_half_rocket_length(vessel):
    result = [norm(part.position(vessel_reference_frame)) for part in vessel.parts.all if part.position(vessel_reference_frame)[1] < 0]
    value_weight_dict = dict(Counter(result))
    total_weight = len(result)
    weighted_sum = sum(value * weight for value, weight in value_weight_dict.items())
    
    return weighted_sum / total_weight

def landed(vessel):
    legs = vessel.parts.legs
    return all(leg.is_grounded for leg in legs)

def ignition_height(target_reference_frame):
    return abs(vessel.flight(target_reference_frame).vertical_speed**2 / (2*(0.6*vessel.available_thrust / vessel.mass - body.surface_gravity)))

def calculate_control_ratio(torque, half_rocket_length, aerodynamic_force):
    max_control_force = torque * half_rocket_length
    max_control_force = sqrt(max_control_force[0,0]**2 + max_control_force[0,2]**2)
    control_ratio = norm(aerodynamic_force[1:3]) / max_control_force
    return control_ratio

target_reference_frame = create_target_reference_frame(target=targets_JNSQ.launchpad)
draw_reference_frame(target_reference_frame)
draw_reference_frame(vessel_surface_reference_frame)
vessel.auto_pilot.reference_frame = vessel_surface_reference_frame

half_rocket_length = get_half_rocket_length(vessel)


conn.krpc.paused = True
conn.ui.message('GENERATING SOLUTION',duration=1)
tf = norm(vessel.velocity(target_reference_frame)) / ((0.1*vessel.available_thrust + norm(vessel.flight(target_reference_frame).aerodynamic_force))/vessel.mass)
result = generate_solution(estimated_landing_time=tf,
                           gravity=g,
                           dry_mass=vessel.dry_mass,
                           fuel_mass=vessel.mass-vessel.dry_mass,
                           max_thrust=vessel.available_thrust,
                           min_throttle=0.2,
                           max_throttle=1.,
                           max_structural_Gs=9,
                           specific_impulse=vessel.specific_impulse,
                           max_velocity=400,
                           glide_slope_cone=9,
                           thrust_pointing_constraint=30,
                           planetary_angular_velocity=body.angular_velocity(target_reference_frame),
                           initial_position=vessel.position(target_reference_frame),
                           initial_velocity=vessel.velocity(target_reference_frame),
                           target_position=(half_rocket_length,0,0),
                           target_velocity=(0,0,0))

trajectory = result['x']
draw_trajectory(result['x'],result['u'],target_reference_frame)
conn.ui.message('SOLUTION GENERATED',duration=1)
vessel.auto_pilot.engage()
conn.krpc.paused = False

pid = PID(0.2,0.,0.)
dt = 0.02
nav_mode = 'GFOLD'
end = False
legs = False
index = 0
start_time = space_center.ut

while not end:
    while nav_mode == 'GFOLD':
        current_gametime = space_center.ut

        velocity = array(vessel.velocity(target_reference_frame))
        position = array(vessel.position(target_reference_frame))
        thrust = vessel.thrust
        mass = vessel.mass
        available_thrust = vessel.available_thrust
        aerodynamic_force = array(vessel.flight(target_reference_frame).aerodynamic_force)
        torque = array(vessel.available_torque)

        waypoints = find_nearest_waypoints(position, trajectory, index)
        waypoint_position_upper = array(waypoints[0])
        waypoint_position_lower = array(waypoints[1])
        waypoint_velocity_upper = array(waypoints[2])
        waypoint_velocity_lower = array(waypoints[3])
        waypoint_acceleration_upper = array(waypoints[4])
        waypoint_acceleration_lower = array(waypoints[5])
        index = waypoints[6]

        position_waypoint = (waypoint_position_upper+waypoint_position_lower)/2
        velocity_waypoint = (waypoint_velocity_upper+waypoint_velocity_lower)/2
        accelration_waypoint = (waypoint_acceleration_upper+waypoint_acceleration_lower)/2

        velocity_error = velocity_waypoint - velocity
        position_error = position_waypoint - position

        target_direction = accelration_waypoint + velocity_error*0.3 + position_error*0.1
        target_direction_x = target_direction[0]
        while target_direction_x <= 0:
            target_direction_x = target_direction_x + g
        target_direction = (target_direction_x, target_direction[1], target_direction[2])
        compensate = norm(aerodynamic_force[1:3])/(available_thrust)
        throttle = norm(target_direction) / (available_thrust/mass) + compensate
        vessel.control.throttle = throttle
        vessel.auto_pilot.target_direction = vec_clamp_yz(target_direction,45)
        print('throttle:%3f | compensate:%3f | index%i' % (throttle,compensate,index),end='\r')

        if velocity[0] >= -2 or landed(vessel):
            nav_mode = 'PID'
            terminal_velocity = velocity
            terminal_position = position - array([half_rocket_length,0,0])
            terminal_time = space_center.ut
            terminal_landing_time = norm(terminal_position)/norm(terminal_velocity)
            vessel.control.legs = True
            print('\nterminal velocity:%s | terminal position:%s' % (terminal_velocity,terminal_position))
            break
        elif norm(position) <= 130 and not legs:
            vessel.control.legs = True
            legs = True

        dt = space_center.ut - current_gametime
    
    while nav_mode == 'PID':
        current_gametime = space_center.ut

        velocity = array(vessel.velocity(target_reference_frame))
        position_error = array(vessel.position(target_reference_frame)) - array([half_rocket_length,0,0])

        velocity_error = -velocity
        position_error = -position

        target_direction = velocity_error*0.3 + position_error*0.1 + array([thrust/mass,0,0])
        dt = max( dt, 0.02 )
        throttle = pid.update(-2-velocity[0],dt)
        vessel.control.throttle = throttle
        vessel.auto_pilot.target_direction = vec_clamp_yz(target_direction, 75)

        if landed(vessel):
            vessel.control.throttle = 0.
            print('END')
            end = True
            break

        dt = space_center.ut - current_gametime