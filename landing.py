import time

import krpc
from numpy import array, cos, sin, deg2rad, sqrt
from numpy.linalg import norm
from collections import Counter
from tqdm import trange

from PID import PID, clamp
from GFOLD_SOLVER import GFOLD
from vector import vec_clamp_yz, normalize, vec_ang

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

#solver utilities
def bundle_data(vessel):
    bundled_data = {}
    bundled_data['gravity'] = g
    bundled_data['dry_mass'] = vessel.dry_mass
    bundled_data['fuel_mass'] = vessel.mass - vessel.dry_mass
    bundled_data['max_thrust'] = vessel.available_thrust
    bundled_data['min_throttle'] = 0.2
    bundled_data['max_throttle'] = 0.9
    bundled_data['max_structural_Gs'] = 9
    bundled_data['specific_impulse'] = vessel.specific_impulse
    bundled_data['max_velocity'] = vessel.flight(target_reference_frame).speed
    bundled_data['glide_slope_cone'] = 20
    bundled_data['thrust_pointing_constraint'] = 30
    bundled_data['planetary_angular_velocity'] = body.angular_velocity(target_reference_frame)
    bundled_data['initial_position'] = vessel.position(target_reference_frame)
    bundled_data['initial_velocity'] = vessel.velocity(target_reference_frame)
    bundled_data['target_position'] = (get_half_rocket_length(vessel),0,0)
    bundled_data['target_velocity'] = (0,0,0)
    bundled_data['prog_flag'] = 'p4'
    bundled_data['solver'] = 'ECOS'
    bundled_data['N_tf'] = 200
    bundled_data['plot'] = False
    bundled_data['min_tf'] = min_tf
    bundled_data['max_tf'] = 2*min_tf
    return bundled_data

target_reference_frame = create_target_reference_frame(target=targets_JNSQ.VAB_A)
half_rocket_length = get_half_rocket_length(vessel)
draw_reference_frame(target_reference_frame)
draw_reference_frame(vessel_surface_reference_frame)
vessel.auto_pilot.reference_frame = vessel_surface_reference_frame
vessel.auto_pilot.engage()

conn.krpc.paused = True
conn.ui.message('GENERATING SOLUTION',duration=1)
min_tf = int(sqrt(2*vessel.position(target_reference_frame)[0]/g))

bundled_data = bundle_data(vessel)
problem = GFOLD(bundled_data)
problem.find_optimal_solution()

result = problem.solution

draw_trajectory(result['x'],result['u'],target_reference_frame)
conn.ui.message('SOLUTION GENERATED',duration=1)
conn.krpc.paused = False

nav_mode = 'GFOLD'
end = False
legs = False

trajectory = array(result['x'])
trajectory_position = [(trajectory[0,i],trajectory[1,i],trajectory[2,i]) for i in range(len(trajectory[0]))]
trajectory_velocity = [(trajectory[3,i],trajectory[4,i],trajectory[5,i]) for i in range(len(trajectory[0]))]
trajectory_acceleration = [(result['u'][0,i],result['u'][1,i],result['u'][2,i]) for i in range(len(result['u'][0]))]

while not end:
    while nav_mode == 'GFOLD':

        velocity = array(vessel.velocity(target_reference_frame))
        position = array(vessel.position(target_reference_frame))
        thrust = vessel.thrust
        mass = vessel.mass
        available_thrust = vessel.available_thrust
        aerodynamic_force = array(vessel.flight(target_reference_frame).aerodynamic_force)
        direction = array(vessel.direction(vessel_surface_reference_frame))

        #get the index of nearest waypoints
        results_position = []
        for point in trajectory_position:
            results_position.append(norm(point - position))
        min_index = results_position.index(min(results_position))

        #define waypoints
        position_waypoint = array(trajectory_position[min_index])
        velocity_waypoint = array(trajectory_velocity[min_index])
        acceleration_waypoint = array(trajectory_acceleration[min_index])

        velocity_error = velocity_waypoint - velocity
        position_error = position_waypoint - position

        target_direction = acceleration_waypoint + velocity_error*0.3 + position_error*0.1
        target_direction_x = target_direction[0]
        while target_direction_x <= 0:
            target_direction_x = target_direction_x + g
        target_direction = (target_direction_x, target_direction[1], target_direction[2])
        compensation = norm(aerodynamic_force[1:3])/available_thrust
        throttle = norm(target_direction)/(available_thrust/mass) + compensation
        throttle = clamp(throttle,0.2,1.)
        vessel.control.throttle = throttle
        vessel.auto_pilot.target_direction = vec_clamp_yz(target_direction,45)
        print('throttle:%3f | compensation:%3f | index%i' % (throttle,compensation,min_index),end='\r')

        if norm(position) <= 4*half_rocket_length:
            nav_mode = 'PID'
            terminal_velocity = velocity
            terminal_position = position - array([half_rocket_length,0,0])
            vessel.control.legs = True
            print('\nterminal velocity:%s | terminal position:%s' % (terminal_velocity,terminal_position))
            break
        elif norm(position) <= 130 and not legs:
            vessel.control.legs = True
            legs = True
    
    while nav_mode == 'PID':
        direction = vessel.direction(target_reference_frame)
        velocity = array(vessel.velocity(target_reference_frame))
        position = array(vessel.position(target_reference_frame)) - array([half_rocket_length,0,0])
        mass = vessel.mass
        available_thrust = vessel.available_thrust

        velocity_error = -velocity
        position_error = -position

        landing_time = position[0]/velocity[0]

        target_direction = velocity_error*0.3 + position_error*0.1
        target_direction_x = target_direction[0]
        while target_direction_x <= 0:
            target_direction_x += g
        target_direction = (target_direction_x,target_direction[1],target_direction[2])
        throttle = 0.1*(-2-velocity[0])
        vessel.control.throttle = throttle
        vessel.auto_pilot.target_direction = vec_clamp_yz(target_direction, 85)
        print('velocity error:%s | position error:%s' % (velocity_error,position_error),end='\r')

        if landed(vessel):
            vessel.control.throttle = 0.
            print('\ntouchdown velocity:%s' % (velocity))
            print('END')
            end = True
            break
