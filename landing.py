import time

import krpc
from numpy import array, cos, sin, deg2rad
from numpy.linalg import norm

from PID import PID, clamp
from GFOLD_SOLVER import generate_solution
from vector import vec_ang, vec_around, vec_clamp, vec_clamp_yz

#define some basic KRPC things
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
    direction.thickness = 0.5

def draw_line(start, end, reference_frame):
    position = conn.drawing.add_line(reference_frame=reference_frame,start=start,end=end)
    position.thickness = 0.5
    return position

def draw_trajectory(x,reference_frame):
    for i in range(len(x[0])):
        if i >=1:
            draw_line(start=(x[0,i-1],x[1,i-1],x[2,i-1]),end=(x[0,i],x[1,i],x[2,i]),reference_frame=reference_frame)

def find_nearest_waypoint(position, acc, trajectory):
    results_position = []
    position = array(position)
    trajectory = array(trajectory)
    for point in trajectory:
        results_position.append(norm(point - position))
    result = trajectory[results_position.index(min(results_position))]
    thrust = (acc[results_position.index(min(results_position))] - array([-g,0,0])) * vessel.mass
    return result, thrust

def descent_throttle_controller(target_height=0,vt=-2):
        acc = (round(vt**2,5)+round(vessel.velocity(target_reference_frame)[2]**2,5))/round((2*(vessel.position(target_reference_frame)[2]-target_height)),5) + round(body.surface_gravity,5) + round(vessel.flight(vessel_reference_frame).aerodynamic_force[2],5)/round(vessel.mass*body.surface_gravity,5)
        return vessel.mass*acc/vessel.max_thrust


target_reference_frame = create_target_reference_frame(target=targets.launchpad)
draw_reference_frame(target_reference_frame)
draw_reference_frame(vessel_surface_reference_frame)
vessel.auto_pilot.reference_frame = vessel_surface_reference_frame

tf = 30
conn.ui.message('INITIALIZED',duration=0.5)
conn.krpc.paused = True
conn.ui.message('GENERATING SOLUTION',duration=1)
result = generate_solution(estimated_landing_time=tf,
                           gravity=g,
                           dry_mass=vessel.dry_mass,
                           fuel_mass=vessel.mass-vessel.dry_mass,
                           max_thrust=vessel.available_thrust,
                           min_throttle=0.2,
                           max_throttle=1.,
                           max_structural_Gs=15,
                           specific_impulse=vessel.specific_impulse,
                           max_velocity=250,
                           glide_slope_cone=15,
                           thrust_pointing_constraint=120,
                           planetary_angular_velocity=body.angular_velocity(target_reference_frame),
                           initial_position=vessel.position(target_reference_frame),
                           initial_velocity=vessel.velocity(target_reference_frame),
                           plot=False)

trajectory = [(result['x'][0,i],result['x'][1,i],result['x'][2,i]) for i in range(len(result['x'][0]))]
thrust_acc = [(abs(result['u'][0,i]),result['u'][1,i],result['u'][2,i]) for i in range(len(result['u'][0]))]

draw_trajectory(result['x'],target_reference_frame)
conn.ui.message('SOLUTION GENERATED',duration=1)
conn.krpc.paused = False


pid = PID(0.5,0.2,0.)
vessel.auto_pilot.engage()

ut = space_center.ut
while True:
    timespan = space_center.ut - ut
    vessel.control.throttle = norm(array(thrust_acc[int(timespan*len(trajectory)/tf)]) + array([-g,0,0]))*vessel.mass / vessel.available_thrust
    vessel.auto_pilot.target_direction = tuple(vec_clamp_yz( thrust_acc[int(timespan*len(trajectory)/tf)] ,60))
