import krpc
import csv
from tqdm import trange
from math import sqrt, degrees, sin, radians, cos
from time import sleep

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
                                                                       rotation=(0., sin(-radians(target_lon / 2)), 0.,
                                                                                 cos(-radians(target_lon / 2))))
    # spin around z axis by target_lat degrees
    temp_reference_frame = space_center.ReferenceFrame.create_relative(temp_reference_frame,
                                                                       rotation=(0., 0., sin(radians(target_lat / 2)),
                                                                                 cos(radians(target_lat / 2))))
    if body.bedrock_height(target_lat, target_lon) < 0:
        target_reference_frame_height = body.equatorial_radius
    else:
        target_reference_frame_height = body.equatorial_radius + body.surface_height(target_lat, target_lon)
    reference_frame = space_center.ReferenceFrame.create_relative(temp_reference_frame,
                                                                  position=(target_reference_frame_height, 0., 0.))
    return reference_frame

target_reference_frame = create_target_reference_frame(target=Targets.launchpad)
vessel.control.activate_next_stage()

density = []
altitude = []
air_speed = []

while True:
    altitude.append(vessel.flight().mean_altitude)
    density.append(vessel.flight().atmosphere_density)
    air_speed.append(vessel.flight().true_air_speed)
    print(altitude, density, air_speed)
    if vessel.velocity(target_reference_frame)[0] <= 0 or vessel.flight().mean_altitude >= 70000:
        print("Recording finished")
        break

with open(r"D:\Python_projects\KSP-Auto-Landing\Models\stock.csv","w",newline='', encoding='utf-8') as csvfile: 
    writer = csv.writer(csvfile)
    writer.writerow(["altitude","density","air speed"])
    for i in range(len(altitude)):
        writer.writerow([altitude[i],density[i],air_speed[i]])
    print("Writed")