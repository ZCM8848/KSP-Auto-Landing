import krpc
from control.extension import Rocket
from internal.utils import *
from internal.targets import *
import scipy.optimize as optimize

def get_test_data() -> None:
    conn = krpc.connect(name='ffs_test')
    space_center = conn.space_center
    vessel = space_center.active_vessel
    body = vessel.orbit.body
    g = body.surface_gravity

    trf = create_target_reference_frame(conn, Targets_JNSQ.launchpad)
    vessel = Rocket(space_center, vessel, trf)

    with open('test_data.txt', 'w+') as f:
        f.write(f'mass: {vessel.mass}\n')
        f.write(f'dry_mass: {vessel.dry_mass}\n')
        f.write(f'available_thrust: {vessel.available_thrust}\n')
        f.write(f'specific_impulse: {vessel.specific_impulse}\n')
        f.write(f'initial_position: {vessel.position()}\n')
        f.write(f'initial_velocity: {vessel.velocity()}\n')
    return None

mass = 98887.5078125
dry_mass = 26361.478515625
available_thrust = 1902927.25
specific_impulse = 334.99658203125
initial_position = array([55420.7212688, 13440.62966535, 24222.02999502])
initial_velocity = array([1179.84897419, 388.63473972, 692.80155726])

