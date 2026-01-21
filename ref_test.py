import krpc
import numpy as np
from internal.utils import create_solver_reference_frame, create_target_reference_frame
from internal import Targets, Targets_JNSQ

conn = krpc.connect(name='ref_test')
space_center = conn.space_center
vessel = space_center.active_vessel
body = vessel.orbit.body
trf = create_target_reference_frame(conn, Targets_JNSQ.launchpad)
srf = create_solver_reference_frame(conn, trf)

print(vessel.position(srf), vessel.position(trf))
print(vessel.velocity(srf), vessel.velocity(trf))

print(np.array(vessel.position(srf)) * np.array(vessel.position(trf)))