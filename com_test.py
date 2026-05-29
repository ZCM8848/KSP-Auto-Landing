import krpc

conn = krpc.connect(name='0')
space_center = conn.space_center
vessel = space_center.active_vessel
brf = vessel.orbit.body.reference_frame

print(vessel.position(brf), vessel.velocity(brf))
print(vessel.orbit.body.atmosphere_depth)
print(vessel.orbit.body.equatorial_radius)