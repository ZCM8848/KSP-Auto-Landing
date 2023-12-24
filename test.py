import krpc
from PID import PID

conn = krpc.connect()
space_center = conn.space_center
vessel = space_center.active_vessel


target_height = 100
pid = PID(0.3,0.2,0.1)
dt = 0.02
while True:
    ut = space_center.ut
    current_height = vessel.flight().surface_altitude
    current_velocity = vessel.flight().vertical_speed
    error = target_height - (current_velocity + current_height)
    if dt==0:
        dt=0.02
    thr = pid.update(error,dt)
    vessel.control.throttle = thr
    print(thr)
    dt = space_center.ut - ut