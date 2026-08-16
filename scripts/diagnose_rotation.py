"""Verify planetary rotation vector in the target reference frame."""
import numpy as np

from recovery import ConnectionManager
from recovery.data.targets import LAUNCHPAD_JNSQ

with ConnectionManager(address="127.0.0.1") as km:
    b = km.add_booster("rot-check", "VTVL-Cam")
    km.register_target("rot-check", lon=LAUNCHPAD_JNSQ.lon, lat=LAUNCHPAD_JNSQ.lat)
    km.start()

    body = b.raw.orbit.body
    tf = km.frame("rot-check", "target")

    mu = body.gravitational_parameter
    rs = body.rotational_speed
    re = body.equatorial_radius
    surf_h = body.surface_height(LAUNCHPAD_JNSQ.lat, LAUNCHPAD_JNSQ.lon)
    body_radius = re + surf_h

    print(f"mu   = {mu:.2f} m^3/s^2")
    print(f"w    = {rs:.6f} rad/s")
    print(f"Re   = {re:.1f} m")
    print(f"surf = {surf_h:.1f} m")
    print(f"R    = {body_radius:.1f} m")
    print()

    # Method A: direction(target_frame) * rotational_speed
    npole = np.array(body.direction(tf))
    omega_a = npole * rs
    print(f"north-in-target = ({npole[0]:.4f}, {npole[1]:.4f}, {npole[2]:.4f})")
    print(f"w (method A)    = ({omega_a[0]:.6f}, {omega_a[1]:.6f}, {omega_a[2]:.6f})")

    # Method C: angular_velocity(target_frame)
    av_tf = np.array(body.angular_velocity(tf))
    print(f"w-in-target     = ({av_tf[0]:.6f}, {av_tf[1]:.6f}, {av_tf[2]:.6f})")

    # Method D: angular_velocity(body.reference_frame) -- should be zero
    av_bf = np.array(body.angular_velocity(body.reference_frame))
    print(f"w-in-body       = ({av_bf[0]:.6f}, {av_bf[1]:.6f}, {av_bf[2]:.6f})")

    print()
    print(f"|w-in-target|   = {np.linalg.norm(av_tf):.6f}  (expect {rs:.6f})")
    print(f"|w-method-A|    = {np.linalg.norm(omega_a):.6f}  (expect {rs:.6f})")
