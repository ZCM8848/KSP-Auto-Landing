"""Check gfold s_values (slack) vs u norm, and normalized_thrusts semantics."""

import gfold
import numpy as np

cfg = gfold.Config(
    gfold.Spacecraft(
        wet_mass=2000.0, fuel=1700.0,
        initial_position=[450.0, -330.0, 2400.0],
        initial_velocity=[-40.0, 10.0, -10.0],
    ),
    gfold.Environment(gravity=[0.0, 0.0, -3.71], max_angle_deg=25.0),
    gfold.Solver(n=100, time_of_flight=None),
)
traj = gfold.solve(cfg)
u = np.asarray(traj.u_values)
s = np.asarray(traj.s_values)
thr = np.asarray(traj.thrusts)
nt = np.asarray(traj.normalized_thrusts)
z = np.asarray(traj.z_values)
mass = np.exp(z)

print(
    f"{'node':>4} {'|u|':>7} {'s':>7} {'|u|-s':>8} "
    f"{'thrust':>8} {'norm_t':>7} {'s*mass/24000':>14}"
)
for i in [0, 25, 50, 75, 99]:
    s_mass = s[i] * mass[i] / 24000.0
    print(
        f"{i:4d} {np.linalg.norm(u[i]):7.3f} {s[i]:7.3f} "
        f"{np.linalg.norm(u[i]) - s[i]:8.4f} {thr[i]:8.1f} {nt[i]:7.4f} {s_mass:14.4f}"
    )

print()
print("Question: is normalized_thrusts = s*mass/max_thrust (using slack), or |u|*mass/max_thrust?")
print("If they match s*mass (not |u|*mass), then slack matters and we must use normalized_thrusts.")
