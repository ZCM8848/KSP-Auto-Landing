import krpc
import time
import numpy as np

from math import log, sin
from control.extension import Rocket
from internal.utils import *
from internal.targets import *
from solver.GFOLD.compiled_solvers.normal_landing.cpg_solver import cpg_solve
from solver.GFOLD import GFoldSolver, GFoldConfig

throttle_limits = [0.4, 1]

conn = krpc.connect(name='test_gfold')
space_center = conn.space_center
vessel = space_center.active_vessel
body = vessel.orbit.body
g = body.surface_gravity

trf = create_target_reference_frame(conn, Targets_JNSQ.launchpad)
trf = create_solver_reference_frame(conn, trf)
draw_reference_frame(conn, trf)
vessel = Rocket(space_center, vessel, trf)

# attempt 1
for i in range(5):
    random_array = np.random.randint(-10, 10, size=3)
    t0 = time.time()
    params = GFoldConfig()
    params.spacecraft.initial_position = vessel.position() + random_array
    params.spacecraft.initial_velocity = vessel.velocity() + random_array
    params.spacecraft.wet_mass = vessel.vessel.mass
    params.spacecraft.fuel = vessel.vessel.mass - vessel.vessel.dry_mass
    params.spacecraft.real_max_thrust = vessel.vessel.available_thrust
    params.spacecraft.fuel_consumption = 1 / (g*vessel.vessel.specific_impulse)
    params.environment.max_angle = 10
    params.environment.gravity = array([0, 0, -g])
    params.environment.glide_slope_angle = 45
    params.solver.n = 100
    solver = GFoldSolver(params)
    solver.solve(verbose=False)
    print(f"Py solver has status: {solver.problem.status}, time: {time.time() - t0}")
# draw_trajectory(conn, solver.variables['x'].value[:, 0:3], solver.variables['u'].value, trf)

for i in range(5):
    # attempt 2
    random_array = np.random.randint(-10, 10, size=3)
    t0 = time.time()
    params = GFoldConfig()
    params.spacecraft.initial_position = vessel.position() + random_array
    params.spacecraft.initial_velocity = vessel.velocity() + random_array
    params.spacecraft.wet_mass = vessel.vessel.mass
    params.spacecraft.fuel = vessel.vessel.mass - vessel.vessel.dry_mass
    params.spacecraft.real_max_thrust = vessel.vessel.available_thrust
    params.spacecraft.fuel_consumption = 1 / (g*vessel.vessel.specific_impulse)
    params.environment.max_angle = 10
    params.environment.gravity = array([0, 0, -g])
    params.environment.glide_slope_angle = 45
    params.solver.n = 100
    solver = GFoldSolver(params)
    solver.problem.register_solve('CPG', cpg_solve)
    solver.problem.solve(method='CPG')
    print(f"CPG solver1 has status: {solver.problem.status}, time: {time.time() - t0}")
plot = draw_trajectory(conn, solver.variables['x'].value[:, 0:3], solver.variables['u'].value, trf)
input("Press Enter to continue...")
clear_lines(plot)

solver = GFoldSolver()
problem = solver.problem
problem.register_solve('CPG', cpg_solve)

for i in range(5):
    # attempt 3
    random_array = np.random.randint(-10, 10, size=3)
    t0 = time.time()
    available_thrust = vessel.vessel.available_thrust
    solver.update_parameter("initial_position", vessel.position() + random_array)
    solver.update_parameter("initial_velocity", vessel.velocity() + random_array)
    solver.update_parameter('target_velocity', array([0, 0, 0]))
    solver.update_parameter("log_mass", log(vessel.vessel.mass))
    solver.update_parameter("log_dry_mass", log(vessel.vessel.dry_mass))
    solver.update_parameter("max_vel", 1000)
    solver.update_parameter("sin_glide_slope", sin(45))
    solver.update_parameter("min_thrust", available_thrust * throttle_limits[0])
    solver.update_parameter("max_thrust", available_thrust * throttle_limits[1])
    solver.update_parameter('fuel_consumption', 1 / (g*vessel.vessel.specific_impulse))
    solver.update_parameter('max_angle', 10)
    solver.update_parameter('gravity', array([0, 0, -g]))
    # solver.update_parameter('dt', 40/100)
    # solver.update_parameter('dt_squared', (40/100)**2)
    # solver.update_parameter('gravity_dt', array([0, 0, -g]) * (40/100))
    # solver.update_parameter('gravity_dt_squared', (array([0, 0, -g]) * (40/100))**2)
    problem.solve(method='CPG')
    print(f"CPG solver2 has status: {problem.status}, time: {time.time() - t0}")
plot = draw_trajectory(conn, solver.variables['x'].value[:, 0:3], solver.variables['u'].value, trf)
input("Press Enter to continue...")
clear_lines(plot)

solver = GFoldSolver()
problem = solver.problem
problem.register_solve('CPG', cpg_solve)

# attempt 4
for i in range(5):
    random_array = np.random.randint(-10, 10, size=3)
    t0 = time.time()
    solver.update_config(initial_position=vessel.position() + random_array,
                          initial_velocity=vessel.velocity() + random_array,
                          target_velocity=array([0, 0, 0]),
                          wet_mass=vessel.vessel.mass,
                          fuel=vessel.vessel.mass - vessel.vessel.dry_mass,
                          max_velocity=1000,
                          glide_slope_angle=45,
                          min_thrust_pct = throttle_limits[0],
                          max_thrust_pct = throttle_limits[1],
                          fuel_consumption=1 / (g*vessel.vessel.specific_impulse),
                          max_angle=10,
                          gravity=array([0, 0, -g]),
                          n = 100
                          )
    solver.problem.solve(method='CPG')
    print(f"CPG solver3 has status: {solver.problem.status}, time: {time.time() - t0}")
plot = draw_trajectory(conn, solver.problem.solution.primal_vars[find_key_by_shape(solver.problem.solution.primal_vars, (100, 6))][:, 0:3], solver.problem.solution.primal_vars[find_key_by_shape(solver.problem.solution.primal_vars, (100, 3))], trf)
input("Press Enter to continue...")
clear_lines(plot)
quit()

# attempt 5
available_thrust = vessel.vessel.available_thrust
params = {'initial_position': vessel.position(),
          'initial_velocity': vessel.velocity(),
          'target_velocity': array([0, 0, 0]),
          'log_mass': log(vessel.vessel.mass),
          'log_dry_mass': log(vessel.vessel.dry_mass),
          'max_vel': 1000,
          'sin_glide_slope': sin(45),
          'fuel_consumption': 1 / (g*vessel.vessel.specific_impulse),
          'max_angle': 10,
          'gravity': array([0, 0, -g]),
          'n': 100
          }
solution = cpg_solve(problem, params)