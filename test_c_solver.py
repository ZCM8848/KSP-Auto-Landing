from math import log, sin
from random import uniform
from compiled_solvers.tower_catch.cpg_solver import cpg_solve
from solver import GFoldSolver, config, visualization
from numpy import array
import time

update_params = False
g = 9.80665

# register solver
solver = GFoldSolver()
problem = solver.problem
problem.register_solve('CPG', cpg_solve)

t0 = time.time()
if update_params:
    # Falcon 9 specific parameters
    m_dry = 22.2 * 1e3      # dry mass kg
    m_fuel = 13.4 * 1e3     # fuel mass kg
    T_max = 845000    # maximum thrust N
    Isp = 300               # specific impulse s
    throt = [0.4, 1.0]      # throttle range

    # Initial conditions (coordinate system: x-downrange, y-crossrange, z-altitude)
    initial_position = array([5, 10, 2000])  # r_ from Falcon 9 data 
    initial_velocity = array([2, -10, -150])  # v0 from Falcon 9 data 

    # Mass and fuel parameters
    fuel_consumption = 1 / (Isp * g)  # alpha parameter
    wet_mass = m_dry + m_fuel  # total wet mass
    fuel = m_fuel  # fuel mass

    # Thrust parameters
    min_thrust_pct = throt[0]  # minimum throttle
    max_thrust_pct = throt[1]  # maximum throttle
    real_max_thrust = T_max  # maximum thrust

    # Target conditions
    target_position = array([0, 0, 23.58])  # rf from Falcon 9 data
    target_velocity = array([0, 0, 0])  # vf from Falcon 9 data

    # Environment parameters
    gravity = array([0, 0, -g])  # gravity vector
    glide_slope_angle = 45  # y_gs in degrees
    max_angle = 10  # p_cs constraint

    # Solver parameters
    n = 100  # N nodes in discretization

    # update parameters
    solver.update_parameter('initial_position', initial_position)
    solver.update_parameter('initial_velocity', initial_velocity)
    solver.update_parameter('target_velocity', target_velocity)
    solver.update_parameter('log_mass', log(wet_mass))
    solver.update_parameter('log_dry_mass', log(m_dry))
    solver.update_parameter('fuel_consumption', fuel_consumption)
    solver.update_parameter('min_thrust', min_thrust_pct * real_max_thrust)
    solver.update_parameter('max_thrust', max_thrust_pct * real_max_thrust)
    solver.update_parameter('max_angle', max_angle)
    solver.update_parameter('gravity', gravity)
    solver.update_parameter('sin_glide_slope', sin(glide_slope_angle))

# solve
problem.solve(method='CPG')
t1 = time.time()

# result_direct = solver.solve()
print('\nCVXPYgen\nSolve time: %.3f ms\n' % (1000 * (t1 - t0)))

print(f"solution status: {problem.status}")
print(f"{problem.solution.__dict__}")
print(f"x value: {solver.variables['x'].value}")
print(f"u value: {solver.variables['u'].value}")
print(f"z value: {solver.variables['z'].value}")
print(f"s value: {solver.variables['s'].value}")

print(solver.variables['x'].value[:, 0:3].shape)