"""
GFOLD 求解器对比测试 - 验证参数更新方法 vs 重建问题方法
"""
import krpc
import csv
import time
import numpy as np
from numpy import array, log, sin, radians

from control.extension import Rocket
from internal.utils import create_target_reference_frame, draw_reference_frame, draw_trajectory
from internal.targets import Targets_JNSQ
from solver.GFOLD.compiled_solvers.normal_landing.cpg_solver import cpg_solve
from solver.GFOLD import GFoldSolver, GFoldConfig

STT = array([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0]])
conn = krpc.connect("RTLS")
# tgt = (conn.space_center.target_vessel.flight().longitude, conn.space_center.target_vessel.flight().latitude)
tgt = Targets_JNSQ.launchpad
trf = create_target_reference_frame(conn, tgt)
space_center = conn.space_center
vessel = Rocket(space_center, space_center.active_vessel, trf)
body = vessel.vessel.orbit.body
brf = body.reference_frame
bnrf = body.non_rotating_reference_frame
g0 = body.surface_gravity
target_origion = array(space_center.transform_position((0,0,0), trf, brf))

g=9.80665
glide_slope_angle = 45
n = 100
time_of_flight = 30
max_angle = 10
throttle_limits = [0.4, 1.]

def gfold_method0(position:np.ndarray,
                  velocity:np.ndarray,
                  mass:float,
                  dry_mass:float,
                  available_thrust:float,
                  specific_impulse:float) -> str:
    params = GFoldConfig()
    params.spacecraft.initial_position = position
    params.spacecraft.initial_velocity = velocity
    params.spacecraft.wet_mass = mass
    params.spacecraft.fuel = mass - dry_mass
    params.spacecraft.real_max_thrust = available_thrust
    params.spacecraft.fuel_consumption = 1 / (g*specific_impulse)
    params.environment.max_angle = max_angle
    params.environment.gravity = array([-g, 0, 0])
    params.environment.glide_slope_angle = glide_slope_angle
    params.solver.n = n
    params.solver.time_of_flight = time_of_flight
    solver = GFoldSolver(params)
    solver.solve(verbose=False)
    return solver.problem.status

def gfold_method1(position:np.ndarray,
                  velocity:np.ndarray,
                  mass:float,
                  dry_mass:float,
                  available_thrust:float,
                  specific_impulse:float) -> str:
    params = GFoldConfig()
    params.spacecraft.initial_position = position
    params.spacecraft.initial_velocity = velocity
    params.spacecraft.wet_mass = mass
    params.spacecraft.fuel = mass - dry_mass
    params.spacecraft.real_max_thrust = available_thrust
    params.spacecraft.fuel_consumption = 1 / (g*specific_impulse)
    params.environment.max_angle = max_angle
    params.environment.gravity = array([-g, 0, 0])
    params.environment.glide_slope_angle = glide_slope_angle
    params.solver.n = n
    params.solver.time_of_flight = time_of_flight
    solver = GFoldSolver(params)
    solver.problem.register_solve('CPG', cpg_solve)
    solver.problem.solve(method='CPG')
    return solver.problem.status

def gfold_method2(solver:GFoldSolver,
                  position:np.ndarray,
                  velocity:np.ndarray,
                  mass:float,
                  dry_mass:float,
                  available_thrust:float,
                  specific_impulse:float) -> str:
    dt = time_of_flight / n
    solver.update_parameter("initial_position", position)
    solver.update_parameter("initial_velocity", velocity)
    solver.update_parameter('target_velocity', array([0, 0, 0]))
    solver.update_parameter("log_mass", log(mass))
    solver.update_parameter("log_dry_mass", log(dry_mass))
    solver.update_parameter("max_vel", 1000)
    solver.update_parameter("sin_glide_slope", sin(radians(glide_slope_angle)))
    solver.update_parameter("min_thrust", available_thrust * throttle_limits[0])
    solver.update_parameter("max_thrust", available_thrust * throttle_limits[1])
    solver.update_parameter('fuel_consumption', 1 / (g*specific_impulse))
    solver.update_parameter('max_angle', max_angle)
    solver.update_parameter('gravity', array([-g, 0, 0]))
    solver.update_parameter('dt', dt)
    solver.update_parameter('dt_squared', dt**2)
    solver.update_parameter('gravity_dt', array([-g, 0, 0]) * dt)
    solver.update_parameter('gravity_dt_squared', (array([-g, 0, 0]) * dt)**2)
    solver.problem.solve(method='CPG')
    return solver.problem.status

def control():
        pos = vessel.position()
        vel = vessel.velocity()
        pos_x = pos[2]
        pos_y = pos[1]
        pos_z = pos[0]
        vel_x = vel[2]
        vel_y = vel[1]
        vel_z = vel[0]
        available_thrust = vessel.vessel.available_thrust
        specific_impulse = vessel.vessel.specific_impulse
        dry_mass = vessel.vessel.dry_mass
        wet_mass = vessel.vessel.mass
        
        position = array([pos_x, pos_y, pos_z])
        velocity = array([vel_x, vel_y, vel_z])
        t0 = time.time()
        try:
            # solution0 = gfold_method0(position,
            #                           velocity,
            #                           wet_mass,
            #                           dry_mass,
            #                           available_thrust,
            #                           specific_impulse)
            solution0 = '6'
        except Exception as e:
            solution0 = '5'
        time0 = time.time() - t0
        t1 = time.time()
        try:
            solution1 = gfold_method1(position,
                                      velocity,
                                      wet_mass,
                                      dry_mass,
                                      available_thrust,
                                      specific_impulse)
        except:
            solution1 = '5'
        time1 = time.time() - t1
        t2 = time.time()
        try:
            solution2 = gfold_method2(_solver,
                                      position,
                                      velocity,
                                      wet_mass,
                                      dry_mass,
                                      available_thrust,
                                      specific_impulse)
        except:
            solution2 = '5'
        time2 = time.time() - t2
        print('%s\t%s\t%s\t%2f\t%2f\t%2f\t' % (solution0, solution1, solution2, time0, time1, time2))
  
if __name__ == "__main__":
    print('s0\ts1\ts2\tt0\t\tt1\t\tt2')
    _solver = GFoldSolver()
    _solver.problem.register_solve('CPG', cpg_solve)
    while True:
        control()