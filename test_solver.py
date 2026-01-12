from math import radians
from solver import GFoldSolver, config, visualization
from numpy import array
import time

t0 = time.time()
vessel_config = config.GFoldConfig()
g = 9.80665

# Falcon 9 specific parameters
m_dry = 22.2 * 1e3      # dry mass kg
m_fuel = 13.4 * 1e3     # fuel mass kg
T_max = 845000*3    # maximum thrust N
Isp = 300               # specific impulse s
throt = [0.4, 1.0]      # throttle range
    
# Initial conditions (coordinate system: x-downrange, y-crossrange, z-altitude)
vessel_config.spacecraft.initial_position = array([5, 10, 2000])  # r_ from Falcon 9 data 
vessel_config.spacecraft.initial_velocity = array([2, -10, -150])  # v0 from Falcon 9 data 
    
# Mass and fuel parameters
vessel_config.spacecraft.fuel_consumption = 1 / (Isp * g)  # alpha parameter
vessel_config.spacecraft.wet_mass = m_dry + m_fuel  # total wet mass
vessel_config.spacecraft.fuel = m_fuel  # fuel mass
    
# Thrust parameters
vessel_config.spacecraft.min_thrust_pct = throt[0]  # minimum throttle
vessel_config.spacecraft.max_thrust_pct = throt[1]  # maximum throttle
vessel_config.spacecraft.real_max_thrust = T_max  # maximum thrust
    
# Target conditions
vessel_config.spacecraft.target_position = array([0, 0, 23.58])  # rf from Falcon 9 data
vessel_config.spacecraft.target_velocity = array([0, 0, 0])  # vf from Falcon 9 data
    
# Environment parameters
vessel_config.environment.gravity = array([0, 0, -g])  # gravity vector
vessel_config.environment.glide_slope_angle = 1  # y_gs in degrees
vessel_config.environment.max_angle = 10  # p_cs constraint
    
# Solver parameters
vessel_config.solver.n = 100  # N nodes in discretization

solution = GFoldSolver(vessel_config).solve(verbose=True)

visualization.plot_results(solution)