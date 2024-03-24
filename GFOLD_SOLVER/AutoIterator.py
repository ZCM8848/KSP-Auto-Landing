from tqdm import trange
from numpy import sqrt

from .GFOLD_Enterance import generate_solution

class GFOLD:
    def __init__(self,
                 gravity:float,
                 dry_mass:float,
                 fuel_mass:float,
                 max_thrust:float,
                 min_throttle:float,
                 max_throttle:float,
                 max_structural_Gs:float,
                 specific_impulse:float,
                 max_velocity:float,
                 glide_slope_cone:float,
                 thrust_pointing_constraint:float,
                 planetary_angular_velocity:tuple,
                 initial_position:tuple,
                 initial_velocity:tuple,
                 target_position:tuple,
                 target_velocity:tuple,
                 prog_flag:str,
                 solver:str,
                 N_tf:int,
                 plot:bool,
                 min_tf:int) -> None:
        self.gravity = gravity
        self.dry_mass = dry_mass
        self.fuel_mass = fuel_mass
        self.max_thrust = max_thrust
        self.min_throttle = min_throttle
        self.max_throttle = max_throttle
        self.max_structural_Gs = max_structural_Gs
        self.specific_impulse = specific_impulse
        self.max_velocity = max_velocity
        self.glide_slope_cone = glide_slope_cone
        self.thrust_pointing_constraint = thrust_pointing_constraint
        self.planetary_angular_velocity = planetary_angular_velocity
        self.initial_position = initial_position
        self.initial_velocity = initial_velocity
        self.target_position = target_position
        self.target_velocity = target_velocity
        self.prog_flag = prog_flag
        self.solver = solver
        self.N_tf = N_tf
        self.plot = plot
        self.min_tf = min_tf

    def estimate_time(self):
        t = []
        o = []
        for i in trange(int(self.min_tf),80,desc='solving'):
            opt = generate_solution(estimated_landing_time=i,
                                    gravity=self.gravity,
                                    dry_mass=self.dry_mass,
                                    fuel_mass=self.fuel_mass,
                                    max_thrust=self.max_thrust,
                                    min_throttle=self.min_throttle,
                                    max_throttle=self.max_throttle,
                                    max_structural_Gs=self.max_structural_Gs,
                                    specific_impulse=self.specific_impulse,
                                    max_velocity=self.max_velocity,
                                    glide_slope_cone=self.glide_slope_cone,
                                    thrust_pointing_constraint=self.thrust_pointing_constraint,
                                    planetary_angular_velocity=self.planetary_angular_velocity,
                                    initial_position=self.initial_position,
                                    initial_velocity=self.initial_velocity,
                                    target_position=self.target_position,
                                    target_velocity=self.target_velocity,
                                    prog_flag=self.prog_flag,
                                    solver=self.solver,
                                    N_tf=20)
            if opt['opt'] is not None:
                o.append(opt['opt'])
                t.append(i)
                #print(f"    TIME:{t[-1]}, COST:{o[-1]}")
        return t[o.index(max(o))]
    
    def solve(self):
        tf = self.estimate_time()
        result = generate_solution(estimated_landing_time=tf,
                                   gravity=self.gravity,
                                   dry_mass=self.dry_mass,
                                   fuel_mass=self.fuel_mass,
                                   max_thrust=self.max_thrust,
                                   min_throttle=self.min_throttle,
                                   max_throttle=self.max_throttle,
                                   max_structural_Gs=self.max_structural_Gs,
                                   specific_impulse=self.specific_impulse,
                                   max_velocity=self.max_velocity,
                                   glide_slope_cone=self.glide_slope_cone,
                                   thrust_pointing_constraint=self.thrust_pointing_constraint,
                                   planetary_angular_velocity=self.planetary_angular_velocity,
                                   initial_position=self.initial_position,
                                   initial_velocity=self.initial_velocity,
                                   target_position=self.target_position,
                                   target_velocity=self.target_velocity,
                                   prog_flag=self.prog_flag,
                                   solver=self.solver,
                                   N_tf=self.N_tf,
                                   plot=self.plot)
        return result