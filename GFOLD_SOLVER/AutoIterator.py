from concurrent.futures import ThreadPoolExecutor

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
                 target_velocity:tuple) -> None:
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

    def pointor(self, tf):
        try:
            return generate_solution(tf, 
                                     self.gravity, 
                                     self.dry_mass, 
                                     self.fuel_mass, 
                                     self.max_thrust,
                                     self.min_throttle, 
                                     self.max_throttle, 
                                     self.max_structural_Gs,
                                     self.specific_impulse, 
                                     self.max_velocity, 
                                     self.glide_slope_cone,
                                     self.thrust_pointing_constraint,
                                     self.planetary_angular_velocity,
                                     self.initial_position,
                                     self.initial_velocity, 
                                     self.target_position, 
                                     self.target_velocity)
        except:
            print(f'{tf}:E',flush=True)
            return None

    def find_best_result(self):
        pool = ThreadPoolExecutor(max_workers=200)
        results =pool.map(self.pointor,range(10,31))
        valid_results = []

        for result in results:
            if result is None:
                continue
            else:
                valid_results.append(result)

        best_result = min(valid_results, key=lambda k: abs(k['opt']))

        #print(f"The optimal tf is {best_result['tf']} seconds, with opt value of {best_result['opt']}")
        return best_result