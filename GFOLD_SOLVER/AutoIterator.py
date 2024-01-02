from concurrent.futures import ThreadPoolExecutor
from .GFOLD_Enterance import generate_solution

class GFOLD:
    def __init__(self,
                 gravity,
                 dry_mass,
                 fuel_mass,
                 max_thrust,
                 min_throttle,
                 max_throttle,
                 max_structural_Gs,
                 specific_impulse,
                 max_velocity,
                 glide_slope_cone,
                 thrust_pointing_constraint,
                 planetary_angular_velocity,
                 initial_position,
                 initial_velocity,
                 target_position,
                 target_velocity) -> None:
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
            print(f'{tf}:E')
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