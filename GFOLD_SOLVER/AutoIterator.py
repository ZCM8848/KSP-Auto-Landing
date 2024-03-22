import numpy as np

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
        self.alpha = specific_impulse * 9.8065
        self.thrust_lower_bound = min_throttle * max_thrust
        self.thrust_upper_bound = max_throttle * max_thrust

    def estimate_time(self, N):
        # golden section search to estimate flight time with lowest cost for trajectory calculations
        golden_ratio = (np.sqrt(5) - 1) * 0.5
        thrust_lower = self.thrust_lower_bound
        thrust_upper = self.thrust_upper_bound
        initial_speed = np.linalg.norm(self.initial_velocity)
        max_fuel_burn_time = self.fuel_mass / (self.alpha * thrust_lower)
        min_dry_mass_time = self.dry_mass * np.linalg.norm(initial_speed) / thrust_upper
        time_lower_bound, time_upper_bound = min_dry_mass_time, max_fuel_burn_time
        iteration_count = 0
        
        while not (time_upper_bound - time_lower_bound) ** 2 <= 10:
            iteration_count += 1
            print("[Golden Search]: Iteration Count:", iteration_count)
            time_difference = (time_upper_bound - time_lower_bound) * golden_ratio
            time_1, time_2 = time_lower_bound + time_difference, time_upper_bound - time_difference

            cost_1 = generate_solution(estimated_landing_time=time_1,
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
                                       N_tf=N)
            
            cost_2 = generate_solution(estimated_landing_time=time_1,
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
                                       N_tf=N)
            if cost_1 > cost_2:
                time_upper_bound = time_1
            elif cost_2 > cost_1:
                time_lower_bound = time_2

        optimal_time = (time_upper_bound + time_lower_bound) * 0.5
        return optimal_time
    
    def solve(self):
        estimated_flight_time = self.estimate_time(20)
        result = generate_solution(estimated_landing_time=estimated_flight_time,
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
                                   N_tf=160)
        return result

if __name__ == '__mian__':
    GFOLD(gravity=9.80665,
          dry_mass=(22.2)*1e3,
          fuel_mass=(13.4)*1e3,
          max_thrust=845000*3,
          min_throttle=0.4,
          max_throttle=1.,
          max_structural_Gs=9,
          specific_impulse=282,
          max_velocity=900,
          glide_slope_cone=1,
          thrust_pointing_constraint=120,
          planetary_angular_velocity=(2.53*1e-5, 0, 6.62*1e-5),
          initial_position=(2400, 20, 50),
          initial_velocity=(-80,  3,   1),
          plot=True)
    result = GFOLD.solve()
    print(result['x'])
    print(result['tf'])