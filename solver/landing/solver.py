import numpy as np
from scipy.integrate import solve_ivp
from math import pi

class SuicideBurnSolver():
    def __init__(self, 
                 mass: float,
                 dry_mass: float,
                 position: tuple[float, float, float],
                 velocity: tuple[float, float, float],
                 gravitational_parameter: float,
                 planetary_angular_velocity: tuple[float, float, float],
                 target_position: tuple[float, float, float],
                 available_thrust: float,
                 specific_impulse: float,
                 diameter: float,
                 length: float,
                 density_profile: list[tuple[float, float]],
                 drag_coefficient: float,
                 equatorial_radius: float,
                 g0:float = 9.80665):
        self.mass = mass
        self.dry_mass = dry_mass
        self.position = np.array(position)
        self.velocity = np.array(velocity)
        self.gravitational_parameter = gravitational_parameter
        self.planetary_angular_velocity = np.array(planetary_angular_velocity)
        self.target_position = np.array(target_position)
        self.available_thrust = 0.9*available_thrust
        self.specific_impulse = specific_impulse
        self.diameter = diameter
        self.length = length
        self.density_profile = density_profile
        self.drag_coefficient = drag_coefficient
        self.equatorial_radius = equatorial_radius
        self.g0 = g0
        self.max_burn_time = (self.mass - self.dry_mass) * self.specific_impulse * self.g0 / self.available_thrust

        if density_profile:
            altitudes = np.array([d[0] for d in density_profile])
            densities = np.array([d[1] for d in density_profile])
            sort_idx = np.argsort(altitudes)
            self._density_altitudes = altitudes[sort_idx]
            self._density_values = densities[sort_idx]
    
    def get_density(self, altitude: float) -> float:
        return float(np.interp(altitude, self._density_altitudes, self._density_values))
    
    def dynamics(self, t, X):
        r = X[0:3]
        v = X[3:6]
        m = X[6]
        g = self.gravitational_parameter / np.linalg.norm(r)**2
        F = -2*m*np.cross(self.planetary_angular_velocity, v) - m*np.cross(self.planetary_angular_velocity, np.cross(self.planetary_angular_velocity, r))
        rho = self.get_density(np.linalg.norm(r) - self.equatorial_radius)
        drag = pi*(0.5*self.diameter)**2 * self.drag_coefficient * rho * 0.5 * -np.linalg.norm(v) * v
        thrust = self.available_thrust * (-v / np.linalg.norm(v)) if m > self.dry_mass else np.zeros(3)
        a = -r/np.linalg.norm(r)*g + (F + drag + thrust)/m
        mass_flow = -self.available_thrust / (self.specific_impulse * self.g0) if m > self.dry_mass else 0
        return np.concatenate([v, a, [mass_flow]])
    
    def predict_impact_point(self):
        def zero_velocity_event(t, X):
            r = X[0:3]
            v = X[3:6]
            v_radial = np.dot(v, r) / np.linalg.norm(r)
            return v_radial
        zero_velocity_event.terminal = True
        zero_velocity_event.direction = -1
        solution = solve_ivp(
            fun=self.dynamics,
            t_span=[0, self.max_burn_time],
            events=zero_velocity_event,
            y0=np.concatenate([self.position, self.velocity, [self.mass]]),
            dense_output=True,
        )
        return solution.y_events