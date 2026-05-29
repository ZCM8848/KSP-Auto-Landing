import numpy as np
from math import pi, cos, sin, radians
from scipy.integrate import solve_ivp

class ImapctPointSolver():
    def __init__(self, 
                 mass: float,
                 position: tuple[float, float, float],
                 velocity: tuple[float, float, float],
                 gravitational_parameter: float,
                 planetary_angular_velocity: tuple[float, float, float],
                 target_position: tuple[float, float, float],
                 drag_coefficient: float,
                 lift_coefficient: float,
                 diameter: float,
                 length: float,
                 equatorial_radius: float,
                 max_AoA: float,
                 AoA: float,
                 direction: tuple[float, float, float],
                 density_profile: list[tuple[float, float]],
                 local_altitude: float = 0,):
        self.mass = mass
        self.position = np.array(position)
        self.velocity = np.array(velocity)
        self.gravitational_parameter = gravitational_parameter
        self.planetary_angular_velocity = np.array(planetary_angular_velocity)
        self.target_position = np.array(target_position)
        self.drag_coefficient = drag_coefficient
        self.lift_coefficient = lift_coefficient
        self.diameter = diameter
        self.length = length
        self.equatorial_radius = equatorial_radius
        self.density_altitudes = np.array([d[0] for d in density_profile])
        self.density_values = np.array([d[1] for d in density_profile])
        self.max_AoA = max_AoA
        self.AoA = AoA
        self.dynamic_pressure = list()
        self.aerodynamic_force = list()
        self.Sref = self.calculate_Sref()
        self.local_altitude = local_altitude
        self.direction = np.array(direction)
    
    def get_density(self, altitude: float) -> float:
        return float(np.interp(altitude, self.density_altitudes, self.density_values))
    
    def calculate_Sref(self) -> float:
        return pi*self.diameter**2*abs(cos(radians(self.AoA)))/4 + self.diameter*self.length*abs(sin(radians(self.AoA)))
    
    def dynamics(self, t, X):
        r = X[0:3]
        v = X[3:6]
        r_norm = np.linalg.norm(r)
        v_norm = np.linalg.norm(v)
        r_hat = r / r_norm
        v_hat = v / v_norm
        
        g = self.gravitational_parameter / r_norm**2
        rho = self.get_density(r_norm - self.equatorial_radius)
        
        # Drag (opposite to velocity)
        drag = pi * (0.5*self.diameter)**2 * self.drag_coefficient * rho * 0.5 * (-v_norm) * v
        
        # Lift (perpendicular to velocity, in the plane containing r and v, pointing "up" away from planet center)
        # lift_dir = cross(v_hat, cross(r_hat, v_hat)) normalized
        temp = np.cross(r_hat, v_hat)
        lift_dir = np.cross(v_hat, temp)
        lift_dir_norm = np.linalg.norm(lift_dir)
        if lift_dir_norm > 1e-10:
            lift_dir = lift_dir / lift_dir_norm
        else:
            lift_dir = np.zeros(3)
        lift = 0.5 * rho * self.lift_coefficient * self.Sref * v_norm**2 * lift_dir
        drag = np.zeros(3)
        lift = np.zeros(3)
        F = -2*self.mass*np.cross(self.planetary_angular_velocity, v) - self.mass*np.cross(self.planetary_angular_velocity, np.cross(self.planetary_angular_velocity, r))
        a = -r/np.linalg.norm(r)*g + (F + drag + lift)/self.mass
        q = 0.5 * rho * np.linalg.norm(v)**2

        self.dynamic_pressure.append(q)
        self.aerodynamic_force.append(drag+lift)
        return np.concatenate([v, a])
    
    def predict_impact_point(self):
        def impact_event(t, X):
            return (np.linalg.norm(X[0:3]) - self.local_altitude - 0.5*self.length) - np.linalg.norm(self.target_position)
        
        impact_event.terminal = True
        impact_event.direction = -1
        solution = solve_ivp(
            fun=self.dynamics,
            t_span=[0, 600],
            events=impact_event,
            y0=np.concatenate([self.position, self.velocity]),
            dense_output=True
        )
        return {
            'solution': solution,
            'event_t': solution.t_events[0] if solution.t_events else None,
            'event_X': solution.y_events[0] if solution.y_events else None
        }