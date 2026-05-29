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
                 target_position: tuple[float, float, float],):
        self.mass = mass
        self.position = np.array(position)
        self.velocity = np.array(velocity)
        self.gravitational_parameter = gravitational_parameter
        self.planetary_angular_velocity = np.array(planetary_angular_velocity)
        self.target_position = np.array(target_position)
    
    def dynamics(self, t, X):
        r = X[0:3]
        v = X[3:6]
        r_norm = np.linalg.norm(r)
        g = self.gravitational_parameter / r_norm**2
        F = -2*self.mass*np.cross(self.planetary_angular_velocity, v) - self.mass*np.cross(self.planetary_angular_velocity, np.cross(self.planetary_angular_velocity, r))
        a = -r/np.linalg.norm(r)*g + F/self.mass
        return np.concatenate([v, a])
    
    def predict_impact_point(self):
        def impact_event(t, X):
            return np.linalg.norm(X[0:3]) - np.linalg.norm(self.target_position)
        
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