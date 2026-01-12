import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import minimize_scalar

class ImapctPointSolver():
    def __init__(self, 
                 mass: float,
                 position: tuple[float, float, float],
                 velocity: tuple[float, float, float],
                 gravitational_parameter: float,
                 planetary_angular_velocity: tuple[float, float, float],
                 target_position: tuple[float, float, float]):
        self.mass = mass
        self.position = np.array(position)
        self.velocity = np.array(velocity)
        self.gravitational_parameter = gravitational_parameter
        self.planetary_angular_velocity = np.array(planetary_angular_velocity)
        self.target_position = np.array(target_position)
        
    def dynamics(self, t, X):
        r = X[0:3]
        v = X[3:6]
        g = self.gravitational_parameter / np.linalg.norm(r)**2
        F = -2*self.mass*np.cross(self.planetary_angular_velocity, v)
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
        )
        return solution.y_events