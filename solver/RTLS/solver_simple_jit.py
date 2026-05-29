import numpy as np
from math import pi, cos, sin, radians
from scipy.integrate import solve_ivp
from numba import njit

@njit(cache=True)
def _dynamics_core(r, v, planetary_angular_velocity, gravitational_parameter):
    """JIT编译的核心动力学计算"""
    r_norm = np.sqrt(r[0]**2 + r[1]**2 + r[2]**2)
    g = gravitational_parameter / (r_norm ** 2)
    
    # 计算科里奥利力和离心力: F = -2m(ω×v) - m(ω×(ω×r))
    omega_cross_v = np.cross(planetary_angular_velocity, v)
    omega_cross_r = np.cross(planetary_angular_velocity, r)
    omega_cross_omega_cross_r = np.cross(planetary_angular_velocity, omega_cross_r)
    
    F = -2.0 * omega_cross_v - omega_cross_omega_cross_r
    a = -r / r_norm * g + F
    
    return a

@njit(cache=True)
def _compute_norm(vec):
    """JIT编译的向量范数计算"""
    return np.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)


class ImapctPointSolver():
    def __init__(self, 
                 mass: float,
                 position: tuple[float, float, float],
                 velocity: tuple[float, float, float],
                 gravitational_parameter: float,
                 planetary_angular_velocity: tuple[float, float, float],
                 target_position: tuple[float, float, float],):
        self.mass = mass
        self.position = np.array(position, dtype=np.float64)
        self.velocity = np.array(velocity, dtype=np.float64)
        self.gravitational_parameter = gravitational_parameter
        self.planetary_angular_velocity = np.array(planetary_angular_velocity, dtype=np.float64)
        self.target_position = np.array(target_position, dtype=np.float64)
        self._target_norm = _compute_norm(self.target_position)
    
    def dynamics(self, t, X):
        r = X[0:3]
        v = X[3:6]
        a = _dynamics_core(r, v, self.planetary_angular_velocity, self.gravitational_parameter)
        return np.concatenate([v, a])
    
    def predict_impact_point(self):
        def impact_event(t, X):
            r = X[0:3]
            r_norm = np.sqrt(r[0]**2 + r[1]**2 + r[2]**2)
            return r_norm - self._target_norm
        
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
