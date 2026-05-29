import numpy as np
import scipy as cp
from scipy.optimize import minimize
from scipy.integrate import solve_bvp
from typing import List, Tuple
from math import pi, sin, cos, radians, degrees

class AGSolver:
    def __init__(self,
                 diameter:float,
                 length:float,
                 AoA:float,
                 density_profile:List[List[float], List[float]],
                 mass: float,
                 position: Tuple[float, float, float],
                 velocity: Tuple[float, float, float],
                 gravitational_parameter: float,
                 planetary_angular_velocity: Tuple[float, float, float],
                 drag_coefficient: float,
                 lift_coefficient: float,
                 avaliable_thrust: float):
        self.diameter = diameter
        self.length = length
        self.AoA = AoA
        self.density_profile = density_profile
        self.mass = mass
        self.position = np.array(position)
        self.velocity = np.array(velocity)
        self.gravitational_parameter = gravitational_parameter
        self.planetary_angular_velocity = np.array(planetary_angular_velocity)
        self.drag_coefficient = drag_coefficient
        self.lift_coefficient = lift_coefficient
        self.avaliable_thrust = avaliable_thrust

    def calculate_Sref(self) -> float:
        return pi*self.diameter**2*abs(cos(radians(self.AoA)))/4 + self.diameter*self.length*abs(sin(radians(self.AoA)))
    
    def dynamics(self, t, X):
        r = X[0:3]
        v = X[3:6]
        p = X[6:9]
        g = self.gravitational_parameter / np.linalg.norm(r)**2
        F = -2*self.mass*np.cross(self.planetary_angular_velocity, v)
        drag = 
        lift = 
        a = -r/np.linalg.norm(r)*g + F/self.mass
        return np.concatenate([v, a])