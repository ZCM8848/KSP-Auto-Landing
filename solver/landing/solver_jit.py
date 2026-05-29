import numpy as np
from scipy.integrate import solve_ivp
from math import pi
from numba import njit


@njit(cache=True)
def _interp_density(altitude: float, altitudes: np.ndarray, densities: np.ndarray) -> float:
    """Numba-compatible linear interpolation for density."""
    n = altitudes.shape[0]
    if altitude <= altitudes[0]:
        return densities[0]
    if altitude >= altitudes[n - 1]:
        return densities[n - 1]
    
    # Binary search for interpolation interval
    lo = 0
    hi = n - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if altitude < altitudes[mid]:
            hi = mid
        else:
            lo = mid
    
    # Linear interpolation
    t = (altitude - altitudes[lo]) / (altitudes[hi] - altitudes[lo])
    return densities[lo] + t * (densities[hi] - densities[lo])


@njit(cache=True)
def _norm(v: np.ndarray) -> float:
    """Compute L2 norm of a 3D vector."""
    return np.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


@njit(cache=True)
def _cross(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Compute cross product of two 3D vectors."""
    return np.array([
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    ])


@njit(cache=True)
def _dot(a: np.ndarray, b: np.ndarray) -> float:
    """Compute dot product of two 3D vectors."""
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


@njit(cache=True)
def _dynamics_core(X: np.ndarray,
                   gravitational_parameter: float,
                   planetary_angular_velocity: np.ndarray,
                   equatorial_radius: float,
                   density_altitudes: np.ndarray,
                   density_values: np.ndarray,
                   diameter: float,
                   drag_coefficient: float,
                   available_thrust: float,
                   dry_mass: float,
                   specific_impulse: float,
                   g0: float) -> np.ndarray:
    """Numba-compiled core dynamics computation."""
    r = X[0:3].copy()
    v = X[3:6].copy()
    m = X[6]
    
    r_norm = _norm(r)
    v_norm = _norm(v)
    
    # Gravity
    g = gravitational_parameter / (r_norm * r_norm)
    
    # Coriolis and centrifugal forces
    omega_cross_v = _cross(planetary_angular_velocity, v)
    omega_cross_r = _cross(planetary_angular_velocity, r)
    omega_cross_omega_cross_r = _cross(planetary_angular_velocity, omega_cross_r)
    
    F = -2.0 * m * omega_cross_v - m * omega_cross_omega_cross_r
    
    # Atmospheric density and drag
    altitude = r_norm - equatorial_radius
    rho = _interp_density(altitude, density_altitudes, density_values)
    
    drag_factor = pi * (0.5 * diameter) ** 2 * drag_coefficient * rho * 0.5 * (-v_norm)
    drag = drag_factor * v
    
    # Thrust (always opposite to velocity direction)
    if m > dry_mass:
        thrust_mag = available_thrust
        thrust = np.array([-thrust_mag * v[0] / v_norm,
                          -thrust_mag * v[1] / v_norm,
                          -thrust_mag * v[2] / v_norm])
        mass_flow = -available_thrust / (specific_impulse * g0)
    else:
        thrust = np.zeros(3)
        mass_flow = 0.0
    
    # Acceleration
    g_vec = np.array([-r[0] / r_norm * g,
                     -r[1] / r_norm * g,
                     -r[2] / r_norm * g])
    a = g_vec + (F + drag + thrust) / m
    
    result = np.empty(7)
    result[0:3] = v
    result[3:6] = a
    result[6] = mass_flow
    
    return result


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
        self.position = np.array(position, dtype=np.float64)
        self.velocity = np.array(velocity, dtype=np.float64)
        self.gravitational_parameter = gravitational_parameter
        self.planetary_angular_velocity = np.array(planetary_angular_velocity, dtype=np.float64)
        self.target_position = np.array(target_position, dtype=np.float64)
        self.available_thrust = 0.9 * available_thrust
        self.specific_impulse = specific_impulse
        self.diameter = diameter
        self.length = length
        self.density_profile = density_profile
        self.drag_coefficient = drag_coefficient
        self.equatorial_radius = equatorial_radius
        self.g0 = g0
        self.max_burn_time = (self.mass - self.dry_mass) * self.specific_impulse * self.g0 / self.available_thrust

        if density_profile:
            altitudes = np.array([d[0] for d in density_profile], dtype=np.float64)
            densities = np.array([d[1] for d in density_profile], dtype=np.float64)
            sort_idx = np.argsort(altitudes)
            self._density_altitudes = altitudes[sort_idx]
            self._density_values = densities[sort_idx]
        else:
            self._density_altitudes = np.array([], dtype=np.float64)
            self._density_values = np.array([], dtype=np.float64)
    
    def get_density(self, altitude: float) -> float:
        return float(_interp_density(altitude, self._density_altitudes, self._density_values))
    
    def dynamics(self, t: float, X: np.ndarray) -> np.ndarray:
        return _dynamics_core(X.astype(np.float64),
                             self.gravitational_parameter,
                             self.planetary_angular_velocity,
                             self.equatorial_radius,
                             self._density_altitudes,
                             self._density_values,
                             self.diameter,
                             self.drag_coefficient,
                             self.available_thrust,
                             self.dry_mass,
                             self.specific_impulse,
                             self.g0)
    
    def predict_impact_point(self):
        def zero_velocity_event(t, X):
            r = X[0:3]
            v = X[3:6]
            r_norm = np.linalg.norm(r)
            v_radial = np.dot(v, r) / r_norm
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