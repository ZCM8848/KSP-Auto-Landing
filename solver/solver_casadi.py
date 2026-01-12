import casadi as ca
import numpy as np
import os
# import mosek
from cvxpygen import cpg
from .config import GFoldConfig
import warnings
warnings.filterwarnings("ignore")

class GFoldSolver:
    """
    G-FOLD solver that implements the Fuel Optimal Large Divert Guidance Algorithm.
    """
    
    def __init__(self, config=None):
        """
        Initialize the G-FOLD solver.
        
        Args:
            config (GFoldConfig): Configuration object with problem parameters
        """
        self.config = config if config is not None else GFoldConfig()
            
        self.parameters = {}
        self.variables = {}
        self.constraints = []
        self.opti = ca.Opti()
        self._setup_problem()

    def _setup_problem(self):
        """Set up the G-FOLD optimization problem."""
        config = self.config
        n = config.solver.n
        t = config.solver.time_of_flight
        
        # Variables
        x = self.opti.variable(n, 6)
        u = self.opti.variable(n, 3)
        s = self.opti.variable(n)
        z = self.opti.variable(n)
        
        self.variables = {
            "x": x,
            "u": u,
            "s": s,
            "z": z
        }
        
        # Parameters
        log_mass = config.spacecraft.log_wet_mass
        max_vel = config.spacecraft.max_velocity
        sin_glide_slope = config.environment.sin_glide_slope # nonneg=True
        log_dry_mass = config.spacecraft.log_dry_mass # nonneg=True
        min_t = config.spacecraft.min_thrust # nonneg=True
        max_t = config.spacecraft.max_thrust # nonneg=True
        dt = t / n
        a = config.spacecraft.fuel_consumption
        a_dt = a * dt

        # Derived parameters
        z0 = self.opti.parameter(n)
        exp_z0 = self.opti.parameter(n) # nonneg=True
        max_exp = self.opti.parameter(n) # nonneg=True
        min_exp = self.opti.parameter(n) # nonneg=True
        
        # Calculate values
        c_z0 = []
        c_exp_z0 = []
        c_max_exp = []
        c_min_exp = []

        for i in range(n):
            z00 = np.log(config.spacecraft.wet_mass - a*dt*max_t*i)
            c_z0.append(z00)
            c_exp_z0.append(np.exp(-z00))
            c_max_exp.append(1/(np.exp(-z00) * max_t))
            if min_t != 0:
                c_min_exp.append(1/(np.exp(-z00) * min_t))

        z0 = c_z0
        exp_z0 = c_exp_z0
        max_exp = c_max_exp
        min_exp = c_min_exp

        # More parameters
        max_angle = config.environment.cos_max_angle 
        initial_position = np.array(config.spacecraft.initial_position).reshape((1, 3))
        initial_velocity = np.array(config.spacecraft.initial_velocity).reshape((1, 3))
        target_vel = np.array(config.spacecraft.target_velocity).reshape((1, 3))
        g = np.array(config.environment.gravity).reshape((1,3))
        dt_squared = dt** 2  
        g_dt = g * dt       
        g_dt_sq = g * dt_squared
        
        # Store parameters
        self.parameters = {
            "log_mass": log_mass,
            "max_vel": max_vel,
            "sin_glide_slope": sin_glide_slope,
            "log_dry_mass": log_dry_mass,
            "min_thrust": min_t,
            "max_thrust": max_t,
            "dt": dt,
            "fuel_consumption": a,
            "fuel_consumption_dt": a_dt,
            "z0": z0,
            "exp_z0": exp_z0,
            "max_exp_z0": max_exp,
            "min_exp_z0": min_exp,
            "max_angle": max_angle,
            "dt_squared": dt_squared,
            "initial_position": initial_position,
            "initial_velocity": initial_velocity,
            "target_velocity": target_vel,
            "gravity": g,
            "gravity_dt": g_dt,
            "gravity_dt_squared": g_dt_sq,
        }
        
        # Constraints
        self.opti.subject_to(x[0, :3] == initial_position)
        self.opti.subject_to(x[0, 3:] == initial_velocity)
        self.opti.subject_to(z[0] == log_mass)

        # Timestep constraints
        for i in range(n):
            self.opti.subject_to(ca.norm_2(x[i, 3:]) <= max_vel) # never exceed the maximum velocity
            self.opti.subject_to(x[i, 2] >= ca.norm_2(x[i, :3]) * sin_glide_slope)  # glide slope constraint
            self.opti.subject_to(s[i] >= ca.norm_2(u[i, :]))  # |u| = s
            self.opti.subject_to((1 - (z[i]-z0[i]) + (z[i]-z0[i])**2/2) <= s[i] * min_exp[i])
            self.opti.subject_to(s[i] * max_exp[i] <= (1 - (z[i]-z0[i])))  # upper bound for s 
            self.opti.subject_to(s[i] * min_exp[i] >= (1 - (z[i]-z0[i])))  # lower bound for s
            self.opti.subject_to(x[i, 2] >= 0)  # stay above ground
            if i != n - 1:
                acc = (u[i+1, :] + u[i, :])/2
                self.opti.subject_to(x[i+1, :3] == x[i, :3] + (x[i, 3:] + x[i+1, 3:]) * dt / 2 + (acc*dt_squared+g_dt_sq) * (1/2))
                self.opti.subject_to(x[i+1, 3:] == x[i, 3:] + acc*dt + g_dt)
                self.opti.subject_to(z[i+1] == z[i] - (s[i] + s[i+1]) / 2 * a_dt)
                self.opti.subject_to(z[i+1] <= z[i])
                self.opti.subject_to(x[i+1, 2] <= x[i, 2])

        # Constraints on the last step
        self.opti.subject_to(x[n-1, :3] == np.array(config.spacecraft.target_position).reshape((1, 3)))
        self.opti.subject_to(x[n-1, 3:] == target_vel)
        self.opti.subject_to(z[n-1] >= log_dry_mass)
        self.opti.subject_to(u[n-1, 0] == 0)
        self.opti.subject_to(u[n-1, 1] == 0)
        
        # Objective: maximize final mass
        self.opti.minimize(-z[n-1])
        
    def guess_solution(self):
        """
        Generate an initial guess for the G-FOLD optimization problem using 
        polynomial trajectory interpolation.
        """
        config = self.config
        n = config.solver.n
        t_f = config.solver.time_of_flight
        dt = t_f / n

        # Time points for discretization
        time_points = np.linspace(0, t_f, n)

        # Extract boundary conditions from config
        r0 = np.array(config.spacecraft.initial_position, dtype=float)
        v0 = np.array(config.spacecraft.initial_velocity, dtype=float)
        rf = np.array(config.spacecraft.target_position, dtype=float)
        vf = np.array(config.spacecraft.target_velocity, dtype=float)

        # Mass bounds
        m_wet = config.spacecraft.wet_mass
        m_dry = config.spacecraft.wet_mass - config.spacecraft.fuel
        mass_range = m_wet - m_dry

        # Thrust bounds
        T_min = config.spacecraft.min_thrust
        T_max = config.spacecraft.max_thrust

        # Physical constants
        g_vec = np.array(config.environment.gravity, dtype=float)
        a = config.spacecraft.fuel_consumption  # fuel consumption coefficient

        # --- 1. Position and Velocity Guess ---
        # Use cubic polynomial for position: r(t) = a3*t^3 + a2*t^2 + a1*t + a0
        # Solve for coefficients that satisfy boundary conditions
        positions = np.zeros((n, 3))
        velocities = np.zeros((n, 3))

        for axis in range(3):
            # Set up system of equations for cubic polynomial coefficients
            # r(0) = r0, r(t_f) = rf
            # v(0) = v0, v(t_f) = vf
            # where v(t) = 3*a3*t^2 + 2*a2*t + a1

            A = np.array([
                [0, 0, 0, 1],                    # r(0) = a0
                [t_f**3, t_f**2, t_f, 1],        # r(t_f)
                [0, 0, 1, 0],                    # v(0) = a1
                [3*t_f**2, 2*t_f, 1, 0]          # v(t_f)
            ])

            b = np.array([
                r0[axis],  # initial position
                rf[axis],  # final position
                v0[axis],  # initial velocity
                vf[axis]   # final velocity
            ])

            # Solve for polynomial coefficients
            coeffs = np.linalg.solve(A, b)
            a3, a2, a1, a0 = coeffs

            # Evaluate position and velocity at each time point
            for i, t in enumerate(time_points):
                positions[i, axis] = a3 * t**3 + a2 * t**2 + a1 * t + a0
                velocities[i, axis] = 3 * a3 * t**2 + 2 * a2 * t + a1

        # Apply glide slope constraint (ensure altitude is above line)
        for i in range(n):
            r_norm = np.linalg.norm(positions[i, :2])  # horizontal distance
            min_altitude = r_norm * config.environment.sin_glide_slope
            positions[i, 2] = max(positions[i, 2], min_altitude)

        # Ensure altitude is non-negative
        positions[:, 2] = np.maximum(positions[:, 2], 0.0)

        # --- 2. Mass Profile Guess ---
        # Linear interpolation between wet and dry mass
        # In reality, mass decreases faster during high-thrust phases
        mass_guess = np.linspace(m_wet, m_dry, n) * 0.8 + m_dry * 0.2  # Conservative estimate
        log_mass_guess = np.log(mass_guess)

        # --- 3. Thrust Profile Guess ---
        # Estimate thrust from dynamics: T/m = a - g
        # Compute acceleration from velocity profile
        thrusts = np.zeros((n, 3))
        thrust_magnitudes = np.zeros(n)

        for i in range(n):
            if i == 0:
                acc = (velocities[i+1] - velocities[i]) / dt
            elif i == n-1:
                acc = (velocities[i] - velocities[i-1]) / dt
            else:
                acc = (velocities[i+1] - velocities[i-1]) / (2 * dt)

            # Required thrust per unit mass
            thrust_per_mass = acc - g_vec

            # Total thrust magnitude
            thrust_mag = np.linalg.norm(thrust_per_mass) * mass_guess[i]

            # Clip thrust to allowable range (with some margin for initial guess)
            thrust_mag_clipped = np.clip(thrust_mag, T_min * 1.2, T_max * 0.8)

            # Store thrust magnitude
            thrust_magnitudes[i] = thrust_mag_clipped

            # Store thrust vector (aligned with thrust_per_mass direction)
            if np.linalg.norm(thrust_per_mass) > 1e-6:
                thrusts[i, :] = thrust_per_mass / np.linalg.norm(thrust_per_mass) * thrust_mag_clipped
            else:
                # If no acceleration needed, assume upward thrust to counter gravity
                thrusts[i, :] = np.array([0, 0, -g_vec[2] * mass_guess[i]])

        # Adjust thrust for final step (should be around hover thrust)
        thrusts[-1, :] = np.array([0, 0, -g_vec[2] * mass_guess[-1]])
        thrust_magnitudes[-1] = np.linalg.norm(thrusts[-1, :])

        # --- 4. Compute normalized thrust 's' and 'z' profiles ---
        s_guess = np.zeros(n)
        z_guess = log_mass_guess.copy()

        # Precompute z0, exp_z0, max_exp, min_exp (same as in _setup_problem)
        dt = config.solver.time_of_flight / n
        c_z0 = []
        c_exp_z0 = []
        c_max_exp = []
        c_min_exp = []

        for i in range(n):
            z00 = np.log(config.spacecraft.wet_mass - a * dt * T_max * i)
            c_z0.append(z00)
            c_exp_z0.append(np.exp(-z00))
            c_max_exp.append(1 / (np.exp(-z00) * T_max))
            if T_min != 0:
                c_min_exp.append(1 / (np.exp(-z00) * T_min))
            else:
                c_min_exp.append(np.inf)

        # Initial guess for s from thrust magnitude
        for i in range(n):
            if i < n-1:
                s_guess[i] = thrust_magnitudes[i] / mass_guess[i]
            else:
                s_guess[-1] = T_min / mass_guess[-1]

        # Adjust s to satisfy constraints (roughly)
        for i in range(n):
            dz = z_guess[i] - c_z0[i]
            if dz > 0:
                s_guess[i] = max(s_guess[i], (1 - dz + dz**2/2) * c_min_exp[i])
                s_guess[i] = min(s_guess[i], (1 - dz) * c_max_exp[i])

        # --- 5. Set initial guesses for CasADi variables ---
        x_init = np.zeros((n, 6))
        u_init = np.zeros((n, 3))

        x_init[:, :3] = positions
        x_init[:, 3:] = velocities
        u_init = thrusts

        # Set initial guesses
        self.opti.set_initial(self.variables["x"], x_init)
        self.opti.set_initial(self.variables["u"], u_init)
        self.opti.set_initial(self.variables["s"], s_guess)
        self.opti.set_initial(self.variables["z"], z_guess)

        # Return the initial guess as a dictionary for inspection
        return {
            "positions": positions,
            "velocities": velocities,
            "thrusts": thrusts,
            "thrust_magnitudes": thrust_magnitudes,
            "mass": mass_guess,
            "s": s_guess,
            "z": z_guess,
            "time_points": time_points
        }

    def solve(self, verbose=False):
        """
        Solve the G-FOLD optimization problem.
        
        Args:
            verbose (bool): Whether to print verbose output
            
        Returns:
            dict: Solution containing positions, velocities, thrusts, and other data
        """

        # solution_val = self.problem.solve(verbose=verbose, solver=cp.MOSEK)
        self.opti.solver('qpsol', {'qpsol': {'solver': 'qrqp'}})
        self.guess_solution()
        solution_val = self.opti.solve()
        # solution_val = self.problem.solve(verbose=verbose, solver=cp.ECOS)
        
        # Extract solution data
        x_val = self.variables["x"]
        u_val = self.variables["u"]
        z_val = self.variables["z"]
        s_val = self.variables["s"]
        
        positions = x_val[:, :3]
        velocities = x_val[:, 3:]
        thrusts = np.array([np.linalg.norm(u) for u in u_val])
        
        # Adjust thrust for mass
        for i in range(self.config.solver.n):
            thrusts[i] *= np.exp(z_val[i])
        
        return {
            "solution_value": solution_val,
            "positions": positions,
            "velocities": velocities,
            "thrusts": thrusts,
            "normalized_thrusts": thrusts / self.config.spacecraft.real_max_thrust,
            "final_mass": np.exp(z_val[-1]),
            "z_values": z_val,
            "x_values": x_val,
            "u_values": u_val,
            "s_values": s_val,
            "time_points": np.arange(0, self.parameters["dt"].value * self.config.solver.n, self.parameters["dt"].value)
        }
        
    def generate_code(self, code_dir="code"):
        """
        Generate C++/Python code for the solver using cvxpygen.
        
        Args:
            code_dir (str): Directory to save the generated code
            
        Returns:
            str: Path to the generated code
        """
            
        # Create directory if it doesn't exist
        os.makedirs(code_dir, exist_ok=True)
        
        # Generate code
        cpg.generate_code(self.problem, code_dir=code_dir, solver=cp.QOCO)
        # cpg.generate_code(self.problem, code_dir=code_dir, solver=cp.ECOS)
        return code_dir

    def update_parameter(self, param_name, new_value):
        """
        Update a parameter value.
        
        Args:
            param_name (str): Name of the parameter to update
            new_value: New value for the parameter
        """
        if param_name not in self.parameters:
            raise ValueError(f"Parameter {param_name} not found")
            
        self.parameters[param_name].value = new_value
    
    def update_config(self, **kwargs):
        """
        Update configuration parameters and rebuild the problem.
        
        Args:
            **kwargs: Configuration parameters to update
        """
        self.config._process_kwargs(kwargs)
        
        # Rebuild the problem with updated configuration
        self._setup_problem()
