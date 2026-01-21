import numpy as np
from scipy.integrate import solve_ivp

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
    
if __name__ == '__main__':
    """
    Test script for ImpactPointSolver 
    (note: class name has a typo: "ImapctPointSolver")
    
    Simulates a ballistic projectile under Earth's gravity and Coriolis effect.
    """
    
    import numpy as np
    
    # Earth physical constants
    EARTH_MU = 3.986004418e14  # Earth's gravitational parameter [m³/s²]
    EARTH_RADIUS = 6371e3  # Earth's mean radius [m]
    EARTH_ROTATION_RATE = 7.2921159e-5  # Earth's angular velocity [rad/s]
    
    print("="*50)
    print("IMPACT POINT SOLVER TEST")
    print("="*50)
    
    # Test case: Projectile launched downward from 100 km altitude
    print("\n[Test Case: Suborbital projectile from 100 km altitude]")
    
    mass = 50.0  # [kg]
    
    # Initial position: 100 km above Earth's surface (at equator)
    initial_altitude = 100e3  # [m]
    initial_position = (EARTH_RADIUS + initial_altitude, 0, 0)
    
    # Initial velocity: 2 km/s radially downward
    initial_speed = 2000.0  # [m/s]
    initial_velocity = (-initial_speed, 0, 0)
    
    # Target: Point on Earth's surface directly below
    target_position = (EARTH_RADIUS, 0, 0)
    
    # Earth's rotation vector (along z-axis)
    planetary_angular_velocity = (0, 0, EARTH_ROTATION_RATE)
    
    print(f"\nInitial Conditions:")
    print(f"  Position: {initial_position} m")
    print(f"  Velocity: {initial_velocity} m/s")
    print(f"  Target radius: {np.linalg.norm(target_position):.2f} m")
    print(f"  Planetary angular velocity: {planetary_angular_velocity} rad/s")
    
    # Instantiate the solver
    # Note: Using the class as defined (with the typo in its name)
    solver = ImapctPointSolver(
        mass=mass,
        position=initial_position,
        velocity=initial_velocity,
        gravitational_parameter=EARTH_MU,
        planetary_angular_velocity=planetary_angular_velocity,
        target_position=target_position
    )
    
    try:
        # Run impact prediction
        print("\nRunning simulation...")
        impact_events = solver.predict_impact_point()
        
        # Check results
        if impact_events and len(impact_events) > 0 and len(impact_events[0]) > 0:
            print("✅ Impact detected!")
            
            # Extract state at impact
            impact_state = impact_events[0][0]  # First event, first occurrence
            impact_position = impact_state[0:3]
            impact_velocity = impact_state[3:6]
            
            print(f"\nImpact Details:")
            print(f"  Position (m):     {impact_position}")
            print(f"    Altitude:       {np.linalg.norm(impact_position) - EARTH_RADIUS:.2f} m")
            print(f"    Target radius:  {np.linalg.norm(target_position):.2f} m")
            print(f"  Velocity (m/s):   {impact_velocity}")
            print(f"    Speed:          {np.linalg.norm(impact_velocity):.2f} m/s")
            
            # Note: To get impact time, modify predict_impact_point to return solution.t_events
            # For now, we only get the state at impact
            
        else:
            print("❌ No impact detected within the 600-second simulation window.")
            print("   Suggestions:")
            print("   - Increase time span in predict_impact_point()")
            print("   - Use a more negative initial vertical velocity")
            print("   - Check if projectile is in stable orbit")
            
    except Exception as e:
        print(f"❌ Simulation failed with error:")
        print(f"   {type(e).__name__}: {e}")
        print("\nTroubleshooting:")
        print("   - Verify physical constants are realistic")
        print("   - Check that initial position is outside target radius")
        print("   - Ensure velocity is directed toward target")