from solver.RTLS.solver import ImapctPointSolver

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
    
    for i in range(10):
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