import krpc
from threading import Lock

from Control import *
from Solver import GFOLD
from .utils import *

class KAL:
    def __init__(self, name:str, target, params:dict, lock:Lock):
        self.conn = krpc.connect(name=f'KAL-{name}')
        self.target = target
        self.params = params
        self.lock = lock

        # define params
        self.skip_boosterback:bool = params['skip_boosterback']
        self.skip_entryburn:bool = params['skip_entryburn']
        self.skip_aerodynamic_guidance:bool = params['skip_aerodynamic_guidance']
        self.max_tilt:float = params['max_tilt']
        self.throttle_limit:list = params['throttle_limit']
        self.target_roll:float = params['target_roll']
        self.gfold_start_velocity:float = params['gfold_start_velocity']
        self.land_confirm:bool = params['land_confirm']
        self.landing_gear:bool = params['landing_gear']
        self.final_altitude:float = params['final_altitude']

        #define target reference frame
        self.target_reference_frame = create_target_reference_frame(self.conn, self.target)

        # define basic KRPC consts
        self.space_center = self.conn.space_center
        self.vessel = Rocket(self.space_center, find_vessel_by_name(self.conn, name), self.target_reference_frame)
        self.body = self.vessel.orbit.body
        self.g = self.body.surface_gravity

        # define vessel reference frame
        self.vessel_reference_frame = self.vessel.reference_frame
        self.vessel_surface_reference_frame = self.vessel.surface_reference_frame
        
    def _boosterback(self):
        self.vessel.control.rcs = True
        error = [float('inf')]
        print(f"BOOTERBACK MANEUVER STARTED FOR {self.vessel.name}")
        while True:
            position = array(self.vessel.position())
            velocity = array(self.vessel.velocity())
            available_thrust = self.vessel.available_thrust
            mass = self.vessel.mass
            time_to_apoapsis = max(velocity[0] / self.g, 0)
            apoapsis_altitude = position[0] + 0.5 * self.g * time_to_apoapsis**2

            estimated_landing_point = impact_point(self.vessel, self.target_reference_frame)
            horizontal_error = norm(estimated_landing_point[1:3])

            target_direction = (0, -estimated_landing_point[1], -estimated_landing_point[2])
            with self.lock: self.vessel.update_ap(target_direction, self.target_roll)
            target_velocity = horizontal_error / time_to_apoapsis
            target_acceleration = (norm(target_velocity)**2 - norm(velocity)**2) / (2 * apoapsis_altitude)
            throttle =target_acceleration * mass / available_thrust
            throttle = clamp(throttle, self.throttle_limit[0], self.throttle_limit[1])
            if horizontal_error <= 1000:
                error.append(horizontal_error)
                throttle = self.throttle_limit[0]
            with self.lock: self.vessel.control.throttle = throttle
            # print("\tERROR:%.1f | TIME TO APOAPSOS:%.1f | THROTTLE:%.3f" % (horizontal_error, time_to_apoapsis, throttle))
            if horizontal_error > min(error):
                with self.lock: self.vessel.control.throttle = 0
                self.vessel.control.brakes = True
                if self.vessel.flight(self.target_reference_frame).surface_altitude <= self.body.atmosphere_depth:
                    self.skip_entryburn = True
                return True

    def _entryburn(self):
        while self.vessel.flight(self.target_reference_frame).vertical_speed >= 0:
            with self.lock: self.vessel.update_ap((1, 0, 0))
        while self.vessel.flight(self.target_reference_frame).surface_altitude >= self.body.atmosphere_depth:
            with self.lock: self.vessel.update_ap((1, 0, 0))
        print(f"ENTRY BURN MANEUVER STARTED FOR {self.vessel.name}")
        while True:
            position = array(self.vessel.position())
            velocity = array(self.vessel.velocity())
            estimated_landing_point = impact_point(self.vessel, self.target_reference_frame)

            target_direction = -(- velocity + array([0, estimated_landing_point[1], estimated_landing_point[2]]))
            target_direction = normalize(target_direction) + normalize(position)
            target_direction = conic_clamp(-velocity, target_direction, 30)
            with self.lock: self.vessel.update_ap(target_direction)
            with self.lock: self.vessel.control.throttle = self.throttle_limit[1]

            #print("\tERROR: %.3f" % (horizontal_error))

            if norm(estimated_landing_point[1:3]) <= 500:
                return True
    
    def _aerodynamic_guidance(self):
        print(f"AERODYNAMIC GUIDANCE MANEUVER STARTED FOR {self.vessel.name}")
        while True:
            position = array(self.vessel.position())
            velocity = array(self.vessel.velocity())
            estimated_landing_point = impact_point(self.vessel, self.target_reference_frame)
            horizontal_error = norm(estimated_landing_point[1:3])
            heading = self.vessel.flight(self.target_reference_frame).heading
            gfold_start_altitude = max(5 * horizontal_error, 3000)
            ignition_altitude = max(ignition_height(self.vessel, self.target_reference_frame, gfold_start_altitude, -self.gfold_start_velocity), 5000)
            altitude = self.vessel.flight(self.target_reference_frame).surface_altitude
            

            target_direction = - velocity + array([0, estimated_landing_point[1], estimated_landing_point[2]])
            target_direction = normalize(target_direction) + normalize(position)
            target_direction = conic_clamp(-velocity, target_direction, self.max_tilt)
            target_roll = roll_controller(heading)
            with self.lock: self.vessel.update_ap(target_direction, target_roll)
            with self.lock: self.vessel.control.throttle = 0
            # print('\tALTITUDE:%.3f | IGNITION ALTITUDE:%.3f | ERROR:%.3f' % (altitude, ignition_altitude, horizontal_error))

            if altitude <= ignition_altitude:
                return True
    
    def _landingburn(self):
        print(f"LANDING BURN MANEUVER STARTED FOR {self.vessel.name}")
        self.vessel.control.rcs = False
        while True:
            position = array(self.vessel.position())
            velocity = array(self.vessel.velocity())
            available_thrust = self.vessel.available_thrust
            mass = self.vessel.mass
            altitude = self.vessel.flight(self.target_reference_frame).surface_altitude

            horizontal_error = norm(impact_point(self.vessel, self.target_reference_frame)[1:3])
            gfold_start_altitude = max(5 * horizontal_error, 3000)

            acc = (velocity[0]**2 - self.gfold_start_velocity**2) / (2 * (position[0] - gfold_start_altitude))
            throttle = mass * acc / available_thrust if position[0] >= gfold_start_altitude else self.throttle_limit[1]
            throttle = clamp(throttle, self.throttle_limit[0], self.throttle_limit[1])

            with self.lock: self.vessel.control.throttle = throttle
            with self.lock: self.vessel.update_ap(-velocity, self.target_roll)

            # print('\tALTITUDE:%.3f | THROTTLE:%.3f | ERROR:%.3f' % (position[0], throttle, horizontal_error))

            if altitude <= gfold_start_altitude and velocity[0] >= -self.gfold_start_velocity:
                return True
    
    def _gfold(self): # be aware of solver variable security for multithreading
        print(f"G-FOLD MANEUVER STARTED FOR {self.vessel.name}")
        self.conn.krpc.paused = True
        self.conn.ui.message('GENERATING SOLUTION', duration=1)

        self.lock.acquire()
        bundled_data = bundle_data(self.vessel, self.target_reference_frame, self.params)
        problem = GFOLD(bundled_data)
        problem.find_optimal_solution()

        result = problem.solution

        self.conn.ui.message('SOLUTION GENERATED', duration=1)
        self.conn.krpc.paused = False
        self.lock.release()

        nav_mode = 'GFOLD'
        end = False

        trajectory = array(result['x'])
        trajectory_position = [(trajectory[0, i], trajectory[1, i], trajectory[2, i]) for i in range(len(trajectory[0]))]
        trajectory_velocity = [(trajectory[3, i], trajectory[4, i], trajectory[5, i]) for i in range(len(trajectory[0]))]
        trajectory_acceleration = [(result['u'][0, i], result['u'][1, i], result['u'][2, i]) for i in range(len(result['u'][0]))]

        target_position = trajectory_position[-1]

        while not end:
            while nav_mode == 'GFOLD':
                # gather information
                velocity = array(self.vessel.velocity())
                position = array(self.vessel.position())
                mass = self.vessel.mass
                available_thrust = self.vessel.available_thrust
                aerodynamic_force = array(self.vessel.flight(self.target_reference_frame).aerodynamic_force)

                # get the index of nearest waypoints
                results_position = []
                for point in trajectory_position:
                    results_position.append(norm(point - position))
                min_index = results_position.index(min(results_position))

                # define waypoints
                position_waypoint = array(trajectory_position[min_index])
                velocity_waypoint = array(trajectory_velocity[min_index])
                acceleration_waypoint = array(trajectory_acceleration[min_index])

                # define errors
                velocity_error = velocity_waypoint - velocity
                position_error = position_waypoint - position

                # main control
                target_direction = acceleration_waypoint + velocity_error * 0.3 + position_error * 0.1
                target_direction_x = target_direction[0]
                while target_direction_x <= 0:
                    target_direction_x = target_direction_x + self.g
                target_direction = (target_direction_x, target_direction[1], target_direction[2])
                target_direction = self.space_center.transform_direction(target_direction, from_=self.target_reference_frame,to=self.vessel_surface_reference_frame)
                compensation = norm(aerodynamic_force[1:3]) / available_thrust
                throttle = norm(target_direction) / (available_thrust / mass) + compensation
                throttle = clamp(throttle, self.throttle_limit[0], self.throttle_limit[1])
                with self.lock: self.vessel.control.throttle = throttle
                target_direction = conic_clamp(array([1, 0, 0]), target_direction, self.max_tilt)
                with self.lock: self.vessel.update_ap(target_direction, self.target_roll)
                # print('\tTHROTTLE:%3f | COMPENSATION:%3f | INDEX:%i' % (throttle, compensation, min_index))

                if position[0] <= target_position[0]:
                    if not has_legs(self.vessel):
                        self.land_confirm = False
                        self.landing_gear= False
                    nav_mode = 'terminal'
                    half_rocket_length = get_half_rocket_length(self.vessel)
                    break
                
                if self.landing_gear and norm(position) / norm(velocity) <= 4:
                    self.vessel.control.legs = True

            while nav_mode == 'terminal':
                half_rocket_length = self.final_altitude if self.final_altitude != 0 else half_rocket_length
                velocity = array(self.vessel.velocity())
                position = array(self.vessel.position())
                available_thrust = self.vessel.available_thrust
                thrust = self.vessel.thrust
                mass = self.vessel.mass

                time = velocity[0] / (thrust / (mass * self.g) - self.g)
                prediction = position + velocity * time
                if norm(position[1:3]) <= 2 and norm(velocity[1:3]) <= 5: prediction = position
                position = array([position[0], prediction[1], prediction[2]])

                acc_hor = - position[1:3] * 0.3 - velocity[1:3] * 0.5
                acc_ver = available_thrust / mass
                acc_ver = max(acc_ver, norm(acc_hor) * 1.5)
                acc = array([0, acc_hor[0], acc_hor[1]]) + array([acc_ver, 0, 0])
                target_direction = conic_clamp((1,0,0), acc, 5)
                throttle = descent_throttle(self.vessel, target_height=half_rocket_length) if velocity[0] <= -2 else mass * self.g / available_thrust
                with self.lock: self.vessel.update_ap(target_direction, self.target_roll)
                with self.lock: self.vessel.control.throttle = throttle

                # print('\tTHROTTLE:%.3f | ERROR:%.3f | TIME:%.3f' % (throttle, norm(position[1:3]), time))

                if self.landing_gear and position[0] <= 100:
                    self.vessel.control.legs = True

                if (self.land_confirm and landed(self.vessel)) or velocity[0] >= 0:
                    with self.lock: self.vessel.control.throttle = 0.
                    self.vessel.control.rcs = True
                    print(f"TOUCHDOWN VELOCITY OF {self.vessel.name}:{velocity}")
                    print(f"FINAL ERROR OF {self.vessel.name}:{norm(position[1:3])}")
                    while True:
                        with self.lock: self.vessel.update_ap((1, 0, 0))
                        if norm(array(self.vessel.velocity())) <= 0.2:
                            self.vessel.control.rcs = False
                            break
                    print(f'END OF {self.vessel.name}')
                    end = True
                    return True
    
    def land(self):
        if not self.skip_boosterback:
            self._boosterback()
        if not self.skip_entryburn:
            self._entryburn()
        if not self.skip_aerodynamic_guidance:
            self._aerodynamic_guidance()
        self._landingburn()
        self._gfold()
        return True