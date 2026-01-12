import krpc
from threading import Lock

from control import *
from .utils import *
from .aero import simulate

class KAL:
    def __init__(self, name:str, target, params:dict, lock:Lock):
        self.conn = krpc.connect(name=f'KAL-{name}')
        self.params = params
        self.lock = lock
        self.badge = None
        self.target = None

        # define space center
        self.space_center = self.conn.space_center

        #define body
        self.body = self.space_center.bodies['Kerbin']

        # define g
        self.g = self.body.surface_gravity

        # define target
        if self.space_center.target_vessel:
            print(f"{name} TARGET VESSEL: {self.space_center.target_vessel.name}")
            self.target = (self.space_center.target_vessel.flight(self.body.reference_frame).longitude, self.space_center.target_vessel.flight(self.body.reference_frame).latitude)
            self.badge = True
        else:
            self.target = target
            self.badge = False

        #define target reference frame
        self.target_reference_frame = create_target_reference_frame(self.conn, self.target)

        # define vessel
        if name is None:
            self.vessel = Rocket(self.space_center, self.space_center.active_vessel, self.target_reference_frame)
        else:
            self.vessel = Rocket(self.space_center, find_vessel_by_name(self.conn, name), self.target_reference_frame)

        # define vessel reference frame
        self.vessel_reference_frame = self.vessel.reference_frame
        self.vessel_surface_reference_frame = self.vessel.surface_reference_frame

        # define params
        self.skip_boosterback:bool = params['skip_boosterback']
        self.skip_entryburn:bool = params['skip_entryburn']
        self.skip_aerodynamic_guidance:bool = params['skip_aerodynamic_guidance']
        self.max_tilt:float = params['max_tilt']
        self.throttle_limit:list = params['throttle_limit']
        self.target_roll:float = params['target_roll']
        self.terminal_guidiance_start_velocity:float = params['terminal_guidiance_start_velocity']
        self.land_confirm:bool = params['land_confirm']
        self.landing_gear:bool = params['landing_gear']
        self.final_altitude:float = params['final_altitude']
        if not has_legs(self.vessel):
            self.land_confirm = False
        
    def __boosterback(self):
        self.vessel.control.rcs = True
        error = [float('inf')]
        judged = False
        print(f"BOOTERBACK MANEUVER STARTED FOR {self.vessel.name}")
        _position = array(self.vessel.position())
        while True:
            position = array(self.vessel.position())
            velocity = array(self.vessel.velocity())
            available_thrust = self.vessel.available_thrust
            mass = self.vessel.mass
            time_to_apoapsis = max(velocity[0] / self.g, 0)
            apoapsis_altitude = position[0] + 0.5 * self.g * time_to_apoapsis**2
            impact_time = time_to_apoapsis + sqrt(2*apoapsis_altitude/self.g)
            if not judged and apoapsis_altitude < 50000 and not self.badge:
                self.skip_entryburn = True
                judged = True
            else:
                self.skip_entryburn = False
                judged = True

            # estimated_landing_point = impact_pointB(self.vessel, self.target_reference_frame)
            # _position = array(self.space_center.transform_position(position, self.target_reference_frame, self.body.reference_frame))
            # _velocity = array(self.space_center.transform_velocity(_position, velocity, self.target_reference_frame, self.body.reference_frame))
            # print(_position, _velocity)
            # print(self.body.rotational_speed)
            estimated_landing_point = simulate(array(self.vessel.vessel.position(self.body.reference_frame)), array(self.vessel.flight(self.body.reference_frame).velocity), body_radius=self.body.equatorial_radius, pos_origin=-position, max_t=impact_time)
            estimated_landing_point_dir = impact_point(self.vessel, self.target_reference_frame)
            # while estimated_landing_point[0] > 0:
            #     estimated_landing_point += (velocity + array([-self.g*sqrt(2*apoapsis_altitude/self.g), 0, 0]))
            # ratio = clamp(position[0]/self.body.atmosphere_depth, 0, 1)
            horizontal_error = sqrt(estimated_landing_point[-1][1][0]**2 + estimated_landing_point[-1][1][1]**2)
            _horizontal_error = norm(estimated_landing_point_dir[1:3])
            #for t,r,v,a in estimated_landing_point:
            #    print(f"{t}, {r[0]}, {r[1]}, {r[2]}, {v[0]}, {v[1]}, {v[2]}, {a[0]}, {a[1]}, {a[2]}")
            #print(estimated_landing_point[-1][1])
            #print(horizontal_error)
            # print(estimated_landing_point[-1][1][1], estimated_landing_point[-1][1][0])

            # target_direction = (0, -estimated_landing_point[-1][1][1], -estimated_landing_point[-1][1][0])
            target_direction = (0, -estimated_landing_point_dir[1], -estimated_landing_point_dir[2])
            self.vessel.update_ap(target_direction, self.target_roll)
            #target_velocity = horizontal_error / time_to_apoapsis
            #target_acceleration = (norm(target_velocity)**2 - norm(velocity)**2) / (2 * apoapsis_altitude)
            #throttle = target_acceleration * mass / available_thrust
            #throttle = clamp(throttle, self.throttle_limit[0], self.throttle_limit[1])
            throttle = self.throttle_limit[1]
            if _horizontal_error <= 1000:
                error.append(horizontal_error)
                throttle = self.throttle_limit[0]
            self.vessel.control.throttle = throttle
            # print("\tERROR:%.1f | TIME TO APOAPSOS:%.1f | THROTTLE:%.3f" % (horizontal_error, time_to_apoapsis, throttle))
            if horizontal_error > min(error):
                with self.lock: self.vessel.control.throttle = 0
                self.vessel.control.brakes = True
                return True

    def __entryburn(self):
        while self.vessel.flight(self.target_reference_frame).vertical_speed >= 0:
            with self.lock: self.vessel.update_ap((1, 0, 0))
        while self.vessel.flight(self.target_reference_frame).surface_altitude >= 50000:
            with self.lock: self.vessel.update_ap((1, 0, 0))
        print(f"ENTRY BURN MANEUVER STARTED FOR {self.vessel.name}")
        while True:
            position = array(self.vessel.position())
            velocity = array(self.vessel.velocity())
            estimated_landing_point = impact_point(self.vessel, self.target_reference_frame)

            target_direction = -(- velocity + array([0, estimated_landing_point[1], estimated_landing_point[2]]))
            target_direction = normalize(target_direction) + normalize(position)
            # target_direction = conic_clamp(-velocity, target_direction, 30)
            with self.lock: self.vessel.update_ap(target_direction)
            with self.lock: self.vessel.control.throttle = self.throttle_limit[1]

            #print("\tERROR: %.3f" % (horizontal_error))

            if norm(estimated_landing_point[1:3]) <= 500:
                return True
    
    def __aerodynamic_guidance(self):
        print(f"AERODYNAMIC GUIDANCE MANEUVER STARTED FOR {self.vessel.name}")
        while True:
            position = self.vessel.position()
            velocity = self.vessel.velocity()
            heading = self.vessel.flight(self.target_reference_frame).heading
            estimated_landing_point = impact_point(self.vessel, self.target_reference_frame)
            compensation = normalize(position) * estimated_landing_point[0]
            estimated_landing_point = estimated_landing_point + array([0, compensation[1], compensation[2]])
            altitude = self.vessel.flight(self.target_reference_frame).surface_altitude
            horizontal_error = norm(estimated_landing_point[1:3])
            gfold_start_altitude = max(5 * horizontal_error, 2000)
            ignition_altitude = ignition_height(self.vessel, self.target_reference_frame, gfold_start_altitude, -self.terminal_guidiance_start_velocity)
            ignition_altitude = clamp(ignition_altitude, 3000, 10000)

            target_direction = - velocity + array([0, estimated_landing_point[1], estimated_landing_point[2]])
            target_direction = normalize(target_direction) + normalize(position)
            target_direction = conic_clamp(-velocity, target_direction, self.max_tilt)
            target_roll = roll_controller(heading)
            self.vessel.update_ap(target_direction, target_roll)
            self.vessel.control.throttle = 0
            # print('\tALTITUDE:%.3f | IGNITION ALTITUDE:%.3f | ERROR:%.3f' % (altitude, ignition_altitude, horizontal_error))

            if altitude <= ignition_altitude:
                return True
    
    def __landingburn(self):
        print(f"LANDING BURN MANEUVER STARTED FOR {self.vessel.name}")
        self.vessel.control.rcs = False
        while True:
            position = self.vessel.position()
            velocity = self.vessel.velocity()
            available_thrust = self.vessel.available_thrust
            mass = self.vessel.mass
            altitude = self.vessel.flight(self.target_reference_frame).surface_altitude

            horizontal_error = norm(impact_point(self.vessel, self.target_reference_frame)[1:3])
            gfold_start_altitude = max(5 * horizontal_error, 1000)

            acc = (velocity[0]**2 - self.terminal_guidiance_start_velocity**2) / (2 * (position[0] - gfold_start_altitude))
            throttle = mass * acc / available_thrust if position[0] >= gfold_start_altitude else self.throttle_limit[1]
            throttle = clamp(throttle, self.throttle_limit[0], self.throttle_limit[1])

            self.vessel.control.throttle = throttle
            with self.lock: self.vessel.update_ap(-velocity)

            # print('\tALTITUDE:%.3f | THROTTLE:%.3f | ERROR:%.3f' % (position[0], throttle, horizontal_error))

            if altitude <= gfold_start_altitude and velocity[0] >= -self.terminal_guidiance_start_velocity:
                return True
    
    def __terminal(self):
        print(f"TERMINAL GUIDIANCE STARTED FOR {self.vessel.name}")
        start_vel = self.vessel.velocity()
        start_pos = self.vessel.position()
        target_pos = [get_half_rocket_length(self.vessel), 0, 0] if not self.final_altitude else [self.final_altitude, 0, 0]
        target_vel = [0, 0, 0]
        duration = estimate_duration(start_pos, start_vel, target_pos, target_vel)
        result = generate_cubic_with_vertical_end(
        start_pos, start_vel, target_pos, target_vel,
        duration=duration,
        blend_ratio=0.2  # 最后15%段平滑过渡
        )
        # self.conn.krpc.paused = True
        # draw_trajectory(self.conn, result['x'], result['u'], self.target_reference_frame)
        # self.conn.krpc.paused = False
        

        if result is None:
            nav_mode = "terminal"
        else:
            nav_mode = 'cruise'

            trajectory = array(result['x'])
            trajectory_position = [(trajectory[0, i], trajectory[1, i], trajectory[2, i]) for i in range(len(trajectory[0]))]
            trajectory_velocity = [(trajectory[3, i], trajectory[4, i], trajectory[5, i]) for i in range(len(trajectory[0]))]
            trajectory_acceleration = [(result['u'][0, i], result['u'][1, i], result['u'][2, i]) for i in range(len(result['u'][0]))]

            target_position = trajectory_position[-1]
        end = False
        # draw_trajectory(self.conn, result['x'], result['u'], self.target_reference_frame)
        half_rocket_length = get_half_rocket_length(self.vessel)

        while not end:
            if nav_mode == 'cruise':
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
                self.vessel.control.throttle = 0.5*(-2 - velocity[0]) if min_index > 100*(1-0.2) else throttle
                target_direction = conic_clamp(array([1, 0, 0]), target_direction, self.max_tilt)
                self.vessel.update_ap(target_direction, self.target_roll)
                time = velocity[0] / (self.vessel.thrust / (mass * self.g) - self.g)
                # print('\tTHROTTLE:%3f | COMPENSATION:%3f | INDEX:%i' % (throttle, compensation, min_index))

                if min_index > 100*(1-0.2) and self.landing_gear and time <= 4:
                    self.vessel.control.legs = True

                if (self.land_confirm and landed(self.vessel)) or velocity[0] >= 0:
                    self.vessel.control.throttle = 0.
                    self.vessel.control.rcs = True
                    #print(f"TOUCHDOWN VELOCITY:{velocity}")
                    #print(f"FINAL ERROR:{norm(position[1:3])}")
                    while True:
                        self.vessel.update_ap((1, 0, 0))
                        if norm(array(self.vessel.velocity())) <= 0.2:
                            self.vessel.control.rcs = False
                            break
                    print(f"{self.vessel.name} LANDED")
                    end = True
                    break

            if nav_mode == 'terminal':
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
                target_direction = conic_clamp((1,0,0), acc, self.max_tilt)
                throttle = descent_throttle(self.vessel, half_rocket_length) if velocity[0] <= -2 else mass * self.g / available_thrust
                self.vessel.update_ap(target_direction, radians(self.target_roll))
                self.vessel.control.throttle = throttle

                #print('\tTHROTTLE:%.3f | ERROR:%.3f | TIME:%.3f' % (throttle, norm(position[1:3]), time))

                if self.landing_gear and time <= 4:
                    self.vessel.control.legs = True

                if (self.land_confirm and landed(self.vessel)) or velocity[0] >= 0:
                    self.vessel.control.throttle = 0.
                    self.vessel.control.rcs = True
                    #print(f"TOUCHDOWN VELOCITY:{velocity}")
                    #print(f"FINAL ERROR:{norm(position[1:3])}")
                    while True:
                        self.vessel.update_ap((1, 0, 0))
                        if norm(array(self.vessel.velocity())) <= 0.2:
                            self.vessel.control.rcs = False
                            break
                    print(f"{self.vessel.name} LANDED")
                    end = True
                    break
    
    def land(self):
        if not self.skip_boosterback:
            self.__boosterback()
        if not self.skip_entryburn:
            self.__entryburn()
        if not self.skip_aerodynamic_guidance:
            self.__aerodynamic_guidance()
        self.__landingburn()
        # self.__terminal()
        import gfold_with_c
        return True