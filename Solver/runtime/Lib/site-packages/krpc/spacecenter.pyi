from typing import Tuple, Set, Dict, List, TYPE_CHECKING, Optional, Any, Callable


class SpaceCenter:

    @property
    def active_vessel(self):
        # type: () -> SpaceCenter.Vessel
        """The currently active vessel."""
        ...

    @active_vessel.setter
    def active_vessel(self, value):
        # type: (SpaceCenter.Vessel) -> None
        ...

    @property
    def alarm_manager(self):
        # type: () -> SpaceCenter.AlarmManager
        """The alarm manager."""
        ...

    @property
    def bodies(self):
        # type: () -> Dict[str, SpaceCenter.CelestialBody]
        """A dictionary of all celestial bodies (planets, moons, etc.) in the game,
        keyed by the name of the body."""
        ...

    @property
    def camera(self):
        # type: () -> SpaceCenter.Camera
        """An object that can be used to control the camera."""
        ...

    @property
    def contract_manager(self):
        # type: () -> SpaceCenter.ContractManager
        """The contract manager."""
        ...

    @property
    def far_available(self):
        # type: () -> bool
        """Whether Ferram Aerospace Research is installed."""
        ...

    @property
    def funds(self):
        # type: () -> float
        """The current amount of funds."""
        ...

    @property
    def g(self):
        # type: () -> float
        """The value of the 
        gravitational constant G in N(m/kg)^2."""
        ...

    @property
    def game_mode(self):
        # type: () -> SpaceCenter.GameMode
        """The current mode the game is in."""
        ...

    @property
    def launch_sites(self):
        # type: () -> List[SpaceCenter.LaunchSite]
        """A list of available launch sites."""
        ...

    @property
    def map_filter(self):
        # type: () -> SpaceCenter.MapFilterType
        """The visible objects in map mode."""
        ...

    @map_filter.setter
    def map_filter(self, value):
        # type: (SpaceCenter.MapFilterType) -> None
        ...

    @property
    def maximum_rails_warp_factor(self):
        # type: () -> int
        """The current maximum regular "on-rails" warp factor that can be set.
        A value between 0 and 7 inclusive. See
        the KSP wiki
        for details."""
        ...

    @property
    def navball(self):
        # type: () -> bool
        """Whether the navball is visible."""
        ...

    @navball.setter
    def navball(self, value):
        # type: (bool) -> None
        ...

    @property
    def physics_warp_factor(self):
        # type: () -> int
        """The physical time warp rate. A value between 0 and 3 inclusive. 0 means
        no time warp. Returns 0 if regular "on-rails" time warp is active."""
        ...

    @physics_warp_factor.setter
    def physics_warp_factor(self, value):
        # type: (int) -> None
        ...

    @property
    def rails_warp_factor(self):
        # type: () -> int
        """The time warp rate, using regular "on-rails" time warp. A value between
        0 and 7 inclusive. 0 means no time warp. Returns 0 if physical time warp
        is active.

        If requested time warp factor cannot be set, it will be set to the next
        lowest possible value. For example, if the vessel is too close to a
        planet. See 
        the KSP wiki for details."""
        ...

    @rails_warp_factor.setter
    def rails_warp_factor(self, value):
        # type: (int) -> None
        ...

    @property
    def reputation(self):
        # type: () -> float
        """The current amount of reputation."""
        ...

    @property
    def science(self):
        # type: () -> float
        """The current amount of science."""
        ...

    @property
    def target_body(self):
        # type: () -> Optional[SpaceCenter.CelestialBody]
        """The currently targeted celestial body."""
        ...

    @target_body.setter
    def target_body(self, value):
        # type: (SpaceCenter.CelestialBody) -> None
        ...

    @property
    def target_docking_port(self):
        # type: () -> Optional[SpaceCenter.DockingPort]
        """The currently targeted docking port."""
        ...

    @target_docking_port.setter
    def target_docking_port(self, value):
        # type: (SpaceCenter.DockingPort) -> None
        ...

    @property
    def target_vessel(self):
        # type: () -> Optional[SpaceCenter.Vessel]
        """The currently targeted vessel."""
        ...

    @target_vessel.setter
    def target_vessel(self, value):
        # type: (SpaceCenter.Vessel) -> None
        ...

    @property
    def ui_visible(self):
        # type: () -> bool
        """Whether the UI is visible."""
        ...

    @ui_visible.setter
    def ui_visible(self, value):
        # type: (bool) -> None
        ...

    @property
    def ut(self):
        # type: () -> float
        """The current universal time in seconds."""
        ...

    @property
    def vessels(self):
        # type: () -> List[SpaceCenter.Vessel]
        """A list of all the vessels in the game."""
        ...

    @property
    def warp_factor(self):
        # type: () -> float
        """The current warp factor. This is the index of the rate at which time
        is passing for either regular "on-rails" or physical time warp. Returns 0
        if time warp is not active. When in on-rails time warp, this is equal to
        SpaceCenter#railsWarpFactor, and in physics time warp, this is equal to
        SpaceCenter#physicsWarpFactor."""
        ...

    @property
    def warp_mode(self):
        # type: () -> SpaceCenter.WarpMode
        """The current time warp mode. Returns SpaceCenter.WarpMode#none if time
        warp is not active, SpaceCenter.WarpMode#rails if regular "on-rails" time warp
        is active, or SpaceCenter.WarpMode#physics if physical time warp is active."""
        ...

    @property
    def warp_rate(self):
        # type: () -> float
        """The current warp rate. This is the rate at which time is passing for
        either on-rails or physical time warp. For example, a value of 10 means
        time is passing 10x faster than normal. Returns 1 if time warp is not
        active."""
        ...

    @property
    def waypoint_manager(self):
        # type: () -> SpaceCenter.WaypointManager
        """The waypoint manager."""
        ...

    def can_rails_warp_at(self, factor=1):
        # type: (int) -> bool
        """Returns {@code true} if regular "on-rails" time warp can be used, at the specified warp
        factor. The maximum time warp rate is limited by various things,
        including how close the active vessel is to a planet. See
        the KSP wiki
        for details.
        :factor The warp factor to check."""
        ...

    def can_revert_to_launch(self):
        # type: () -> bool
        """Whether the current flight can be reverted to launch."""
        ...

    def clear_target(self):
        # type: () -> None
        """Clears the current target."""
        ...

    def create_kerbal(self, name, job, male):
        # type: (str, str, bool) -> None
        """Creates a Kerbal.
        :name 
        :job 
        :male """
        ...

    def get_kerbal(self, name):
        # type: (str) -> Optional[SpaceCenter.CrewMember]
        """Find a Kerbal by name.
        :name 
        @return {@code null} if no Kerbal with the given name exists."""
        ...

    def launch_vessel(self, craft_directory, name, launch_site, recover=True, crew=None, flag_url=''):
        # type: (str, str, str, bool, List[str], str) -> None
        """Launch a vessel.
        :craft_directory Name of the directory in the current saves
        "Ships" directory, that contains the craft file.
        For example {@code "VAB"} or {@code "SPH"}.
        :name Name of the vessel to launch. This is the name of the ".craft" file
        in the save directory, without the ".craft" file extension.
        :launch_site Name of the launch site. For example {@code "LaunchPad"} or
        {@code "Runway"}.
        :recover If true and there is a vessel on the launch site,
        recover it before launching.
        :crew If not {@code null}, a list of names of Kerbals to place in the craft. Otherwise the crew will use default assignments.
        :flag_url If not {@code null}, the asset URL of the mission flag to use for the launch.

        Throws an exception if any of the games pre-flight checks fail."""
        ...

    def launch_vessel_from_sph(self, name, recover=True):
        # type: (str, bool) -> None
        """Launch a new vessel from the SPH onto the runway.
        :name Name of the vessel to launch.
        :recover If true and there is a vessel on the runway,
        recover it before launching.

        This is equivalent to calling SpaceCenter#launchVessel with the craft directory
        set to "SPH" and the launch site set to "Runway".
        Throws an exception if any of the games pre-flight checks fail."""
        ...

    def launch_vessel_from_vab(self, name, recover=True):
        # type: (str, bool) -> None
        """Launch a new vessel from the VAB onto the launchpad.
        :name Name of the vessel to launch.
        :recover If true and there is a vessel on the launch pad,
        recover it before launching.

        This is equivalent to calling SpaceCenter#launchVessel with the craft directory
        set to "VAB" and the launch site set to "LaunchPad".
        Throws an exception if any of the games pre-flight checks fail."""
        ...

    def launchable_vessels(self, craft_directory):
        # type: (str) -> List[str]
        """Returns a list of vessels from the given craft_directory
        that can be launched.
        :craft_directory Name of the directory in the current saves
        "Ships" directory. For example {@code "VAB"} or {@code "SPH"}."""
        ...

    def load(self, name):
        # type: (str) -> None
        """Load the game with the given name.
        This will create a load a save file called {@code name.sfs} from the folder of the
        current save game."""
        ...

    def load_space_center(self):
        # type: () -> None
        """Switch to the spacecenter view."""
        ...

    def quickload(self):
        # type: () -> None
        """Load a quicksave.

        This is the same as calling SpaceCenter#load with the name "quicksave"."""
        ...

    def quicksave(self):
        # type: () -> None
        """Save a quicksave.

        This is the same as calling SpaceCenter#save with the name "quicksave"."""
        ...

    def raycast_distance(self, position, direction, reference_frame):
        # type: (Tuple[float,float,float], Tuple[float,float,float], SpaceCenter.ReferenceFrame) -> float
        """Cast a ray from a given position in a given direction, and return the distance to the hit point.
        If no hit occurs, returns infinity.
        :position Position, as a vector, of the origin of the ray.
        :direction Direction of the ray, as a unit vector.
        :reference_frame The reference frame that the position and direction are in.
        @return The distance to the hit, in meters, or infinity if there was no hit."""
        ...

    def raycast_part(self, position, direction, reference_frame):
        # type: (Tuple[float,float,float], Tuple[float,float,float], SpaceCenter.ReferenceFrame) -> Optional[SpaceCenter.Part]
        """Cast a ray from a given position in a given direction, and return the part that it hits.
        If no hit occurs, returns {@code null}.
        :position Position, as a vector, of the origin of the ray.
        :direction Direction of the ray, as a unit vector.
        :reference_frame The reference frame that the position and direction are in.
        @return The part that was hit or {@code null} if there was no hit."""
        ...

    def revert_to_launch(self):
        # type: () -> None
        """Revert the current flight to launch."""
        ...

    def save(self, name):
        # type: (str) -> None
        """Save the game with a given name.
        This will create a save file called {@code name.sfs} in the folder of the
        current save game."""
        ...

    def screenshot(self, file_path, scale=1):
        # type: (str, int) -> None
        """Saves a screenshot.
        :file_path The path of the file to save.
        :scale Resolution scaling factor"""
        ...

    def transfer_crew(self, crew_member, target_part):
        # type: (SpaceCenter.CrewMember, SpaceCenter.Part) -> None
        """Tranfsers a crew member to a different part.
        :crew_member The crew member to transfer.
        :target_part The part to move them to."""
        ...

    def transform_direction(self, direction, from__, to):
        # type: (Tuple[float,float,float], SpaceCenter.ReferenceFrame, SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
        """Converts a direction from one reference frame to another.
        :direction Direction, as a vector, in reference frame
        from.
        :from The reference frame that the direction is in.
        :to The reference frame to covert the direction to.
        @return The corresponding direction, as a vector, in reference frame
        to."""
        ...

    def transform_position(self, position, from__, to):
        # type: (Tuple[float,float,float], SpaceCenter.ReferenceFrame, SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
        """Converts a position from one reference frame to another.
        :position Position, as a vector, in reference frame
        from.
        :from The reference frame that the position is in.
        :to The reference frame to covert the position to.
        @return The corresponding position, as a vector, in reference frame
        to."""
        ...

    def transform_rotation(self, rotation, from__, to):
        # type: (Tuple[float,float,float,float], SpaceCenter.ReferenceFrame, SpaceCenter.ReferenceFrame) -> Tuple[float,float,float,float]
        """Converts a rotation from one reference frame to another.
        :rotation Rotation, as a quaternion of the form (x, y, z, w),
        in reference frame from.
        :from The reference frame that the rotation is in.
        :to The reference frame to covert the rotation to.
        @return The corresponding rotation, as a quaternion of the form
        (x, y, z, w), in reference frame to."""
        ...

    def transform_velocity(self, position, velocity, from__, to):
        # type: (Tuple[float,float,float], Tuple[float,float,float], SpaceCenter.ReferenceFrame, SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
        """Converts a velocity (acting at the specified position) from one reference frame
        to another. The position is required to take the relative angular velocity of the
        reference frames into account.
        :position Position, as a vector, in reference frame
        from.
        :velocity Velocity, as a vector that points in the direction of travel and
        whose magnitude is the speed in meters per second, in reference frame
        from.
        :from The reference frame that the position and velocity are in.
        :to The reference frame to covert the velocity to.
        @return The corresponding velocity, as a vector, in reference frame
        to."""
        ...

    def warp_to(self, ut, max_rails_rate=100000.0, max_physics_rate=2.0):
        # type: (float, float, float) -> None
        """Uses time acceleration to warp forward to a time in the future, specified
        by universal time ut. This call blocks until the desired
        time is reached. Uses regular "on-rails" or physical time warp as appropriate.
        For example, physical time warp is used when the active vessel is traveling
        through an atmosphere. When using regular "on-rails" time warp, the warp
        rate is limited by max_rails_rate, and when using physical
        time warp, the warp rate is limited by max_physics_rate.
        :ut The universal time to warp to, in seconds.
        :max_rails_rate The maximum warp rate in regular "on-rails" time warp.
        :max_physics_rate The maximum warp rate in physical time warp.
        @return When the time warp is complete."""
        ...

    class Alarm:
        @property
        def description(self):
            # type: () -> str
            """Description of the alarm."""
            ...

        @property
        def event_offset(self):
            # type: () -> float
            """Seconds between the alarm going off and the event it references."""
            ...

        @property
        def id(self):
            # type: () -> Optional[int]
            """Unique identifier of the alarm.
            KSP destroys and recreates an alarm when it is edited.
            This id will remain constant between the old and new alarms."""
            ...

        @property
        def time(self):
            # type: () -> float
            """Time the alarm will trigger."""
            ...

        @property
        def time_until(self):
            # type: () -> float
            """Time until the alarm triggers."""
            ...

        @property
        def title(self):
            # type: () -> str
            """Title of the alarm"""
            ...

        @property
        def type(self):
            # type: () -> str
            """Type of alarm"""
            ...

        @property
        def vessel(self):
            # type: () -> Optional[SpaceCenter.Vessel]
            """Vessel the alarm references. {@code null} if it does not reference a vesssel."""
            ...

    class AlarmManager:
        @property
        def alarms(self):
            # type: () -> List[SpaceCenter.Alarm]
            """A list of all alarms."""
            ...

        @staticmethod
        def add_alarm(, time, title='Raw Alarm', description=''):
            # type: (float, str, str) -> SpaceCenter.Alarm
            """Create an alarm.
            :time Number of seconds from now that the alarm should trigger.
            :title Title for the alarm.
            :description Description for the alarm."""
            ...

        @staticmethod
        def add_apoapsis_alarm(, vessel, offset=60.0, title='APA Alarm', description=''):
            # type: (SpaceCenter.Vessel, float, str, str) -> SpaceCenter.Alarm
            """Create an alarm for the given vessel's next apoapsis.
            :vessel The vessel.
            :offset Time in seconds to offset the alarm by.
            :title Title for the alarm.
            :description Description for the alarm."""
            ...

        @staticmethod
        def add_maneuver_node_alarm(, vessel, node, offset=60.0, add_burn_time=True, title='Maneuver Alarm', description=''):
            # type: (SpaceCenter.Vessel, SpaceCenter.Node, float, bool, str, str) -> SpaceCenter.Alarm
            """Create an alarm for the given vessel and maneuver node.
            :vessel The vessel.
            :node The maneuver node.
            :offset Time in seconds to offset the alarm by.
            :add_burn_time Whether the node's burn time should be included in the alarm.
            :title Title for the alarm.
            :description Description for the alarm."""
            ...

        @staticmethod
        def add_periapsis_alarm(, vessel, offset=60.0, title='PEA Alarm', description=''):
            # type: (SpaceCenter.Vessel, float, str, str) -> SpaceCenter.Alarm
            """Create an alarm for the given vessel's next periapsis.
            :vessel The vessel.
            :offset Time in seconds to offset the alarm by.
            :title Title for the alarm.
            :description Description for the alarm."""
            ...

        @staticmethod
        def add_soi_alarm(, vessel, offset=60.0, title='SOI Change', description=''):
            # type: (SpaceCenter.Vessel, float, str, str) -> SpaceCenter.Alarm
            """Create an alarm for the given vessel's next sphere of influence change.
            :vessel The vessel.
            :offset Time in seconds to offset the alarm by.
            :title Title for the alarm.
            :description Description for the alarm."""
            ...

        @staticmethod
        def add_vessel_alarm(, time, vessel, title='Raw Alarm', description=''):
            # type: (float, SpaceCenter.Vessel, str, str) -> SpaceCenter.Alarm
            """Create an alarm linked to a vessel.
            :time Number of seconds from now that the alarm should trigger.
            :vessel Vessel to link the alarm to.
            :title Title for the alarm.
            :description Description for the alarm."""
            ...

    class Antenna:
        @property
        def allow_partial(self):
            # type: () -> bool
            """Whether partial data transmission is permitted."""
            ...

        @allow_partial.setter
        def allow_partial(self, value):
            # type: (bool) -> None
            ...

        @property
        def can_transmit(self):
            # type: () -> bool
            """Whether data can be transmitted by this antenna."""
            ...

        @property
        def combinable(self):
            # type: () -> bool
            """Whether the antenna can be combined with other antennae on the vessel
            to boost the power."""
            ...

        @property
        def combinable_exponent(self):
            # type: () -> float
            """Exponent used to calculate the combined power of multiple antennae on a vessel."""
            ...

        @property
        def deployable(self):
            # type: () -> bool
            """Whether the antenna is deployable."""
            ...

        @property
        def deployed(self):
            # type: () -> bool
            """Whether the antenna is deployed.

            Fixed antennas are always deployed.
            Returns an error if you try to deploy a fixed antenna."""
            ...

        @deployed.setter
        def deployed(self, value):
            # type: (bool) -> None
            ...

        @property
        def packet_interval(self):
            # type: () -> float
            """Interval between sending packets in seconds."""
            ...

        @property
        def packet_resource_cost(self):
            # type: () -> float
            """Units of electric charge consumed per packet sent."""
            ...

        @property
        def packet_size(self):
            # type: () -> float
            """Amount of data sent per packet in Mits."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this antenna."""
            ...

        @property
        def power(self):
            # type: () -> float
            """The power of the antenna."""
            ...

        @property
        def state(self):
            # type: () -> SpaceCenter.AntennaState
            """The current state of the antenna."""
            ...

        def cancel(self):
            # type: () -> None
            """Cancel current transmission of data."""
            ...

        def transmit(self):
            # type: () -> None
            """Transmit data."""
            ...

    class AutoPilot:
        @property
        def attenuation_angle(self):
            # type: () -> Tuple[float,float,float]
            """The angle at which the autopilot considers the vessel to be pointing
            close to the target.
            This determines the midpoint of the target velocity attenuation function.
            A vector of three angles, in degrees, one for each of the pitch, roll and yaw axes.
            Defaults to 1° for each axis."""
            ...

        @attenuation_angle.setter
        def attenuation_angle(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def auto_tune(self):
            # type: () -> bool
            """Whether the rotation rate controllers PID parameters should be automatically tuned
            using the vessels moment of inertia and available torque. Defaults to {@code true}.
            See SpaceCenter.AutoPilot#timeToPeak and SpaceCenter.AutoPilot#overshoot."""
            ...

        @auto_tune.setter
        def auto_tune(self, value):
            # type: (bool) -> None
            ...

        @property
        def deceleration_time(self):
            # type: () -> Tuple[float,float,float]
            """The time the vessel should take to come to a stop pointing in the target direction.
            This determines the angular acceleration used to decelerate the vessel.
            A vector of three times, in seconds, one for each of the pitch, roll and yaw axes.
            Defaults to 5 seconds for each axis."""
            ...

        @deceleration_time.setter
        def deceleration_time(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def error(self):
            # type: () -> float
            """The error, in degrees, between the direction the ship has been asked
            to point in and the direction it is pointing in. Throws an exception if the auto-pilot
            has not been engaged and SAS is not enabled or is in stability assist mode."""
            ...

        @property
        def heading_error(self):
            # type: () -> float
            """The error, in degrees, between the vessels current and target heading.
            Throws an exception if the auto-pilot has not been engaged."""
            ...

        @property
        def overshoot(self):
            # type: () -> Tuple[float,float,float]
            """The target overshoot percentage used to autotune the PID controllers.
            A vector of three values, between 0 and 1, for each of the pitch, roll and yaw axes.
            Defaults to 0.01 for each axis."""
            ...

        @overshoot.setter
        def overshoot(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def pitch_error(self):
            # type: () -> float
            """The error, in degrees, between the vessels current and target pitch.
            Throws an exception if the auto-pilot has not been engaged."""
            ...

        @property
        def pitch_pid_gains(self):
            # type: () -> Tuple[float,float,float]
            """Gains for the pitch PID controller.

            When SpaceCenter.AutoPilot#autoTune is true, these values are updated automatically,
            which will overwrite any manual changes."""
            ...

        @pitch_pid_gains.setter
        def pitch_pid_gains(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """The reference frame for the target direction (SpaceCenter.AutoPilot#targetDirection).

            An error will be thrown if this property is set to a reference frame that rotates with
            the vessel being controlled, as it is impossible to rotate the vessel in such a
            reference frame."""
            ...

        @reference_frame.setter
        def reference_frame(self, value):
            # type: (SpaceCenter.ReferenceFrame) -> None
            ...

        @property
        def roll_error(self):
            # type: () -> float
            """The error, in degrees, between the vessels current and target roll.
            Throws an exception if the auto-pilot has not been engaged or no target roll is set."""
            ...

        @property
        def roll_pid_gains(self):
            # type: () -> Tuple[float,float,float]
            """Gains for the roll PID controller.

            When SpaceCenter.AutoPilot#autoTune is true, these values are updated automatically,
            which will overwrite any manual changes."""
            ...

        @roll_pid_gains.setter
        def roll_pid_gains(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def roll_threshold(self):
            # type: () -> float
            """The threshold at which the autopilot will try to match the target roll angle, if any.
            Defaults to 5 degrees."""
            ...

        @roll_threshold.setter
        def roll_threshold(self, value):
            # type: (float) -> None
            ...

        @property
        def sas(self):
            # type: () -> bool
            """The state of SAS.

            Equivalent to SpaceCenter.Control#sAS"""
            ...

        @sas.setter
        def sas(self, value):
            # type: (bool) -> None
            ...

        @property
        def sas_mode(self):
            # type: () -> SpaceCenter.SASMode
            """The current SpaceCenter.SASMode.
            These modes are equivalent to the mode buttons to the left of the navball that appear
            when SAS is enabled.

            Equivalent to SpaceCenter.Control#sASMode"""
            ...

        @sas_mode.setter
        def sas_mode(self, value):
            # type: (SpaceCenter.SASMode) -> None
            ...

        @property
        def stopping_time(self):
            # type: () -> Tuple[float,float,float]
            """The maximum amount of time that the vessel should need to come to a complete stop.
            This determines the maximum angular velocity of the vessel.
            A vector of three stopping times, in seconds, one for each of the pitch, roll
            and yaw axes. Defaults to 0.5 seconds for each axis."""
            ...

        @stopping_time.setter
        def stopping_time(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def target_direction(self):
            # type: () -> Tuple[float,float,float]
            """Direction vector corresponding to the target pitch and heading.
            This is in the reference frame specified by SpaceCenter.ReferenceFrame."""
            ...

        @target_direction.setter
        def target_direction(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def target_heading(self):
            # type: () -> float
            """The target heading, in degrees, between 0° and 360°."""
            ...

        @target_heading.setter
        def target_heading(self, value):
            # type: (float) -> None
            ...

        @property
        def target_pitch(self):
            # type: () -> float
            """The target pitch, in degrees, between -90° and +90°."""
            ...

        @target_pitch.setter
        def target_pitch(self, value):
            # type: (float) -> None
            ...

        @property
        def target_roll(self):
            # type: () -> float
            """The target roll, in degrees. {@code NaN} if no target roll is set."""
            ...

        @target_roll.setter
        def target_roll(self, value):
            # type: (float) -> None
            ...

        @property
        def time_to_peak(self):
            # type: () -> Tuple[float,float,float]
            """The target time to peak used to autotune the PID controllers.
            A vector of three times, in seconds, for each of the pitch, roll and yaw axes.
            Defaults to 3 seconds for each axis."""
            ...

        @time_to_peak.setter
        def time_to_peak(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def yaw_pid_gains(self):
            # type: () -> Tuple[float,float,float]
            """Gains for the yaw PID controller.

            When SpaceCenter.AutoPilot#autoTune is true, these values are updated automatically,
            which will overwrite any manual changes."""
            ...

        @yaw_pid_gains.setter
        def yaw_pid_gains(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        def disengage(self):
            # type: () -> None
            """Disengage the auto-pilot."""
            ...

        def engage(self):
            # type: () -> None
            """Engage the auto-pilot."""
            ...

        def target_pitch_and_heading(self, pitch, heading):
            # type: (float, float) -> None
            """Set target pitch and heading angles.
            :pitch Target pitch angle, in degrees between -90° and +90°.
            :heading Target heading angle, in degrees between 0° and 360°."""
            ...

        def wait(self):
            # type: () -> None
            """Blocks until the vessel is pointing in the target direction and has
            the target roll (if set). Throws an exception if the auto-pilot has not been engaged."""
            ...

    class Camera:
        @property
        def default_distance(self):
            # type: () -> float
            """Default distance from the camera to the subject, in meters."""
            ...

        @property
        def distance(self):
            # type: () -> float
            """The distance from the camera to the subject, in meters.
            A value between SpaceCenter.Camera#minDistance and SpaceCenter.Camera#maxDistance."""
            ...

        @distance.setter
        def distance(self, value):
            # type: (float) -> None
            ...

        @property
        def focussed_body(self):
            # type: () -> Optional[SpaceCenter.CelestialBody]
            """In map mode, the celestial body that the camera is focussed on.
            Returns {@code null} if the camera is not focussed on a celestial body.
            Returns an error is the camera is not in map mode."""
            ...

        @focussed_body.setter
        def focussed_body(self, value):
            # type: (SpaceCenter.CelestialBody) -> None
            ...

        @property
        def focussed_node(self):
            # type: () -> Optional[SpaceCenter.Node]
            """In map mode, the maneuver node that the camera is focussed on.
            Returns {@code null} if the camera is not focussed on a maneuver node.
            Returns an error is the camera is not in map mode."""
            ...

        @focussed_node.setter
        def focussed_node(self, value):
            # type: (SpaceCenter.Node) -> None
            ...

        @property
        def focussed_vessel(self):
            # type: () -> Optional[SpaceCenter.Vessel]
            """In map mode, the vessel that the camera is focussed on.
            Returns {@code null} if the camera is not focussed on a vessel.
            Returns an error is the camera is not in map mode."""
            ...

        @focussed_vessel.setter
        def focussed_vessel(self, value):
            # type: (SpaceCenter.Vessel) -> None
            ...

        @property
        def heading(self):
            # type: () -> float
            """The heading of the camera, in degrees."""
            ...

        @heading.setter
        def heading(self, value):
            # type: (float) -> None
            ...

        @property
        def max_distance(self):
            # type: () -> float
            """Maximum distance from the camera to the subject, in meters."""
            ...

        @property
        def max_pitch(self):
            # type: () -> float
            """The maximum pitch of the camera."""
            ...

        @property
        def min_distance(self):
            # type: () -> float
            """Minimum distance from the camera to the subject, in meters."""
            ...

        @property
        def min_pitch(self):
            # type: () -> float
            """The minimum pitch of the camera."""
            ...

        @property
        def mode(self):
            # type: () -> SpaceCenter.CameraMode
            """The current mode of the camera."""
            ...

        @mode.setter
        def mode(self, value):
            # type: (SpaceCenter.CameraMode) -> None
            ...

        @property
        def pitch(self):
            # type: () -> float
            """The pitch of the camera, in degrees.
            A value between SpaceCenter.Camera#minPitch and SpaceCenter.Camera#maxPitch"""
            ...

        @pitch.setter
        def pitch(self, value):
            # type: (float) -> None
            ...

    class CargoBay:
        @property
        def open(self):
            # type: () -> bool
            """Whether the cargo bay is open."""
            ...

        @open.setter
        def open(self, value):
            # type: (bool) -> None
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this cargo bay."""
            ...

        @property
        def state(self):
            # type: () -> SpaceCenter.CargoBayState
            """The state of the cargo bay."""
            ...

    class CelestialBody:
        @property
        def atmosphere_depth(self):
            # type: () -> float
            """The depth of the atmosphere, in meters."""
            ...

        @property
        def biomes(self):
            # type: () -> Set[str]
            """The biomes present on this body."""
            ...

        @property
        def equatorial_radius(self):
            # type: () -> float
            """The equatorial radius of the body, in meters."""
            ...

        @property
        def flying_high_altitude_threshold(self):
            # type: () -> float
            """The altitude, in meters, above which a vessel is considered to be
            flying "high" when doing science."""
            ...

        @property
        def gravitational_parameter(self):
            # type: () -> float
            """The standard
            gravitational parameter of the body in m^3s^{-2}."""
            ...

        @property
        def has_atmosphere(self):
            # type: () -> bool
            """{@code true} if the body has an atmosphere."""
            ...

        @property
        def has_atmospheric_oxygen(self):
            # type: () -> bool
            """{@code true} if there is oxygen in the atmosphere, required for air-breathing engines."""
            ...

        @property
        def has_solid_surface(self):
            # type: () -> bool
            """Whether or not the body has a solid surface."""
            ...

        @property
        def initial_rotation(self):
            # type: () -> float
            """The initial rotation angle of the body (at UT 0), in radians.
            A value between 0 and 2\pi"""
            ...

        @property
        def is_star(self):
            # type: () -> bool
            """Whether or not the body is a star."""
            ...

        @property
        def mass(self):
            # type: () -> float
            """The mass of the body, in kilograms."""
            ...

        @property
        def name(self):
            # type: () -> str
            """The name of the body."""
            ...

        @property
        def non_rotating_reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """The reference frame that is fixed relative to this celestial body, and
            orientated in a fixed direction (it does not rotate with the body).
            <p><ul>
            <li>The origin is at the center of the body.
            <li>The axes do not rotate.
            <li>The x-axis points in an arbitrary direction through the
              equator.
            <li>The y-axis points from the center of the body towards
              the north pole.
            <li>The z-axis points in an arbitrary direction through the
              equator.
            </ul></p>"""
            ...

        @property
        def orbit(self):
            # type: () -> Optional[SpaceCenter.Orbit]
            """The orbit of the body."""
            ...

        @property
        def orbital_reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """The reference frame that is fixed relative to this celestial body, but
            orientated with the body's orbital prograde/normal/radial directions.
            <p><ul>
            <li>The origin is at the center of the body.
            <li>The axes rotate with the orbital prograde/normal/radial
              directions.
            <li>The x-axis points in the orbital anti-radial direction.
            <li>The y-axis points in the orbital prograde direction.
            <li>The z-axis points in the orbital normal direction.
            </ul></p>"""
            ...

        @property
        def reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """The reference frame that is fixed relative to the celestial body.
            <p><ul>
            <li>The origin is at the center of the body.
            <li>The axes rotate with the body.
            <li>The x-axis points from the center of the body
              towards the intersection of the prime meridian and equator (the
              position at 0° longitude, 0° latitude).
            <li>The y-axis points from the center of the body
              towards the north pole.
            <li>The z-axis points from the center of the body
              towards the equator at 90°E longitude.
            </ul></p>"""
            ...

        @property
        def rotation_angle(self):
            # type: () -> float
            """The current rotation angle of the body, in radians.
            A value between 0 and 2\pi"""
            ...

        @property
        def rotational_period(self):
            # type: () -> float
            """The sidereal rotational period of the body, in seconds."""
            ...

        @property
        def rotational_speed(self):
            # type: () -> float
            """The rotational speed of the body, in radians per second."""
            ...

        @property
        def satellites(self):
            # type: () -> List[SpaceCenter.CelestialBody]
            """A list of celestial bodies that are in orbit around this celestial body."""
            ...

        @property
        def space_high_altitude_threshold(self):
            # type: () -> float
            """The altitude, in meters, above which a vessel is considered to be
            in "high" space when doing science."""
            ...

        @property
        def sphere_of_influence(self):
            # type: () -> float
            """The radius of the sphere of influence of the body, in meters."""
            ...

        @property
        def surface_gravity(self):
            # type: () -> float
            """The acceleration due to gravity at sea level (mean altitude) on the body,
            in m/s^2."""
            ...

        def altitude_at_position(self, position, reference_frame):
            # type: (Tuple[float,float,float], SpaceCenter.ReferenceFrame) -> float
            """The altitude, in meters, of the given position in the given reference frame.
            :position Position as a vector.
            :reference_frame Reference frame for the position vector."""
            ...

        def angular_velocity(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The angular velocity of the body in the specified reference frame.
            @return The angular velocity as a vector. The magnitude of the vector is the rotational
            speed of the body, in radians per second. The direction of the vector indicates the axis
            of rotation, using the right-hand rule.
            :reference_frame The reference frame the returned
            angular velocity is in."""
            ...

        def atmospheric_density_at_position(self, position, reference_frame):
            # type: (Tuple[float,float,float], SpaceCenter.ReferenceFrame) -> float
            """The atmospheric density at the given position, in kg/m^3,
            in the given reference frame.
            :position The position vector at which to measure the density.
            :reference_frame Reference frame that the position vector is in."""
            ...

        def bedrock_height(self, latitude, longitude):
            # type: (float, float) -> float
            """The height of the surface relative to mean sea level, in meters,
            at the given position. When over water, this is the height
            of the sea-bed and is therefore  negative value.
            :latitude Latitude in degrees.
            :longitude Longitude in degrees."""
            ...

        def bedrock_position(self, latitude, longitude, reference_frame):
            # type: (float, float, SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The position of the surface at the given latitude and longitude, in the given
            reference frame. When over water, this is the position at the bottom of the sea-bed.
            @return Position as a vector.
            :latitude Latitude in degrees.
            :longitude Longitude in degrees.
            :reference_frame Reference frame for the returned position vector."""
            ...

        def biome_at(self, latitude, longitude):
            # type: (float, float) -> str
            """The biome at the given latitude and longitude, in degrees."""
            ...

        def density_at(self, altitude):
            # type: (float) -> float
            """Gets the air density, in kg/m^3, for the specified
            altitude above sea level, in meters.

            This is an approximation, because actual calculations, taking sun exposure into account
            to compute air temperature, require us to know the exact point on the body where the
            density is to be computed (knowing the altitude is not enough).
            However, the difference is small for high altitudes, so it makes very little difference
            for trajectory prediction."""
            ...

        def direction(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The direction in which the north pole of the celestial body is pointing,
            in the specified reference frame.
            @return The direction as a unit vector.
            :reference_frame The reference frame that the returned
            direction is in."""
            ...

        def latitude_at_position(self, position, reference_frame):
            # type: (Tuple[float,float,float], SpaceCenter.ReferenceFrame) -> float
            """The latitude of the given position, in the given reference frame.
            :position Position as a vector.
            :reference_frame Reference frame for the position vector."""
            ...

        def longitude_at_position(self, position, reference_frame):
            # type: (Tuple[float,float,float], SpaceCenter.ReferenceFrame) -> float
            """The longitude of the given position, in the given reference frame.
            :position Position as a vector.
            :reference_frame Reference frame for the position vector."""
            ...

        def msl_position(self, latitude, longitude, reference_frame):
            # type: (float, float, SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The position at mean sea level at the given latitude and longitude,
            in the given reference frame.
            @return Position as a vector.
            :latitude Latitude in degrees.
            :longitude Longitude in degrees.
            :reference_frame Reference frame for the returned position vector."""
            ...

        def position(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The position of the center of the body, in the specified reference frame.
            @return The position as a vector.
            :reference_frame The reference frame that the returned
            position vector is in."""
            ...

        def position_at_altitude(self, latitude, longitude, altitude, reference_frame):
            # type: (float, float, float, SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The position at the given latitude, longitude and altitude, in the given reference frame.
            @return Position as a vector.
            :latitude Latitude in degrees.
            :longitude Longitude in degrees.
            :altitude Altitude in meters above sea level.
            :reference_frame Reference frame for the returned position vector."""
            ...

        def pressure_at(self, altitude):
            # type: (float) -> float
            """Gets the air pressure, in Pascals, for the specified
            altitude above sea level, in meters."""
            ...

        def rotation(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float,float]
            """The rotation of the body, in the specified reference frame.
            @return The rotation as a quaternion of the form (x, y, z, w).
            :reference_frame The reference frame that the returned
            rotation is in."""
            ...

        def surface_height(self, latitude, longitude):
            # type: (float, float) -> float
            """The height of the surface relative to mean sea level, in meters,
            at the given position. When over water this is equal to 0.
            :latitude Latitude in degrees.
            :longitude Longitude in degrees."""
            ...

        def surface_position(self, latitude, longitude, reference_frame):
            # type: (float, float, SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The position of the surface at the given latitude and longitude, in the given
            reference frame. When over water, this is the position of the surface of the water.
            @return Position as a vector.
            :latitude Latitude in degrees.
            :longitude Longitude in degrees.
            :reference_frame Reference frame for the returned position vector."""
            ...

        def temperature_at(self, position, reference_frame):
            # type: (Tuple[float,float,float], SpaceCenter.ReferenceFrame) -> float
            """The temperature on the body at the given position, in the given reference frame.
            :position Position as a vector.
            :reference_frame The reference frame that the position is in.

            This calculation is performed using the bodies current position, which means that
            the value could be wrong if you want to know the temperature in the far future."""
            ...

        def velocity(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The linear velocity of the body, in the specified reference frame.
            @return The velocity as a vector. The vector points in the direction of travel,
            and its magnitude is the speed of the body in meters per second.
            :reference_frame The reference frame that the returned
            velocity vector is in."""
            ...

    class CommLink:
        @property
        def end(self):
            # type: () -> SpaceCenter.CommNode
            """Start point of the link."""
            ...

        @property
        def signal_strength(self):
            # type: () -> float
            """Signal strength of the link."""
            ...

        @property
        def start(self):
            # type: () -> SpaceCenter.CommNode
            """Start point of the link."""
            ...

        @property
        def type(self):
            # type: () -> SpaceCenter.CommLinkType
            """The type of link."""
            ...

    class CommNode:
        @property
        def is_control_point(self):
            # type: () -> bool
            """Whether the communication node is a control point, for example a manned vessel."""
            ...

        @property
        def is_home(self):
            # type: () -> bool
            """Whether the communication node is on Kerbin."""
            ...

        @property
        def is_vessel(self):
            # type: () -> bool
            """Whether the communication node is a vessel."""
            ...

        @property
        def name(self):
            # type: () -> str
            """Name of the communication node."""
            ...

        @property
        def vessel(self):
            # type: () -> SpaceCenter.Vessel
            """The vessel for this communication node."""
            ...

    class Comms:
        @property
        def can_communicate(self):
            # type: () -> bool
            """Whether the vessel can communicate with KSC."""
            ...

        @property
        def can_transmit_science(self):
            # type: () -> bool
            """Whether the vessel can transmit science data to KSC."""
            ...

        @property
        def control_path(self):
            # type: () -> List[SpaceCenter.CommLink]
            """The communication path used to control the vessel."""
            ...

        @property
        def power(self):
            # type: () -> float
            """The combined power of all active antennae on the vessel."""
            ...

        @property
        def signal_delay(self):
            # type: () -> float
            """Signal delay to KSC in seconds."""
            ...

        @property
        def signal_strength(self):
            # type: () -> float
            """Signal strength to KSC."""
            ...

    class Contract:
        @property
        def active(self):
            # type: () -> bool
            """Whether the contract is active."""
            ...

        @property
        def can_be_canceled(self):
            # type: () -> bool
            """Whether the contract can be canceled."""
            ...

        @property
        def can_be_declined(self):
            # type: () -> bool
            """Whether the contract can be declined."""
            ...

        @property
        def can_be_failed(self):
            # type: () -> bool
            """Whether the contract can be failed."""
            ...

        @property
        def description(self):
            # type: () -> str
            """Description of the contract."""
            ...

        @property
        def failed(self):
            # type: () -> bool
            """Whether the contract has been failed."""
            ...

        @property
        def funds_advance(self):
            # type: () -> float
            """Funds received when accepting the contract."""
            ...

        @property
        def funds_completion(self):
            # type: () -> float
            """Funds received on completion of the contract."""
            ...

        @property
        def funds_failure(self):
            # type: () -> float
            """Funds lost if the contract is failed."""
            ...

        @property
        def keywords(self):
            # type: () -> List[str]
            """Keywords for the contract."""
            ...

        @property
        def notes(self):
            # type: () -> str
            """Notes for the contract."""
            ...

        @property
        def parameters(self):
            # type: () -> List[SpaceCenter.ContractParameter]
            """Parameters for the contract."""
            ...

        @property
        def read(self):
            # type: () -> bool
            """Whether the contract has been read."""
            ...

        @property
        def reputation_completion(self):
            # type: () -> float
            """Reputation gained on completion of the contract."""
            ...

        @property
        def reputation_failure(self):
            # type: () -> float
            """Reputation lost if the contract is failed."""
            ...

        @property
        def science_completion(self):
            # type: () -> float
            """Science gained on completion of the contract."""
            ...

        @property
        def seen(self):
            # type: () -> bool
            """Whether the contract has been seen."""
            ...

        @property
        def state(self):
            # type: () -> SpaceCenter.ContractState
            """State of the contract."""
            ...

        @property
        def synopsis(self):
            # type: () -> str
            """Synopsis for the contract."""
            ...

        @property
        def title(self):
            # type: () -> str
            """Title of the contract."""
            ...

        @property
        def type(self):
            # type: () -> str
            """Type of the contract."""
            ...

        def accept(self):
            # type: () -> None
            """Accept an offered contract."""
            ...

        def cancel(self):
            # type: () -> None
            """Cancel an active contract."""
            ...

        def decline(self):
            # type: () -> None
            """Decline an offered contract."""
            ...

    class ContractManager:
        @property
        def active_contracts(self):
            # type: () -> List[SpaceCenter.Contract]
            """A list of all active contracts."""
            ...

        @property
        def all_contracts(self):
            # type: () -> List[SpaceCenter.Contract]
            """A list of all contracts."""
            ...

        @property
        def completed_contracts(self):
            # type: () -> List[SpaceCenter.Contract]
            """A list of all completed contracts."""
            ...

        @property
        def failed_contracts(self):
            # type: () -> List[SpaceCenter.Contract]
            """A list of all failed contracts."""
            ...

        @property
        def offered_contracts(self):
            # type: () -> List[SpaceCenter.Contract]
            """A list of all offered, but unaccepted, contracts."""
            ...

        @property
        def types(self):
            # type: () -> Set[str]
            """A list of all contract types."""
            ...

    class ContractParameter:
        @property
        def children(self):
            # type: () -> List[SpaceCenter.ContractParameter]
            """Child contract parameters."""
            ...

        @property
        def completed(self):
            # type: () -> bool
            """Whether the parameter has been completed."""
            ...

        @property
        def failed(self):
            # type: () -> bool
            """Whether the parameter has been failed."""
            ...

        @property
        def funds_completion(self):
            # type: () -> float
            """Funds received on completion of the contract parameter."""
            ...

        @property
        def funds_failure(self):
            # type: () -> float
            """Funds lost if the contract parameter is failed."""
            ...

        @property
        def notes(self):
            # type: () -> str
            """Notes for the parameter."""
            ...

        @property
        def optional(self):
            # type: () -> bool
            """Whether the contract parameter is optional."""
            ...

        @property
        def reputation_completion(self):
            # type: () -> float
            """Reputation gained on completion of the contract parameter."""
            ...

        @property
        def reputation_failure(self):
            # type: () -> float
            """Reputation lost if the contract parameter is failed."""
            ...

        @property
        def science_completion(self):
            # type: () -> float
            """Science gained on completion of the contract parameter."""
            ...

        @property
        def title(self):
            # type: () -> str
            """Title of the parameter."""
            ...

    class Control:
        @property
        def abort(self):
            # type: () -> bool
            """The state of the abort action group."""
            ...

        @abort.setter
        def abort(self, value):
            # type: (bool) -> None
            ...

        @property
        def antennas(self):
            # type: () -> bool
            """Returns whether all antennas on the vessel are deployed,
            and sets the deployment state of all antennas.
            See SpaceCenter.Antenna#deployed."""
            ...

        @antennas.setter
        def antennas(self, value):
            # type: (bool) -> None
            ...

        @property
        def brakes(self):
            # type: () -> bool
            """The state of the wheel brakes."""
            ...

        @brakes.setter
        def brakes(self, value):
            # type: (bool) -> None
            ...

        @property
        def cargo_bays(self):
            # type: () -> bool
            """Returns whether any of the cargo bays on the vessel are open,
            and sets the open state of all cargo bays.
            See SpaceCenter.CargoBay#open."""
            ...

        @cargo_bays.setter
        def cargo_bays(self, value):
            # type: (bool) -> None
            ...

        @property
        def current_stage(self):
            # type: () -> int
            """The current stage of the vessel. Corresponds to the stage number in
            the in-game UI."""
            ...

        @property
        def custom_axis01(self):
            # type: () -> float
            """The state of CustomAxis01.
            A value between -1 and 1."""
            ...

        @custom_axis01.setter
        def custom_axis01(self, value):
            # type: (float) -> None
            ...

        @property
        def custom_axis02(self):
            # type: () -> float
            """The state of CustomAxis02.
            A value between -1 and 1."""
            ...

        @custom_axis02.setter
        def custom_axis02(self, value):
            # type: (float) -> None
            ...

        @property
        def custom_axis03(self):
            # type: () -> float
            """The state of CustomAxis03.
            A value between -1 and 1."""
            ...

        @custom_axis03.setter
        def custom_axis03(self, value):
            # type: (float) -> None
            ...

        @property
        def custom_axis04(self):
            # type: () -> float
            """The state of CustomAxis04.
            A value between -1 and 1."""
            ...

        @custom_axis04.setter
        def custom_axis04(self, value):
            # type: (float) -> None
            ...

        @property
        def forward(self):
            # type: () -> float
            """The state of the forward translational control.
            A value between -1 and 1.
            Equivalent to the h and n keys."""
            ...

        @forward.setter
        def forward(self, value):
            # type: (float) -> None
            ...

        @property
        def gear(self):
            # type: () -> bool
            """The state of the landing gear/legs."""
            ...

        @gear.setter
        def gear(self, value):
            # type: (bool) -> None
            ...

        @property
        def input_mode(self):
            # type: () -> SpaceCenter.ControlInputMode
            """Sets the behavior of the pitch, yaw, roll and translation control inputs.
            When set to additive, these inputs are added to the vessels current inputs.
            This mode is the default.
            When set to override, these inputs (if non-zero) override the vessels inputs.
            This mode prevents keyboard control, or SAS, from interfering with the controls when
            they are set."""
            ...

        @input_mode.setter
        def input_mode(self, value):
            # type: (SpaceCenter.ControlInputMode) -> None
            ...

        @property
        def intakes(self):
            # type: () -> bool
            """Returns whether all of the air intakes on the vessel are open,
            and sets the open state of all air intakes.
            See SpaceCenter.Intake#open."""
            ...

        @intakes.setter
        def intakes(self, value):
            # type: (bool) -> None
            ...

        @property
        def legs(self):
            # type: () -> bool
            """Returns whether all landing legs on the vessel are deployed,
            and sets the deployment state of all landing legs.
            Does not include wheels (for example landing gear).
            See SpaceCenter.Leg#deployed."""
            ...

        @legs.setter
        def legs(self, value):
            # type: (bool) -> None
            ...

        @property
        def lights(self):
            # type: () -> bool
            """The state of the lights."""
            ...

        @lights.setter
        def lights(self, value):
            # type: (bool) -> None
            ...

        @property
        def nodes(self):
            # type: () -> List[SpaceCenter.Node]
            """Returns a list of all existing maneuver nodes, ordered by time from first to last."""
            ...

        @property
        def parachutes(self):
            # type: () -> bool
            """Returns whether all parachutes on the vessel are deployed,
            and sets the deployment state of all parachutes.
            Cannot be set to {@code false}.
            See SpaceCenter.Parachute#deployed."""
            ...

        @parachutes.setter
        def parachutes(self, value):
            # type: (bool) -> None
            ...

        @property
        def pitch(self):
            # type: () -> float
            """The state of the pitch control.
            A value between -1 and 1.
            Equivalent to the w and s keys."""
            ...

        @pitch.setter
        def pitch(self, value):
            # type: (float) -> None
            ...

        @property
        def rcs(self):
            # type: () -> bool
            """The state of RCS."""
            ...

        @rcs.setter
        def rcs(self, value):
            # type: (bool) -> None
            ...

        @property
        def radiators(self):
            # type: () -> bool
            """Returns whether all radiators on the vessel are deployed,
            and sets the deployment state of all radiators.
            See SpaceCenter.Radiator#deployed."""
            ...

        @radiators.setter
        def radiators(self, value):
            # type: (bool) -> None
            ...

        @property
        def reaction_wheels(self):
            # type: () -> bool
            """Returns whether all reactive wheels on the vessel are active,
            and sets the active state of all reaction wheels.
            See SpaceCenter.ReactionWheel#active."""
            ...

        @reaction_wheels.setter
        def reaction_wheels(self, value):
            # type: (bool) -> None
            ...

        @property
        def resource_harvesters(self):
            # type: () -> bool
            """Returns whether all of the resource harvesters on the vessel are deployed,
            and sets the deployment state of all resource harvesters.
            See SpaceCenter.ResourceHarvester#deployed."""
            ...

        @resource_harvesters.setter
        def resource_harvesters(self, value):
            # type: (bool) -> None
            ...

        @property
        def resource_harvesters_active(self):
            # type: () -> bool
            """Returns whether any of the resource harvesters on the vessel are active,
            and sets the active state of all resource harvesters.
            See SpaceCenter.ResourceHarvester#active."""
            ...

        @resource_harvesters_active.setter
        def resource_harvesters_active(self, value):
            # type: (bool) -> None
            ...

        @property
        def right(self):
            # type: () -> float
            """The state of the right translational control.
            A value between -1 and 1.
            Equivalent to the j and l keys."""
            ...

        @right.setter
        def right(self, value):
            # type: (float) -> None
            ...

        @property
        def roll(self):
            # type: () -> float
            """The state of the roll control.
            A value between -1 and 1.
            Equivalent to the q and e keys."""
            ...

        @roll.setter
        def roll(self, value):
            # type: (float) -> None
            ...

        @property
        def sas(self):
            # type: () -> bool
            """The state of SAS.

            Equivalent to SpaceCenter.AutoPilot#sAS"""
            ...

        @sas.setter
        def sas(self, value):
            # type: (bool) -> None
            ...

        @property
        def sas_mode(self):
            # type: () -> SpaceCenter.SASMode
            """The current SpaceCenter.SASMode.
            These modes are equivalent to the mode buttons to
            the left of the navball that appear when SAS is enabled.

            Equivalent to SpaceCenter.AutoPilot#sASMode"""
            ...

        @sas_mode.setter
        def sas_mode(self, value):
            # type: (SpaceCenter.SASMode) -> None
            ...

        @property
        def solar_panels(self):
            # type: () -> bool
            """Returns whether all solar panels on the vessel are deployed,
            and sets the deployment state of all solar panels.
            See SpaceCenter.SolarPanel#deployed."""
            ...

        @solar_panels.setter
        def solar_panels(self, value):
            # type: (bool) -> None
            ...

        @property
        def source(self):
            # type: () -> SpaceCenter.ControlSource
            """The source of the vessels control, for example by a kerbal or a probe core."""
            ...

        @property
        def speed_mode(self):
            # type: () -> SpaceCenter.SpeedMode
            """The current SpaceCenter.SpeedMode of the navball.
            This is the mode displayed next to the speed at the top of the navball."""
            ...

        @speed_mode.setter
        def speed_mode(self, value):
            # type: (SpaceCenter.SpeedMode) -> None
            ...

        @property
        def stage_lock(self):
            # type: () -> bool
            """Whether staging is locked on the vessel.

            This is equivalent to locking the staging using Alt+L"""
            ...

        @stage_lock.setter
        def stage_lock(self, value):
            # type: (bool) -> None
            ...

        @property
        def state(self):
            # type: () -> SpaceCenter.ControlState
            """The control state of the vessel."""
            ...

        @property
        def throttle(self):
            # type: () -> float
            """The state of the throttle. A value between 0 and 1."""
            ...

        @throttle.setter
        def throttle(self, value):
            # type: (float) -> None
            ...

        @property
        def up(self):
            # type: () -> float
            """The state of the up translational control.
            A value between -1 and 1.
            Equivalent to the i and k keys."""
            ...

        @up.setter
        def up(self, value):
            # type: (float) -> None
            ...

        @property
        def wheel_steering(self):
            # type: () -> float
            """The state of the wheel steering.
            A value between -1 and 1.
            A value of 1 steers to the left, and a value of -1 steers to the right."""
            ...

        @wheel_steering.setter
        def wheel_steering(self, value):
            # type: (float) -> None
            ...

        @property
        def wheel_throttle(self):
            # type: () -> float
            """The state of the wheel throttle.
            A value between -1 and 1.
            A value of 1 rotates the wheels forwards, a value of -1 rotates
            the wheels backwards."""
            ...

        @wheel_throttle.setter
        def wheel_throttle(self, value):
            # type: (float) -> None
            ...

        @property
        def wheels(self):
            # type: () -> bool
            """Returns whether all wheels on the vessel are deployed,
            and sets the deployment state of all wheels.
            Does not include landing legs.
            See SpaceCenter.Wheel#deployed."""
            ...

        @wheels.setter
        def wheels(self, value):
            # type: (bool) -> None
            ...

        @property
        def yaw(self):
            # type: () -> float
            """The state of the yaw control.
            A value between -1 and 1.
            Equivalent to the a and d keys."""
            ...

        @yaw.setter
        def yaw(self, value):
            # type: (float) -> None
            ...

        def activate_next_stage(self):
            # type: () -> List[SpaceCenter.Vessel]
            """Activates the next stage. Equivalent to pressing the space bar in-game.
            @return A list of vessel objects that are jettisoned from the active vessel.

            When called, the active vessel may change. It is therefore possible that,
            after calling this function, the object(s) returned by previous call(s) to
            SpaceCenter#activeVessel no longer refer to the active vessel.
            Throws an exception if staging is locked."""
            ...

        def add_node(self, ut, prograde=0.0, normal=0.0, radial=0.0):
            # type: (float, float, float, float) -> SpaceCenter.Node
            """Creates a maneuver node at the given universal time, and returns a
            SpaceCenter.Node object that can be used to modify it.
            Optionally sets the magnitude of the delta-v for the maneuver node
            in the prograde, normal and radial directions.
            :ut Universal time of the maneuver node.
            :prograde Delta-v in the prograde direction.
            :normal Delta-v in the normal direction.
            :radial Delta-v in the radial direction."""
            ...

        def get_action_group(self, group):
            # type: (int) -> bool
            """Returns {@code true} if the given action group is enabled.
            :group A number between 0 and 9 inclusive,
            or between 0 and 250 inclusive when the Extended Action Groups mod is installed."""
            ...

        def remove_nodes(self):
            # type: () -> None
            """Remove all maneuver nodes."""
            ...

        def set_action_group(self, group, state):
            # type: (int, bool) -> None
            """Sets the state of the given action group.
            :group A number between 0 and 9 inclusive,
            or between 0 and 250 inclusive when the Extended Action Groups mod is installed.
            :state """
            ...

        def toggle_action_group(self, group):
            # type: (int) -> None
            """Toggles the state of the given action group.
            :group A number between 0 and 9 inclusive,
            or between 0 and 250 inclusive when the Extended Action Groups mod is installed."""
            ...

    class ControlSurface:
        @property
        def authority_limiter(self):
            # type: () -> float
            """The authority limiter for the control surface, which controls how far the
            control surface will move."""
            ...

        @authority_limiter.setter
        def authority_limiter(self, value):
            # type: (float) -> None
            ...

        @property
        def available_torque(self):
            # type: () -> Tuple[Tuple[float,float,float],Tuple[float,float,float]]
            """The available torque, in Newton meters, that can be produced by this control surface,
            in the positive and negative pitch, roll and yaw axes of the vessel. These axes
            correspond to the coordinate axes of the SpaceCenter.Vessel#referenceFrame."""
            ...

        @property
        def deployed(self):
            # type: () -> bool
            """Whether the control surface has been fully deployed."""
            ...

        @deployed.setter
        def deployed(self, value):
            # type: (bool) -> None
            ...

        @property
        def inverted(self):
            # type: () -> bool
            """Whether the control surface movement is inverted."""
            ...

        @inverted.setter
        def inverted(self, value):
            # type: (bool) -> None
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this control surface."""
            ...

        @property
        def pitch_enabled(self):
            # type: () -> bool
            """Whether the control surface has pitch control enabled."""
            ...

        @pitch_enabled.setter
        def pitch_enabled(self, value):
            # type: (bool) -> None
            ...

        @property
        def roll_enabled(self):
            # type: () -> bool
            """Whether the control surface has roll control enabled."""
            ...

        @roll_enabled.setter
        def roll_enabled(self, value):
            # type: (bool) -> None
            ...

        @property
        def surface_area(self):
            # type: () -> float
            """Surface area of the control surface in m^2."""
            ...

        @property
        def yaw_enabled(self):
            # type: () -> bool
            """Whether the control surface has yaw control enabled."""
            ...

        @yaw_enabled.setter
        def yaw_enabled(self, value):
            # type: (bool) -> None
            ...

    class CrewMember:
        @property
        def badass(self):
            # type: () -> bool
            """Whether the crew member is a badass."""
            ...

        @badass.setter
        def badass(self, value):
            # type: (bool) -> None
            ...

        @property
        def career_log_flights(self):
            # type: () -> List[int]
            """The flight IDs for each entry in the career flight log."""
            ...

        @property
        def career_log_targets(self):
            # type: () -> List[str]
            """The body name for each entry in the career flight log."""
            ...

        @property
        def career_log_types(self):
            # type: () -> List[str]
            """The type for each entry in the career flight log."""
            ...

        @property
        def courage(self):
            # type: () -> float
            """The crew members courage."""
            ...

        @courage.setter
        def courage(self, value):
            # type: (float) -> None
            ...

        @property
        def experience(self):
            # type: () -> float
            """The crew members experience."""
            ...

        @experience.setter
        def experience(self, value):
            # type: (float) -> None
            ...

        @property
        def gender(self):
            # type: () -> SpaceCenter.CrewMemberGender
            """The crew member's gender."""
            ...

        @property
        def name(self):
            # type: () -> str
            """The crew members name."""
            ...

        @name.setter
        def name(self, value):
            # type: (str) -> None
            ...

        @property
        def on_mission(self):
            # type: () -> bool
            """Whether the crew member is on a mission."""
            ...

        @property
        def roster_status(self):
            # type: () -> SpaceCenter.RosterStatus
            """The crew member's current roster status."""
            ...

        @property
        def stupidity(self):
            # type: () -> float
            """The crew members stupidity."""
            ...

        @stupidity.setter
        def stupidity(self, value):
            # type: (float) -> None
            ...

        @property
        def suit_type(self):
            # type: () -> SpaceCenter.SuitType
            """The crew member's suit type."""
            ...

        @suit_type.setter
        def suit_type(self, value):
            # type: (SpaceCenter.SuitType) -> None
            ...

        @property
        def trait(self):
            # type: () -> str
            """The crew member's job."""
            ...

        @property
        def type(self):
            # type: () -> SpaceCenter.CrewMemberType
            """The type of crew member."""
            ...

        @property
        def veteran(self):
            # type: () -> bool
            """Whether the crew member is a veteran."""
            ...

        @veteran.setter
        def veteran(self, value):
            # type: (bool) -> None
            ...

    class Decoupler:
        @property
        def decoupled(self):
            # type: () -> bool
            """Whether the decoupler has fired."""
            ...

        @property
        def impulse(self):
            # type: () -> float
            """The impulse that the decoupler imparts when it is fired, in Newton seconds."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this decoupler."""
            ...

        @property
        def staged(self):
            # type: () -> bool
            """Whether the decoupler is enabled in the staging sequence."""
            ...

        def decouple(self):
            # type: () -> SpaceCenter.Vessel
            """Fires the decoupler. Returns the new vessel created when the decoupler fires.
            Throws an exception if the decoupler has already fired.

            When called, the active vessel may change. It is therefore possible that,
            after calling this function, the object(s) returned by previous call(s) to
            SpaceCenter#activeVessel no longer refer to the active vessel."""
            ...

    class DockingPort:
        @property
        def can_rotate(self):
            # type: () -> bool
            """Whether the docking port can be commanded to rotate while docked."""
            ...

        @property
        def docked_part(self):
            # type: () -> Optional[SpaceCenter.Part]
            """The part that this docking port is docked to. Returns {@code null} if this
            docking port is not docked to anything."""
            ...

        @property
        def has_shield(self):
            # type: () -> bool
            """Whether the docking port has a shield."""
            ...

        @property
        def maximum_rotation(self):
            # type: () -> float
            """Maximum rotation angle in degrees."""
            ...

        @property
        def minimum_rotation(self):
            # type: () -> float
            """Minimum rotation angle in degrees."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this docking port."""
            ...

        @property
        def reengage_distance(self):
            # type: () -> float
            """The distance a docking port must move away when it undocks before it
            becomes ready to dock with another port, in meters."""
            ...

        @property
        def reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """The reference frame that is fixed relative to this docking port, and
            oriented with the port.
            <p><ul>
            <li>The origin is at the position of the docking port.
            <li>The axes rotate with the docking port.
            <li>The x-axis points out to the right side of the docking port.
            <li>The y-axis points in the direction the docking port is facing.
            <li>The z-axis points out of the bottom off the docking port.
            </ul></p>

            This reference frame is not necessarily equivalent to the reference frame
            for the part, returned by SpaceCenter.Part#referenceFrame."""
            ...

        @property
        def rotation_locked(self):
            # type: () -> bool
            """Lock rotation. When locked, allows auto-strut to work across the joint."""
            ...

        @rotation_locked.setter
        def rotation_locked(self, value):
            # type: (bool) -> None
            ...

        @property
        def rotation_target(self):
            # type: () -> float
            """Rotation target angle in degrees."""
            ...

        @rotation_target.setter
        def rotation_target(self, value):
            # type: (float) -> None
            ...

        @property
        def shielded(self):
            # type: () -> bool
            """The state of the docking ports shield, if it has one.

            Returns {@code true} if the docking port has a shield, and the shield is
            closed. Otherwise returns {@code false}. When set to {@code true}, the shield is
            closed, and when set to {@code false} the shield is opened. If the docking
            port does not have a shield, setting this attribute has no effect."""
            ...

        @shielded.setter
        def shielded(self, value):
            # type: (bool) -> None
            ...

        @property
        def state(self):
            # type: () -> SpaceCenter.DockingPortState
            """The current state of the docking port."""
            ...

        def direction(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The direction that docking port points in, in the given reference frame.
            @return The direction as a unit vector.
            :reference_frame The reference frame that the returned
            direction is in."""
            ...

        def position(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The position of the docking port, in the given reference frame.
            @return The position as a vector.
            :reference_frame The reference frame that the returned
            position vector is in."""
            ...

        def rotation(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float,float]
            """The rotation of the docking port, in the given reference frame.
            @return The rotation as a quaternion of the form (x, y, z, w).
            :reference_frame The reference frame that the returned
            rotation is in."""
            ...

        def undock(self):
            # type: () -> SpaceCenter.Vessel
            """Undocks the docking port and returns the new SpaceCenter.Vessel that is created.
            This method can be called for either docking port in a docked pair.
            Throws an exception if the docking port is not docked to anything.

            When called, the active vessel may change. It is therefore possible that,
            after calling this function, the object(s) returned by previous call(s) to
            SpaceCenter#activeVessel no longer refer to the active vessel."""
            ...

    class Engine:
        @property
        def active(self):
            # type: () -> bool
            """Whether the engine is active. Setting this attribute may have no effect,
            depending on SpaceCenter.Engine#canShutdown and SpaceCenter.Engine#canRestart."""
            ...

        @active.setter
        def active(self, value):
            # type: (bool) -> None
            ...

        @property
        def auto_mode_switch(self):
            # type: () -> bool
            """Whether the engine will automatically switch modes."""
            ...

        @auto_mode_switch.setter
        def auto_mode_switch(self, value):
            # type: (bool) -> None
            ...

        @property
        def available_thrust(self):
            # type: () -> float
            """The amount of thrust, in Newtons, that would be produced by the engine
            when activated and with its throttle set to 100%.
            Returns zero if the engine does not have any fuel.
            Takes the engine's current SpaceCenter.Engine#thrustLimit and atmospheric conditions
            into account."""
            ...

        @property
        def available_torque(self):
            # type: () -> Tuple[Tuple[float,float,float],Tuple[float,float,float]]
            """The available torque, in Newton meters, that can be produced by this engine,
            in the positive and negative pitch, roll and yaw axes of the vessel. These axes
            correspond to the coordinate axes of the SpaceCenter.Vessel#referenceFrame.
            Returns zero if the engine is inactive, or not gimballed."""
            ...

        @property
        def can_restart(self):
            # type: () -> bool
            """Whether the engine can be restarted once shutdown. If the engine cannot be shutdown,
            returns {@code false}. For example, this is {@code true} for liquid fueled rockets
            and {@code false} for solid rocket boosters."""
            ...

        @property
        def can_shutdown(self):
            # type: () -> bool
            """Whether the engine can be shutdown once activated. For example, this is
            {@code true} for liquid fueled rockets and {@code false} for solid rocket boosters."""
            ...

        @property
        def gimbal_limit(self):
            # type: () -> float
            """The gimbal limiter of the engine. A value between 0 and 1.
            Returns 0 if the gimbal is locked."""
            ...

        @gimbal_limit.setter
        def gimbal_limit(self, value):
            # type: (float) -> None
            ...

        @property
        def gimbal_locked(self):
            # type: () -> bool
            """Whether the engines gimbal is locked in place. Setting this attribute has
            no effect if the engine is not gimballed."""
            ...

        @gimbal_locked.setter
        def gimbal_locked(self, value):
            # type: (bool) -> None
            ...

        @property
        def gimbal_range(self):
            # type: () -> float
            """The range over which the gimbal can move, in degrees.
            Returns 0 if the engine is not gimballed."""
            ...

        @property
        def gimballed(self):
            # type: () -> bool
            """Whether the engine is gimballed."""
            ...

        @property
        def has_fuel(self):
            # type: () -> bool
            """Whether the engine has any fuel available."""
            ...

        @property
        def has_modes(self):
            # type: () -> bool
            """Whether the engine has multiple modes of operation."""
            ...

        @property
        def kerbin_sea_level_specific_impulse(self):
            # type: () -> float
            """The specific impulse of the engine at sea level on Kerbin, in seconds."""
            ...

        @property
        def max_thrust(self):
            # type: () -> float
            """The amount of thrust, in Newtons, that would be produced by the engine
            when activated and fueled, with its throttle and throttle limiter set to 100%."""
            ...

        @property
        def max_vacuum_thrust(self):
            # type: () -> float
            """The maximum amount of thrust that can be produced by the engine in a
            vacuum, in Newtons. This is the amount of thrust produced by the engine
            when activated, SpaceCenter.Engine#thrustLimit is set to 100%, the main
            vessel's throttle is set to 100% and the engine is in a vacuum."""
            ...

        @property
        def mode(self):
            # type: () -> str
            """The name of the current engine mode."""
            ...

        @mode.setter
        def mode(self, value):
            # type: (str) -> None
            ...

        @property
        def modes(self):
            # type: () -> Dict[str, SpaceCenter.Engine]
            """The available modes for the engine.
            A dictionary mapping mode names to SpaceCenter.Engine objects."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this engine."""
            ...

        @property
        def propellant_names(self):
            # type: () -> List[str]
            """The names of the propellants that the engine consumes."""
            ...

        @property
        def propellant_ratios(self):
            # type: () -> Dict[str, float]
            """The ratio of resources that the engine consumes. A dictionary mapping resource names
            to the ratio at which they are consumed by the engine.

            For example, if the ratios are 0.6 for LiquidFuel and 0.4 for Oxidizer, then for every
            0.6 units of LiquidFuel that the engine burns, it will burn 0.4 units of Oxidizer."""
            ...

        @property
        def propellants(self):
            # type: () -> List[SpaceCenter.Propellant]
            """The propellants that the engine consumes."""
            ...

        @property
        def specific_impulse(self):
            # type: () -> float
            """The current specific impulse of the engine, in seconds. Returns zero
            if the engine is not active."""
            ...

        @property
        def throttle(self):
            # type: () -> float
            """The current throttle setting for the engine. A value between 0 and 1.
            This is not necessarily the same as the vessel's main throttle
            setting, as some engines take time to adjust their throttle
            (such as jet engines)."""
            ...

        @property
        def throttle_locked(self):
            # type: () -> bool
            """Whether the SpaceCenter.Control#throttle affects the engine. For example,
            this is {@code true} for liquid fueled rockets, and {@code false} for solid rocket
            boosters."""
            ...

        @property
        def thrust(self):
            # type: () -> float
            """The current amount of thrust being produced by the engine, in Newtons."""
            ...

        @property
        def thrust_limit(self):
            # type: () -> float
            """The thrust limiter of the engine. A value between 0 and 1. Setting this
            attribute may have no effect, for example the thrust limit for a solid
            rocket booster cannot be changed in flight."""
            ...

        @thrust_limit.setter
        def thrust_limit(self, value):
            # type: (float) -> None
            ...

        @property
        def thrusters(self):
            # type: () -> List[SpaceCenter.Thruster]
            """The components of the engine that generate thrust.

            For example, this corresponds to the rocket nozzel on a solid rocket booster,
            or the individual nozzels on a RAPIER engine.
            The overall thrust produced by the engine, as reported by SpaceCenter.Engine#availableThrust,
            SpaceCenter.Engine#maxThrust and others, is the sum of the thrust generated by each thruster."""
            ...

        @property
        def vacuum_specific_impulse(self):
            # type: () -> float
            """The vacuum specific impulse of the engine, in seconds."""
            ...

        def available_thrust_at(self, pressure):
            # type: (float) -> float
            """The amount of thrust, in Newtons, that would be produced by the engine
            when activated and with its throttle set to 100%.
            Returns zero if the engine does not have any fuel.
            Takes the given pressure into account.
            :pressure Atmospheric pressure in atmospheres"""
            ...

        def max_thrust_at(self, pressure):
            # type: (float) -> float
            """The amount of thrust, in Newtons, that would be produced by the engine
            when activated and fueled, with its throttle and throttle limiter set to 100%.
            Takes the given pressure into account.
            :pressure Atmospheric pressure in atmospheres"""
            ...

        def specific_impulse_at(self, pressure):
            # type: (float) -> float
            """The specific impulse of the engine under the given pressure, in seconds. Returns zero
            if the engine is not active.
            :pressure Atmospheric pressure in atmospheres"""
            ...

        def toggle_mode(self):
            # type: () -> None
            """Toggle the current engine mode."""
            ...

    class Experiment:
        @property
        def available(self):
            # type: () -> bool
            """Determines if the experiment is available given the current conditions."""
            ...

        @property
        def biome(self):
            # type: () -> str
            """The name of the biome the experiment is currently in."""
            ...

        @property
        def data(self):
            # type: () -> List[SpaceCenter.ScienceData]
            """The data contained in this experiment."""
            ...

        @property
        def deployed(self):
            # type: () -> bool
            """Whether the experiment has been deployed."""
            ...

        @property
        def has_data(self):
            # type: () -> bool
            """Whether the experiment contains data."""
            ...

        @property
        def inoperable(self):
            # type: () -> bool
            """Whether the experiment is inoperable."""
            ...

        @property
        def name(self):
            # type: () -> str
            """Internal name of the experiment, as used in
            part cfg files."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this experiment."""
            ...

        @property
        def rerunnable(self):
            # type: () -> bool
            """Whether the experiment can be re-run."""
            ...

        @property
        def science_subject(self):
            # type: () -> SpaceCenter.ScienceSubject
            """Containing information on the corresponding specific science result for the current
            conditions. Returns {@code null} if the experiment is unavailable."""
            ...

        @property
        def title(self):
            # type: () -> str
            """Title of the experiment, as shown on the in-game UI."""
            ...

        def dump(self):
            # type: () -> None
            """Dump the experimental data contained by the experiment."""
            ...

        def reset(self):
            # type: () -> None
            """Reset the experiment."""
            ...

        def run(self):
            # type: () -> None
            """Run the experiment."""
            ...

        def transmit(self):
            # type: () -> None
            """Transmit all experimental data contained by this part."""
            ...

    class Fairing:
        @property
        def jettisoned(self):
            # type: () -> bool
            """Whether the fairing has been jettisoned."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this fairing."""
            ...

        def jettison(self):
            # type: () -> None
            """Jettison the fairing. Has no effect if it has already been jettisoned."""
            ...

    class Flight:
        @property
        def aerodynamic_force(self):
            # type: () -> Tuple[float,float,float]
            """The total aerodynamic forces acting on the vessel,
            in reference frame SpaceCenter.ReferenceFrame.
            @return A vector pointing in the direction that the force acts,
            with its magnitude equal to the strength of the force in Newtons."""
            ...

        @property
        def angle_of_attack(self):
            # type: () -> float
            """The pitch angle between the orientation of the vessel and its velocity vector,
            in degrees."""
            ...

        @property
        def anti_normal(self):
            # type: () -> Tuple[float,float,float]
            """The direction opposite to the normal of the vessels orbit,
            in the reference frame SpaceCenter.ReferenceFrame.
            @return The direction as a unit vector."""
            ...

        @property
        def anti_radial(self):
            # type: () -> Tuple[float,float,float]
            """The direction opposite to the radial direction of the vessels orbit,
            in the reference frame SpaceCenter.ReferenceFrame.
            @return The direction as a unit vector."""
            ...

        @property
        def atmosphere_density(self):
            # type: () -> float
            """The current density of the atmosphere around the vessel, in kg/m^3."""
            ...

        @property
        def ballistic_coefficient(self):
            # type: () -> float
            """The ballistic coefficient.

            Requires Ferram Aerospace Research."""
            ...

        @property
        def bedrock_altitude(self):
            # type: () -> float
            """The altitude above the surface of the body, in meters. When over water, this is the altitude above the sea floor.
            Measured from the center of mass of the vessel."""
            ...

        @property
        def center_of_mass(self):
            # type: () -> Tuple[float,float,float]
            """The position of the center of mass of the vessel,
            in the reference frame SpaceCenter.ReferenceFrame
            @return The position as a vector."""
            ...

        @property
        def direction(self):
            # type: () -> Tuple[float,float,float]
            """The direction that the vessel is pointing in,
            in the reference frame SpaceCenter.ReferenceFrame.
            @return The direction as a unit vector."""
            ...

        @property
        def drag(self):
            # type: () -> Tuple[float,float,float]
            """The aerodynamic drag currently acting on the vessel.
            @return A vector pointing in the direction of the force, with its magnitude
            equal to the strength of the force in Newtons."""
            ...

        @property
        def drag_coefficient(self):
            # type: () -> float
            """The coefficient of drag. This is the amount of drag produced by the vessel.
            It depends on air speed, air density and wing area.

            Requires Ferram Aerospace Research."""
            ...

        @property
        def dynamic_pressure(self):
            # type: () -> float
            """The dynamic pressure acting on the vessel, in Pascals. This is a measure of the
            strength of the aerodynamic forces. It is equal to
            \frac{1}{2} . \mbox{air density} . \mbox{velocity}^2.
            It is commonly denoted Q."""
            ...

        @property
        def elevation(self):
            # type: () -> float
            """The elevation of the terrain under the vessel, in meters. This is the height of the terrain above sea level,
            and is negative when the vessel is over the sea."""
            ...

        @property
        def equivalent_air_speed(self):
            # type: () -> float
            """The equivalent air speed
            of the vessel, in meters per second."""
            ...

        @property
        def g_force(self):
            # type: () -> float
            """The current G force acting on the vessel in g."""
            ...

        @property
        def heading(self):
            # type: () -> float
            """The heading of the vessel (its angle relative to north), in degrees.
            A value between 0° and 360°."""
            ...

        @property
        def horizontal_speed(self):
            # type: () -> float
            """The horizontal speed of the vessel in meters per second,
            in the reference frame SpaceCenter.ReferenceFrame."""
            ...

        @property
        def latitude(self):
            # type: () -> float
            """The latitude of the vessel for the body being orbited, in degrees."""
            ...

        @property
        def lift(self):
            # type: () -> Tuple[float,float,float]
            """The aerodynamic lift
            currently acting on the vessel.
            @return A vector pointing in the direction that the force acts,
            with its magnitude equal to the strength of the force in Newtons."""
            ...

        @property
        def lift_coefficient(self):
            # type: () -> float
            """The coefficient of lift. This is the amount of lift produced by the vessel, and
            depends on air speed, air density and wing area.

            Requires Ferram Aerospace Research."""
            ...

        @property
        def longitude(self):
            # type: () -> float
            """The longitude of the vessel for the body being orbited, in degrees."""
            ...

        @property
        def mach(self):
            # type: () -> float
            """The speed of the vessel, in multiples of the speed of sound."""
            ...

        @property
        def mean_altitude(self):
            # type: () -> float
            """The altitude above sea level, in meters.
            Measured from the center of mass of the vessel."""
            ...

        @property
        def normal(self):
            # type: () -> Tuple[float,float,float]
            """The direction normal to the vessels orbit,
            in the reference frame SpaceCenter.ReferenceFrame.
            @return The direction as a unit vector."""
            ...

        @property
        def pitch(self):
            # type: () -> float
            """The pitch of the vessel relative to the horizon, in degrees.
            A value between -90° and +90°."""
            ...

        @property
        def prograde(self):
            # type: () -> Tuple[float,float,float]
            """The prograde direction of the vessels orbit,
            in the reference frame SpaceCenter.ReferenceFrame.
            @return The direction as a unit vector."""
            ...

        @property
        def radial(self):
            # type: () -> Tuple[float,float,float]
            """The radial direction of the vessels orbit,
            in the reference frame SpaceCenter.ReferenceFrame.
            @return The direction as a unit vector."""
            ...

        @property
        def retrograde(self):
            # type: () -> Tuple[float,float,float]
            """The retrograde direction of the vessels orbit,
            in the reference frame SpaceCenter.ReferenceFrame.
            @return The direction as a unit vector."""
            ...

        @property
        def reynolds_number(self):
            # type: () -> float
            """The vessels Reynolds number.

            Requires Ferram Aerospace Research."""
            ...

        @property
        def roll(self):
            # type: () -> float
            """The roll of the vessel relative to the horizon, in degrees.
            A value between -180° and +180°."""
            ...

        @property
        def rotation(self):
            # type: () -> Tuple[float,float,float,float]
            """The rotation of the vessel, in the reference frame SpaceCenter.ReferenceFrame
            @return The rotation as a quaternion of the form (x, y, z, w)."""
            ...

        @property
        def sideslip_angle(self):
            # type: () -> float
            """The yaw angle between the orientation of the vessel and its velocity vector, in degrees."""
            ...

        @property
        def speed(self):
            # type: () -> float
            """The speed of the vessel in meters per second,
            in the reference frame SpaceCenter.ReferenceFrame."""
            ...

        @property
        def speed_of_sound(self):
            # type: () -> float
            """The speed of sound, in the atmosphere around the vessel, in m/s."""
            ...

        @property
        def stall_fraction(self):
            # type: () -> float
            """The current amount of stall, between 0 and 1. A value greater than 0.005 indicates
            a minor stall and a value greater than 0.5 indicates a large-scale stall.

            Requires Ferram Aerospace Research."""
            ...

        @property
        def static_air_temperature(self):
            # type: () -> float
            """The static (ambient)
            temperature of the atmosphere around the vessel, in Kelvin."""
            ...

        @property
        def static_pressure(self):
            # type: () -> float
            """The static atmospheric pressure acting on the vessel, in Pascals."""
            ...

        @property
        def static_pressure_at_msl(self):
            # type: () -> float
            """The static atmospheric pressure at mean sea level, in Pascals."""
            ...

        @property
        def surface_altitude(self):
            # type: () -> float
            """The altitude above the surface of the body or sea level, whichever is closer, in meters.
            Measured from the center of mass of the vessel."""
            ...

        @property
        def terminal_velocity(self):
            # type: () -> float
            """An estimate of the current terminal velocity of the vessel, in meters per second.
            This is the speed at which the drag forces cancel out the force of gravity."""
            ...

        @property
        def thrust_specific_fuel_consumption(self):
            # type: () -> float
            """The thrust specific fuel consumption for the jet engines on the vessel. This is a
            measure of the efficiency of the engines, with a lower value indicating a more
            efficient vessel. This value is the number of Newtons of fuel that are burned,
            per hour, to produce one newton of thrust.

            Requires Ferram Aerospace Research."""
            ...

        @property
        def total_air_temperature(self):
            # type: () -> float
            """The total air temperature
            of the atmosphere around the vessel, in Kelvin.
            This includes the SpaceCenter.Flight#staticAirTemperature and the vessel's kinetic energy."""
            ...

        @property
        def true_air_speed(self):
            # type: () -> float
            """The true air speed
            of the vessel, in meters per second."""
            ...

        @property
        def velocity(self):
            # type: () -> Tuple[float,float,float]
            """The velocity of the vessel, in the reference frame SpaceCenter.ReferenceFrame.
            @return The velocity as a vector. The vector points in the direction of travel,
            and its magnitude is the speed of the vessel in meters per second."""
            ...

        @property
        def vertical_speed(self):
            # type: () -> float
            """The vertical speed of the vessel in meters per second,
            in the reference frame SpaceCenter.ReferenceFrame."""
            ...

        def simulate_aerodynamic_force_at(self, body, position, velocity):
            # type: (SpaceCenter.CelestialBody, Tuple[float,float,float], Tuple[float,float,float]) -> Tuple[float,float,float]
            """Simulate and return the total aerodynamic forces acting on the vessel,
            if it where to be traveling with the given velocity at the given position in the
            atmosphere of the given celestial body.
            @return A vector pointing in the direction that the force acts,
            with its magnitude equal to the strength of the force in Newtons."""
            ...

    class Force:
        @property
        def force_vector(self):
            # type: () -> Tuple[float,float,float]
            """The force vector, in Newtons.
            @return A vector pointing in the direction that the force acts,
            with its magnitude equal to the strength of the force in Newtons."""
            ...

        @force_vector.setter
        def force_vector(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part that this force is applied to."""
            ...

        @property
        def position(self):
            # type: () -> Tuple[float,float,float]
            """The position at which the force acts, in reference frame SpaceCenter.ReferenceFrame.
            @return The position as a vector."""
            ...

        @position.setter
        def position(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """The reference frame of the force vector and position."""
            ...

        @reference_frame.setter
        def reference_frame(self, value):
            # type: (SpaceCenter.ReferenceFrame) -> None
            ...

        def remove(self):
            # type: () -> None
            """Remove the force."""
            ...

    class Intake:
        @property
        def area(self):
            # type: () -> float
            """The area of the intake's opening, in square meters."""
            ...

        @property
        def flow(self):
            # type: () -> float
            """The rate of flow into the intake, in units of resource per second."""
            ...

        @property
        def open(self):
            # type: () -> bool
            """Whether the intake is open."""
            ...

        @open.setter
        def open(self, value):
            # type: (bool) -> None
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this intake."""
            ...

        @property
        def speed(self):
            # type: () -> float
            """Speed of the flow into the intake, in m/s."""
            ...

    class LaunchClamp:
        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this launch clamp."""
            ...

        def release(self):
            # type: () -> None
            """Releases the docking clamp. Has no effect if the clamp has already been released."""
            ...

    class LaunchSite:
        @property
        def body(self):
            # type: () -> SpaceCenter.CelestialBody
            """The celestial body the launch site is on."""
            ...

        @property
        def editor_facility(self):
            # type: () -> SpaceCenter.EditorFacility
            """Which editor is normally used for this launch site."""
            ...

        @property
        def name(self):
            # type: () -> str
            """The name of the launch site."""
            ...

    class Leg:
        @property
        def deployable(self):
            # type: () -> bool
            """Whether the leg is deployable."""
            ...

        @property
        def deployed(self):
            # type: () -> bool
            """Whether the landing leg is deployed.

            Fixed landing legs are always deployed.
            Returns an error if you try to deploy fixed landing gear."""
            ...

        @deployed.setter
        def deployed(self, value):
            # type: (bool) -> None
            ...

        @property
        def is_grounded(self):
            # type: () -> bool
            """Returns whether the leg is touching the ground."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this landing leg."""
            ...

        @property
        def state(self):
            # type: () -> SpaceCenter.LegState
            """The current state of the landing leg."""
            ...

    class Light:
        @property
        def active(self):
            # type: () -> bool
            """Whether the light is switched on."""
            ...

        @active.setter
        def active(self, value):
            # type: (bool) -> None
            ...

        @property
        def blink(self):
            # type: () -> bool
            """Whether blinking is enabled."""
            ...

        @blink.setter
        def blink(self, value):
            # type: (bool) -> None
            ...

        @property
        def blink_rate(self):
            # type: () -> float
            """The blink rate of the light."""
            ...

        @blink_rate.setter
        def blink_rate(self, value):
            # type: (float) -> None
            ...

        @property
        def color(self):
            # type: () -> Tuple[float,float,float]
            """The color of the light, as an RGB triple."""
            ...

        @color.setter
        def color(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this light."""
            ...

        @property
        def power_usage(self):
            # type: () -> float
            """The current power usage, in units of charge per second."""
            ...

    class Module:
        @property
        def actions(self):
            # type: () -> List[str]
            """A list of all the names of the modules actions. These are the parts actions that can
            be assigned to action groups in the in-game editor."""
            ...

        @property
        def events(self):
            # type: () -> List[str]
            """A list of the names of all of the modules events. Events are the clickable buttons
            visible in the right-click menu of the part."""
            ...

        @property
        def fields(self):
            # type: () -> Dict[str, str]
            """The modules field names and their associated values, as a dictionary.
            These are the values visible in the right-click menu of the part."""
            ...

        @property
        def name(self):
            # type: () -> str
            """Name of the PartModule. For example, "ModuleEngines"."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part that contains this module."""
            ...

        def get_field(self, name):
            # type: (str) -> str
            """Returns the value of a field.
            :name Name of the field."""
            ...

        def has_action(self, name):
            # type: (str) -> bool
            """{@code true} if the part has an action with the given name.
            :name """
            ...

        def has_event(self, name):
            # type: (str) -> bool
            """{@code true} if the module has an event with the given name.
            :name """
            ...

        def has_field(self, name):
            # type: (str) -> bool
            """Returns {@code true} if the module has a field with the given name.
            :name Name of the field."""
            ...

        def reset_field(self, name):
            # type: (str) -> None
            """Set the value of a field to its original value.
            :name Name of the field."""
            ...

        def set_action(self, name, value=True):
            # type: (str, bool) -> None
            """Set the value of an action with the given name.
            :name 
            :value """
            ...

        def set_field_float(self, name, value):
            # type: (str, float) -> None
            """Set the value of a field to the given floating point number.
            :name Name of the field.
            :value Value to set."""
            ...

        def set_field_int(self, name, value):
            # type: (str, int) -> None
            """Set the value of a field to the given integer number.
            :name Name of the field.
            :value Value to set."""
            ...

        def set_field_string(self, name, value):
            # type: (str, str) -> None
            """Set the value of a field to the given string.
            :name Name of the field.
            :value Value to set."""
            ...

        def trigger_event(self, name):
            # type: (str) -> None
            """Trigger the named event. Equivalent to clicking the button in the right-click menu
            of the part.
            :name """
            ...

    class Node:
        @property
        def delta_v(self):
            # type: () -> float
            """The delta-v of the maneuver node, in meters per second.

            Does not change when executing the maneuver node. See SpaceCenter.Node#remainingDeltaV."""
            ...

        @delta_v.setter
        def delta_v(self, value):
            # type: (float) -> None
            ...

        @property
        def normal(self):
            # type: () -> float
            """The magnitude of the maneuver nodes delta-v in the normal direction,
            in meters per second."""
            ...

        @normal.setter
        def normal(self, value):
            # type: (float) -> None
            ...

        @property
        def orbit(self):
            # type: () -> SpaceCenter.Orbit
            """The orbit that results from executing the maneuver node."""
            ...

        @property
        def orbital_reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """The reference frame that is fixed relative to the maneuver node, and
            orientated with the orbital prograde/normal/radial directions of the
            original orbit at the maneuver node's position.
            <p><ul>
            <li>The origin is at the position of the maneuver node.
            <li>The x-axis points in the orbital anti-radial direction of the original
              orbit, at the position of the maneuver node.
            <li>The y-axis points in the orbital prograde direction of the original
              orbit, at the position of the maneuver node.
            <li>The z-axis points in the orbital normal direction of the original orbit,
              at the position of the maneuver node.
            </ul></p>"""
            ...

        @property
        def prograde(self):
            # type: () -> float
            """The magnitude of the maneuver nodes delta-v in the prograde direction,
            in meters per second."""
            ...

        @prograde.setter
        def prograde(self, value):
            # type: (float) -> None
            ...

        @property
        def radial(self):
            # type: () -> float
            """The magnitude of the maneuver nodes delta-v in the radial direction,
            in meters per second."""
            ...

        @radial.setter
        def radial(self, value):
            # type: (float) -> None
            ...

        @property
        def reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """The reference frame that is fixed relative to the maneuver node's burn.
            <p><ul>
            <li>The origin is at the position of the maneuver node.
            <li>The y-axis points in the direction of the burn.
            <li>The x-axis and z-axis point in arbitrary but fixed directions.
            </ul></p>"""
            ...

        @property
        def remaining_delta_v(self):
            # type: () -> float
            """Gets the remaining delta-v of the maneuver node, in meters per second. Changes as the
            node is executed. This is equivalent to the delta-v reported in-game."""
            ...

        @property
        def time_to(self):
            # type: () -> float
            """The time until the maneuver node will be encountered, in seconds."""
            ...

        @property
        def ut(self):
            # type: () -> float
            """The universal time at which the maneuver will occur, in seconds."""
            ...

        @ut.setter
        def ut(self, value):
            # type: (float) -> None
            ...

        def burn_vector(self, reference_frame=None):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """Returns the burn vector for the maneuver node.
            :reference_frame The reference frame that the returned vector is in.
            Defaults to SpaceCenter.Vessel#orbitalReferenceFrame.
            @return A vector whose direction is the direction of the maneuver node burn, and
            magnitude is the delta-v of the burn in meters per second.

            Does not change when executing the maneuver node. See SpaceCenter.Node#remainingBurnVector."""
            ...

        def direction(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The direction of the maneuver nodes burn.
            @return The direction as a unit vector.
            :reference_frame The reference frame that the returned
            direction is in."""
            ...

        def position(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The position vector of the maneuver node in the given reference frame.
            @return The position as a vector.
            :reference_frame The reference frame that the returned
            position vector is in."""
            ...

        def remaining_burn_vector(self, reference_frame=None):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """Returns the remaining burn vector for the maneuver node.
            :reference_frame The reference frame that the returned vector is in.
            Defaults to SpaceCenter.Vessel#orbitalReferenceFrame.
            @return A vector whose direction is the direction of the maneuver node burn, and
            magnitude is the delta-v of the burn in meters per second.

            Changes as the maneuver node is executed. See SpaceCenter.Node#burnVector."""
            ...

        def remove(self):
            # type: () -> None
            """Removes the maneuver node."""
            ...

    class Orbit:
        @property
        def apoapsis(self):
            # type: () -> float
            """Gets the apoapsis of the orbit, in meters, from the center of mass
            of the body being orbited.

            For the apoapsis altitude reported on the in-game map view,
            use SpaceCenter.Orbit#apoapsisAltitude."""
            ...

        @property
        def apoapsis_altitude(self):
            # type: () -> float
            """The apoapsis of the orbit, in meters, above the sea level of the body being orbited.

            This is equal to SpaceCenter.Orbit#apoapsis minus the equatorial radius of the body."""
            ...

        @property
        def argument_of_periapsis(self):
            # type: () -> float
            """The argument of
            periapsis, in radians."""
            ...

        @property
        def body(self):
            # type: () -> SpaceCenter.CelestialBody
            """The celestial body (e.g. planet or moon) around which the object is orbiting."""
            ...

        @property
        def eccentric_anomaly(self):
            # type: () -> float
            """The eccentric anomaly."""
            ...

        @property
        def eccentricity(self):
            # type: () -> float
            """The eccentricity
            of the orbit."""
            ...

        @property
        def epoch(self):
            # type: () -> float
            """The time since the epoch (the point at which the
            mean anomaly at epoch
            was measured, in seconds."""
            ...

        @property
        def inclination(self):
            # type: () -> float
            """The inclination
            of the orbit,
            in radians."""
            ...

        @property
        def longitude_of_ascending_node(self):
            # type: () -> float
            """The longitude of
            the ascending node, in radians."""
            ...

        @property
        def mean_anomaly(self):
            # type: () -> float
            """The mean anomaly."""
            ...

        @property
        def mean_anomaly_at_epoch(self):
            # type: () -> float
            """The mean anomaly at epoch."""
            ...

        @property
        def next_orbit(self):
            # type: () -> Optional[SpaceCenter.Orbit]
            """If the object is going to change sphere of influence in the future, returns the new
            orbit after the change. Otherwise returns {@code null}."""
            ...

        @property
        def orbital_speed(self):
            # type: () -> float
            """The current orbital speed in meters per second."""
            ...

        @property
        def periapsis(self):
            # type: () -> float
            """The periapsis of the orbit, in meters, from the center of mass
            of the body being orbited.

            For the periapsis altitude reported on the in-game map view,
            use SpaceCenter.Orbit#periapsisAltitude."""
            ...

        @property
        def periapsis_altitude(self):
            # type: () -> float
            """The periapsis of the orbit, in meters, above the sea level of the body being orbited.

            This is equal to SpaceCenter.Orbit#periapsis minus the equatorial radius of the body."""
            ...

        @property
        def period(self):
            # type: () -> float
            """The orbital period, in seconds."""
            ...

        @property
        def radius(self):
            # type: () -> float
            """The current radius of the orbit, in meters. This is the distance between the center
            of mass of the object in orbit, and the center of mass of the body around which it
            is orbiting.

            This value will change over time if the orbit is elliptical."""
            ...

        @property
        def semi_major_axis(self):
            # type: () -> float
            """The semi-major axis of the orbit, in meters."""
            ...

        @property
        def semi_minor_axis(self):
            # type: () -> float
            """The semi-minor axis of the orbit, in meters."""
            ...

        @property
        def speed(self):
            # type: () -> float
            """The current orbital speed of the object in meters per second.

            This value will change over time if the orbit is elliptical."""
            ...

        @property
        def time_to_apoapsis(self):
            # type: () -> float
            """The time until the object reaches apoapsis, in seconds."""
            ...

        @property
        def time_to_periapsis(self):
            # type: () -> float
            """The time until the object reaches periapsis, in seconds."""
            ...

        @property
        def time_to_soi_change(self):
            # type: () -> float
            """The time until the object changes sphere of influence, in seconds. Returns {@code NaN}
            if the object is not going to change sphere of influence."""
            ...

        @property
        def true_anomaly(self):
            # type: () -> float
            """The true anomaly."""
            ...

        def distance_at_closest_approach(self, target):
            # type: (SpaceCenter.Orbit) -> float
            """Estimates and returns the distance at closest approach to a target orbit, in meters.
            :target Target orbit."""
            ...

        def eccentric_anomaly_at_ut(self, ut):
            # type: (float) -> float
            """The eccentric anomaly at the given universal time.
            :ut The universal time, in seconds."""
            ...

        def list_closest_approaches(self, target, orbits):
            # type: (SpaceCenter.Orbit, int) -> List[List[float]]
            """Returns the times at closest approach and corresponding distances, to a target orbit.
            @return A list of two lists.
            The first is a list of times at closest approach, as universal times in seconds.
            The second is a list of corresponding distances at closest approach, in meters.
            :target Target orbit.
            :orbits The number of future orbits to search."""
            ...

        def mean_anomaly_at_ut(self, ut):
            # type: (float) -> float
            """The mean anomaly at the given time.
            :ut The universal time in seconds."""
            ...

        def orbital_speed_at(self, time):
            # type: (float) -> float
            """The orbital speed at the given time, in meters per second.
            :time Time from now, in seconds."""
            ...

        def position_at(self, ut, reference_frame):
            # type: (float, SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The position at a given time, in the specified reference frame.
            @return The position as a vector.
            :ut The universal time to measure the position at.
            :reference_frame The reference frame that the returned
            position vector is in."""
            ...

        def radius_at(self, ut):
            # type: (float) -> float
            """The orbital radius at the given time, in meters.
            :ut The universal time to measure the radius at."""
            ...

        def radius_at_true_anomaly(self, true_anomaly):
            # type: (float) -> float
            """The orbital radius at the point in the orbit given by the true anomaly.
            :true_anomaly The true anomaly."""
            ...

        def relative_inclination(self, target):
            # type: (SpaceCenter.Orbit) -> float
            """Relative inclination of this orbit and the target orbit, in radians.
            :target Target orbit."""
            ...

        def time_of_closest_approach(self, target):
            # type: (SpaceCenter.Orbit) -> float
            """Estimates and returns the time at closest approach to a target orbit.
            @return The universal time at closest approach, in seconds.
            :target Target orbit."""
            ...

        def true_anomaly_at_an(self, target):
            # type: (SpaceCenter.Orbit) -> float
            """The true anomaly of the ascending node with the given target orbit.
            :target Target orbit."""
            ...

        def true_anomaly_at_dn(self, target):
            # type: (SpaceCenter.Orbit) -> float
            """The true anomaly of the descending node with the given target orbit.
            :target Target orbit."""
            ...

        def true_anomaly_at_radius(self, radius):
            # type: (float) -> float
            """The true anomaly at the given orbital radius.
            :radius The orbital radius in meters."""
            ...

        def true_anomaly_at_ut(self, ut):
            # type: (float) -> float
            """The true anomaly at the given time.
            :ut The universal time in seconds."""
            ...

        def ut_at_true_anomaly(self, true_anomaly):
            # type: (float) -> float
            """The universal time, in seconds, corresponding to the given true anomaly.
            :true_anomaly True anomaly."""
            ...

        @staticmethod
        def reference_plane_direction(, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The direction from which the orbits longitude of ascending node is measured,
            in the given reference frame.
            @return The direction as a unit vector.
            :reference_frame The reference frame that the returned
            direction is in."""
            ...

        @staticmethod
        def reference_plane_normal(, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The direction that is normal to the orbits reference plane,
            in the given reference frame.
            The reference plane is the plane from which the orbits inclination is measured.
            @return The direction as a unit vector.
            :reference_frame The reference frame that the returned
            direction is in."""
            ...

    class Parachute:
        @property
        def armed(self):
            # type: () -> bool
            """Whether the parachute has been armed or deployed."""
            ...

        @property
        def deploy_altitude(self):
            # type: () -> float
            """The altitude at which the parachute will full deploy, in meters.
            Only applicable to stock parachutes."""
            ...

        @deploy_altitude.setter
        def deploy_altitude(self, value):
            # type: (float) -> None
            ...

        @property
        def deploy_min_pressure(self):
            # type: () -> float
            """The minimum pressure at which the parachute will semi-deploy, in atmospheres.
            Only applicable to stock parachutes."""
            ...

        @deploy_min_pressure.setter
        def deploy_min_pressure(self, value):
            # type: (float) -> None
            ...

        @property
        def deployed(self):
            # type: () -> bool
            """Whether the parachute has been deployed."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this parachute."""
            ...

        @property
        def state(self):
            # type: () -> SpaceCenter.ParachuteState
            """The current state of the parachute."""
            ...

        def arm(self):
            # type: () -> None
            """Deploys the parachute. This has no effect if the parachute has already
            been armed or deployed."""
            ...

        def cut(self):
            # type: () -> None
            """Cuts the parachute."""
            ...

        def deploy(self):
            # type: () -> None
            """Deploys the parachute. This has no effect if the parachute has already
            been deployed."""
            ...

    class Part:
        @property
        def antenna(self):
            # type: () -> Optional[SpaceCenter.Antenna]
            """A SpaceCenter.Antenna if the part is an antenna, otherwise {@code null}."""
            ...

        @property
        def auto_strut_mode(self):
            # type: () -> SpaceCenter.AutoStrutMode
            """Auto-strut mode."""
            ...

        @property
        def available_seats(self):
            # type: () -> int
            """How many open seats the part has."""
            ...

        @property
        def axially_attached(self):
            # type: () -> bool
            """Whether the part is axially attached to its parent, i.e. on the top
            or bottom of its parent. If the part has no parent, returns {@code false}."""
            ...

        @property
        def cargo_bay(self):
            # type: () -> Optional[SpaceCenter.CargoBay]
            """A SpaceCenter.CargoBay if the part is a cargo bay, otherwise {@code null}."""
            ...

        @property
        def center_of_mass_reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """The reference frame that is fixed relative to this part, and centered on its
            center of mass.
            <p><ul>
            <li>The origin is at the center of mass of the part, as returned by
              SpaceCenter.Part#centerOfMass.
            <li>The axes rotate with the part.
            <li>The x, y and z axis directions depend on the design of the part.
            </ul></p>

            For docking port parts, this reference frame is not necessarily equivalent to the
            reference frame for the docking port, returned by
            SpaceCenter.DockingPort#referenceFrame."""
            ...

        @property
        def children(self):
            # type: () -> List[SpaceCenter.Part]
            """The parts children. Returns an empty list if the part has no children.
            This, in combination with SpaceCenter.Part#parent, can be used to traverse the vessels
            parts tree."""
            ...

        @property
        def control_surface(self):
            # type: () -> Optional[SpaceCenter.ControlSurface]
            """A SpaceCenter.ControlSurface if the part is an aerodynamic control surface,
            otherwise {@code null}."""
            ...

        @property
        def cost(self):
            # type: () -> float
            """The cost of the part, in units of funds."""
            ...

        @property
        def crossfeed(self):
            # type: () -> bool
            """Whether this part is crossfeed capable."""
            ...

        @property
        def decouple_stage(self):
            # type: () -> int
            """The stage in which this part will be decoupled. Returns -1 if the part is never
            decoupled from the vessel."""
            ...

        @property
        def decoupler(self):
            # type: () -> Optional[SpaceCenter.Decoupler]
            """A SpaceCenter.Decoupler if the part is a decoupler, otherwise {@code null}."""
            ...

        @property
        def docking_port(self):
            # type: () -> Optional[SpaceCenter.DockingPort]
            """A SpaceCenter.DockingPort if the part is a docking port, otherwise {@code null}."""
            ...

        @property
        def dry_mass(self):
            # type: () -> float
            """The mass of the part, not including any resources it contains, in kilograms.
            Returns zero if the part is massless."""
            ...

        @property
        def dynamic_pressure(self):
            # type: () -> float
            """The dynamic pressure acting on the part, in Pascals."""
            ...

        @property
        def engine(self):
            # type: () -> Optional[SpaceCenter.Engine]
            """An SpaceCenter.Engine if the part is an engine, otherwise {@code null}."""
            ...

        @property
        def experiment(self):
            # type: () -> Optional[SpaceCenter.Experiment]
            """An SpaceCenter.Experiment if the part contains a
            single science experiment, otherwise {@code null}.

            Throws an exception if the part contains more than one experiment.
            In that case, use SpaceCenter.Part#experiments to get the list of experiments in the part."""
            ...

        @property
        def experiments(self):
            # type: () -> Optional[List[SpaceCenter.Experiment]]
            """A list of SpaceCenter.Experiment objects that the part contains."""
            ...

        @property
        def fairing(self):
            # type: () -> Optional[SpaceCenter.Fairing]
            """A SpaceCenter.Fairing if the part is a fairing, otherwise {@code null}."""
            ...

        @property
        def flag_url(self):
            # type: () -> str
            """The asset URL for the part's flag."""
            ...

        @flag_url.setter
        def flag_url(self, value):
            # type: (str) -> None
            ...

        @property
        def fuel_lines_from(self):
            # type: () -> List[SpaceCenter.Part]
            """The parts that are connected to this part via fuel lines, where the direction of the
            fuel line is into this part."""
            ...

        @property
        def fuel_lines_to(self):
            # type: () -> List[SpaceCenter.Part]
            """The parts that are connected to this part via fuel lines, where the direction of the
            fuel line is out of this part."""
            ...

        def glow(self, value):
            # type: (bool) -> None
            """Whether the part is glowing."""
            ...
        glow = property(None, glow)

        @property
        def highlight_color(self):
            # type: () -> Tuple[float,float,float]
            """The color used to highlight the part, as an RGB triple."""
            ...

        @highlight_color.setter
        def highlight_color(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def highlighted(self):
            # type: () -> bool
            """Whether the part is highlighted."""
            ...

        @highlighted.setter
        def highlighted(self, value):
            # type: (bool) -> None
            ...

        @property
        def impact_tolerance(self):
            # type: () -> float
            """The impact tolerance of the part, in meters per second."""
            ...

        @property
        def inertia_tensor(self):
            # type: () -> List[float]
            """The inertia tensor of the part in the parts reference frame
            (SpaceCenter.ReferenceFrame).
            Returns the 3x3 matrix as a list of elements, in row-major order."""
            ...

        @property
        def intake(self):
            # type: () -> Optional[SpaceCenter.Intake]
            """An SpaceCenter.Intake if the part is an intake, otherwise {@code null}.

            This includes any part that generates thrust. This covers many different types
            of engine, including liquid fuel rockets, solid rocket boosters and jet engines.
            For RCS thrusters see SpaceCenter.RCS."""
            ...

        @property
        def is_fuel_line(self):
            # type: () -> bool
            """Whether this part is a fuel line."""
            ...

        @property
        def launch_clamp(self):
            # type: () -> Optional[SpaceCenter.LaunchClamp]
            """A SpaceCenter.LaunchClamp if the part is a launch clamp, otherwise {@code null}."""
            ...

        @property
        def leg(self):
            # type: () -> Optional[SpaceCenter.Leg]
            """A SpaceCenter.Leg if the part is a landing leg, otherwise {@code null}."""
            ...

        @property
        def light(self):
            # type: () -> Optional[SpaceCenter.Light]
            """A SpaceCenter.Light if the part is a light, otherwise {@code null}."""
            ...

        @property
        def mass(self):
            # type: () -> float
            """The current mass of the part, including resources it contains, in kilograms.
            Returns zero if the part is massless."""
            ...

        @property
        def massless(self):
            # type: () -> bool
            """Whether the part is
            massless."""
            ...

        @property
        def max_skin_temperature(self):
            # type: () -> float
            """Maximum temperature that the skin of the part can survive, in Kelvin."""
            ...

        @property
        def max_temperature(self):
            # type: () -> float
            """Maximum temperature that the part can survive, in Kelvin."""
            ...

        @property
        def modules(self):
            # type: () -> List[SpaceCenter.Module]
            """The modules for this part."""
            ...

        @property
        def moment_of_inertia(self):
            # type: () -> Tuple[float,float,float]
            """The moment of inertia of the part in kg.m^2 around its center of mass
            in the parts reference frame (SpaceCenter.ReferenceFrame)."""
            ...

        @property
        def name(self):
            # type: () -> str
            """Internal name of the part, as used in
            part cfg files.
            For example "Mark1-2Pod"."""
            ...

        @property
        def parachute(self):
            # type: () -> Optional[SpaceCenter.Parachute]
            """A SpaceCenter.Parachute if the part is a parachute, otherwise {@code null}."""
            ...

        @property
        def parent(self):
            # type: () -> Optional[SpaceCenter.Part]
            """The parts parent. Returns {@code null} if the part does not have a parent.
            This, in combination with SpaceCenter.Part#children, can be used to traverse the vessels
            parts tree."""
            ...

        @property
        def rcs(self):
            # type: () -> Optional[SpaceCenter.RCS]
            """A SpaceCenter.RCS if the part is an RCS block/thruster, otherwise {@code null}."""
            ...

        @property
        def radially_attached(self):
            # type: () -> bool
            """Whether the part is radially attached to its parent, i.e. on the side of its parent.
            If the part has no parent, returns {@code false}."""
            ...

        @property
        def radiator(self):
            # type: () -> Optional[SpaceCenter.Radiator]
            """A SpaceCenter.Radiator if the part is a radiator, otherwise {@code null}."""
            ...

        @property
        def reaction_wheel(self):
            # type: () -> Optional[SpaceCenter.ReactionWheel]
            """A SpaceCenter.ReactionWheel if the part is a reaction wheel, otherwise {@code null}."""
            ...

        @property
        def reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """The reference frame that is fixed relative to this part, and centered on a fixed
            position within the part, defined by the parts model.
            <p><ul>
            <li>The origin is at the position of the part, as returned by
              SpaceCenter.Part#position.
            <li>The axes rotate with the part.
            <li>The x, y and z axis directions depend on the design of the part.
            </ul></p>

            For docking port parts, this reference frame is not necessarily equivalent to the
            reference frame for the docking port, returned by
            SpaceCenter.DockingPort#referenceFrame."""
            ...

        @property
        def resource_converter(self):
            # type: () -> Optional[SpaceCenter.ResourceConverter]
            """A SpaceCenter.ResourceConverter if the part is a resource converter,
            otherwise {@code null}."""
            ...

        @property
        def resource_drain(self):
            # type: () -> Optional[SpaceCenter.ResourceDrain]
            """A SpaceCenter.ResourceDrain if the part is a resource drain, otherwise {@code null}."""
            ...

        @property
        def resource_harvester(self):
            # type: () -> Optional[SpaceCenter.ResourceHarvester]
            """A SpaceCenter.ResourceHarvester if the part is a resource harvester,
            otherwise {@code null}."""
            ...

        @property
        def resources(self):
            # type: () -> SpaceCenter.Resources
            """A SpaceCenter.Resources object for the part."""
            ...

        @property
        def robotic_controller(self):
            # type: () -> Optional[SpaceCenter.RoboticController]
            """A SpaceCenter.RoboticController if the part is a robotic controller,
            otherwise {@code null}."""
            ...

        @property
        def robotic_hinge(self):
            # type: () -> Optional[SpaceCenter.RoboticHinge]
            """A SpaceCenter.RoboticHinge if the part is a robotic hinge, otherwise {@code null}."""
            ...

        @property
        def robotic_piston(self):
            # type: () -> Optional[SpaceCenter.RoboticPiston]
            """A SpaceCenter.RoboticPiston if the part is a robotic piston, otherwise {@code null}."""
            ...

        @property
        def robotic_rotation(self):
            # type: () -> Optional[SpaceCenter.RoboticRotation]
            """A SpaceCenter.RoboticRotation if the part is a robotic rotation servo, otherwise {@code null}."""
            ...

        @property
        def robotic_rotor(self):
            # type: () -> Optional[SpaceCenter.RoboticRotor]
            """A SpaceCenter.RoboticRotor if the part is a robotic rotor, otherwise {@code null}."""
            ...

        @property
        def sensor(self):
            # type: () -> Optional[SpaceCenter.Sensor]
            """A SpaceCenter.Sensor if the part is a sensor, otherwise {@code null}."""
            ...

        @property
        def shielded(self):
            # type: () -> bool
            """Whether the part is shielded from the exterior of the vessel, for example by a fairing."""
            ...

        @property
        def skin_temperature(self):
            # type: () -> float
            """Temperature of the skin of the part, in Kelvin."""
            ...

        @property
        def solar_panel(self):
            # type: () -> Optional[SpaceCenter.SolarPanel]
            """A SpaceCenter.SolarPanel if the part is a solar panel, otherwise {@code null}."""
            ...

        @property
        def stage(self):
            # type: () -> int
            """The stage in which this part will be activated. Returns -1 if the part is not
            activated by staging."""
            ...

        @property
        def tag(self):
            # type: () -> str
            """The name tag for the part. Can be set to a custom string using the
            in-game user interface.

            This string is shared with
            kOS
            if it is installed."""
            ...

        @tag.setter
        def tag(self, value):
            # type: (str) -> None
            ...

        @property
        def temperature(self):
            # type: () -> float
            """Temperature of the part, in Kelvin."""
            ...

        @property
        def thermal_conduction_flux(self):
            # type: () -> float
            """The rate at which heat energy is conducting into or out of the part via contact with
            other parts. Measured in energy per unit time, or power, in Watts.
            A positive value means the part is gaining heat energy, and negative means it is
            losing heat energy."""
            ...

        @property
        def thermal_convection_flux(self):
            # type: () -> float
            """The rate at which heat energy is convecting into or out of the part from the
            surrounding atmosphere. Measured in energy per unit time, or power, in Watts.
            A positive value means the part is gaining heat energy, and negative means it is
            losing heat energy."""
            ...

        @property
        def thermal_internal_flux(self):
            # type: () -> float
            """The rate at which heat energy is begin generated by the part.
            For example, some engines generate heat by combusting fuel.
            Measured in energy per unit time, or power, in Watts.
            A positive value means the part is gaining heat energy, and negative means it is losing
            heat energy."""
            ...

        @property
        def thermal_mass(self):
            # type: () -> float
            """A measure of how much energy it takes to increase the internal temperature of the part,
            in Joules per Kelvin."""
            ...

        @property
        def thermal_radiation_flux(self):
            # type: () -> float
            """The rate at which heat energy is radiating into or out of the part from the surrounding
            environment. Measured in energy per unit time, or power, in Watts.
            A positive value means the part is gaining heat energy, and negative means it is
            losing heat energy."""
            ...

        @property
        def thermal_resource_mass(self):
            # type: () -> float
            """A measure of how much energy it takes to increase the temperature of the resources
            contained in the part, in Joules per Kelvin."""
            ...

        @property
        def thermal_skin_mass(self):
            # type: () -> float
            """A measure of how much energy it takes to increase the skin temperature of the part,
            in Joules per Kelvin."""
            ...

        @property
        def thermal_skin_to_internal_flux(self):
            # type: () -> float
            """The rate at which heat energy is transferring between the part's skin and its internals.
            Measured in energy per unit time, or power, in Watts.
            A positive value means the part's internals are gaining heat energy,
            and negative means its skin is gaining heat energy."""
            ...

        @property
        def title(self):
            # type: () -> str
            """Title of the part, as shown when the part is right clicked in-game. For example "Mk1-2 Command Pod"."""
            ...

        @property
        def vessel(self):
            # type: () -> SpaceCenter.Vessel
            """The vessel that contains this part."""
            ...

        @property
        def wheel(self):
            # type: () -> Optional[SpaceCenter.Wheel]
            """A SpaceCenter.Wheel if the part is a wheel, otherwise {@code null}."""
            ...

        def add_force(self, force, position, reference_frame):
            # type: (Tuple[float,float,float], Tuple[float,float,float], SpaceCenter.ReferenceFrame) -> SpaceCenter.Force
            """Exert a constant force on the part, acting at the given position.
            @return An object that can be used to remove or modify the force.
            :force A vector pointing in the direction that the force acts,
            with its magnitude equal to the strength of the force in Newtons.
            :position The position at which the force acts, as a vector.
            :reference_frame The reference frame that the
            force and position are in."""
            ...

        def bounding_box(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[Tuple[float,float,float],Tuple[float,float,float]]
            """The axis-aligned bounding box of the part in the given reference frame.
            @return The positions of the minimum and maximum vertices of the box,
            as position vectors.
            :reference_frame The reference frame that the returned
            position vectors are in.

            This is computed from the collision mesh of the part.
            If the part is not collidable, the box has zero volume and is centered on
            the SpaceCenter.Part#position of the part."""
            ...

        def center_of_mass(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The position of the parts center of mass in the given reference frame.
            If the part is physicsless, this is equivalent to SpaceCenter.Part#position.
            @return The position as a vector.
            :reference_frame The reference frame that the returned
            position vector is in."""
            ...

        def direction(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The direction the part points in, in the given reference frame.
            @return The direction as a unit vector.
            :reference_frame The reference frame that the returned
            direction is in."""
            ...

        def instantaneous_force(self, force, position, reference_frame):
            # type: (Tuple[float,float,float], Tuple[float,float,float], SpaceCenter.ReferenceFrame) -> None
            """Exert an instantaneous force on the part, acting at the given position.
            :force A vector pointing in the direction that the force acts,
            with its magnitude equal to the strength of the force in Newtons.
            :position The position at which the force acts, as a vector.
            :reference_frame The reference frame that the
            force and position are in.

            The force is applied instantaneously in a single physics update."""
            ...

        def position(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The position of the part in the given reference frame.
            @return The position as a vector.
            :reference_frame The reference frame that the returned
            position vector is in.

            This is a fixed position in the part, defined by the parts model.
            It s not necessarily the same as the parts center of mass.
            Use SpaceCenter.Part#centerOfMass to get the parts center of mass."""
            ...

        def rotation(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float,float]
            """The rotation of the part, in the given reference frame.
            @return The rotation as a quaternion of the form (x, y, z, w).
            :reference_frame The reference frame that the returned
            rotation is in."""
            ...

        def velocity(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The linear velocity of the part in the given reference frame.
            @return The velocity as a vector. The vector points in the direction of travel,
            and its magnitude is the speed of the body in meters per second.
            :reference_frame The reference frame that the returned
            velocity vector is in."""
            ...

    class Parts:
        @property
        def all(self):
            # type: () -> List[SpaceCenter.Part]
            """A list of all of the vessels parts."""
            ...

        @property
        def antennas(self):
            # type: () -> List[SpaceCenter.Antenna]
            """A list of all antennas in the vessel."""
            ...

        @property
        def cargo_bays(self):
            # type: () -> List[SpaceCenter.CargoBay]
            """A list of all cargo bays in the vessel."""
            ...

        @property
        def control_surfaces(self):
            # type: () -> List[SpaceCenter.ControlSurface]
            """A list of all control surfaces in the vessel."""
            ...

        @property
        def controlling(self):
            # type: () -> SpaceCenter.Part
            """The part from which the vessel is controlled."""
            ...

        @controlling.setter
        def controlling(self, value):
            # type: (SpaceCenter.Part) -> None
            ...

        @property
        def decouplers(self):
            # type: () -> List[SpaceCenter.Decoupler]
            """A list of all decouplers in the vessel."""
            ...

        @property
        def docking_ports(self):
            # type: () -> List[SpaceCenter.DockingPort]
            """A list of all docking ports in the vessel."""
            ...

        @property
        def engines(self):
            # type: () -> List[SpaceCenter.Engine]
            """A list of all engines in the vessel.

            This includes any part that generates thrust. This covers many different types
            of engine, including liquid fuel rockets, solid rocket boosters, jet engines and
            RCS thrusters."""
            ...

        @property
        def experiments(self):
            # type: () -> List[SpaceCenter.Experiment]
            """A list of all science experiments in the vessel."""
            ...

        @property
        def fairings(self):
            # type: () -> List[SpaceCenter.Fairing]
            """A list of all fairings in the vessel."""
            ...

        @property
        def intakes(self):
            # type: () -> List[SpaceCenter.Intake]
            """A list of all intakes in the vessel."""
            ...

        @property
        def launch_clamps(self):
            # type: () -> List[SpaceCenter.LaunchClamp]
            """A list of all launch clamps attached to the vessel."""
            ...

        @property
        def legs(self):
            # type: () -> List[SpaceCenter.Leg]
            """A list of all landing legs attached to the vessel."""
            ...

        @property
        def lights(self):
            # type: () -> List[SpaceCenter.Light]
            """A list of all lights in the vessel."""
            ...

        @property
        def parachutes(self):
            # type: () -> List[SpaceCenter.Parachute]
            """A list of all parachutes in the vessel."""
            ...

        @property
        def rcs(self):
            # type: () -> List[SpaceCenter.RCS]
            """A list of all RCS blocks/thrusters in the vessel."""
            ...

        @property
        def radiators(self):
            # type: () -> List[SpaceCenter.Radiator]
            """A list of all radiators in the vessel."""
            ...

        @property
        def reaction_wheels(self):
            # type: () -> List[SpaceCenter.ReactionWheel]
            """A list of all reaction wheels in the vessel."""
            ...

        @property
        def resource_converters(self):
            # type: () -> List[SpaceCenter.ResourceConverter]
            """A list of all resource converters in the vessel."""
            ...

        @property
        def resource_drains(self):
            # type: () -> List[SpaceCenter.ResourceDrain]
            """A list of all resource drains in the vessel."""
            ...

        @property
        def resource_harvesters(self):
            # type: () -> List[SpaceCenter.ResourceHarvester]
            """A list of all resource harvesters in the vessel."""
            ...

        @property
        def robotic_hinges(self):
            # type: () -> List[SpaceCenter.RoboticHinge]
            """A list of all robotic hinges in the vessel."""
            ...

        @property
        def robotic_pistons(self):
            # type: () -> List[SpaceCenter.RoboticPiston]
            """A list of all robotic pistons in the vessel."""
            ...

        @property
        def robotic_rotations(self):
            # type: () -> List[SpaceCenter.RoboticRotation]
            """A list of all robotic rotations in the vessel."""
            ...

        @property
        def robotic_rotors(self):
            # type: () -> List[SpaceCenter.RoboticRotor]
            """A list of all robotic rotors in the vessel."""
            ...

        @property
        def root(self):
            # type: () -> SpaceCenter.Part
            """The vessels root part."""
            ...

        @property
        def sensors(self):
            # type: () -> List[SpaceCenter.Sensor]
            """A list of all sensors in the vessel."""
            ...

        @property
        def solar_panels(self):
            # type: () -> List[SpaceCenter.SolarPanel]
            """A list of all solar panels in the vessel."""
            ...

        @property
        def wheels(self):
            # type: () -> List[SpaceCenter.Wheel]
            """A list of all wheels in the vessel."""
            ...

        def in_decouple_stage(self, stage):
            # type: (int) -> List[SpaceCenter.Part]
            """A list of all parts that are decoupled in the given stage.
            :stage """
            ...

        def in_stage(self, stage):
            # type: (int) -> List[SpaceCenter.Part]
            """A list of all parts that are activated in the given stage.
            :stage """
            ...

        def modules_with_name(self, module_name):
            # type: (str) -> List[SpaceCenter.Module]
            """A list of modules (combined across all parts in the vessel) whose
            SpaceCenter.Module#name is module_name.
            :module_name """
            ...

        def with_module(self, module_name):
            # type: (str) -> List[SpaceCenter.Part]
            """A list of all parts that contain a SpaceCenter.Module whose
            SpaceCenter.Module#name is module_name.
            :module_name """
            ...

        def with_name(self, name):
            # type: (str) -> List[SpaceCenter.Part]
            """A list of parts whose SpaceCenter.Part#name is name.
            :name """
            ...

        def with_tag(self, tag):
            # type: (str) -> List[SpaceCenter.Part]
            """A list of all parts whose SpaceCenter.Part#tag is tag.
            :tag """
            ...

        def with_title(self, title):
            # type: (str) -> List[SpaceCenter.Part]
            """A list of all parts whose SpaceCenter.Part#title is title.
            :title """
            ...

    class Propellant:
        @property
        def current_amount(self):
            # type: () -> float
            """The current amount of propellant."""
            ...

        @property
        def current_requirement(self):
            # type: () -> float
            """The required amount of propellant."""
            ...

        @property
        def draw_stack_gauge(self):
            # type: () -> bool
            """If this propellant has a stack gauge or not."""
            ...

        @property
        def ignore_for_isp(self):
            # type: () -> bool
            """If this propellant should be ignored when calculating required mass flow
            given specific impulse."""
            ...

        @property
        def ignore_for_thrust_curve(self):
            # type: () -> bool
            """If this propellant should be ignored for thrust curve calculations."""
            ...

        @property
        def is_deprived(self):
            # type: () -> bool
            """If this propellant is deprived."""
            ...

        @property
        def name(self):
            # type: () -> str
            """The name of the propellant."""
            ...

        @property
        def ratio(self):
            # type: () -> float
            """The propellant ratio."""
            ...

        @property
        def total_resource_available(self):
            # type: () -> float
            """The total amount of the underlying resource currently reachable given
            resource flow rules."""
            ...

        @property
        def total_resource_capacity(self):
            # type: () -> float
            """The total vehicle capacity for the underlying propellant resource,
            restricted by resource flow rules."""
            ...

    class RCS:
        @property
        def active(self):
            # type: () -> bool
            """Whether the RCS thrusters are active.
            An RCS thruster is inactive if the RCS action group is disabled
            (SpaceCenter.Control#rCS), the RCS thruster itself is not enabled
            (SpaceCenter.RCS#enabled) or it is covered by a fairing (SpaceCenter.Part#shielded)."""
            ...

        @property
        def available_force(self):
            # type: () -> Tuple[Tuple[float,float,float],Tuple[float,float,float]]
            """The available force, in Newtons, that can be produced by this RCS,
            in the positive and negative x, y and z axes of the vessel. These axes
            correspond to the coordinate axes of the SpaceCenter.Vessel#referenceFrame.
            Returns zero if RCS is disabled."""
            ...

        @property
        def available_thrust(self):
            # type: () -> float
            """The amount of thrust, in Newtons, that would be produced by the thruster when activated.
            Returns zero if the thruster does not have any fuel.
            Takes the thrusters current SpaceCenter.RCS#thrustLimit and atmospheric conditions
            into account."""
            ...

        @property
        def available_torque(self):
            # type: () -> Tuple[Tuple[float,float,float],Tuple[float,float,float]]
            """The available torque, in Newton meters, that can be produced by this RCS,
            in the positive and negative pitch, roll and yaw axes of the vessel. These axes
            correspond to the coordinate axes of the SpaceCenter.Vessel#referenceFrame.
            Returns zero if RCS is disable."""
            ...

        @property
        def enabled(self):
            # type: () -> bool
            """Whether the RCS thrusters are enabled."""
            ...

        @enabled.setter
        def enabled(self, value):
            # type: (bool) -> None
            ...

        @property
        def forward_enabled(self):
            # type: () -> bool
            """Whether the RCS thruster will fire when pitch control input is given."""
            ...

        @forward_enabled.setter
        def forward_enabled(self, value):
            # type: (bool) -> None
            ...

        @property
        def has_fuel(self):
            # type: () -> bool
            """Whether the RCS has fuel available."""
            ...

        @property
        def kerbin_sea_level_specific_impulse(self):
            # type: () -> float
            """The specific impulse of the RCS at sea level on Kerbin, in seconds."""
            ...

        @property
        def max_thrust(self):
            # type: () -> float
            """The maximum amount of thrust that can be produced by the RCS thrusters when active,
            in Newtons.
            Takes the thrusters current SpaceCenter.RCS#thrustLimit and atmospheric conditions
            into account."""
            ...

        @property
        def max_vacuum_thrust(self):
            # type: () -> float
            """The maximum amount of thrust that can be produced by the RCS thrusters when active
            in a vacuum, in Newtons."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this RCS."""
            ...

        @property
        def pitch_enabled(self):
            # type: () -> bool
            """Whether the RCS thruster will fire when pitch control input is given."""
            ...

        @pitch_enabled.setter
        def pitch_enabled(self, value):
            # type: (bool) -> None
            ...

        @property
        def propellant_ratios(self):
            # type: () -> Dict[str, float]
            """The ratios of resources that the RCS consumes. A dictionary mapping resource names
            to the ratios at which they are consumed by the RCS."""
            ...

        @property
        def propellants(self):
            # type: () -> List[str]
            """The names of resources that the RCS consumes."""
            ...

        @property
        def right_enabled(self):
            # type: () -> bool
            """Whether the RCS thruster will fire when roll control input is given."""
            ...

        @right_enabled.setter
        def right_enabled(self, value):
            # type: (bool) -> None
            ...

        @property
        def roll_enabled(self):
            # type: () -> bool
            """Whether the RCS thruster will fire when roll control input is given."""
            ...

        @roll_enabled.setter
        def roll_enabled(self, value):
            # type: (bool) -> None
            ...

        @property
        def specific_impulse(self):
            # type: () -> float
            """The current specific impulse of the RCS, in seconds. Returns zero
            if the RCS is not active."""
            ...

        @property
        def thrust_limit(self):
            # type: () -> float
            """The thrust limiter of the thruster. A value between 0 and 1."""
            ...

        @thrust_limit.setter
        def thrust_limit(self, value):
            # type: (float) -> None
            ...

        @property
        def thrusters(self):
            # type: () -> List[SpaceCenter.Thruster]
            """A list of thrusters, one of each nozzel in the RCS part."""
            ...

        @property
        def up_enabled(self):
            # type: () -> bool
            """Whether the RCS thruster will fire when yaw control input is given."""
            ...

        @up_enabled.setter
        def up_enabled(self, value):
            # type: (bool) -> None
            ...

        @property
        def vacuum_specific_impulse(self):
            # type: () -> float
            """The vacuum specific impulse of the RCS, in seconds."""
            ...

        @property
        def yaw_enabled(self):
            # type: () -> bool
            """Whether the RCS thruster will fire when yaw control input is given."""
            ...

        @yaw_enabled.setter
        def yaw_enabled(self, value):
            # type: (bool) -> None
            ...

    class Radiator:
        @property
        def deployable(self):
            # type: () -> bool
            """Whether the radiator is deployable."""
            ...

        @property
        def deployed(self):
            # type: () -> bool
            """For a deployable radiator, {@code true} if the radiator is extended.
            If the radiator is not deployable, this is always {@code true}."""
            ...

        @deployed.setter
        def deployed(self, value):
            # type: (bool) -> None
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this radiator."""
            ...

        @property
        def state(self):
            # type: () -> SpaceCenter.RadiatorState
            """The current state of the radiator.

            A fixed radiator is always SpaceCenter.RadiatorState#extended."""
            ...

    class ReactionWheel:
        @property
        def active(self):
            # type: () -> bool
            """Whether the reaction wheel is active."""
            ...

        @active.setter
        def active(self, value):
            # type: (bool) -> None
            ...

        @property
        def available_torque(self):
            # type: () -> Tuple[Tuple[float,float,float],Tuple[float,float,float]]
            """The available torque, in Newton meters, that can be produced by this reaction wheel,
            in the positive and negative pitch, roll and yaw axes of the vessel. These axes
            correspond to the coordinate axes of the SpaceCenter.Vessel#referenceFrame.
            Returns zero if the reaction wheel is inactive or broken."""
            ...

        @property
        def broken(self):
            # type: () -> bool
            """Whether the reaction wheel is broken."""
            ...

        @property
        def max_torque(self):
            # type: () -> Tuple[Tuple[float,float,float],Tuple[float,float,float]]
            """The maximum torque, in Newton meters, that can be produced by this reaction wheel,
            when it is active, in the positive and negative pitch, roll and yaw axes of the vessel.
            These axes correspond to the coordinate axes of the SpaceCenter.Vessel#referenceFrame."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this reaction wheel."""
            ...

    class ReferenceFrame:
        @staticmethod
        def create_hybrid(, position, rotation=None, velocity=None, angular_velocity=None):
            # type: (SpaceCenter.ReferenceFrame, SpaceCenter.ReferenceFrame, SpaceCenter.ReferenceFrame, SpaceCenter.ReferenceFrame) -> SpaceCenter.ReferenceFrame
            """Create a hybrid reference frame. This is a custom reference frame
            whose components inherited from other reference frames.
            :position The reference frame providing the position of the origin.
            :rotation The reference frame providing the rotation of the frame.
            :velocity The reference frame providing the linear velocity of the frame.
            :angular_velocity The reference frame providing the angular velocity
            of the frame.

            The position reference frame is required but all other
            reference frames are optional. If omitted, they are set to the
            position reference frame."""
            ...

        @staticmethod
        def create_relative(, reference_frame, position=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0), velocity=(0.0, 0.0, 0.0), angular_velocity=(0.0, 0.0, 0.0)):
            # type: (SpaceCenter.ReferenceFrame, Tuple[float,float,float], Tuple[float,float,float,float], Tuple[float,float,float], Tuple[float,float,float]) -> SpaceCenter.ReferenceFrame
            """Create a relative reference frame. This is a custom reference frame
            whose components offset the components of a parent reference frame.
            :reference_frame The parent reference frame on which to
            base this reference frame.
            :position The offset of the position of the origin,
            as a position vector. Defaults to (0, 0, 0)
            :rotation The rotation to apply to the parent frames rotation,
            as a quaternion of the form (x, y, z, w).
            Defaults to (0, 0, 0, 1) (i.e. no rotation)
            :velocity The linear velocity to offset the parent frame by,
            as a vector pointing in the direction of travel, whose magnitude is the speed in
            meters per second. Defaults to (0, 0, 0).
            :angular_velocity The angular velocity to offset the parent frame by,
            as a vector. This vector points in the direction of the axis of rotation,
            and its magnitude is the speed of the rotation in radians per second.
            Defaults to (0, 0, 0)."""
            ...

    class Resource:
        @property
        def amount(self):
            # type: () -> float
            """The amount of the resource that is currently stored in the part."""
            ...

        @property
        def density(self):
            # type: () -> float
            """The density of the resource, in kg/l."""
            ...

        @property
        def enabled(self):
            # type: () -> bool
            """Whether use of this resource is enabled."""
            ...

        @enabled.setter
        def enabled(self, value):
            # type: (bool) -> None
            ...

        @property
        def flow_mode(self):
            # type: () -> SpaceCenter.ResourceFlowMode
            """The flow mode of the resource."""
            ...

        @property
        def max(self):
            # type: () -> float
            """The total amount of the resource that can be stored in the part."""
            ...

        @property
        def name(self):
            # type: () -> str
            """The name of the resource."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part containing the resource."""
            ...

    class ResourceConverter:
        @property
        def core_temperature(self):
            # type: () -> float
            """The core temperature of the converter, in Kelvin."""
            ...

        @property
        def count(self):
            # type: () -> int
            """The number of converters in the part."""
            ...

        @property
        def optimum_core_temperature(self):
            # type: () -> float
            """The core temperature at which the converter will operate with peak efficiency, in Kelvin."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this converter."""
            ...

        @property
        def thermal_efficiency(self):
            # type: () -> float
            """The thermal efficiency of the converter, as a percentage of its maximum."""
            ...

        def active(self, index):
            # type: (int) -> bool
            """True if the specified converter is active.
            :index Index of the converter."""
            ...

        def inputs(self, index):
            # type: (int) -> List[str]
            """List of the names of resources consumed by the specified converter.
            :index Index of the converter."""
            ...

        def name(self, index):
            # type: (int) -> str
            """The name of the specified converter.
            :index Index of the converter."""
            ...

        def outputs(self, index):
            # type: (int) -> List[str]
            """List of the names of resources produced by the specified converter.
            :index Index of the converter."""
            ...

        def start(self, index):
            # type: (int) -> None
            """Start the specified converter.
            :index Index of the converter."""
            ...

        def state(self, index):
            # type: (int) -> SpaceCenter.ResourceConverterState
            """The state of the specified converter.
            :index Index of the converter."""
            ...

        def status_info(self, index):
            # type: (int) -> str
            """Status information for the specified converter.
            This is the full status message shown in the in-game UI.
            :index Index of the converter."""
            ...

        def stop(self, index):
            # type: (int) -> None
            """Stop the specified converter.
            :index Index of the converter."""
            ...

    class ResourceDrain:
        @property
        def available_resources(self):
            # type: () -> List[SpaceCenter.Resource]
            """List of available resources."""
            ...

        @property
        def drain_mode(self):
            # type: () -> SpaceCenter.DrainMode
            """The drain mode."""
            ...

        @drain_mode.setter
        def drain_mode(self, value):
            # type: (SpaceCenter.DrainMode) -> None
            ...

        @property
        def max_rate(self):
            # type: () -> float
            """Maximum possible drain rate."""
            ...

        @property
        def min_rate(self):
            # type: () -> float
            """Minimum possible drain rate"""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this resource drain."""
            ...

        @property
        def rate(self):
            # type: () -> float
            """Current drain rate."""
            ...

        @rate.setter
        def rate(self, value):
            # type: (float) -> None
            ...

        def check_resource(self, resource):
            # type: (SpaceCenter.Resource) -> bool
            """Whether the provided resource is enabled for draining."""
            ...

        def set_resource(self, resource, enabled):
            # type: (SpaceCenter.Resource, bool) -> None
            """Whether the given resource should be drained."""
            ...

        def start(self):
            # type: () -> None
            """Activates resource draining for all enabled parts."""
            ...

        def stop(self):
            # type: () -> None
            """Turns off resource draining."""
            ...

    class ResourceHarvester:
        @property
        def active(self):
            # type: () -> bool
            """Whether the harvester is actively drilling."""
            ...

        @active.setter
        def active(self, value):
            # type: (bool) -> None
            ...

        @property
        def core_temperature(self):
            # type: () -> float
            """The core temperature of the drill, in Kelvin."""
            ...

        @property
        def deployed(self):
            # type: () -> bool
            """Whether the harvester is deployed."""
            ...

        @deployed.setter
        def deployed(self, value):
            # type: (bool) -> None
            ...

        @property
        def extraction_rate(self):
            # type: () -> float
            """The rate at which the drill is extracting ore, in units per second."""
            ...

        @property
        def optimum_core_temperature(self):
            # type: () -> float
            """The core temperature at which the drill will operate with peak efficiency, in Kelvin."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this harvester."""
            ...

        @property
        def state(self):
            # type: () -> SpaceCenter.ResourceHarvesterState
            """The state of the harvester."""
            ...

        @property
        def thermal_efficiency(self):
            # type: () -> float
            """The thermal efficiency of the drill, as a percentage of its maximum."""
            ...

    class ResourceTransfer:
        @property
        def amount(self):
            # type: () -> float
            """The amount of the resource that has been transferred."""
            ...

        @property
        def complete(self):
            # type: () -> bool
            """Whether the transfer has completed."""
            ...

        @staticmethod
        def start(, from_part, to_part, resource, max_amount):
            # type: (SpaceCenter.Part, SpaceCenter.Part, str, float) -> SpaceCenter.ResourceTransfer
            """Start transferring a resource transfer between a pair of parts. The transfer will move
            at most max_amount units of the resource, depending on how much of
            the resource is available in the source part and how much storage is available in the
            destination part.
            Use SpaceCenter.ResourceTransfer#complete to check if the transfer is complete.
            Use SpaceCenter.ResourceTransfer#amount to see how much of the resource has been transferred.
            :from_part The part to transfer to.
            :to_part The part to transfer from.
            :resource The name of the resource to transfer.
            :max_amount The maximum amount of resource to transfer."""
            ...

    class Resources:
        @property
        def all(self):
            # type: () -> List[SpaceCenter.Resource]
            """All the individual resources that can be stored."""
            ...

        @property
        def enabled(self):
            # type: () -> bool
            """Whether use of all the resources are enabled.

            This is {@code true} if all of the resources are enabled.
            If any of the resources are not enabled, this is {@code false}."""
            ...

        @enabled.setter
        def enabled(self, value):
            # type: (bool) -> None
            ...

        @property
        def names(self):
            # type: () -> List[str]
            """A list of resource names that can be stored."""
            ...

        def amount(self, name):
            # type: (str) -> float
            """Returns the amount of a resource that is currently stored.
            :name The name of the resource."""
            ...

        def has_resource(self, name):
            # type: (str) -> bool
            """Check whether the named resource can be stored.
            :name The name of the resource."""
            ...

        def max(self, name):
            # type: (str) -> float
            """Returns the amount of a resource that can be stored.
            :name The name of the resource."""
            ...

        def with_resource(self, name):
            # type: (str) -> List[SpaceCenter.Resource]
            """All the individual resources with the given name that can be stored."""
            ...

        @staticmethod
        def density(, name):
            # type: (str) -> float
            """Returns the density of a resource, in kg/l.
            :name The name of the resource."""
            ...

        @staticmethod
        def flow_mode(, name):
            # type: (str) -> SpaceCenter.ResourceFlowMode
            """Returns the flow mode of a resource.
            :name The name of the resource."""
            ...

    class RoboticController:
        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this controller."""
            ...

        def add_axis(self, module, field_name):
            # type: (SpaceCenter.Module, str) -> bool
            """Add an axis to the controller.
            @return Returns {@code true} if the axis is added successfully."""
            ...

        def add_key_frame(self, module, field_name, time, value):
            # type: (SpaceCenter.Module, str, float, float) -> bool
            """Add key frame value for controller axis.
            @return Returns {@code true} if the key frame is added successfully."""
            ...

        def axes(self):
            # type: () -> List[List[str]]
            """The axes for the controller."""
            ...

        def clear_axis(self, module, field_name):
            # type: (SpaceCenter.Module, str) -> bool
            """Clear axis.
            @return Returns {@code true} if the axis is cleared successfully."""
            ...

        def has_part(self, part):
            # type: (SpaceCenter.Part) -> bool
            """Whether the controller has a part."""
            ...

    class RoboticHinge:
        @property
        def current_angle(self):
            # type: () -> float
            """Current angle."""
            ...

        @property
        def damping(self):
            # type: () -> float
            """Damping percentage."""
            ...

        @damping.setter
        def damping(self, value):
            # type: (float) -> None
            ...

        @property
        def locked(self):
            # type: () -> bool
            """Lock movement."""
            ...

        @locked.setter
        def locked(self, value):
            # type: (bool) -> None
            ...

        @property
        def motor_engaged(self):
            # type: () -> bool
            """Whether the motor is engaged."""
            ...

        @motor_engaged.setter
        def motor_engaged(self, value):
            # type: (bool) -> None
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this robotic hinge."""
            ...

        @property
        def rate(self):
            # type: () -> float
            """Target movement rate in degrees per second."""
            ...

        @rate.setter
        def rate(self, value):
            # type: (float) -> None
            ...

        @property
        def target_angle(self):
            # type: () -> float
            """Target angle."""
            ...

        @target_angle.setter
        def target_angle(self, value):
            # type: (float) -> None
            ...

        def move_home(self):
            # type: () -> None
            """Move hinge to it's built position."""
            ...

    class RoboticPiston:
        @property
        def current_extension(self):
            # type: () -> float
            """Current extension of the piston."""
            ...

        @property
        def damping(self):
            # type: () -> float
            """Damping percentage."""
            ...

        @damping.setter
        def damping(self, value):
            # type: (float) -> None
            ...

        @property
        def locked(self):
            # type: () -> bool
            """Lock movement."""
            ...

        @locked.setter
        def locked(self, value):
            # type: (bool) -> None
            ...

        @property
        def motor_engaged(self):
            # type: () -> bool
            """Whether the motor is engaged."""
            ...

        @motor_engaged.setter
        def motor_engaged(self, value):
            # type: (bool) -> None
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this robotic piston."""
            ...

        @property
        def rate(self):
            # type: () -> float
            """Target movement rate in degrees per second."""
            ...

        @rate.setter
        def rate(self, value):
            # type: (float) -> None
            ...

        @property
        def target_extension(self):
            # type: () -> float
            """Target extension of the piston."""
            ...

        @target_extension.setter
        def target_extension(self, value):
            # type: (float) -> None
            ...

        def move_home(self):
            # type: () -> None
            """Move piston to it's built position."""
            ...

    class RoboticRotation:
        @property
        def current_angle(self):
            # type: () -> float
            """Current angle."""
            ...

        @property
        def damping(self):
            # type: () -> float
            """Damping percentage."""
            ...

        @damping.setter
        def damping(self, value):
            # type: (float) -> None
            ...

        @property
        def locked(self):
            # type: () -> bool
            """Lock Movement"""
            ...

        @locked.setter
        def locked(self, value):
            # type: (bool) -> None
            ...

        @property
        def motor_engaged(self):
            # type: () -> bool
            """Whether the motor is engaged."""
            ...

        @motor_engaged.setter
        def motor_engaged(self, value):
            # type: (bool) -> None
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this robotic rotation servo."""
            ...

        @property
        def rate(self):
            # type: () -> float
            """Target movement rate in degrees per second."""
            ...

        @rate.setter
        def rate(self, value):
            # type: (float) -> None
            ...

        @property
        def target_angle(self):
            # type: () -> float
            """Target angle."""
            ...

        @target_angle.setter
        def target_angle(self, value):
            # type: (float) -> None
            ...

        def move_home(self):
            # type: () -> None
            """Move rotation servo to it's built position."""
            ...

    class RoboticRotor:
        @property
        def current_rpm(self):
            # type: () -> float
            """Current RPM."""
            ...

        @property
        def inverted(self):
            # type: () -> bool
            """Whether the rotor direction is inverted."""
            ...

        @inverted.setter
        def inverted(self, value):
            # type: (bool) -> None
            ...

        @property
        def locked(self):
            # type: () -> bool
            """Lock movement."""
            ...

        @locked.setter
        def locked(self, value):
            # type: (bool) -> None
            ...

        @property
        def motor_engaged(self):
            # type: () -> bool
            """Whether the motor is engaged."""
            ...

        @motor_engaged.setter
        def motor_engaged(self, value):
            # type: (bool) -> None
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this robotic rotor."""
            ...

        @property
        def target_rpm(self):
            # type: () -> float
            """Target RPM."""
            ...

        @target_rpm.setter
        def target_rpm(self, value):
            # type: (float) -> None
            ...

        @property
        def torque_limit(self):
            # type: () -> float
            """Torque limit percentage."""
            ...

        @torque_limit.setter
        def torque_limit(self, value):
            # type: (float) -> None
            ...

    class ScienceData:
        @property
        def data_amount(self):
            # type: () -> float
            """Data amount."""
            ...

        @property
        def science_value(self):
            # type: () -> float
            """Science value."""
            ...

        @property
        def transmit_value(self):
            # type: () -> float
            """Transmit value."""
            ...

    class ScienceSubject:
        @property
        def data_scale(self):
            # type: () -> float
            """Multiply science value by this to determine data amount in mits."""
            ...

        @property
        def is_complete(self):
            # type: () -> bool
            """Whether the experiment has been completed."""
            ...

        @property
        def science(self):
            # type: () -> float
            """Amount of science already earned from this subject, not updated until after
            transmission/recovery."""
            ...

        @property
        def science_cap(self):
            # type: () -> float
            """Total science allowable for this subject."""
            ...

        @property
        def scientific_value(self):
            # type: () -> float
            """Diminishing value multiplier for decreasing the science value returned from repeated
            experiments."""
            ...

        @property
        def subject_value(self):
            # type: () -> float
            """Multiplier for specific Celestial Body/Experiment Situation combination."""
            ...

        @property
        def title(self):
            # type: () -> str
            """Title of science subject, displayed in science archives"""
            ...

    class Sensor:
        @property
        def active(self):
            # type: () -> bool
            """Whether the sensor is active."""
            ...

        @active.setter
        def active(self, value):
            # type: (bool) -> None
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this sensor."""
            ...

        @property
        def value(self):
            # type: () -> str
            """The current value of the sensor."""
            ...

    class SolarPanel:
        @property
        def deployable(self):
            # type: () -> bool
            """Whether the solar panel is deployable."""
            ...

        @property
        def deployed(self):
            # type: () -> bool
            """Whether the solar panel is extended."""
            ...

        @deployed.setter
        def deployed(self, value):
            # type: (bool) -> None
            ...

        @property
        def energy_flow(self):
            # type: () -> float
            """The current amount of energy being generated by the solar panel, in
            units of charge per second."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this solar panel."""
            ...

        @property
        def state(self):
            # type: () -> SpaceCenter.SolarPanelState
            """The current state of the solar panel."""
            ...

        @property
        def sun_exposure(self):
            # type: () -> float
            """The current amount of sunlight that is incident on the solar panel,
            as a percentage. A value between 0 and 1."""
            ...

    class Thruster:
        @property
        def gimbal_angle(self):
            # type: () -> Tuple[float,float,float]
            """The current gimbal angle in the pitch, roll and yaw axes, in degrees."""
            ...

        @property
        def gimballed(self):
            # type: () -> bool
            """Whether the thruster is gimballed."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The SpaceCenter.Part that contains this thruster."""
            ...

        @property
        def thrust_reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """A reference frame that is fixed relative to the thruster and orientated with
            its thrust direction (SpaceCenter.Thruster#thrustDirection).
            For gimballed engines, this takes into account the current rotation of the gimbal.
            <p><ul>
            <li>The origin is at the position of thrust for this thruster
              (SpaceCenter.Thruster#thrustPosition).
            <li>The axes rotate with the thrust direction.
              This is the direction in which the thruster expels propellant, including any gimballing.
            <li>The y-axis points along the thrust direction.
            <li>The x-axis and z-axis are perpendicular to the thrust direction.
            </ul></p>"""
            ...

        def gimbal_position(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """Position around which the gimbal pivots.
            @return The position as a vector.
            :reference_frame The reference frame that the returned
            position vector is in."""
            ...

        def initial_thrust_direction(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The direction of the force generated by the thruster, when the engine is in its
            initial position (no gimballing), in the given reference frame.
            This is opposite to the direction in which the thruster expels propellant.
            @return The direction as a unit vector.
            :reference_frame The reference frame that the returned
            direction is in."""
            ...

        def initial_thrust_position(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The position at which the thruster generates thrust, when the engine is in its
            initial position (no gimballing), in the given reference frame.
            @return The position as a vector.
            :reference_frame The reference frame that the returned
            position vector is in.

            This position can move when the gimbal rotates. This is because the thrust position and
            gimbal position are not necessarily the same."""
            ...

        def thrust_direction(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The direction of the force generated by the thruster, in the given reference frame.
            This is opposite to the direction in which the thruster expels propellant.
            For gimballed engines, this takes into account the current rotation of the gimbal.
            @return The direction as a unit vector.
            :reference_frame The reference frame that the returned
            direction is in."""
            ...

        def thrust_position(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The position at which the thruster generates thrust, in the given reference frame.
            For gimballed engines, this takes into account the current rotation of the gimbal.
            @return The position as a vector.
            :reference_frame The reference frame that the returned
            position vector is in."""
            ...

    class Vessel:
        @property
        def auto_pilot(self):
            # type: () -> SpaceCenter.AutoPilot
            """An SpaceCenter.AutoPilot object, that can be used to perform
            simple auto-piloting of the vessel."""
            ...

        @property
        def available_control_surface_torque(self):
            # type: () -> Tuple[Tuple[float,float,float],Tuple[float,float,float]]
            """The maximum torque that the aerodynamic control surfaces can generate.
            Returns the torques in N.m around each of the coordinate axes of the
            vessels reference frame (SpaceCenter.ReferenceFrame).
            These axes are equivalent to the pitch, roll and yaw axes of the vessel."""
            ...

        @property
        def available_engine_torque(self):
            # type: () -> Tuple[Tuple[float,float,float],Tuple[float,float,float]]
            """The maximum torque that the currently active and gimballed engines can generate.
            Returns the torques in N.m around each of the coordinate axes of the
            vessels reference frame (SpaceCenter.ReferenceFrame).
            These axes are equivalent to the pitch, roll and yaw axes of the vessel."""
            ...

        @property
        def available_other_torque(self):
            # type: () -> Tuple[Tuple[float,float,float],Tuple[float,float,float]]
            """The maximum torque that parts (excluding reaction wheels, gimballed engines,
            RCS and control surfaces) can generate.
            Returns the torques in N.m around each of the coordinate axes of the
            vessels reference frame (SpaceCenter.ReferenceFrame).
            These axes are equivalent to the pitch, roll and yaw axes of the vessel."""
            ...

        @property
        def available_rcs_force(self):
            # type: () -> Tuple[Tuple[float,float,float],Tuple[float,float,float]]
            """The maximum force that the currently active RCS thrusters can generate.
            Returns the forces in N along each of the coordinate axes of the
            vessels reference frame (SpaceCenter.ReferenceFrame).
            These axes are equivalent to the right, forward and bottom directions of the vessel."""
            ...

        @property
        def available_rcs_torque(self):
            # type: () -> Tuple[Tuple[float,float,float],Tuple[float,float,float]]
            """The maximum torque that the currently active RCS thrusters can generate.
            Returns the torques in N.m around each of the coordinate axes of the
            vessels reference frame (SpaceCenter.ReferenceFrame).
            These axes are equivalent to the pitch, roll and yaw axes of the vessel."""
            ...

        @property
        def available_reaction_wheel_torque(self):
            # type: () -> Tuple[Tuple[float,float,float],Tuple[float,float,float]]
            """The maximum torque that the currently active and powered reaction wheels can generate.
            Returns the torques in N.m around each of the coordinate axes of the
            vessels reference frame (SpaceCenter.ReferenceFrame).
            These axes are equivalent to the pitch, roll and yaw axes of the vessel."""
            ...

        @property
        def available_thrust(self):
            # type: () -> float
            """Gets the total available thrust that can be produced by the vessel's
            active engines, in Newtons. This is computed by summing
            SpaceCenter.Engine#availableThrust for every active engine in the vessel."""
            ...

        @property
        def available_torque(self):
            # type: () -> Tuple[Tuple[float,float,float],Tuple[float,float,float]]
            """The maximum torque that the vessel generates. Includes contributions from
            reaction wheels, RCS, gimballed engines and aerodynamic control surfaces.
            Returns the torques in N.m around each of the coordinate axes of the
            vessels reference frame (SpaceCenter.ReferenceFrame).
            These axes are equivalent to the pitch, roll and yaw axes of the vessel."""
            ...

        @property
        def biome(self):
            # type: () -> str
            """The name of the biome the vessel is currently in."""
            ...

        @property
        def comms(self):
            # type: () -> SpaceCenter.Comms
            """Returns a SpaceCenter.Comms object that can be used to interact
            with CommNet for this vessel."""
            ...

        @property
        def control(self):
            # type: () -> SpaceCenter.Control
            """Returns a SpaceCenter.Control object that can be used to manipulate
            the vessel's control inputs. For example, its pitch/yaw/roll controls,
            RCS and thrust."""
            ...

        @property
        def crew(self):
            # type: () -> List[SpaceCenter.CrewMember]
            """The crew in the vessel."""
            ...

        @property
        def crew_capacity(self):
            # type: () -> int
            """The number of crew that can occupy the vessel."""
            ...

        @property
        def crew_count(self):
            # type: () -> int
            """The number of crew that are occupying the vessel."""
            ...

        @property
        def dry_mass(self):
            # type: () -> float
            """The total mass of the vessel, excluding resources, in kg."""
            ...

        @property
        def inertia_tensor(self):
            # type: () -> List[float]
            """The inertia tensor of the vessel around its center of mass,
            in the vessels reference frame (SpaceCenter.ReferenceFrame).
            Returns the 3x3 matrix as a list of elements, in row-major order."""
            ...

        @property
        def kerbin_sea_level_specific_impulse(self):
            # type: () -> float
            """The combined specific impulse of all active engines at sea level on Kerbin, in seconds.
            This is computed using the formula
            described here."""
            ...

        @property
        def met(self):
            # type: () -> float
            """The mission elapsed time in seconds."""
            ...

        @property
        def mass(self):
            # type: () -> float
            """The total mass of the vessel, including resources, in kg."""
            ...

        @property
        def max_thrust(self):
            # type: () -> float
            """The total maximum thrust that can be produced by the vessel's active
            engines, in Newtons. This is computed by summing
            SpaceCenter.Engine#maxThrust for every active engine."""
            ...

        @property
        def max_vacuum_thrust(self):
            # type: () -> float
            """The total maximum thrust that can be produced by the vessel's active
            engines when the vessel is in a vacuum, in Newtons. This is computed by
            summing SpaceCenter.Engine#maxVacuumThrust for every active engine."""
            ...

        @property
        def moment_of_inertia(self):
            # type: () -> Tuple[float,float,float]
            """The moment of inertia of the vessel around its center of mass in kg.m^2.
            The inertia values in the returned 3-tuple are around the
            pitch, roll and yaw directions respectively.
            This corresponds to the vessels reference frame (SpaceCenter.ReferenceFrame)."""
            ...

        @property
        def name(self):
            # type: () -> str
            """The name of the vessel."""
            ...

        @name.setter
        def name(self, value):
            # type: (str) -> None
            ...

        @property
        def orbit(self):
            # type: () -> SpaceCenter.Orbit
            """The current orbit of the vessel."""
            ...

        @property
        def orbital_reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """The reference frame that is fixed relative to the vessel,
            and orientated with the vessels orbital prograde/normal/radial directions.
            <p><ul>
            <li>The origin is at the center of mass of the vessel.
            <li>The axes rotate with the orbital prograde/normal/radial directions.
            <li>The x-axis points in the orbital anti-radial direction.
            <li>The y-axis points in the orbital prograde direction.
            <li>The z-axis points in the orbital normal direction.
            </ul></p>

            Be careful not to confuse this with 'orbit' mode on the navball."""
            ...

        @property
        def parts(self):
            # type: () -> SpaceCenter.Parts
            """A SpaceCenter.Parts object, that can used to interact with the parts that make up this vessel."""
            ...

        @property
        def recoverable(self):
            # type: () -> bool
            """Whether the vessel is recoverable."""
            ...

        @property
        def reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """The reference frame that is fixed relative to the vessel,
            and orientated with the vessel.
            <p><ul>
            <li>The origin is at the center of mass of the vessel.
            <li>The axes rotate with the vessel.
            <li>The x-axis points out to the right of the vessel.
            <li>The y-axis points in the forward direction of the vessel.
            <li>The z-axis points out of the bottom off the vessel.
            </ul></p>"""
            ...

        @property
        def resources(self):
            # type: () -> SpaceCenter.Resources
            """A SpaceCenter.Resources object, that can used to get information
            about resources stored in the vessel."""
            ...

        @property
        def situation(self):
            # type: () -> SpaceCenter.VesselSituation
            """The situation the vessel is in."""
            ...

        @property
        def specific_impulse(self):
            # type: () -> float
            """The combined specific impulse of all active engines, in seconds. This is computed using the formula
            described here."""
            ...

        @property
        def surface_reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """The reference frame that is fixed relative to the vessel,
            and orientated with the surface of the body being orbited.
            <p><ul>
            <li>The origin is at the center of mass of the vessel.
            <li>The axes rotate with the north and up directions on the surface of the body.
            <li>The x-axis points in the zenith
              direction (upwards, normal to the body being orbited, from the center of the body towards the center of
              mass of the vessel).
            <li>The y-axis points northwards towards the
              astronomical horizon (north, and tangential to the
              surface of the body -- the direction in which a compass would point when on the surface).
            <li>The z-axis points eastwards towards the
              astronomical horizon (east, and tangential to the
              surface of the body -- east on a compass when on the surface).
            </ul></p>

            Be careful not to confuse this with 'surface' mode on the navball."""
            ...

        @property
        def surface_velocity_reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """The reference frame that is fixed relative to the vessel,
            and orientated with the velocity vector of the vessel relative
            to the surface of the body being orbited.
            <p><ul>
            <li>The origin is at the center of mass of the vessel.
            <li>The axes rotate with the vessel's velocity vector.
            <li>The y-axis points in the direction of the vessel's velocity vector,
              relative to the surface of the body being orbited.
            <li>The z-axis is in the plane of the
              astronomical horizon.
            <li>The x-axis is orthogonal to the other two axes.
            </ul></p>"""
            ...

        @property
        def thrust(self):
            # type: () -> float
            """The total thrust currently being produced by the vessel's engines, in
            Newtons. This is computed by summing SpaceCenter.Engine#thrust for
            every engine in the vessel."""
            ...

        @property
        def type(self):
            # type: () -> SpaceCenter.VesselType
            """The type of the vessel."""
            ...

        @type.setter
        def type(self, value):
            # type: (SpaceCenter.VesselType) -> None
            ...

        @property
        def vacuum_specific_impulse(self):
            # type: () -> float
            """The combined vacuum specific impulse of all active engines, in seconds. This is computed using the formula
            described here."""
            ...

        def angular_velocity(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The angular velocity of the vessel, in the given reference frame.
            @return The angular velocity as a vector. The magnitude of the vector is the rotational
            speed of the vessel, in radians per second. The direction of the vector indicates the
            axis of rotation, using the right-hand rule.
            :reference_frame The reference frame the returned
            angular velocity is in."""
            ...

        def available_thrust_at(self, pressure):
            # type: (float) -> float
            """Gets the total available thrust that can be produced by the vessel's
            active engines, in Newtons. This is computed by summing
            SpaceCenter.Engine#availableThrustAt for every active engine in the vessel.
            Takes the given pressure into account.
            :pressure Atmospheric pressure in atmospheres"""
            ...

        def bounding_box(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[Tuple[float,float,float],Tuple[float,float,float]]
            """The axis-aligned bounding box of the vessel in the given reference frame.
            @return The positions of the minimum and maximum vertices of the box,
            as position vectors.
            :reference_frame The reference frame that the returned
            position vectors are in."""
            ...

        def direction(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The direction in which the vessel is pointing, in the given reference frame.
            @return The direction as a unit vector.
            :reference_frame The reference frame that the returned
            direction is in."""
            ...

        def flight(self, reference_frame=None):
            # type: (SpaceCenter.ReferenceFrame) -> SpaceCenter.Flight
            """Returns a SpaceCenter.Flight object that can be used to get flight
            telemetry for the vessel, in the specified reference frame.
            :reference_frame Reference frame. Defaults to the vessel's surface reference frame
            (SpaceCenter.Vessel#surfaceReferenceFrame)."""
            ...

        def max_thrust_at(self, pressure):
            # type: (float) -> float
            """The total maximum thrust that can be produced by the vessel's active
            engines, in Newtons. This is computed by summing
            SpaceCenter.Engine#maxThrustAt for every active engine.
            Takes the given pressure into account.
            :pressure Atmospheric pressure in atmospheres"""
            ...

        def position(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The position of the center of mass of the vessel, in the given reference frame.
            @return The position as a vector.
            :reference_frame The reference frame that the returned
            position vector is in."""
            ...

        def recover(self):
            # type: () -> None
            """Recover the vessel."""
            ...

        def resources_in_decouple_stage(self, stage, cumulative=True):
            # type: (int, bool) -> SpaceCenter.Resources
            """Returns a SpaceCenter.Resources object, that can used to get
            information about resources stored in a given stage.
            :stage Get resources for parts that are decoupled in this stage.
            :cumulative When {@code false}, returns the resources for parts
            decoupled in just the given stage. When {@code true} returns the resources decoupled in
            the given stage and all subsequent stages combined."""
            ...

        def rotation(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float,float]
            """The rotation of the vessel, in the given reference frame.
            @return The rotation as a quaternion of the form (x, y, z, w).
            :reference_frame The reference frame that the returned
            rotation is in."""
            ...

        def specific_impulse_at(self, pressure):
            # type: (float) -> float
            """The combined specific impulse of all active engines, in seconds. This is computed using the formula
            described here.
            Takes the given pressure into account.
            :pressure Atmospheric pressure in atmospheres"""
            ...

        def velocity(self, reference_frame):
            # type: (SpaceCenter.ReferenceFrame) -> Tuple[float,float,float]
            """The velocity of the center of mass of the vessel, in the given reference frame.
            @return The velocity as a vector. The vector points in the direction of travel,
            and its magnitude is the speed of the body in meters per second.
            :reference_frame The reference frame that the returned
            velocity vector is in."""
            ...

    class Waypoint:
        @property
        def bedrock_altitude(self):
            # type: () -> float
            """The altitude of the waypoint above the surface of the body, in meters.
            When over water, this is the altitude above the sea floor."""
            ...

        @bedrock_altitude.setter
        def bedrock_altitude(self, value):
            # type: (float) -> None
            ...

        @property
        def body(self):
            # type: () -> SpaceCenter.CelestialBody
            """The celestial body the waypoint is attached to."""
            ...

        @body.setter
        def body(self, value):
            # type: (SpaceCenter.CelestialBody) -> None
            ...

        @property
        def clustered(self):
            # type: () -> bool
            """{@code true} if this waypoint is part of a set of clustered waypoints with greek letter
            names appended (Alpha, Beta, Gamma, etc).
            If {@code true}, there is a one-to-one correspondence with the greek letter name and
            the SpaceCenter.Waypoint#index."""
            ...

        @property
        def color(self):
            # type: () -> int
            """The seed of the icon color. See SpaceCenter.WaypointManager#colors for example colors."""
            ...

        @color.setter
        def color(self, value):
            # type: (int) -> None
            ...

        @property
        def contract(self):
            # type: () -> SpaceCenter.Contract
            """The associated contract."""
            ...

        @property
        def grounded(self):
            # type: () -> bool
            """{@code true} if the waypoint is attached to the ground."""
            ...

        @property
        def has_contract(self):
            # type: () -> bool
            """Whether the waypoint belongs to a contract."""
            ...

        @property
        def icon(self):
            # type: () -> str
            """The icon of the waypoint."""
            ...

        @icon.setter
        def icon(self, value):
            # type: (str) -> None
            ...

        @property
        def index(self):
            # type: () -> int
            """The integer index of this waypoint within its cluster of sibling waypoints.
            In other words, when you have a cluster of waypoints called "Somewhere Alpha",
            "Somewhere Beta" and "Somewhere Gamma", the alpha site has index 0, the beta
            site has index 1 and the gamma site has index 2.
            When SpaceCenter.Waypoint#clustered is {@code false}, this is zero."""
            ...

        @property
        def latitude(self):
            # type: () -> float
            """The latitude of the waypoint."""
            ...

        @latitude.setter
        def latitude(self, value):
            # type: (float) -> None
            ...

        @property
        def longitude(self):
            # type: () -> float
            """The longitude of the waypoint."""
            ...

        @longitude.setter
        def longitude(self, value):
            # type: (float) -> None
            ...

        @property
        def mean_altitude(self):
            # type: () -> float
            """The altitude of the waypoint above sea level, in meters."""
            ...

        @mean_altitude.setter
        def mean_altitude(self, value):
            # type: (float) -> None
            ...

        @property
        def name(self):
            # type: () -> str
            """The name of the waypoint as it appears on the map and the contract."""
            ...

        @name.setter
        def name(self, value):
            # type: (str) -> None
            ...

        @property
        def near_surface(self):
            # type: () -> bool
            """{@code true} if the waypoint is near to the surface of a body."""
            ...

        @property
        def surface_altitude(self):
            # type: () -> float
            """The altitude of the waypoint above the surface of the body or sea level,
            whichever is closer, in meters."""
            ...

        @surface_altitude.setter
        def surface_altitude(self, value):
            # type: (float) -> None
            ...

        def remove(self):
            # type: () -> None
            """Removes the waypoint."""
            ...

    class WaypointManager:
        @property
        def colors(self):
            # type: () -> Dict[str, int]
            """An example map of known color - seed pairs.
            Any other integers may be used as seed."""
            ...

        @property
        def icons(self):
            # type: () -> List[str]
            """Returns all available icons (from "GameData/Squad/Contracts/Icons/")."""
            ...

        @property
        def waypoints(self):
            # type: () -> List[SpaceCenter.Waypoint]
            """A list of all existing waypoints."""
            ...

        def add_waypoint(self, latitude, longitude, body, name):
            # type: (float, float, SpaceCenter.CelestialBody, str) -> SpaceCenter.Waypoint
            """Creates a waypoint at the given position at ground level, and returns a
            SpaceCenter.Waypoint object that can be used to modify it.
            :latitude Latitude of the waypoint.
            :longitude Longitude of the waypoint.
            :body Celestial body the waypoint is attached to.
            :name Name of the waypoint.
            @return """
            ...

        def add_waypoint_at_altitude(self, latitude, longitude, altitude, body, name):
            # type: (float, float, float, SpaceCenter.CelestialBody, str) -> SpaceCenter.Waypoint
            """Creates a waypoint at the given position and altitude, and returns a
            SpaceCenter.Waypoint object that can be used to modify it.
            :latitude Latitude of the waypoint.
            :longitude Longitude of the waypoint.
            :altitude Altitude (above sea level) of the waypoint.
            :body Celestial body the waypoint is attached to.
            :name Name of the waypoint.
            @return """
            ...

    class Wheel:
        @property
        def auto_friction_control(self):
            # type: () -> bool
            """Whether automatic friction control is enabled."""
            ...

        @auto_friction_control.setter
        def auto_friction_control(self, value):
            # type: (bool) -> None
            ...

        @property
        def brakes(self):
            # type: () -> float
            """The braking force, as a percentage of maximum, when the brakes are applied."""
            ...

        @brakes.setter
        def brakes(self, value):
            # type: (float) -> None
            ...

        @property
        def broken(self):
            # type: () -> bool
            """Whether the wheel is broken."""
            ...

        @property
        def deflection(self):
            # type: () -> float
            """Current deflection of the wheel."""
            ...

        @property
        def deployable(self):
            # type: () -> bool
            """Whether the wheel is deployable."""
            ...

        @property
        def deployed(self):
            # type: () -> bool
            """Whether the wheel is deployed."""
            ...

        @deployed.setter
        def deployed(self, value):
            # type: (bool) -> None
            ...

        @property
        def drive_limiter(self):
            # type: () -> float
            """Manual setting for the motor limiter.
            Only takes effect if the wheel has automatic traction control disabled.
            A value between 0 and 100 inclusive."""
            ...

        @drive_limiter.setter
        def drive_limiter(self, value):
            # type: (float) -> None
            ...

        @property
        def grounded(self):
            # type: () -> bool
            """Whether the wheel is touching the ground."""
            ...

        @property
        def has_brakes(self):
            # type: () -> bool
            """Whether the wheel has brakes."""
            ...

        @property
        def has_suspension(self):
            # type: () -> bool
            """Whether the wheel has suspension."""
            ...

        @property
        def manual_friction_control(self):
            # type: () -> float
            """Manual friction control value. Only has an effect if automatic friction control is disabled.
            A value between 0 and 5 inclusive."""
            ...

        @manual_friction_control.setter
        def manual_friction_control(self, value):
            # type: (float) -> None
            ...

        @property
        def motor_enabled(self):
            # type: () -> bool
            """Whether the motor is enabled."""
            ...

        @motor_enabled.setter
        def motor_enabled(self, value):
            # type: (bool) -> None
            ...

        @property
        def motor_inverted(self):
            # type: () -> bool
            """Whether the direction of the motor is inverted."""
            ...

        @motor_inverted.setter
        def motor_inverted(self, value):
            # type: (bool) -> None
            ...

        @property
        def motor_output(self):
            # type: () -> float
            """The output of the motor. This is the torque currently being generated, in Newton meters."""
            ...

        @property
        def motor_state(self):
            # type: () -> SpaceCenter.MotorState
            """Whether the direction of the motor is inverted."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part object for this wheel."""
            ...

        @property
        def powered(self):
            # type: () -> bool
            """Whether the wheel is powered by a motor."""
            ...

        @property
        def radius(self):
            # type: () -> float
            """Radius of the wheel, in meters."""
            ...

        @property
        def repairable(self):
            # type: () -> bool
            """Whether the wheel is repairable."""
            ...

        @property
        def slip(self):
            # type: () -> float
            """Current slip of the wheel."""
            ...

        @property
        def state(self):
            # type: () -> SpaceCenter.WheelState
            """The current state of the wheel."""
            ...

        @property
        def steerable(self):
            # type: () -> bool
            """Whether the wheel has steering."""
            ...

        @property
        def steering_angle_limit(self):
            # type: () -> float
            """The steering angle limit."""
            ...

        @steering_angle_limit.setter
        def steering_angle_limit(self, value):
            # type: (float) -> None
            ...

        @property
        def steering_enabled(self):
            # type: () -> bool
            """Whether the wheel steering is enabled."""
            ...

        @steering_enabled.setter
        def steering_enabled(self, value):
            # type: (bool) -> None
            ...

        @property
        def steering_inverted(self):
            # type: () -> bool
            """Whether the wheel steering is inverted."""
            ...

        @steering_inverted.setter
        def steering_inverted(self, value):
            # type: (bool) -> None
            ...

        @property
        def steering_response_time(self):
            # type: () -> float
            """Steering response time."""
            ...

        @steering_response_time.setter
        def steering_response_time(self, value):
            # type: (float) -> None
            ...

        @property
        def stress(self):
            # type: () -> float
            """Current stress on the wheel."""
            ...

        @property
        def stress_percentage(self):
            # type: () -> float
            """Current stress on the wheel as a percentage of its stress tolerance."""
            ...

        @property
        def stress_tolerance(self):
            # type: () -> float
            """Stress tolerance of the wheel."""
            ...

        @property
        def suspension_damper_strength(self):
            # type: () -> float
            """Suspension damper strength, as set in the editor."""
            ...

        @property
        def suspension_spring_strength(self):
            # type: () -> float
            """Suspension spring strength, as set in the editor."""
            ...

        @property
        def traction_control(self):
            # type: () -> float
            """Setting for the traction control.
            Only takes effect if the wheel has automatic traction control enabled.
            A value between 0 and 5 inclusive."""
            ...

        @traction_control.setter
        def traction_control(self, value):
            # type: (float) -> None
            ...

        @property
        def traction_control_enabled(self):
            # type: () -> bool
            """Whether automatic traction control is enabled.
            A wheel only has traction control if it is powered."""
            ...

        @traction_control_enabled.setter
        def traction_control_enabled(self, value):
            # type: (bool) -> None
            ...

    class AntennaState:
        """The state of an antenna. See SpaceCenter.Antenna#state."""
        deployed = 0
        retracted = 1
        deploying = 2
        retracting = 3
        broken = 4

    class AutoStrutMode:
        """The state of an auto-strut. SpaceCenter.Part#autoStrutMode"""
        off = 0
        root = 1
        heaviest = 2
        grandparent = 3
        force_root = 4
        force_heaviest = 5
        force_grandparent = 6

    class CameraMode:
        """See SpaceCenter.Camera#mode."""
        automatic = 0
        free = 1
        chase = 2
        locked = 3
        orbital = 4
        iva = 5
        map = 6

    class CargoBayState:
        """The state of a cargo bay. See SpaceCenter.CargoBay#state."""
        open = 0
        closed = 1
        opening = 2
        closing = 3

    class CommLinkType:
        """The type of a communication link.
        See SpaceCenter.CommLink#type."""
        home = 0
        control = 1
        relay = 2

    class ContractState:
        """The state of a contract. See SpaceCenter.Contract#state."""
        active = 0
        canceled = 1
        completed = 2
        deadline_expired = 3
        declined = 4
        failed = 5
        generated = 6
        offered = 7
        offer_expired = 8
        withdrawn = 9

    class ControlInputMode:
        """See SpaceCenter.Control#inputMode."""
        additive = 0
        override = 1

    class ControlSource:
        """The control source of a vessel.
        See SpaceCenter.Control#source."""
        kerbal = 0
        probe = 1
        none__ = 2

    class ControlState:
        """The control state of a vessel.
        See SpaceCenter.Control#state."""
        full = 0
        partial = 1
        none__ = 2

    class CrewMemberGender:
        """A crew member's gender.
        See SpaceCenter.CrewMember#gender."""
        male = 0
        female = 1

    class CrewMemberType:
        """The type of a crew member.
        See SpaceCenter.CrewMember#type."""
        applicant = 0
        crew = 1
        tourist = 2
        unowned = 3

    class DockingPortState:
        """The state of a docking port. See SpaceCenter.DockingPort#state."""
        ready = 0
        docked = 1
        docking = 2
        undocking = 3
        shielded = 4
        moving = 5

    class DrainMode:
        """Resource drain mode.
        See SpaceCenter.ResourceDrain#drainMode."""
        part = 0
        vessel = 1

    class EditorFacility:
        """Editor facility.
        See SpaceCenter.LaunchSite#editorFacility."""
        vab = 1
        sph = 2
        none__ = 0

    class GameMode:
        """The game mode.
        Returned by SpaceCenter.GameMode"""
        sandbox = 0
        career = 1
        science = 2
        science_sandbox = 3
        mission = 4
        mission_builder = 5
        scenario = 6
        scenario_non_resumable = 7

    class LegState:
        """The state of a landing leg. See SpaceCenter.Leg#state."""
        deployed = 0
        retracted = 1
        deploying = 2
        retracting = 3
        broken = 4

    class MapFilterType:
        """The set of things that are visible in map mode.
        These may be combined with bitwise logic."""
        all = -1
        none__ = 0
        debris = 1
        unknown = 2
        space_objects = 4
        probes = 8
        rovers = 16
        landers = 32
        ships = 64
        stations = 128
        bases = 256
        ev_as = 512
        flags = 1024
        plane = 2048
        relay = 4096
        site = 8192
        deployed_science_controller = 16384

    class MotorState:
        """The state of the motor on a powered wheel. See SpaceCenter.Wheel#motorState."""
        idle = 0
        running = 1
        disabled = 2
        inoperable = 3
        not_enough_resources = 4

    class ParachuteState:
        """The state of a parachute. See SpaceCenter.Parachute#state."""
        stowed = 0
        armed = 1
        semi_deployed = 2
        deployed = 3
        cut = 4

    class RadiatorState:
        """The state of a radiator. SpaceCenter.Radiator#state"""
        extended = 0
        retracted = 1
        extending = 2
        retracting = 3
        broken = 4

    class ResourceConverterState:
        """The state of a resource converter. See SpaceCenter.ResourceConverter#state."""
        running = 0
        idle = 1
        missing_resource = 2
        storage_full = 3
        capacity = 4
        unknown = 5

    class ResourceFlowMode:
        """The way in which a resource flows between parts. See SpaceCenter.Resources#flowMode."""
        vessel = 0
        stage = 1
        adjacent = 2
        none__ = 3

    class ResourceHarvesterState:
        """The state of a resource harvester. See SpaceCenter.ResourceHarvester#state."""
        deploying = 0
        deployed = 1
        retracting = 2
        retracted = 3
        active = 4

    class RosterStatus:
        """A crew member's roster status.
        See SpaceCenter.CrewMember#rosterStatus."""
        available = 0
        assigned = 1
        dead = 2
        missing = 3

    class SASMode:
        """The behavior of the SAS auto-pilot. See SpaceCenter.AutoPilot#sASMode."""
        stability_assist = 0
        maneuver = 1
        prograde = 2
        retrograde = 3
        normal = 4
        anti_normal = 5
        radial = 6
        anti_radial = 7
        target = 8
        anti_target = 9

    class SolarPanelState:
        """The state of a solar panel. See SpaceCenter.SolarPanel#state."""
        extended = 0
        retracted = 1
        extending = 2
        retracting = 3
        broken = 4

    class SpeedMode:
        """The mode of the speed reported in the navball.
        See SpaceCenter.Control#speedMode."""
        orbit = 0
        surface = 1
        target = 2

    class SuitType:
        """A crew member's suit type.
        See SpaceCenter.CrewMember#suitType."""
        default = 0
        vintage = 1
        future = 2
        slim = 3

    class VesselSituation:
        """The situation a vessel is in.
        See SpaceCenter.Vessel#situation."""
        pre_launch = 0
        orbiting = 1
        sub_orbital = 2
        escaping = 3
        flying = 4
        landed = 5
        splashed = 6
        docked = 7

    class VesselType:
        """The type of a vessel.
        See SpaceCenter.Vessel#type."""
        base = 0
        debris = 1
        lander = 2
        plane = 3
        probe = 4
        relay = 5
        rover = 6
        ship = 7
        station = 8
        space_object = 9
        unknown = 10
        eva = 11
        flag = 12
        deployed_science_controller = 13
        deployed_science_part = 14
        dropped_part = 15
        deployed_ground_part = 16

    class WarpMode:
        """The time warp mode.
        Returned by SpaceCenter.WarpMode"""
        rails = 0
        physics = 1
        none__ = 2

    class WheelState:
        """The state of a wheel. See SpaceCenter.Wheel#state."""
        deployed = 0
        retracted = 1
        deploying = 2
        retracting = 3
        broken = 4
