from typing import Tuple, Set, Dict, List, TYPE_CHECKING, Optional, Any, Callable
from krpc.spacecenter import SpaceCenter


class InfernalRobotics:

    @property
    def available(self):
        # type: () -> bool
        """Whether Infernal Robotics is installed."""
        ...

    @property
    def ready(self):
        # type: () -> bool
        """Whether Infernal Robotics API is ready."""
        ...

    def servo_group_with_name(self, vessel, name):
        # type: (SpaceCenter.Vessel, str) -> Optional[InfernalRobotics.ServoGroup]
        """Returns the servo group in the given vessel with the given name,
        or {@code null} if none exists. If multiple servo groups have the same name, only one of them is returned.
        :vessel Vessel to check.
        :name Name of servo group to find."""
        ...

    def servo_groups(self, vessel):
        # type: (SpaceCenter.Vessel) -> List[InfernalRobotics.ServoGroup]
        """A list of all the servo groups in the given vessel."""
        ...

    def servo_with_name(self, vessel, name):
        # type: (SpaceCenter.Vessel, str) -> Optional[InfernalRobotics.Servo]
        """Returns the servo in the given vessel with the given name or
        {@code null} if none exists. If multiple servos have the same name, only one of them is returned.
        :vessel Vessel to check.
        :name Name of the servo to find."""
        ...

    class Servo:
        @property
        def acceleration(self):
            # type: () -> float
            """The current speed multiplier set in the UI."""
            ...

        @acceleration.setter
        def acceleration(self, value):
            # type: (float) -> None
            ...

        @property
        def config_speed(self):
            # type: () -> float
            """The speed multiplier of the servo, specified by the part configuration."""
            ...

        @property
        def current_speed(self):
            # type: () -> float
            """The current speed at which the servo is moving."""
            ...

        def highlight(self, value):
            # type: (bool) -> None
            """Whether the servo should be highlighted in-game."""
            ...
        highlight = property(None, highlight)

        @property
        def is_axis_inverted(self):
            # type: () -> bool
            """Whether the servos axis is inverted."""
            ...

        @is_axis_inverted.setter
        def is_axis_inverted(self, value):
            # type: (bool) -> None
            ...

        @property
        def is_free_moving(self):
            # type: () -> bool
            """Whether the servo is freely moving."""
            ...

        @property
        def is_locked(self):
            # type: () -> bool
            """Whether the servo is locked."""
            ...

        @is_locked.setter
        def is_locked(self, value):
            # type: (bool) -> None
            ...

        @property
        def is_moving(self):
            # type: () -> bool
            """Whether the servo is moving."""
            ...

        @property
        def max_config_position(self):
            # type: () -> float
            """The maximum position of the servo, specified by the part configuration."""
            ...

        @property
        def max_position(self):
            # type: () -> float
            """The maximum position of the servo, specified by the in-game tweak menu."""
            ...

        @max_position.setter
        def max_position(self, value):
            # type: (float) -> None
            ...

        @property
        def min_config_position(self):
            # type: () -> float
            """The minimum position of the servo, specified by the part configuration."""
            ...

        @property
        def min_position(self):
            # type: () -> float
            """The minimum position of the servo, specified by the in-game tweak menu."""
            ...

        @min_position.setter
        def min_position(self, value):
            # type: (float) -> None
            ...

        @property
        def name(self):
            # type: () -> str
            """The name of the servo."""
            ...

        @name.setter
        def name(self, value):
            # type: (str) -> None
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """The part containing the servo."""
            ...

        @property
        def position(self):
            # type: () -> float
            """The position of the servo."""
            ...

        @property
        def speed(self):
            # type: () -> float
            """The speed multiplier of the servo, specified by the in-game tweak menu."""
            ...

        @speed.setter
        def speed(self, value):
            # type: (float) -> None
            ...

        def move_center(self):
            # type: () -> None
            """Moves the servo to the center."""
            ...

        def move_left(self):
            # type: () -> None
            """Moves the servo to the left."""
            ...

        def move_right(self):
            # type: () -> None
            """Moves the servo to the right."""
            ...

        def move_to(self, position, speed):
            # type: (float, float) -> None
            """Moves the servo to position and sets the
            speed multiplier to speed.
            :position The position to move the servo to.
            :speed Speed multiplier for the movement."""
            ...

        def stop(self):
            # type: () -> None
            """Stops the servo."""
            ...

    class ServoGroup:
        @property
        def expanded(self):
            # type: () -> bool
            """Whether the group is expanded in the InfernalRobotics UI."""
            ...

        @expanded.setter
        def expanded(self, value):
            # type: (bool) -> None
            ...

        @property
        def forward_key(self):
            # type: () -> str
            """The key assigned to be the "forward" key for the group."""
            ...

        @forward_key.setter
        def forward_key(self, value):
            # type: (str) -> None
            ...

        @property
        def name(self):
            # type: () -> str
            """The name of the group."""
            ...

        @name.setter
        def name(self, value):
            # type: (str) -> None
            ...

        @property
        def parts(self):
            # type: () -> List[SpaceCenter.Part]
            """The parts containing the servos in the group."""
            ...

        @property
        def reverse_key(self):
            # type: () -> str
            """The key assigned to be the "reverse" key for the group."""
            ...

        @reverse_key.setter
        def reverse_key(self, value):
            # type: (str) -> None
            ...

        @property
        def servos(self):
            # type: () -> List[InfernalRobotics.Servo]
            """The servos that are in the group."""
            ...

        @property
        def speed(self):
            # type: () -> float
            """The speed multiplier for the group."""
            ...

        @speed.setter
        def speed(self, value):
            # type: (float) -> None
            ...

        def move_center(self):
            # type: () -> None
            """Moves all of the servos in the group to the center."""
            ...

        def move_left(self):
            # type: () -> None
            """Moves all of the servos in the group to the left."""
            ...

        def move_next_preset(self):
            # type: () -> None
            """Moves all of the servos in the group to the next preset."""
            ...

        def move_prev_preset(self):
            # type: () -> None
            """Moves all of the servos in the group to the previous preset."""
            ...

        def move_right(self):
            # type: () -> None
            """Moves all of the servos in the group to the right."""
            ...

        def servo_with_name(self, name):
            # type: (str) -> Optional[InfernalRobotics.Servo]
            """Returns the servo with the given name from this group,
            or {@code null} if none exists.
            :name Name of servo to find."""
            ...

        def stop(self):
            # type: () -> None
            """Stops the servos in the group."""
            ...
