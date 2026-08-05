# ServoGroup

class ServoGroup
:   A group of servos, obtained by calling [`servo_groups()`](./infernal-robotics.md#InfernalRobotics.servo_groups "InfernalRobotics.servo_groups")
    or [`servo_group_with_name()`](./infernal-robotics.md#InfernalRobotics.servo_group_with_name "InfernalRobotics.servo_group_with_name"). Represents the “Servo Groups”
    in the InfernalRobotics UI.

    name
    :   The name of the group.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Flight

    forward\_key
    :   The key assigned to be the “forward” key for the group.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Flight

    reverse\_key
    :   The key assigned to be the “reverse” key for the group.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Flight

    speed
    :   The speed multiplier for the group.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    expanded
    :   Whether the group is expanded in the InfernalRobotics UI.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    vessel
    :   The vessel the group belongs to, or `None` if it is not available.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`SpaceCenter.Vessel`](../space-center/vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")

        Game Scenes:
        :   Flight

    moving\_direction
    :   The direction the group is currently moving in: -1 for reverse, 0 for stopped
        and 1 for forward.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   int

        Game Scenes:
        :   Flight

    advanced\_mode
    :   Whether the group is in advanced mode.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    electric\_charge\_required
    :   The total rate at which the servos in the group consume electric charge, in units
        per second, when moving.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    build\_aid
    :   Whether the build aid is enabled for the group.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    ik\_active
    :   Whether inverse kinematics is active for the group.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    servos
    :   The servos that are in the group.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Servo`](./servo.md#InfernalRobotics.Servo "InfernalRobotics.Servo"))

        Game Scenes:
        :   Flight

    servo\_with\_name(*name*)
    :   Returns the servo with the given *name* from this group,
        or `None` if none exists.

        Parameters:
        :   **name** (*str*) – Name of servo to find.

        Return type:
        :   [`Servo`](./servo.md#InfernalRobotics.Servo "InfernalRobotics.Servo")

        Game Scenes:
        :   Flight

    parts
    :   The parts containing the servos in the group.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`SpaceCenter.Part`](../space-center/parts.md#SpaceCenter.Part "SpaceCenter.Part"))

        Game Scenes:
        :   Flight

    move\_right()
    :   Moves all of the servos in the group to the right.

        Game Scenes:
        :   Flight

    move\_left()
    :   Moves all of the servos in the group to the left.

        Game Scenes:
        :   Flight

    move\_center()
    :   Moves all of the servos in the group to the center.

        Game Scenes:
        :   Flight

    move\_next\_preset()
    :   Moves all of the servos in the group to the next preset.

        Game Scenes:
        :   Flight

    move\_prev\_preset()
    :   Moves all of the servos in the group to the previous preset.

        Game Scenes:
        :   Flight

    stop()
    :   Stops the servos in the group.

        Game Scenes:
        :   Flight
