# Servo

class Servo
:   Represents a servo. Obtained using
    [`ServoGroup.servos`](./servo-group.md#InfernalRobotics.ServoGroup.servos "InfernalRobotics.ServoGroup.servos"),
    [`ServoGroup.servo_with_name()`](./servo-group.md#InfernalRobotics.ServoGroup.servo_with_name "InfernalRobotics.ServoGroup.servo_with_name")
    or [`servo_with_name()`](./infernal-robotics.md#InfernalRobotics.servo_with_name "InfernalRobotics.servo_with_name").

    name
    :   The name of the servo.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Flight

    uid
    :   The unique identifier of the servo.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   int

        Game Scenes:
        :   Flight

    part
    :   The part containing the servo.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`SpaceCenter.Part`](../space-center/parts.md#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight

    mode
    :   Whether the part acts as a servo or a rotor.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ServoMode`](#InfernalRobotics.ServoMode "InfernalRobotics.ServoMode")

        Game Scenes:
        :   Flight

    highlight
    :   Whether the servo should be highlighted in-game.

        Attribute:
        :   Write-only, cannot be read

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    position
    :   The position of the servo.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    min\_config\_position
    :   The minimum position of the servo, specified by the part configuration.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_config\_position
    :   The maximum position of the servo, specified by the part configuration.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    min\_position
    :   The minimum position of the servo, specified by the in-game tweak menu.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_position
    :   The maximum position of the servo, specified by the in-game tweak menu.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    config\_speed
    :   The speed multiplier of the servo, specified by the part configuration.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    speed
    :   The speed multiplier of the servo, specified by the in-game tweak menu.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    current\_speed
    :   The current speed at which the servo is moving.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    acceleration
    :   The current speed multiplier set in the UI.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    is\_moving
    :   Whether the servo is moving.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    is\_free\_moving
    :   Whether the servo is freely moving.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    is\_locked
    :   Whether the servo is locked.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    is\_axis\_inverted
    :   Whether the servos axis is inverted.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    target\_position
    :   The target position the servo is moving towards.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    target\_speed
    :   The target speed the servo is moving at.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    commanded\_position
    :   The position the servo is currently being commanded to move to.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    default\_position
    :   The default (built) position of the servo.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    force\_limit
    :   The force limit of the servo, as a percentage of the maximum force.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_force
    :   The maximum force the servo can generate.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_acceleration
    :   The maximum acceleration the servo can achieve.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_speed
    :   The maximum speed the servo can achieve.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    electric\_charge\_required
    :   The rate at which the servo consumes electric charge, in units per second, when moving.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    spring\_power
    :   The strength of the servo’s spring, when it has one.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    damping\_power
    :   The strength of the servo’s damping.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    rotor\_acceleration
    :   The acceleration of the servo when operating as a rotor.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    is\_limited
    :   Whether the servo’s range of movement is limited to the configured minimum and
        maximum positions.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    is\_rotational
    :   Whether the servo moves rotationally (as opposed to linearly).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    is\_servo
    :   Whether the part is operating as a servo (rather than a rotor).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    can\_have\_limits
    :   Whether the servo can have its range of movement limited.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    has\_spring
    :   Whether the servo has a spring.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    is\_running
    :   Whether the servo is running, when operating as a rotor.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    preset\_positions
    :   The list of preset positions configured for the servo.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list(float)

        Game Scenes:
        :   Flight

    move\_right()
    :   Moves the servo to the right.

        Game Scenes:
        :   Flight

    move\_left()
    :   Moves the servo to the left.

        Game Scenes:
        :   Flight

    move\_center()
    :   Moves the servo to the center.

        Game Scenes:
        :   Flight

    move\_to(*position*, *speed*)
    :   Moves the servo to *position* and sets the
        speed multiplier to *speed*.

        Parameters:
        :   - **position** (*float*) – The position to move the servo to.
            - **speed** (*float*) – Speed multiplier for the movement.

        Game Scenes:
        :   Flight

    stop()
    :   Stops the servo.

        Game Scenes:
        :   Flight

    add\_preset(*position*)
    :   Adds a preset position to the servo.

        Parameters:
        :   **position** (*float*) – The position of the preset.

        Game Scenes:
        :   Flight

    remove\_preset\_at(*index*)
    :   Removes the preset position at the given index.

        Parameters:
        :   **index** (*int*) – The index of the preset to remove.

        Game Scenes:
        :   Flight

    sort\_presets()
    :   Sorts the preset positions of the servo into ascending order.

        Game Scenes:
        :   Flight

class ServoMode
:   The mode a servo is operating in. See [`Servo.mode`](#InfernalRobotics.Servo.mode "InfernalRobotics.Servo.mode").

    servo
    :   The part acts as a servo, driving towards a target position.

    rotor
    :   The part acts as a rotor, spinning continuously.
