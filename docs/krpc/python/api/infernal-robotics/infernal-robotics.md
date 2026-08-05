# InfernalRobotics

This service provides functionality to interact with
[Infernal Robotics](https://forum.kerbalspaceprogram.com/index.php?/topic/184787-infernal-robotics-next/).

available
:   Whether Infernal Robotics is installed.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   bool

ready
:   Whether Infernal Robotics API is ready.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   bool

    Game Scenes:
    :   Flight

static servo\_groups(*vessel*)
:   A list of all the servo groups in the given *vessel*.

    Parameters:
    :   **vessel** ([*SpaceCenter.Vessel*](../space-center/vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel"))

    Return type:
    :   list([`ServoGroup`](./servo-group.md#InfernalRobotics.ServoGroup "InfernalRobotics.ServoGroup"))

    Game Scenes:
    :   Flight

    > **Note**
    >
    > Works for any loaded vessel, not just the active one. Groups on a non-active vessel
    > support movement and per-servo control, but not preset, key, speed-factor or
    > expanded state, which are only tracked by Infernal Robotics for the active vessel.

static servo\_group\_with\_name(*vessel*, *name*)
:   Returns the servo group in the given *vessel* with the given *name*,
    or `None` if none exists. If multiple servo groups have the same name, only one of them is returned.

    Parameters:
    :   - **vessel** ([*SpaceCenter.Vessel*](../space-center/vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")) – Vessel to check.
        - **name** (*str*) – Name of servo group to find.

    Return type:
    :   [`ServoGroup`](./servo-group.md#InfernalRobotics.ServoGroup "InfernalRobotics.ServoGroup")

    Game Scenes:
    :   Flight

static servo\_with\_name(*vessel*, *name*)
:   Returns the servo in the given *vessel* with the given *name* or
    `None` if none exists. If multiple servos have the same name, only one of them is returned.

    Parameters:
    :   - **vessel** ([*SpaceCenter.Vessel*](../space-center/vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")) – Vessel to check.
        - **name** (*str*) – Name of the servo to find.

    Return type:
    :   [`Servo`](./servo.md#InfernalRobotics.Servo "InfernalRobotics.Servo")

    Game Scenes:
    :   Flight
