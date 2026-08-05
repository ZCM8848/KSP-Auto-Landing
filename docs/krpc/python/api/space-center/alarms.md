# Alarms

class AlarmManager
:   Alarm manager.
    Obtained by calling [`alarm_manager`](./space-center.md#SpaceCenter.alarm_manager "SpaceCenter.alarm_manager").

    alarms
    :   A list of all alarms.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Alarm`](#SpaceCenter.Alarm "SpaceCenter.Alarm"))

    alarms\_with\_type(*type*)
    :   A list of all alarms of the given type.

        Parameters:
        :   **type** ([*AlarmType*](#SpaceCenter.AlarmType "SpaceCenter.AlarmType")) – The type of alarm to return.

        Return type:
        :   list([`Alarm`](#SpaceCenter.Alarm "SpaceCenter.Alarm"))

    alarm\_with\_name(*name*)
    :   Returns the first alarm with the given title, or `None` if no such
        alarm exists. Alarm titles are not guaranteed to be unique; if more than
        one alarm shares the given title, the first one found is returned.

        Parameters:
        :   **name** (*str*) – The title of the alarm to return.

        Return type:
        :   [`Alarm`](#SpaceCenter.Alarm "SpaceCenter.Alarm")

    static add\_alarm(*time*[, *title='Alarm'*][, *description=''*])
    :   Create an alarm.

        Parameters:
        :   - **time** (*float*) – Number of seconds from now that the alarm should trigger.
            - **title** (*str*) – Title for the alarm.
            - **description** (*str*) – Description for the alarm.

        Return type:
        :   [`Alarm`](#SpaceCenter.Alarm "SpaceCenter.Alarm")

    static add\_vessel\_alarm(*time*, *vessel*[, *title='Vessel Alarm'*][, *description=''*])
    :   Create an alarm linked to a vessel.

        Parameters:
        :   - **time** (*float*) – Number of seconds from now that the alarm should trigger.
            - **vessel** ([*Vessel*](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")) – Vessel to link the alarm to.
            - **title** (*str*) – Title for the alarm.
            - **description** (*str*) – Description for the alarm.

        Return type:
        :   [`Alarm`](#SpaceCenter.Alarm "SpaceCenter.Alarm")

    static add\_apoapsis\_alarm(*vessel*[, *offset=60.0*][, *title='Apoapsis Alarm'*][, *description=''*])
    :   Create an alarm for the given vessel’s next apoapsis.

        Parameters:
        :   - **vessel** ([*Vessel*](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")) – The vessel.
            - **offset** (*float*) – Time in seconds to offset the alarm by.
            - **title** (*str*) – Title for the alarm.
            - **description** (*str*) – Description for the alarm.

        Return type:
        :   [`Alarm`](#SpaceCenter.Alarm "SpaceCenter.Alarm")

    static add\_periapsis\_alarm(*vessel*[, *offset=60.0*][, *title='Periapsis Alarm'*][, *description=''*])
    :   Create an alarm for the given vessel’s next periapsis.

        Parameters:
        :   - **vessel** ([*Vessel*](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")) – The vessel.
            - **offset** (*float*) – Time in seconds to offset the alarm by.
            - **title** (*str*) – Title for the alarm.
            - **description** (*str*) – Description for the alarm.

        Return type:
        :   [`Alarm`](#SpaceCenter.Alarm "SpaceCenter.Alarm")

    static add\_maneuver\_node\_alarm(*vessel*, *node*[, *offset=60.0*][, *add\_burn\_time=True*][, *title='Maneuver Node Alarm'*][, *description=''*])
    :   Create an alarm for the given vessel and maneuver node.

        Parameters:
        :   - **vessel** ([*Vessel*](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")) – The vessel.
            - **node** ([*Node*](./node.md#SpaceCenter.Node "SpaceCenter.Node")) – The maneuver node.
            - **offset** (*float*) – Time in seconds to offset the alarm by.
            - **add\_burn\_time** (*bool*) – Whether the node’s burn time should be included in the alarm.
            - **title** (*str*) – Title for the alarm.
            - **description** (*str*) – Description for the alarm.

        Return type:
        :   [`Alarm`](#SpaceCenter.Alarm "SpaceCenter.Alarm")

    static add\_soi\_alarm(*vessel*[, *offset=60.0*][, *title='SOI Change Alarm'*][, *description=''*])
    :   Create an alarm for the given vessel’s next sphere of influence change.

        Parameters:
        :   - **vessel** ([*Vessel*](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")) – The vessel.
            - **offset** (*float*) – Time in seconds to offset the alarm by.
            - **title** (*str*) – Title for the alarm.
            - **description** (*str*) – Description for the alarm.

        Return type:
        :   [`Alarm`](#SpaceCenter.Alarm "SpaceCenter.Alarm")

    static add\_transfer\_window\_alarm(*vessel*, *target*[, *title='Transfer Window Alarm'*][, *description=''*])
    :   Create an alarm for the next planetary transfer window from the vessel’s
        current parent body to the target body.

        Parameters:
        :   - **vessel** ([*Vessel*](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")) – The vessel.
            - **target** ([*CelestialBody*](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")) – The target body.
            - **title** (*str*) – Title for the alarm.
            - **description** (*str*) – Description for the alarm.

        Return type:
        :   [`Alarm`](#SpaceCenter.Alarm "SpaceCenter.Alarm")

        > **Note**
        >
        > This relies on KSP’s stock transfer-window alarm logic. If KSP cannot
        > compute a transfer from the vessel’s current parent body to the target,
        > the resulting alarm may not fire at a useful time; in that case the
        > properties on the returned alarm can still be used to inspect or adjust it.

class Alarm
:   An alarm. Can be accessed using [`alarm_manager`](./space-center.md#SpaceCenter.alarm_manager "SpaceCenter.alarm_manager").

    id
    :   Unique identifier of the alarm.
        KSP destroys and recreates an alarm when it is edited.
        This id will remain constant between the old and new alarms.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   int

    type
    :   Type of alarm.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`AlarmType`](#SpaceCenter.AlarmType "SpaceCenter.AlarmType")

    title
    :   Title of the alarm.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

    description
    :   Description of the alarm.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

    time
    :   Time the alarm will trigger, in seconds since epoch.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

    time\_until
    :   Time until the alarm triggers, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    event\_offset
    :   Seconds between the alarm going off and the event it references.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

    vessel
    :   Vessel the alarm references. `None` if it does not reference a vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Vessel`](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")

    node
    :   Maneuver node the alarm references. Only valid for alarms of type
        [`AlarmType.maneuver`](#SpaceCenter.AlarmType.maneuver "SpaceCenter.AlarmType.maneuver").

        Attribute:
        :   Can be read or written

        Return type:
        :   [`Node`](./node.md#SpaceCenter.Node "SpaceCenter.Node")

        > **Note**
        >
        > Throws an exception if the alarm is not of
        > type [`AlarmType.maneuver`](#SpaceCenter.AlarmType.maneuver "SpaceCenter.AlarmType.maneuver").

    origin\_body
    :   Origin body for the transfer window. Only valid for alarms of type
        [`AlarmType.transfer_window`](#SpaceCenter.AlarmType.transfer_window "SpaceCenter.AlarmType.transfer_window").

        Attribute:
        :   Can be read or written

        Return type:
        :   [`CelestialBody`](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")

        > **Note**
        >
        > Throws an exception if the alarm is not of
        > type [`AlarmType.transfer_window`](#SpaceCenter.AlarmType.transfer_window "SpaceCenter.AlarmType.transfer_window").

    destination\_body
    :   Destination body for the transfer window. Only valid for alarms of type
        [`AlarmType.transfer_window`](#SpaceCenter.AlarmType.transfer_window "SpaceCenter.AlarmType.transfer_window").

        Attribute:
        :   Can be read or written

        Return type:
        :   [`CelestialBody`](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")

        > **Note**
        >
        > Throws an exception if the alarm is not of
        > type [`AlarmType.transfer_window`](#SpaceCenter.AlarmType.transfer_window "SpaceCenter.AlarmType.transfer_window").

    warp\_action
    :   The action taken on time warp when the alarm fires.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`AlarmWarpAction`](#SpaceCenter.AlarmWarpAction "SpaceCenter.AlarmWarpAction")

    message\_action
    :   The on-screen message behavior when the alarm fires.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`AlarmMessageAction`](#SpaceCenter.AlarmMessageAction "SpaceCenter.AlarmMessageAction")

    play\_sound
    :   Whether the alarm plays a sound when it fires.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    delete\_on\_dismiss
    :   Whether the alarm is deleted automatically once the player has dismissed
        the triggered message.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    triggered
    :   Whether the time of the alarm has passed and its actions have been triggered.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    actioned
    :   Whether the alarm’s actions were triggered and then completed or closed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    remove()
    :   Removes the alarm.

class AlarmType
:   The type of an alarm. See [`Alarm.type`](#SpaceCenter.Alarm.type "SpaceCenter.Alarm.type").

    raw
    :   An alarm for a specific date/time or a specific period in the future.

    apoapsis
    :   An alarm for the next apoapsis of a vessel.

    periapsis
    :   An alarm for the next periapsis of a vessel.

    maneuver
    :   An alarm based on a maneuver node on the vessel’s flight path.

    soi\_change
    :   An alarm for the next sphere of influence change on the vessel’s flight path.

    transfer\_window
    :   An alarm for the next planetary transfer window from the vessel’s current
        orbit to a target body.

    unknown
    :   The alarm is of a type not recognized by kRPC. Typically this is a type
        introduced by a mod.

class AlarmWarpAction
:   The warp action taken when an alarm fires.
    See [`Alarm.warp_action`](#SpaceCenter.Alarm.warp_action "SpaceCenter.Alarm.warp_action").

    no\_change
    :   Do not change time warp when the alarm fires.

    stop\_warp
    :   Drop out of time warp when the alarm fires.

    pause\_game
    :   Pause the game when the alarm fires.

class AlarmMessageAction
:   The on-screen message action taken when an alarm fires.
    See [`Alarm.message_action`](#SpaceCenter.Alarm.message_action "SpaceCenter.Alarm.message_action").

    no\_message
    :   Do not display a message when the alarm fires.

    message
    :   Display a message when the alarm fires.

    message\_if\_not\_active\_vessel
    :   Display a message only if the alarm’s vessel is not the currently active vessel.
