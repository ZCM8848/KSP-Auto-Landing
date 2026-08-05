# Alarm

class Alarm
:   Represents an alarm. Obtained by calling
    [`alarms`](./kerbal-alarm-clock.md#KerbalAlarmClock.alarms "KerbalAlarmClock.alarms"),
    [`alarm_with_name()`](./kerbal-alarm-clock.md#KerbalAlarmClock.alarm_with_name "KerbalAlarmClock.alarm_with_name") or
    [`alarms_with_type()`](./kerbal-alarm-clock.md#KerbalAlarmClock.alarms_with_type "KerbalAlarmClock.alarms_with_type").

    action
    :   The action that the alarm triggers.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`AlarmAction`](./alarm-action.md#KerbalAlarmClock.AlarmAction "KerbalAlarmClock.AlarmAction")

    margin
    :   The number of seconds before the event that the alarm will fire.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

    time
    :   The time at which the alarm will fire.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

    type
    :   The type of the alarm.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`AlarmType`](./alarm-type.md#KerbalAlarmClock.AlarmType "KerbalAlarmClock.AlarmType")

    id
    :   The unique identifier for the alarm.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    name
    :   The short name of the alarm.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

    notes
    :   The long description of the alarm.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

    remaining
    :   The number of seconds until the alarm will fire.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    enabled
    :   Whether the alarm is enabled. A disabled alarm does not fire.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    play\_sound
    :   Whether the alarm plays a sound when it fires.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    triggered
    :   Whether the alarm has fired. Remains true once the alarm has fired;
        stream this or use it in an event expression to react to the alarm
        firing.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    repeat
    :   Whether the alarm will be repeated after it has fired.
        Only has an effect for alarm types that support repeating
        (see [`Alarm.supports_repeat`](#KerbalAlarmClock.Alarm.supports_repeat "KerbalAlarmClock.Alarm.supports_repeat")).

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    supports\_repeat
    :   Whether this alarm’s type supports repeating
        (see [`Alarm.repeat`](#KerbalAlarmClock.Alarm.repeat "KerbalAlarmClock.Alarm.repeat")).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    repeat\_period
    :   The time delay to automatically create an alarm after it has fired.
        Only has an effect for alarm types that support a repeat period
        (see [`Alarm.supports_repeat_period`](#KerbalAlarmClock.Alarm.supports_repeat_period "KerbalAlarmClock.Alarm.supports_repeat_period")).

        Attribute:
        :   Can be read or written

        Return type:
        :   float

    supports\_repeat\_period
    :   Whether this alarm’s type supports a repeat period
        (see [`Alarm.repeat_period`](#KerbalAlarmClock.Alarm.repeat_period "KerbalAlarmClock.Alarm.repeat_period")).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    vessel
    :   The vessel that the alarm is attached to.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`SpaceCenter.Vessel`](../space-center/vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")

    xfer\_origin\_body
    :   The celestial body the vessel is departing from.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`SpaceCenter.CelestialBody`](../space-center/celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")

    xfer\_target\_body
    :   The celestial body the vessel is arriving at.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`SpaceCenter.CelestialBody`](../space-center/celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")

    remove()
    :   Removes the alarm. Any further use of this object throws an exception.
