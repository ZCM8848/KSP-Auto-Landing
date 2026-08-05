# KerbalAlarmClock

This service provides functionality to interact with
[Kerbal Alarm Clock](https://forum.kerbalspaceprogram.com/index.php?/topic/22809-13x-kerbal-alarm-clock-v3850-may-30/).

available
:   Whether Kerbal Alarm Clock is available.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   bool

alarms
:   A list of all the alarms.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   list([`Alarm`](./alarm.md#KerbalAlarmClock.Alarm "KerbalAlarmClock.Alarm"))

static alarm\_with\_name(*name*)
:   Get the alarm with the given *name*, or `None`
    if no alarms have that name. If more than one alarm has the name,
    only returns one of them.

    Parameters:
    :   **name** (*str*) – Name of the alarm to search for.

    Return type:
    :   [`Alarm`](./alarm.md#KerbalAlarmClock.Alarm "KerbalAlarmClock.Alarm")

static alarms\_with\_type(*type*)
:   Get a list of alarms of the specified *type*.

    Parameters:
    :   **type** ([*AlarmType*](./alarm-type.md#KerbalAlarmClock.AlarmType "KerbalAlarmClock.AlarmType")) – Type of alarm to return.

    Return type:
    :   list([`Alarm`](./alarm.md#KerbalAlarmClock.Alarm "KerbalAlarmClock.Alarm"))

static create\_alarm(*type*, *name*, *ut*)
:   Create a new alarm and return it.

    Parameters:
    :   - **type** ([*AlarmType*](./alarm-type.md#KerbalAlarmClock.AlarmType "KerbalAlarmClock.AlarmType")) – Type of the new alarm.
        - **name** (*str*) – Name of the new alarm.
        - **ut** (*float*) – Time at which the new alarm should trigger.

    Return type:
    :   [`Alarm`](./alarm.md#KerbalAlarmClock.Alarm "KerbalAlarmClock.Alarm")
