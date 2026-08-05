# Kerbal Alarm Clock API

Provides RPCs to interact with the [Kerbal Alarm Clock](https://forum.kerbalspaceprogram.com/index.php?/topic/22809-13x-kerbal-alarm-clock-v3850-may-30/) mod. Provides the
following classes:

- [KerbalAlarmClock](./kerbal-alarm-clock/kerbal-alarm-clock.md)
- [Alarm](./kerbal-alarm-clock/alarm.md)
- [AlarmType](./kerbal-alarm-clock/alarm-type.md)
- [AlarmAction](./kerbal-alarm-clock/alarm-action.md)

## Example

The following example creates a new alarm for the active vessel. The alarm is
set to trigger after 10 seconds have passed, and display a message.

```py
import krpc

conn = krpc.connect(name="Kerbal Alarm Clock Example")

alarm = conn.kerbal_alarm_clock.create_alarm(
    conn.kerbal_alarm_clock.AlarmType.raw, "My New Alarm", conn.space_center.ut + 10
)

alarm.notes = "10 seconds have now passed since the alarm was created."
alarm.action = conn.kerbal_alarm_clock.AlarmAction.message_only
```
