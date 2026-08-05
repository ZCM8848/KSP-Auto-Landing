# InfernalRobotics API

Provides RPCs to interact with [Infernal Robotics Next](https://forum.kerbalspaceprogram.com/index.php?/topic/170898-wip-infernal-robotics-next/). Provides the following classes:

- [InfernalRobotics](./infernal-robotics/infernal-robotics.md)
- [ServoGroup](./infernal-robotics/servo-group.md)
- [Servo](./infernal-robotics/servo.md)

## Example

The following example gets the control group named “MyGroup”, prints out the
names and positions of all of the servos in the group, then moves all of the
servos to the right for 1 second.

```py
import sys
import time
import krpc

conn = krpc.connect(name="InfernalRobotics Example")
vessel = conn.space_center.active_vessel

group = conn.infernal_robotics.servo_group_with_name(vessel, "MyGroup")
if group is None:
    print("Group not found")
    sys.exit(1)

for servo in group.servos:
    print(servo.name, servo.position)

group.move_right()
time.sleep(1)
group.stop()
```
