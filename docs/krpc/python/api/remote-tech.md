# RemoteTech API

Provides RPCs to interact with the [RemoteTech](https://forum.kerbalspaceprogram.com/index.php?/topic/139167-13-remotetech-v188-2017-09-03/) mod. Provides the following classes:

- [RemoteTech](./remote-tech/remote-tech.md)
- [Comms](./remote-tech/comms.md)
- [Antenna](./remote-tech/antenna.md)

## Example

The following example sets the target of a dish on the active vessel then prints
out the signal delay to the active vessel.

```py
import krpc

conn = krpc.connect(name="RemoteTech Example")
vessel = conn.space_center.active_vessel

# Set a dish target
part = vessel.parts.with_title("Reflectron KR-7")[0]
antenna = conn.remote_tech.antenna(part)
antenna.target_body = conn.space_center.bodies["Jool"]

# Get info about the vessels communications
comms = conn.remote_tech.comms(vessel)
print("Signal delay = %.4f seconds" % comms.signal_delay)
```
