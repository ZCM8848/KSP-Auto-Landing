# Antenna

class Antenna
:   A RemoteTech antenna. Obtained by calling [`Comms.antennas`](./comms.md#RemoteTech.Comms.antennas "RemoteTech.Comms.antennas") or [`antenna()`](./remote-tech.md#RemoteTech.antenna "RemoteTech.antenna").

    part
    :   Get the part containing this antenna.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`SpaceCenter.Part`](../space-center/parts.md#SpaceCenter.Part "SpaceCenter.Part")

    has\_connection
    :   Whether the antenna has a connection.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    target
    :   The object that the antenna is targetting.
        This property can be used to set the target to [`Target.none`](#RemoteTech.Target.none "RemoteTech.Target.none") or [`Target.active_vessel`](#RemoteTech.Target.active_vessel "RemoteTech.Target.active_vessel").
        To set the target to a celestial body, ground station or vessel see [`Antenna.target_body`](#RemoteTech.Antenna.target_body "RemoteTech.Antenna.target_body"),
        [`Antenna.target_ground_station`](#RemoteTech.Antenna.target_ground_station "RemoteTech.Antenna.target_ground_station") and [`Antenna.target_vessel`](#RemoteTech.Antenna.target_vessel "RemoteTech.Antenna.target_vessel").

        Attribute:
        :   Can be read or written

        Return type:
        :   [`Target`](#RemoteTech.Target "RemoteTech.Target")

    target\_body
    :   The celestial body the antenna is targetting.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`SpaceCenter.CelestialBody`](../space-center/celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")

    target\_ground\_station
    :   The ground station the antenna is targetting.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

    target\_vessel
    :   The vessel the antenna is targetting.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`SpaceCenter.Vessel`](../space-center/vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")

class Target
:   The type of object an antenna is targetting.
    See [`Antenna.target`](#RemoteTech.Antenna.target "RemoteTech.Antenna.target").

    active\_vessel
    :   The active vessel.

    celestial\_body
    :   A celestial body.

    ground\_station
    :   A ground station.

    vessel
    :   A specific vessel.

    none
    :   No target.
