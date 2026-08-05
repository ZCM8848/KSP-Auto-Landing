# Comms

class Comms
:   Communications for a vessel.

    vessel
    :   Get the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`SpaceCenter.Vessel`](../space-center/vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")

    has\_local\_control
    :   Whether the vessel can be controlled locally.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    has\_flight\_computer
    :   Whether the vessel has a flight computer on board.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    has\_connection
    :   Whether the vessel has any connection.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    has\_connection\_to\_ground\_station
    :   Whether the vessel has a connection to a ground station.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    signal\_delay
    :   The shortest signal delay to the vessel, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    signal\_delay\_to\_ground\_station
    :   The signal delay between the vessel and the closest ground station, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    signal\_delay\_to\_vessel(*other*)
    :   The signal delay between the this vessel and another vessel, in seconds.

        Parameters:
        :   **other** ([*SpaceCenter.Vessel*](../space-center/vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel"))

        Return type:
        :   float

    antennas
    :   The antennas for this vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Antenna`](./antenna.md#RemoteTech.Antenna "RemoteTech.Antenna"))
