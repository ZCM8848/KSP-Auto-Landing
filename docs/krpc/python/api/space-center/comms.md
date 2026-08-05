# Communications

class Comms
:   Used to interact with CommNet for a given vessel.
    Obtained by calling [`Vessel.comms`](./vessel.md#SpaceCenter.Vessel.comms "SpaceCenter.Vessel.comms").

    can\_communicate
    :   Whether the vessel can communicate with KSC.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    can\_transmit\_science
    :   Whether the vessel can transmit science data to KSC.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    signal\_strength
    :   Signal strength to KSC.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    signal\_delay
    :   Signal delay to KSC in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    power
    :   The combined power of all active antennae on the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    control\_path
    :   The communication path used to control the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`CommLink`](#SpaceCenter.CommLink "SpaceCenter.CommLink"))

        Game Scenes:
        :   Flight

class CommLink
:   Represents a communication node in the network. For example, a vessel or the KSC.

    type
    :   The type of link.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`CommLinkType`](#SpaceCenter.CommLinkType "SpaceCenter.CommLinkType")

    signal\_strength
    :   Signal strength of the link.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    start
    :   Start point of the link.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`CommNode`](#SpaceCenter.CommNode "SpaceCenter.CommNode")

    end
    :   Start point of the link.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`CommNode`](#SpaceCenter.CommNode "SpaceCenter.CommNode")

class CommLinkType
:   The type of a communication link.
    See [`CommLink.type`](#SpaceCenter.CommLink.type "SpaceCenter.CommLink.type").

    home
    :   Link is to a base station on Kerbin.

    control
    :   Link is to a control source, for example a manned spacecraft.

    relay
    :   Link is to a relay satellite.

class CommNode
:   Represents a communication node in the network. For example, a vessel or the KSC.

    name
    :   Name of the communication node.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    is\_home
    :   Whether the communication node is on Kerbin.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    is\_control\_point
    :   Whether the communication node is a control point, for example a manned vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    is\_vessel
    :   Whether the communication node is a vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    vessel
    :   The vessel for this communication node.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Vessel`](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")
