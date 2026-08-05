# Waypoints

class WaypointManager
:   Waypoints are the location markers you can see on the map view showing you where contracts are targeted for.
    With this structure, you can obtain coordinate data for the locations of these waypoints.
    Obtained by calling [`waypoint_manager`](./space-center.md#SpaceCenter.waypoint_manager "SpaceCenter.waypoint_manager").

    waypoints
    :   A list of all existing waypoints.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Waypoint`](#SpaceCenter.Waypoint "SpaceCenter.Waypoint"))

    add\_waypoint(*latitude*, *longitude*, *body*, *name*)
    :   Creates a waypoint at the given position at ground level, and returns a
        [`Waypoint`](#SpaceCenter.Waypoint "SpaceCenter.Waypoint") object that can be used to modify it.

        Parameters:
        :   - **latitude** (*float*) – Latitude of the waypoint.
            - **longitude** (*float*) – Longitude of the waypoint.
            - **body** ([*CelestialBody*](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")) – Celestial body the waypoint is attached to.
            - **name** (*str*) – Name of the waypoint.

        Return type:
        :   [`Waypoint`](#SpaceCenter.Waypoint "SpaceCenter.Waypoint")

    add\_waypoint\_at\_altitude(*latitude*, *longitude*, *altitude*, *body*, *name*)
    :   Creates a waypoint at the given position and altitude, and returns a
        [`Waypoint`](#SpaceCenter.Waypoint "SpaceCenter.Waypoint") object that can be used to modify it.

        Parameters:
        :   - **latitude** (*float*) – Latitude of the waypoint.
            - **longitude** (*float*) – Longitude of the waypoint.
            - **altitude** (*float*) – Altitude (above sea level) of the waypoint.
            - **body** ([*CelestialBody*](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")) – Celestial body the waypoint is attached to.
            - **name** (*str*) – Name of the waypoint.

        Return type:
        :   [`Waypoint`](#SpaceCenter.Waypoint "SpaceCenter.Waypoint")

    colors
    :   An example map of known color - seed pairs.
        Any other integers may be used as seed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   dict(str, int)

    icons
    :   Returns all available icons (from “GameData/Squad/Contracts/Icons/”).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list(str)

class Waypoint
:   Represents a waypoint. Can be created using [`WaypointManager.add_waypoint()`](#SpaceCenter.WaypointManager.add_waypoint "SpaceCenter.WaypointManager.add_waypoint").

    body
    :   The celestial body the waypoint is attached to.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`CelestialBody`](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")

        Game Scenes:
        :   Flight

    name
    :   The name of the waypoint as it appears on the map and the contract.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Flight

    color
    :   The seed of the icon color. See [`WaypointManager.colors`](#SpaceCenter.WaypointManager.colors "SpaceCenter.WaypointManager.colors") for example colors.

        Attribute:
        :   Can be read or written

        Return type:
        :   int

        Game Scenes:
        :   Flight

    icon
    :   The icon of the waypoint.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Flight

    latitude
    :   The latitude of the waypoint.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    longitude
    :   The longitude of the waypoint.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    mean\_altitude
    :   The altitude of the waypoint above sea level, in meters.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    surface\_altitude
    :   The altitude of the waypoint above the surface of the body or sea level,
        whichever is closer, in meters.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    bedrock\_altitude
    :   The altitude of the waypoint above the surface of the body, in meters.
        When over water, this is the altitude above the sea floor.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    near\_surface
    :   `True` if the waypoint is near to the surface of a body.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    grounded
    :   `True` if the waypoint is attached to the ground.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    index
    :   The integer index of this waypoint within its cluster of sibling waypoints.
        In other words, when you have a cluster of waypoints called “Somewhere Alpha”,
        “Somewhere Beta” and “Somewhere Gamma”, the alpha site has index 0, the beta
        site has index 1 and the gamma site has index 2.
        When [`Waypoint.clustered`](#SpaceCenter.Waypoint.clustered "SpaceCenter.Waypoint.clustered") is `False`, this is zero.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   int

        Game Scenes:
        :   Flight

    clustered
    :   `True` if this waypoint is part of a set of clustered waypoints with greek letter
        names appended (Alpha, Beta, Gamma, etc).
        If `True`, there is a one-to-one correspondence with the greek letter name and
        the [`Waypoint.index`](#SpaceCenter.Waypoint.index "SpaceCenter.Waypoint.index").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    has\_contract
    :   Whether the waypoint belongs to a contract.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    contract
    :   The associated contract.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Contract`](./contracts.md#SpaceCenter.Contract "SpaceCenter.Contract")

        Game Scenes:
        :   Flight

    remove()
    :   Removes the waypoint.

        Game Scenes:
        :   Flight
