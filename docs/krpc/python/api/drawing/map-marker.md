# MapMarker

class MapMarker
:   A marker drawn in the map view, with one of the game’s own map icons. Created using
    [`add_map_marker()`](./drawing.md#Drawing.add_map_marker "Drawing.add_map_marker").

    position
    :   Position of the marker.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight, Tracking Station

    reference\_frame
    :   Reference frame that the position of the marker is in.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`SpaceCenter.ReferenceFrame`](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Game Scenes:
        :   Flight, Tracking Station

    visible
    :   Whether the marker is visible.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Tracking Station

    icon
    :   The icon that the marker is drawn with.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`MapMarkerIcon`](#Drawing.MapMarkerIcon "Drawing.MapMarkerIcon")

        Game Scenes:
        :   Flight, Tracking Station

    image
    :   The image that the marker is drawn with in place of its icon, or null to draw the
        icon.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`Image`](./image.md#Drawing.Image "Drawing.Image") or None

        Game Scenes:
        :   Flight, Tracking Station

    caption
    :   The caption shown when the mouse is over the marker.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Flight, Tracking Station

    color
    :   The color of the marker.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`StdLib.ColorRGB`](../stdlib.md#StdLib.ColorRGB "StdLib.ColorRGB")

        Game Scenes:
        :   Flight, Tracking Station

    remove()
    :   Remove the marker.

        Game Scenes:
        :   Flight, Tracking Station

class MapMarkerIcon
:   The icon a [`MapMarker`](#Drawing.MapMarker "Drawing.MapMarker") is drawn with, from the game’s own map icons.

    generic
    :   A plain marker.

    celestial\_body
    :   A celestial body.

    apoapsis
    :   An apoapsis.

    periapsis
    :   A periapsis.

    ascending\_node
    :   An ascending node.

    descending\_node
    :   A descending node.

    maneuver\_node
    :   A maneuver node.

    encounter
    :   An encounter with a sphere of influence.

    escape
    :   An escape from a sphere of influence.

    impact
    :   An impact with the surface.

    close\_approach
    :   A closest approach to a target.

    intersect
    :   An intersection with the orbit of a target.

    ship
    :   A ship.

    probe
    :   A probe.

    relay
    :   A relay.

    station
    :   A station.

    base
    :   A base.

    lander
    :   A lander.

    rover
    :   A rover.

    plane
    :   A plane.

    eva
    :   A kerbal on EVA.

    flag
    :   A flag.

    debris
    :   Debris.

    space\_object
    :   An asteroid or comet.

    deployed\_science
    :   Deployed science equipment.

    launch\_site
    :   A launch site.

    runway
    :   A runway.

    ksc
    :   The Kerbal Space Center.
