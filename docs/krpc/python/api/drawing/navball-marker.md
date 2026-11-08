# NavballMarker

class NavballMarker
:   A marker drawn on the navball. Created using [`add_navball_marker()`](./drawing.md#Drawing.add_navball_marker "Drawing.add_navball_marker").

    direction
    :   Direction that the marker is drawn at.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    reference\_frame
    :   Reference frame that the direction of the marker is in.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`SpaceCenter.ReferenceFrame`](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Game Scenes:
        :   Flight

    visible
    :   Whether the marker is visible.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

        > **Note**
        >
        > A visible marker is still hidden while it is on the far side of the navball.

    color
    :   Set the color

        Attribute:
        :   Can be read or written

        Return type:
        :   [`StdLib.ColorRGB`](../stdlib.md#StdLib.ColorRGB "StdLib.ColorRGB")

        Game Scenes:
        :   Flight

    icon
    :   The icon that the marker is drawn with.
        See [`NavballMarker.available_icons()`](#Drawing.NavballMarker.available_icons "Drawing.NavballMarker.available_icons") for the icons that can be used.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Flight

    image
    :   The image that the marker is drawn with in place of its icon, or null to draw the
        icon.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`Image`](./image.md#Drawing.Image "Drawing.Image") or None

        Game Scenes:
        :   Flight

    static available\_icons()
    :   A list of all available icons, taken from “GameData/Squad/Contracts/Icons/”.

        Return type:
        :   list(str)

        Game Scenes:
        :   Flight

    size
    :   Size of the marker, relative to the size of the navball’s own markers.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    remove()
    :   Remove the marker.

        Game Scenes:
        :   Flight
