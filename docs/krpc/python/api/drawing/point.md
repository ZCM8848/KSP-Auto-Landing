# Point

class Point
:   A point, drawn as a dot that keeps its size on screen. Created using
    [`add_point()`](./drawing.md#Drawing.add_point "Drawing.add_point").

    position
    :   Position of the point.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    reference\_frame
    :   Reference frame for the positions of the object.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`SpaceCenter.ReferenceFrame`](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    visible
    :   Whether the object is visible.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    color
    :   The color of the object.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`StdLib.ColorRGB`](../stdlib.md#StdLib.ColorRGB "StdLib.ColorRGB")

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    size
    :   Diameter of the point, in pixels.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    material
    :   Material used to render the object.
        Creates the material from a shader with the given name.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    remove()
    :   Remove the object.

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph
