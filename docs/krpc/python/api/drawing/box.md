# Box

class Box
:   The edges of a box. Created using [`add_box()`](./drawing.md#Drawing.add_box "Drawing.add_box").

    center
    :   Position of the center of the box.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    size
    :   Lengths of the sides of the box along its x, y and z axes, in meters.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    rotation
    :   Rotation of the box, as a quaternion.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float, float)

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    reference\_frame
    :   Reference frame for the position and rotation of the box.

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
    :   The color of the edges.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`StdLib.ColorRGB`](../stdlib.md#StdLib.ColorRGB "StdLib.ColorRGB")

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    thickness
    :   The thickness of the edges.

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
