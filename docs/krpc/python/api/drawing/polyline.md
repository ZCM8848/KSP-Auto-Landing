# Polyline

class Polyline
:   A line through a list of vertices, open at its ends. Created using
    [`add_polyline()`](./drawing.md#Drawing.add_polyline "Drawing.add_polyline").

    vertices
    :   Vertices for the polyline.

        Attribute:
        :   Can be read or written

        Return type:
        :   list(tuple(float, float, float))

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
    :   The color of the path.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`StdLib.ColorRGB`](../stdlib.md#StdLib.ColorRGB "StdLib.ColorRGB")

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

        > **Note**
        >
        > For a path drawn with more than one color, this is the color at its start.

    colors
    :   The colors of the path, blended evenly from its start to its end.

        Attribute:
        :   Can be read or written

        Return type:
        :   list([`StdLib.ColorRGB`](../stdlib.md#StdLib.ColorRGB "StdLib.ColorRGB"))

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

        > **Note**
        >
        > Throws an exception if the list is empty or holds more than 8 colors.

    thickness
    :   The thickness of the path.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    start\_arrow
    :   Whether an arrow head is drawn at the start.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    end\_arrow
    :   Whether an arrow head is drawn at the end.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    arrow\_size
    :   The size of the arrow heads, relative to the thickness.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    dashed
    :   Whether the path is drawn as a dashed line.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    label
    :   Text drawn halfway along the path, facing the camera. Empty for no label.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

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
