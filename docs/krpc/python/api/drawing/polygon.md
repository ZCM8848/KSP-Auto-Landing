# Polygon

class Polygon
:   A polygon. Created using [`add_polygon()`](./drawing.md#Drawing.add_polygon "Drawing.add_polygon").

    vertices
    :   Vertices for the polygon.

        Attribute:
        :   Can be read or written

        Return type:
        :   list(tuple(float, float, float))

        Game Scenes:
        :   Flight

    reference\_frame
    :   Reference frame for the positions of the object.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`SpaceCenter.ReferenceFrame`](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Game Scenes:
        :   Flight

    visible
    :   Whether the object is visible.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    remove()
    :   Remove the object.

        Game Scenes:
        :   Flight

    color
    :   Set the color

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    material
    :   Material used to render the object.
        Creates the material from a shader with the given name.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Flight

    thickness
    :   Set the thickness

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight
