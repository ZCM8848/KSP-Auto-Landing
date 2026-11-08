# Sphere

class Sphere
:   A solid sphere. Created using [`add_sphere()`](./drawing.md#Drawing.add_sphere "Drawing.add_sphere").

    > **Note**
    >
    > The sphere is drawn with an additive material, so a darker color draws it more
    > transparent.

    center
    :   Position of the center of the sphere.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    radius
    :   Radius of the sphere, in meters.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

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
