# OrbitPath

class OrbitPath
:   The path of an orbit. Created using [`add_orbit()`](./drawing.md#Drawing.add_orbit "Drawing.add_orbit").

    orbit
    :   The orbit.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`SpaceCenter.Orbit`](../space-center/orbit.md#SpaceCenter.Orbit "SpaceCenter.Orbit")

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    whole\_orbit
    :   Whether the whole orbit is drawn, up to where it leaves the sphere of influence of
        its body. When false, the orbit is drawn from [`OrbitPath.start_true_anomaly`](#Drawing.OrbitPath.start_true_anomaly "Drawing.OrbitPath.start_true_anomaly")
        to [`OrbitPath.end_true_anomaly`](#Drawing.OrbitPath.end_true_anomaly "Drawing.OrbitPath.end_true_anomaly").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    start\_true\_anomaly
    :   The true anomaly at which the path starts, in radians.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

        > **Note**
        >
        > Setting this sets [`OrbitPath.whole_orbit`](#Drawing.OrbitPath.whole_orbit "Drawing.OrbitPath.whole_orbit") to false.

    end\_true\_anomaly
    :   The true anomaly at which the path ends, in radians.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

        > **Note**
        >
        > Setting this sets [`OrbitPath.whole_orbit`](#Drawing.OrbitPath.whole_orbit "Drawing.OrbitPath.whole_orbit") to false.

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
