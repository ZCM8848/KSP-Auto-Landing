# Text

class Text
:   Text. Created using [`add_text()`](./drawing.md#Drawing.add_text "Drawing.add_text").

    position
    :   Position of the text.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    rotation
    :   Rotation of the text as a quaternion.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float, float)

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    face\_camera
    :   Whether the text turns to face the camera, in place of its [`Text.rotation`](#Drawing.Text.rotation "Drawing.Text.rotation").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

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

    remove()
    :   Remove the object.

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    content
    :   The text string

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    font
    :   Name of the font

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    static available\_fonts()
    :   A list of all available fonts.

        Return type:
        :   list(str)

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    size
    :   Font size.

        Attribute:
        :   Can be read or written

        Return type:
        :   int

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    character\_size
    :   Character size.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    style
    :   Font style.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`UI.FontStyle`](../ui/text.md#UI.FontStyle "UI.FontStyle")

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    color
    :   Set the color

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

    alignment
    :   Alignment.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`UI.TextAlignment`](../ui/text.md#UI.TextAlignment "UI.TextAlignment")

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    line\_spacing
    :   Line spacing.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    anchor
    :   Anchor.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`UI.TextAnchor`](../ui/text.md#UI.TextAnchor "UI.TextAnchor")

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph
