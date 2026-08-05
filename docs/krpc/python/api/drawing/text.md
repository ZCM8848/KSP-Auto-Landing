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
        :   Flight

    rotation
    :   Rotation of the text as a quaternion.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float, float)

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

    content
    :   The text string

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Flight

    font
    :   Name of the font

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Flight

    static available\_fonts()
    :   A list of all available fonts.

        Return type:
        :   list(str)

        Game Scenes:
        :   Flight

    size
    :   Font size.

        Attribute:
        :   Can be read or written

        Return type:
        :   int

        Game Scenes:
        :   Flight

    character\_size
    :   Character size.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    style
    :   Font style.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`UI.FontStyle`](../ui/text.md#UI.FontStyle "UI.FontStyle")

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

    alignment
    :   Alignment.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`UI.TextAlignment`](../ui/text.md#UI.TextAlignment "UI.TextAlignment")

        Game Scenes:
        :   Flight

    line\_spacing
    :   Line spacing.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    anchor
    :   Anchor.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`UI.TextAnchor`](../ui/text.md#UI.TextAnchor "UI.TextAnchor")

        Game Scenes:
        :   Flight
