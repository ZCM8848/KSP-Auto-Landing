# Text

class Text
:   A text label.
    Added to a [`Canvas`](./canvas.md#UI.Canvas "UI.Canvas") or a [`Panel`](./panel.md#UI.Panel "UI.Panel").

    rect\_transform
    :   The rect transform for the UI object.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`RectTransform`](./rect-transform.md#UI.RectTransform "UI.RectTransform")

    layout\_element
    :   How much space the UI object asks a layout for.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`LayoutElement`](./layout-element.md#UI.LayoutElement "UI.LayoutElement")

        > **Note**
        >
        > Only has an effect when the object is in a panel that has a layout. A canvas is
        > not inside a layout and does not have one.

    visible
    :   Whether the UI object is visible.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        > **Note**
        >
        > This is the object’s own setting, which is what it was last set to. An object is
        > only drawn when everything it is inside is visible as well, so an interface can
        > be built with its parts visible, inside a panel that is not, and shown all at
        > once by making that panel visible.

    content
    :   The text string

        Attribute:
        :   Can be read or written

        Return type:
        :   str

    word\_wrap
    :   Whether the text wraps onto further lines when it is wider than the space it
        has. Text that does not wrap carries on past the edge, and its preferred size
        is that of a single line, so a changing value leaves a layout in place.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    font
    :   Name of the font

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        > **Note**
        >
        > Empty if the game provided no font to draw the text with, in which case it is
        > not drawn. See [`Text.available_fonts`](#UI.Text.available_fonts "UI.Text.available_fonts") for the names that can be set.

    available\_fonts
    :   A list of all available fonts.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list(str)

    size
    :   Font size.

        Attribute:
        :   Can be read or written

        Return type:
        :   int

    style
    :   Font style.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`FontStyle`](#UI.FontStyle "UI.FontStyle")

    color
    :   The color the text is drawn in.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`StdLib.ColorRGBA`](../stdlib.md#StdLib.ColorRGBA "StdLib.ColorRGBA")

    alignment
    :   Alignment.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`TextAnchor`](#UI.TextAnchor "UI.TextAnchor")

    line\_spacing
    :   Line spacing.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

    remove()
    :   Remove the UI object.

class FontStyle
:   Font style.

    normal
    :   Normal.

    bold
    :   Bold.

    italic
    :   Italic.

    bold\_and\_italic
    :   Bold and italic.

class TextAlignment
:   Text alignment.

    left
    :   Left aligned.

    right
    :   Right aligned.

    center
    :   Center aligned.

class TextAnchor
:   Text alignment.

    lower\_center
    :   Lower center.

    lower\_left
    :   Lower left.

    lower\_right
    :   Lower right.

    middle\_center
    :   Middle center.

    middle\_left
    :   Middle left.

    middle\_right
    :   Middle right.

    upper\_center
    :   Upper center.

    upper\_left
    :   Upper left.

    upper\_right
    :   Upper right.
