# Text

class Text
:   A text label. See [`Panel.add_text()`](./panel.md#UI.Panel.add_text "UI.Panel.add_text").

    rect\_transform
    :   The rect transform for the text.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`RectTransform`](./rect-transform.md#UI.RectTransform "UI.RectTransform")

    visible
    :   Whether the UI object is visible.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    content
    :   The text string

        Attribute:
        :   Can be read or written

        Return type:
        :   str

    font
    :   Name of the font

        Attribute:
        :   Can be read or written

        Return type:
        :   str

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
    :   Set the color

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

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
