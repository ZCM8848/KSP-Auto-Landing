# Panel

class Panel
:   A container for user interface elements. See [`Canvas.add_panel()`](./canvas.md#UI.Canvas.add_panel "UI.Canvas.add_panel").

    rect\_transform
    :   The rect transform for the panel.

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

    add\_panel([*visible=True*])
    :   Create a panel within this panel.

        Parameters:
        :   **visible** (*bool*) – Whether the new panel is visible.

        Return type:
        :   [`Panel`](#UI.Panel "UI.Panel")

    add\_text(*content*[, *visible=True*])
    :   Add text to the panel.

        Parameters:
        :   - **content** (*str*) – The text.
            - **visible** (*bool*) – Whether the text is visible.

        Return type:
        :   [`Text`](./text.md#UI.Text "UI.Text")

    add\_input\_field([*visible=True*])
    :   Add an input field to the panel.

        Parameters:
        :   **visible** (*bool*) – Whether the input field is visible.

        Return type:
        :   [`InputField`](./input-field.md#UI.InputField "UI.InputField")

    add\_button(*content*[, *visible=True*])
    :   Add a button to the panel.

        Parameters:
        :   - **content** (*str*) – The label for the button.
            - **visible** (*bool*) – Whether the button is visible.

        Return type:
        :   [`Button`](./button.md#UI.Button "UI.Button")

    remove()
    :   Remove the UI object.
