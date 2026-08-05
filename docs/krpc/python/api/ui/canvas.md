# Canvas

class Canvas
:   A canvas for user interface elements. See [`stock_canvas`](./ui.md#UI.stock_canvas "UI.stock_canvas") and [`add_canvas()`](./ui.md#UI.add_canvas "UI.add_canvas").

    rect\_transform
    :   The rect transform for the canvas.

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
    :   Create a new container for user interface elements.

        Parameters:
        :   **visible** (*bool*) – Whether the panel is visible.

        Return type:
        :   [`Panel`](./panel.md#UI.Panel "UI.Panel")

    add\_text(*content*[, *visible=True*])
    :   Add text to the canvas.

        Parameters:
        :   - **content** (*str*) – The text.
            - **visible** (*bool*) – Whether the text is visible.

        Return type:
        :   [`Text`](./text.md#UI.Text "UI.Text")

    add\_input\_field([*visible=True*])
    :   Add an input field to the canvas.

        Parameters:
        :   **visible** (*bool*) – Whether the input field is visible.

        Return type:
        :   [`InputField`](./input-field.md#UI.InputField "UI.InputField")

    add\_button(*content*[, *visible=True*])
    :   Add a button to the canvas.

        Parameters:
        :   - **content** (*str*) – The label for the button.
            - **visible** (*bool*) – Whether the button is visible.

        Return type:
        :   [`Button`](./button.md#UI.Button "UI.Button")

    remove()
    :   Remove the UI object.
