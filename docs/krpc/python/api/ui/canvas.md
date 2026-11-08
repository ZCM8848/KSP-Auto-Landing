# Canvas

class Canvas
:   A canvas for user interface elements. See [`stock_canvas`](./ui.md#UI.stock_canvas "UI.stock_canvas") and [`add_canvas()`](./ui.md#UI.add_canvas "UI.add_canvas").

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

    add\_panel([*visible=True*])
    :   Create a panel within this object.

        Parameters:
        :   **visible** (*bool*) – Whether the new panel is visible.

        Return type:
        :   [`Panel`](./panel.md#UI.Panel "UI.Panel")

    add\_text(*content*[, *visible=True*])
    :   Add text to this object.

        Parameters:
        :   - **content** (*str*) – The text.
            - **visible** (*bool*) – Whether the text is visible.

        Return type:
        :   [`Text`](./text.md#UI.Text "UI.Text")

    add\_input\_field([*visible=True*])
    :   Add an input field to this object.

        Parameters:
        :   **visible** (*bool*) – Whether the input field is visible.

        Return type:
        :   [`InputField`](./input-field.md#UI.InputField "UI.InputField")

    add\_button(*content*[, *visible=True*])
    :   Add a button to this object.

        Parameters:
        :   - **content** (*str*) – The label for the button.
            - **visible** (*bool*) – Whether the button is visible.

        Return type:
        :   [`Button`](./button.md#UI.Button "UI.Button")

    add\_toggle(*content*[, *visible=True*])
    :   Add a toggle to this object.

        Parameters:
        :   - **content** (*str*) – The label for the toggle.
            - **visible** (*bool*) – Whether the toggle is visible.

        Return type:
        :   [`Toggle`](./toggle.md#UI.Toggle "UI.Toggle")

    add\_toggle\_group()
    :   Add a toggle group to this object, to make a set of toggles into radio buttons.

        Return type:
        :   [`ToggleGroup`](./toggle-group.md#UI.ToggleGroup "UI.ToggleGroup")

        > **Note**
        >
        > The group is not drawn and does not take up any space. The toggles that belong
        > to it can be added anywhere.

    add\_slider([*vertical=False*][, *visible=True*])
    :   Add a slider to this object.

        Parameters:
        :   - **vertical** (*bool*) – Whether the slider runs from bottom to top, rather than from left to right.
            - **visible** (*bool*) – Whether the slider is visible.

        Return type:
        :   [`Slider`](./slider.md#UI.Slider "UI.Slider")

    add\_dropdown([*visible=True*])
    :   Add a dropdown to this object.

        Parameters:
        :   **visible** (*bool*) – Whether the dropdown is visible.

        Return type:
        :   [`Dropdown`](./dropdown.md#UI.Dropdown "UI.Dropdown")

    add\_image([*visible=True*])
    :   Add an image to this object.

        Parameters:
        :   **visible** (*bool*) – Whether the image is visible.

        Return type:
        :   [`Image`](./image.md#UI.Image "UI.Image")

    add\_scroll\_view([*visible=True*])
    :   Add a scroll view to this object.

        Parameters:
        :   **visible** (*bool*) – Whether the scroll view is visible.

        Return type:
        :   [`ScrollView`](./scroll-view.md#UI.ScrollView "UI.ScrollView")

    remove()
    :   Remove the UI object.
