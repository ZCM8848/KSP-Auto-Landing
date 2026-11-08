# Panel

class Panel
:   A container for user interface elements.
    Added to a [`Canvas`](./canvas.md#UI.Canvas "UI.Canvas") or another panel.

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

    style
    :   How the background of the panel is drawn. A box is the frame to group related
        controls in, and is how a group box is made.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`PanelStyle`](#UI.PanelStyle "UI.PanelStyle")

    color
    :   The color the background of the panel is tinted with.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`StdLib.ColorRGBA`](../stdlib.md#StdLib.ColorRGBA "StdLib.ColorRGBA")

        > **Note**
        >
        > Has no effect while the style of the panel is [`PanelStyle.none`](#UI.PanelStyle.none "UI.PanelStyle.none"),
        > as the panel is then not drawn at all.

    draggable
    :   Whether the user can move the panel by dragging it.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        > **Note**
        >
        > Dragging anywhere on the panel moves it, including over the elements it
        > contains, unless one of them takes the pointer for itself. Give a panel a title
        > bar of its own if only part of it should move it.
        > A panel whose style is [`PanelStyle.none`](#UI.PanelStyle.none "UI.PanelStyle.none") is not drawn, so it takes
        > no pointer events of its own and can only be dragged by the elements it
        > contains.

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
        :   [`Panel`](#UI.Panel "UI.Panel")

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

    layout
    :   The layout that arranges the elements of the panel, or `None` if they are
        positioned by hand.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Layout`](./layout.md#UI.Layout "UI.Layout") or None

    add\_horizontal\_layout()
    :   Arrange the elements of the panel in a row.

        Return type:
        :   [`Layout`](./layout.md#UI.Layout "UI.Layout")

    add\_vertical\_layout()
    :   Arrange the elements of the panel in a column.

        Return type:
        :   [`Layout`](./layout.md#UI.Layout "UI.Layout")

    add\_grid\_layout()
    :   Arrange the elements of the panel in a grid of equally sized cells.

        Return type:
        :   [`Layout`](./layout.md#UI.Layout "UI.Layout")

        > **Note**
        >
        > The number of columns follows from the size of the panel and the size of a cell.
        > Use [`Layout.constraint`](./layout.md#UI.Layout.constraint "UI.Layout.constraint") to fix it instead.

    size\_fitter
    :   Sizes the panel to fit what it contains.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`SizeFitter`](./size-fitter.md#UI.SizeFitter "UI.SizeFitter")

        > **Note**
        >
        > Both directions start out unconstrained, which leaves the size of the panel
        > alone.

    bring\_to\_front()
    :   Draw the panel in front of the other elements beside it. Elements are drawn
        in the order they were added, so of two overlapping panels the newer hides
        part of the older until the older is brought to the front.

        > **Note**
        >
        > A draggable panel also brings itself to the front when the user starts
        > dragging it, the way a window comes to the front when it is clicked.

    remove()
    :   Remove the UI object.

class PanelStyle
:   How the background of a panel is drawn.

    window
    :   As a window, the heavier of the two frames the game uses.

    box
    :   As a box, the lighter frame the game groups related controls in.

    none
    :   Not drawn at all, leaving whatever is behind the panel showing through.
