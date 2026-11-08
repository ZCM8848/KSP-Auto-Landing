# Button

class Button
:   A button that the user can click.
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

    text
    :   The text for the button.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Text`](./text.md#UI.Text "UI.Text")

    clicked
    :   Whether the button has been clicked.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        > **Note**
        >
        > This property is set to true when the user clicks the button.
        > A client script should reset the property to false in order to detect subsequent button presses.

    pressed
    :   Whether the user is holding the button down right now.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        > **Note**
        >
        > True from the button being pressed until the pointer is released, however far
        > it has moved by then. Polling this repeats an action for as long as the
        > button is held, for nudging a value; [`Button.clicked`](#UI.Button.clicked "UI.Button.clicked") reports each
        > completed click instead.

    interactable
    :   Whether the control responds to the user.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        > **Note**
        >
        > A control that is not interactable is drawn grayed out and ignores the user,
        > but is still visible. Set its visibility to false to hide it instead.

    color
    :   The color the control is tinted with. White, the default, leaves the control drawn
        as the skin has it.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`StdLib.ColorRGBA`](../stdlib.md#StdLib.ColorRGBA "StdLib.ColorRGBA")

        > **Note**
        >
        > The tint multiplies the sprite the control is drawn with, in every state, so
        > a control can show a state of its own, an engaged autopilot for instance, by
        > color. It is applied to the part of the control that responds to the user:
        > the body of a button, dropdown or input field, the box of a toggle, and the
        > handle of a slider. A control’s label is tinted through the color of its
        > text.

    tooltip
    :   A short piece of text shown beside the pointer while it rests on the
        control. Empty, the default, shows nothing.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

    remove()
    :   Remove the UI object.
