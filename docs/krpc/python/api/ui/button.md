# Button

class Button
:   A text label. See [`Panel.add_button()`](./panel.md#UI.Panel.add_button "UI.Panel.add_button").

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

    remove()
    :   Remove the UI object.
