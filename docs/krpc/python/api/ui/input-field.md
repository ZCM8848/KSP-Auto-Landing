# InputField

class InputField
:   An input field. See [`Panel.add_input_field()`](./panel.md#UI.Panel.add_input_field "UI.Panel.add_input_field").

    rect\_transform
    :   The rect transform for the input field.

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

    value
    :   The value of the input field.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

    text
    :   The text component of the input field.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Text`](./text.md#UI.Text "UI.Text")

        > **Note**
        >
        > Use [`InputField.value`](#UI.InputField.value "UI.InputField.value") to get and set the value in the field.
        > This object can be used to alter the style of the input field’s text.

    changed
    :   Whether the input field has been changed.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        > **Note**
        >
        > This property is set to true when the user modifies the value of the input field.
        > A client script should reset the property to false in order to detect subsequent changes.

    remove()
    :   Remove the UI object.
