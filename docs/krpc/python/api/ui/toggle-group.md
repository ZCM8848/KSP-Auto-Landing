# Toggle Group

class ToggleGroup
:   A set of toggles of which only one can be checked at a time, which is what turns
    a set of [`Toggle`](./toggle.md#UI.Toggle "UI.Toggle") controls into radio buttons.
    Added to a [`Canvas`](./canvas.md#UI.Canvas "UI.Canvas") or a [`Panel`](./panel.md#UI.Panel "UI.Panel").

    > **Note**
    >
    > A group does not contain its toggles, it only refers to them, so toggles can be
    > grouped independently of how they are laid out. A column of radio buttons in a grid
    > of them is one group, even though each row is laid out separately.

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

    selected
    :   The toggle in the group that is checked, or `None` if none of them are.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Toggle`](./toggle.md#UI.Toggle "UI.Toggle") or None

    allow\_switch\_off
    :   Whether all of the toggles in the group are allowed to be unchecked at once.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        > **Note**
        >
        > When this is false, the user cannot clear the checked toggle, they can only
        > check a different one. It does not stop a client setting
        > [`Toggle.checked`](./toggle.md#UI.Toggle.checked "UI.Toggle.checked"), which is obeyed either way.
        > A group starts out with no toggle checked either way.

    remove()
    :   Remove the UI object.
