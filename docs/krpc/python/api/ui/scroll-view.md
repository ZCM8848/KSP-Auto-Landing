# Scroll View

class ScrollView
:   A view onto a panel that is larger than the space available for it, with scroll bars
    to move around it.
    Added to a [`Canvas`](./canvas.md#UI.Canvas "UI.Canvas") or a [`Panel`](./panel.md#UI.Panel "UI.Panel").

    > **Note**
    >
    > Add the elements to [`ScrollView.content`](#UI.ScrollView.content "UI.ScrollView.content"), not to the scroll view itself. Giving
    > the content a layout and setting its size fitter to the preferred size is the
    > simplest way to have it grow to hold what is put in it.

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
    :   The panel that is scrolled, which the elements to show are added to.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Panel`](./panel.md#UI.Panel "UI.Panel")

    horizontal
    :   Whether the view can be scrolled from side to side.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    vertical
    :   Whether the view can be scrolled up and down.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    remove()
    :   Remove the UI object.
