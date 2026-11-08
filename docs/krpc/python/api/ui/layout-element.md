# Layout Element

class LayoutElement
:   How much space a user interface element asks a [`Layout`](./layout.md#UI.Layout "UI.Layout") for.

    > **Note**
    >
    > A layout gives every element its minimum size first. Any space left over is shared
    > out to bring elements up to their preferred size, and anything still left over is
    > shared between the elements with a flexible size, in proportion to it.

    min\_size
    :   The smallest size the element can be given, in pixels. A negative value in either
        direction leaves that direction unconstrained.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float)

    preferred\_size
    :   The size the element asks for, in pixels. A negative value in either direction
        leaves that direction unconstrained.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float)

    flexible\_size
    :   The share the element takes of the space left over once every element has its
        preferred size. A negative value in either direction takes no share.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float)

    ignore\_layout
    :   Whether the layout leaves the element out and lets it be positioned by hand.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool
