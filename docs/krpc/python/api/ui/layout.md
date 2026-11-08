# Layout

class Layout
:   Arranges the elements of a panel automatically, so that they do not have to be
    positioned one by one. See [`Panel.add_horizontal_layout()`](./panel.md#UI.Panel.add_horizontal_layout "UI.Panel.add_horizontal_layout"),
    [`Panel.add_vertical_layout()`](./panel.md#UI.Panel.add_vertical_layout "UI.Panel.add_vertical_layout") and [`Panel.add_grid_layout()`](./panel.md#UI.Panel.add_grid_layout "UI.Panel.add_grid_layout").

    spacing
    :   The gap left between the elements, in pixels. A grid layout uses it in both
        directions.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

    padding
    :   The gap left inside the edges of the panel.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`Padding`](#UI.Padding "UI.Padding")

    child\_alignment
    :   Where the elements are placed within the space the layout has to fill.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`TextAnchor`](./text.md#UI.TextAnchor "UI.TextAnchor")

    cell\_size
    :   The size of each cell of a grid layout, in pixels.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float)

        > **Note**
        >
        > Only applies to a grid layout. Throws an exception for the other layouts, whose
        > elements are sized individually using their layout elements.

    constraint
    :   What fixes the shape of a grid layout.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`GridConstraint`](#UI.GridConstraint "UI.GridConstraint")

        > **Note**
        >
        > Only applies to a grid layout. Throws an exception for the other layouts.

    constraint\_count
    :   The number of columns or rows that [`Layout.constraint`](#UI.Layout.constraint "UI.Layout.constraint") fixes, ignored when
        the constraint is [`GridConstraint.flexible`](#UI.GridConstraint.flexible "UI.GridConstraint.flexible").

        Attribute:
        :   Can be read or written

        Return type:
        :   int

        > **Note**
        >
        > Only applies to a grid layout. Throws an exception for the other layouts.

class Padding(*left*, *right*, *top*, *bottom*)
:   The gap left inside each edge of a layout, in whole pixels.

    > **Note**
    >
    > See [`Layout.padding`](#UI.Layout.padding "UI.Layout.padding").

    left
    :   The gap inside the left edge.

        Return type:
        :   int

    right
    :   The gap inside the right edge.

        Return type:
        :   int

    top
    :   The gap inside the top edge.

        Return type:
        :   int

    bottom
    :   The gap inside the bottom edge.

        Return type:
        :   int

class GridConstraint
:   What fixes the shape of a grid layout.

    flexible
    :   Fit as many columns and rows as the size of the panel allows.

    fixed\_column\_count
    :   Use a fixed number of columns, and as many rows as are needed.

    fixed\_row\_count
    :   Use a fixed number of rows, and as many columns as are needed.
