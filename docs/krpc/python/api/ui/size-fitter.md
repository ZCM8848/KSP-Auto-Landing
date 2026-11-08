# Size Fitter

class SizeFitter
:   Sizes a panel to fit what it contains, rather than the other way round.
    See [`Panel.size_fitter`](./panel.md#UI.Panel.size_fitter "UI.Panel.size_fitter").

    horizontal\_fit
    :   How the width of the panel is chosen.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`ContentSizeFit`](#UI.ContentSizeFit "UI.ContentSizeFit")

    vertical\_fit
    :   How the height of the panel is chosen.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`ContentSizeFit`](#UI.ContentSizeFit "UI.ContentSizeFit")

class ContentSizeFit
:   How a size fitter sizes a panel along one direction.

    unconstrained
    :   Leave the size alone.

    min\_size
    :   Use the smallest size the contents will fit in.

    preferred\_size
    :   Use the size the contents ask for.
