# UI

Provides functionality for drawing and interacting with in-game user interface elements.

> **Note**
>
> For drawing 3D objects in the flight scene, see the Drawing service.
> User interface elements created by a client are removed when that client
> disconnects.

stock\_canvas
:   The stock UI canvas.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   [`Canvas`](./canvas.md#UI.Canvas "UI.Canvas")

static add\_canvas()
:   Add a new canvas.

    Return type:
    :   [`Canvas`](./canvas.md#UI.Canvas "UI.Canvas")

    > **Note**
    >
    > If you want to add UI elements to KSPs stock UI canvas, use [`stock_canvas`](#UI.stock_canvas "UI.stock_canvas").

static message(*content*[, *duration=1.0*][, *position=MessagePosition(1)*][, *color=(1.0*, *0.92*, *0.016)*][, *size=20.0*])
:   Display a message on the screen.

    Parameters:
    :   - **content** (*str*) – Message content.
        - **duration** (*float*) – Duration before the message disappears, in seconds.
        - **position** ([*MessagePosition*](#UI.MessagePosition "UI.MessagePosition")) – Position to display the message.
        - **color** (*tuple*) – The color of the message.
        - **size** (*float*) – Size of the message, differs per position.

    > **Note**
    >
    > The message appears just like a stock message, for example quicksave or quickload messages.

static clear([*client\_only=False*])
:   Remove all user interface elements.

    Parameters:
    :   **client\_only** (*bool*) – If true, only remove objects created by the calling client.

class MessagePosition
:   Message position.

    top\_left
    :   Top left.

    top\_center
    :   Top center.

    top\_right
    :   Top right.

    bottom\_center
    :   Bottom center.
