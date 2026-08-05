# Drawing

Provides functionality for drawing objects in the flight scene.

> **Note**
>
> For drawing and interacting with the user interface, see the UI service.
> Objects drawn by a client are removed when that client disconnects.

static add\_line(*start*, *end*, *reference\_frame*[, *visible=True*])
:   Draw a line in the scene.

    Parameters:
    :   - **start** (*tuple*) – Position of the start of the line.
        - **end** (*tuple*) – Position of the end of the line.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame that the positions are in.
        - **visible** (*bool*) – Whether the line is visible.

    Return type:
    :   [`Line`](./line.md#Drawing.Line "Drawing.Line")

    Game Scenes:
    :   Flight

static add\_direction(*direction*, *reference\_frame*[, *length=10.0*][, *visible=True*])
:   Draw a direction vector in the scene, starting from the origin of the given reference frame.

    Parameters:
    :   - **direction** (*tuple*) – Direction to draw the line in.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame that the direction is in and defines the start position.
        - **length** (*float*) – The length of the line.
        - **visible** (*bool*) – Whether the line is visible.

    Return type:
    :   [`Line`](./line.md#Drawing.Line "Drawing.Line")

    Game Scenes:
    :   Flight

static add\_direction\_from\_com(*direction*, *reference\_frame*[, *length=10.0*][, *visible=True*])
:   Draw a direction vector in the scene, from the center of mass of the active vessel.

    Parameters:
    :   - **direction** (*tuple*) – Direction to draw the line in.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame that the direction is in.
        - **length** (*float*) – The length of the line.
        - **visible** (*bool*) – Whether the line is visible.

    Return type:
    :   [`Line`](./line.md#Drawing.Line "Drawing.Line")

    Game Scenes:
    :   Flight

static add\_polygon(*vertices*, *reference\_frame*[, *visible=True*])
:   Draw a polygon in the scene, defined by a list of vertices.

    Parameters:
    :   - **vertices** (*list*) – Vertices of the polygon.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame that the vertices are in.
        - **visible** (*bool*) – Whether the polygon is visible.

    Return type:
    :   [`Polygon`](./polygon.md#Drawing.Polygon "Drawing.Polygon")

    Game Scenes:
    :   Flight

static add\_text(*text*, *reference\_frame*, *position*, *rotation*[, *visible=True*])
:   Draw text in the scene.

    Parameters:
    :   - **text** (*str*) – The string to draw.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame that the text position is in.
        - **position** (*tuple*) – Position of the text.
        - **rotation** (*tuple*) – Rotation of the text, as a quaternion.
        - **visible** (*bool*) – Whether the text is visible.

    Return type:
    :   [`Text`](./text.md#Drawing.Text "Drawing.Text")

    Game Scenes:
    :   Flight

static clear([*client\_only=False*])
:   Remove all objects being drawn.

    Parameters:
    :   **client\_only** (*bool*) – If true, only remove objects created by the calling client.

    Game Scenes:
    :   Flight
