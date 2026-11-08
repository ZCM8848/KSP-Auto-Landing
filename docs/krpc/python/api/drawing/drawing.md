# Drawing

Provides functionality for drawing objects in the flight scene, the map view, the editor
and the tracking station.

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
    :   Flight, Tracking Station, Editor Vab, Editor Sph

static add\_direction(*direction*, *reference\_frame*[, *length=1e+01*][, *visible=True*])
:   Draw a direction vector in the scene, starting from the origin of the given reference frame.

    Parameters:
    :   - **direction** (*tuple*) – Direction to draw the line in.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame that the direction is in and defines the start position.
        - **length** (*float*) – The length of the line.
        - **visible** (*bool*) – Whether the line is visible.

    Return type:
    :   [`Line`](./line.md#Drawing.Line "Drawing.Line")

    Game Scenes:
    :   Flight, Tracking Station, Editor Vab, Editor Sph

static add\_direction\_from\_com(*direction*, *reference\_frame*[, *length=1e+01*][, *visible=True*])
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

static add\_axes(*reference\_frame*[, *length=1e+01*][, *visible=True*])
:   Draw the axes of a reference frame in the scene, as arrows from its origin.

    Parameters:
    :   - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame.
        - **length** (*float*) – The length of each axis.
        - **visible** (*bool*) – Whether the axes are visible.

    Returns:
    :   The lines for the x, y and z axes, colored red, green and blue.

    Return type:
    :   list([`Line`](./line.md#Drawing.Line "Drawing.Line"))

    Game Scenes:
    :   Flight, Tracking Station, Editor Vab, Editor Sph

static add\_polygon(*vertices*, *reference\_frame*[, *visible=True*])
:   Draw a polygon in the scene, defined by a list of vertices.

    Parameters:
    :   - **vertices** (*list*) – Vertices of the polygon.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame that the vertices are in.
        - **visible** (*bool*) – Whether the polygon is visible.

    Return type:
    :   [`Polygon`](./polygon.md#Drawing.Polygon "Drawing.Polygon")

    Game Scenes:
    :   Flight, Tracking Station, Editor Vab, Editor Sph

static add\_polyline(*vertices*, *reference\_frame*[, *visible=True*])
:   Draw an open line through a list of vertices in the scene.

    Parameters:
    :   - **vertices** (*list*) – Vertices of the line.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame that the vertices are in.
        - **visible** (*bool*) – Whether the line is visible.

    Return type:
    :   [`Polyline`](./polyline.md#Drawing.Polyline "Drawing.Polyline")

    Game Scenes:
    :   Flight, Tracking Station, Editor Vab, Editor Sph

static add\_circle(*center*, *normal*, *radius*, *reference\_frame*[, *visible=True*])
:   Draw a circle in the scene.

    Parameters:
    :   - **center** (*tuple*) – Position of the center of the circle.
        - **normal** (*tuple*) – Normal of the plane of the circle.
        - **radius** (*float*) – Radius of the circle, in meters.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame that the position and normal are in.
        - **visible** (*bool*) – Whether the circle is visible.

    Return type:
    :   [`Circle`](./circle.md#Drawing.Circle "Drawing.Circle")

    Game Scenes:
    :   Flight, Tracking Station, Editor Vab, Editor Sph

    > **Note**
    >
    > Throws an exception if the normal is zero.

static add\_arc(*center*, *normal*, *start\_direction*, *angle*, *radius*, *reference\_frame*[, *visible=True*])
:   Draw an arc of a circle in the scene.

    Parameters:
    :   - **center** (*tuple*) – Position of the center of the arc.
        - **normal** (*tuple*) – Normal of the plane of the arc.
        - **start\_direction** (*tuple*) – Direction from the center to the start of the arc.
        - **angle** (*float*) – Angle of the arc, in radians.
        - **radius** (*float*) – Radius of the arc, in meters.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame that the position and directions are in.
        - **visible** (*bool*) – Whether the arc is visible.

    Return type:
    :   [`Arc`](./arc.md#Drawing.Arc "Drawing.Arc")

    Game Scenes:
    :   Flight, Tracking Station, Editor Vab, Editor Sph

    > **Note**
    >
    > The arc turns counterclockwise about the normal, seen from the side it points to.
    > Throws an exception if the normal is zero, or the start direction is parallel to it.

static add\_orbit(*orbit*[, *visible=True*])
:   Draw the path of an orbit in the scene.

    Parameters:
    :   - **orbit** ([*SpaceCenter.Orbit*](../space-center/orbit.md#SpaceCenter.Orbit "SpaceCenter.Orbit")) – The orbit.
        - **visible** (*bool*) – Whether the path is visible.

    Return type:
    :   [`OrbitPath`](./orbit-path.md#Drawing.OrbitPath "Drawing.OrbitPath")

    Game Scenes:
    :   Flight, Tracking Station

    > **Note**
    >
    > The path follows the orbit as it changes, and ends where the orbit leaves the
    > sphere of influence of its body.

static add\_point(*position*, *reference\_frame*[, *visible=True*])
:   Draw a point in the scene, as a dot that keeps its size on screen.

    Parameters:
    :   - **position** (*tuple*) – Position of the point.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame that the position is in.
        - **visible** (*bool*) – Whether the point is visible.

    Return type:
    :   [`Point`](./point.md#Drawing.Point "Drawing.Point")

    Game Scenes:
    :   Flight, Tracking Station, Editor Vab, Editor Sph

static add\_box(*center*, *size*, *reference\_frame*[, *rotation=(0.0*, *0.0*, *0.0*, *1.0)*][, *visible=True*])
:   Draw the edges of a box in the scene.

    Parameters:
    :   - **center** (*tuple*) – Position of the center of the box.
        - **size** (*tuple*) – Lengths of the sides of the box along its x, y and z axes, in meters.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame that the position and rotation are in.
        - **rotation** (*tuple*) – Rotation of the box, as a quaternion.
        - **visible** (*bool*) – Whether the box is visible.

    Return type:
    :   [`Box`](./box.md#Drawing.Box "Drawing.Box")

    Game Scenes:
    :   Flight, Tracking Station, Editor Vab, Editor Sph

static add\_sphere(*center*, *radius*, *reference\_frame*[, *visible=True*])
:   Draw a solid sphere in the scene.

    Parameters:
    :   - **center** (*tuple*) – Position of the center of the sphere.
        - **radius** (*float*) – Radius of the sphere, in meters.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame that the position is in.
        - **visible** (*bool*) – Whether the sphere is visible.

    Return type:
    :   [`Sphere`](./sphere.md#Drawing.Sphere "Drawing.Sphere")

    Game Scenes:
    :   Flight, Tracking Station, Editor Vab, Editor Sph

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
    :   Flight, Tracking Station, Editor Vab, Editor Sph

static add\_navball\_marker(*direction*, *reference\_frame*[, *visible=True*])
:   Draw a marker on the navball, pointing in the given direction.

    Parameters:
    :   - **direction** (*tuple*) – Direction that the marker points in.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame that the direction is in.
        - **visible** (*bool*) – Whether the marker is visible.

    Return type:
    :   [`NavballMarker`](./navball-marker.md#Drawing.NavballMarker "Drawing.NavballMarker")

    Game Scenes:
    :   Flight

static add\_map\_marker(*position*, *reference\_frame*[, *icon=MapMarkerIcon.generic*][, *caption=''*][, *visible=True*])
:   Draw a marker in the map view, with one of the game’s own map icons.

    Parameters:
    :   - **position** (*tuple*) – Position of the marker.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame that the position is in.
        - **icon** ([*MapMarkerIcon*](./map-marker.md#Drawing.MapMarkerIcon "Drawing.MapMarkerIcon")) – The icon that the marker is drawn with.
        - **caption** (*str*) – The caption shown when the mouse is over the marker.
        - **visible** (*bool*) – Whether the marker is visible.

    Return type:
    :   [`MapMarker`](./map-marker.md#Drawing.MapMarker "Drawing.MapMarker")

    Game Scenes:
    :   Flight, Tracking Station

static add\_image(*data*)
:   Load an image that markers can be drawn with, from the contents of a PNG or JPEG
    file. The transparency of a PNG image is kept.

    Parameters:
    :   **data** (*bytes*) – The contents of the image file.

    Return type:
    :   [`Image`](./image.md#Drawing.Image "Drawing.Image")

    Game Scenes:
    :   Flight, Tracking Station, Editor Vab, Editor Sph

    > **Note**
    >
    > Throws an exception if the data is not a PNG or JPEG image.

static clear([*client\_only=False*])
:   Remove all objects being drawn.

    Parameters:
    :   **client\_only** (*bool*) – If true, only remove objects created by the calling client.

    Game Scenes:
    :   Flight, Tracking Station, Editor Vab, Editor Sph
