# Image

class Image
:   A picture, or a plain colored rectangle.
    Added to a [`Canvas`](./canvas.md#UI.Canvas "UI.Canvas") or a [`Panel`](./panel.md#UI.Panel "UI.Panel").

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
    :   The picture to show, as the contents of a PNG or JPEG file. Set it to an empty
        array to show a plain rectangle instead.

        Attribute:
        :   Can be read or written

        Return type:
        :   bytes

        > **Note**
        >
        > Reading this returns what was last set, not what is on the screen, and returns
        > an empty array if no picture has been set. A picture drawn with
        > [`Image.set_pixels()`](#UI.Image.set_pixels "UI.Image.set_pixels") is not returned here either.

    set\_pixels(*data*, *width*, *height*)
    :   Draw the picture from raw pixels rather than a file, so that a client can
        draw whatever it likes, a graph for instance, and redraw it as often as it
        likes.

        Parameters:
        :   - **data** (*bytes*) – The pixels, 4 bytes per pixel: red, green, blue and alpha, each 0 to 255. Pixels run left to right and rows run top to bottom, the way image files store them, so the array must hold width times height times 4 bytes.
            - **width** (*int*) – How many pixels wide the picture is.
            - **height** (*int*) – How many pixels tall the picture is.

        > **Note**
        >
        > Redrawing at an unchanged size draws into the picture already there, so a
        > picture redrawn every update does not build a new one each time.

    update\_pixels(*data*, *x*, *y*, *width*, *height*)
    :   Redraw part of a picture drawn with [`Image.set_pixels()`](#UI.Image.set_pixels "UI.Image.set_pixels"): a block of the
        given size, with its top left corner x pixels in from the left edge and y
        pixels down from the top. The rest of the picture is left as it is, so a
        change to a small part of a large picture, one new column of a scrolling
        graph for instance, only sends the part that changed.

        Parameters:
        :   - **data** (*bytes*) – The pixels of the block, laid out as for [`Image.set_pixels()`](#UI.Image.set_pixels "UI.Image.set_pixels"): 4 bytes per pixel, rows top to bottom, width times height times 4 bytes in all.
            - **x** (*int*) – How many pixels the block sits in from the left edge.
            - **y** (*int*) – How many pixels the block sits down from the top.
            - **width** (*int*) – How many pixels wide the block is.
            - **height** (*int*) – How many pixels tall the block is.

        > **Note**
        >
        > The image must already be showing a picture drawn from raw pixels, which is
        > what sets the size, and the block must fit inside it. A picture loaded from
        > a file cannot be redrawn this way: the file contents a client reads back
        > would no longer be what is on the screen.

    color
    :   The color the image is tinted with, or the color of the rectangle when no
        picture is set.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`StdLib.ColorRGBA`](../stdlib.md#StdLib.ColorRGBA "StdLib.ColorRGBA")

    remove()
    :   Remove the UI object.
