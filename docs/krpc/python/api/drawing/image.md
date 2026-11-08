# Image

class Image
:   An image that markers can be drawn with. Created using [`add_image()`](./drawing.md#Drawing.add_image "Drawing.add_image").

    > **Note**
    >
    > An image is kept across scene changes, until it is removed or the client that
    > created it disconnects.

    width
    :   The width of the image, in pixels.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   int

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    height
    :   The height of the image, in pixels.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   int

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph

    remove()
    :   Remove the image. A marker drawn with it goes back to its own icon.

        Game Scenes:
        :   Flight, Tracking Station, Editor Vab, Editor Sph
