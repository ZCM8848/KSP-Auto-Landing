# Rect Transform

class RectTransform
:   A Unity engine Rect Transform for a UI object.
    See the [Unity manual](https://docs.unity3d.com/Manual/class-RectTransform.html) for more details.

    position
    :   Position of the rectangles pivot point relative to the anchors.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float)

    local\_position
    :   Position of the rectangles pivot point relative to the anchors.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

    size
    :   Width and height of the rectangle.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float)

    upper\_right
    :   Position of the rectangles upper right corner relative to the anchors.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float)

    lower\_left
    :   Position of the rectangles lower left corner relative to the anchors.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float)

    anchor
    :   Set the minimum and maximum anchor points as a fraction of the size of the parent rectangle.

        Attribute:
        :   Write-only, cannot be read

        Return type:
        :   tuple(float, float)

    anchor\_max
    :   The anchor point for the lower left corner of the rectangle defined as a fraction of the size of the parent rectangle.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float)

    anchor\_min
    :   The anchor point for the upper right corner of the rectangle defined as a fraction of the size of the parent rectangle.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float)

    pivot
    :   Location of the pivot point around which the rectangle rotates, defined as a fraction of the size of the rectangle itself.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float)

    rotation
    :   Rotation, as a quaternion, of the object around its pivot point.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float, float)

    scale
    :   Scale factor applied to the object in the x, y and z dimensions.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)
