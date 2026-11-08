# Standard Library

A standard library of mathematics and printing for server side functions, and of the
colors other services use.

> **Note**
>
> Vectors are position and direction tuples, and rotations are unit quaternion tuples
> in the order x, y, z, w the SpaceCenter service uses. Angles are in radians unless
> stated otherwise.

pi
:   The constant pi.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   float

e
:   The constant e, the base of the natural logarithm.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   float

static abs(*x*)
:   The absolute value of a number.

    Parameters:
    :   **x** (*float*) – The number.

    Return type:
    :   float

static sign(*x*)
:   The sign of a number: -1 if it is negative, 1 if it is positive,
    and 0 if it is zero or not a number.

    Parameters:
    :   **x** (*float*) – The number.

    Return type:
    :   int

static copy\_sign(*x*, *y*)
:   The magnitude of the first number with the sign of the second.

    Parameters:
    :   - **x** (*float*) – The number giving the magnitude.
        - **y** (*float*) – The number giving the sign.

    Return type:
    :   float

static is\_nan(*x*)
:   Whether a number is not a number.

    Parameters:
    :   **x** (*float*) – The number.

    Return type:
    :   bool

static is\_finite(*x*)
:   Whether a number is neither infinite nor not a number.

    Parameters:
    :   **x** (*float*) – The number.

    Return type:
    :   bool

static floor(*x*)
:   The largest whole number less than or equal to the given number.

    Parameters:
    :   **x** (*float*) – The number.

    Return type:
    :   float

static ceiling(*x*)
:   The smallest whole number greater than or equal to the given number.

    Parameters:
    :   **x** (*float*) – The number.

    Return type:
    :   float

static round(*x*[, *digits=0*])
:   The given number rounded to the given number of decimal places,
    with a halfway value rounded to the nearer even number.

    Parameters:
    :   - **x** (*float*) – The number.
        - **digits** (*int*) – The number of decimal places to round to. Negative to round to tens, hundreds and beyond.

    Return type:
    :   float

    > **Note**
    >
    > Throws an exception if the rounded value is too large for a double.

static round\_half\_away\_from\_zero(*x*[, *digits=0*])
:   The given number rounded to the given number of decimal places,
    with a halfway value rounded away from zero.

    Parameters:
    :   - **x** (*float*) – The number.
        - **digits** (*int*) – The number of decimal places to round to. Negative to round to tens, hundreds and beyond.

    Return type:
    :   float

    > **Note**
    >
    > Throws an exception if the rounded value is too large for a double.

static round\_half\_up(*x*[, *digits=0*])
:   The given number rounded to the given number of decimal places,
    with a halfway value rounded toward positive infinity.

    Parameters:
    :   - **x** (*float*) – The number.
        - **digits** (*int*) – The number of decimal places to round to. Negative to round to tens, hundreds and beyond.

    Return type:
    :   float

    > **Note**
    >
    > Throws an exception if the rounded value is too large for a double.

static sqrt(*x*)
:   The square root of a number.

    Parameters:
    :   **x** (*float*) – The number.

    Return type:
    :   float

static hypot(*x*, *y*)
:   The length of the hypotenuse of a right triangle with the given sides.

    Parameters:
    :   - **x** (*float*) – The first side.
        - **y** (*float*) – The second side.

    Return type:
    :   float

static sin(*x*)
:   The sine of an angle, in radians.

    Parameters:
    :   **x** (*float*) – The angle.

    Return type:
    :   float

static cos(*x*)
:   The cosine of an angle, in radians.

    Parameters:
    :   **x** (*float*) – The angle.

    Return type:
    :   float

static tan(*x*)
:   The tangent of an angle, in radians.

    Parameters:
    :   **x** (*float*) – The angle.

    Return type:
    :   float

static asin(*x*)
:   The angle, in radians, whose sine is the given number.

    Parameters:
    :   **x** (*float*) – The number.

    Return type:
    :   float

static acos(*x*)
:   The angle, in radians, whose cosine is the given number.

    Parameters:
    :   **x** (*float*) – The number.

    Return type:
    :   float

static atan(*x*)
:   The angle, in radians, whose tangent is the given number.

    Parameters:
    :   **x** (*float*) – The number.

    Return type:
    :   float

static atan2(*y*, *x*)
:   The angle, in radians, between the positive x axis and the point (x, y).

    Parameters:
    :   - **y** (*float*) – The y coordinate.
        - **x** (*float*) – The x coordinate.

    Return type:
    :   float

static log(*x*[, *new\_base=2.718281828459045*])
:   The logarithm of a number, natural unless another base is given.

    Parameters:
    :   - **x** (*float*) – The number.
        - **new\_base** (*float*) – The base of the logarithm.

    Return type:
    :   float

static log10(*x*)
:   The base 10 logarithm of a number.

    Parameters:
    :   **x** (*float*) – The number.

    Return type:
    :   float

static exp(*x*)
:   The number e raised to the given power.

    Parameters:
    :   **x** (*float*) – The power.

    Return type:
    :   float

static min(*x*, *y*)
:   The smaller of two numbers.

    Parameters:
    :   - **x** (*float*) – The first number.
        - **y** (*float*) – The second number.

    Return type:
    :   float

static max(*x*, *y*)
:   The larger of two numbers.

    Parameters:
    :   - **x** (*float*) – The first number.
        - **y** (*float*) – The second number.

    Return type:
    :   float

static clamp(*x*, *min*, *max*)
:   The given number limited to the given range.

    Parameters:
    :   - **x** (*float*) – The number.
        - **min** (*float*) – The minimum of the range.
        - **max** (*float*) – The maximum of the range.

    Return type:
    :   float

    > **Note**
    >
    > Throws an exception if the minimum is greater than the maximum.

static lerp(*a*, *b*, *t*)
:   Linear interpolation between two numbers.

    Parameters:
    :   - **a** (*float*) – The number when t is 0.
        - **b** (*float*) – The number when t is 1.
        - **t** (*float*) – The interpolation parameter.

    Return type:
    :   float

    > **Note**
    >
    > The parameter is not limited to the range 0 to 1, and a value outside it
    > extrapolates.

static inverse\_lerp(*a*, *b*, *x*)
:   The interpolation parameter of a number between two others,
    limited to the range 0 to 1.

    Parameters:
    :   - **a** (*float*) – The number at 0.
        - **b** (*float*) – The number at 1.
        - **x** (*float*) – The number.

    Return type:
    :   float

    > **Note**
    >
    > Returns 0 if the two numbers are equal.

static degrees\_to\_radians(*x*)
:   The angle in radians of an angle in degrees.

    Parameters:
    :   **x** (*float*) – The angle in degrees.

    Return type:
    :   float

static radians\_to\_degrees(*x*)
:   The angle in degrees of an angle in radians.

    Parameters:
    :   **x** (*float*) – The angle in radians.

    Return type:
    :   float

static vector\_add(*u*, *v*)
:   The sum of two vectors.

    Parameters:
    :   - **u** (*tuple*) – The first vector.
        - **v** (*tuple*) – The second vector.

    Return type:
    :   tuple(float, float, float)

static vector\_subtract(*u*, *v*)
:   The difference of two vectors.

    Parameters:
    :   - **u** (*tuple*) – The first vector.
        - **v** (*tuple*) – The second vector.

    Return type:
    :   tuple(float, float, float)

static vector\_scale(*v*, *scale*)
:   A vector scaled by a number.

    Parameters:
    :   - **v** (*tuple*) – The vector.
        - **scale** (*float*) – The scaling factor.

    Return type:
    :   tuple(float, float, float)

static vector\_dot(*u*, *v*)
:   The dot product of two vectors.

    Parameters:
    :   - **u** (*tuple*) – The first vector.
        - **v** (*tuple*) – The second vector.

    Return type:
    :   float

static vector\_cross(*u*, *v*)
:   The cross product of two vectors.

    Parameters:
    :   - **u** (*tuple*) – The first vector.
        - **v** (*tuple*) – The second vector.

    Return type:
    :   tuple(float, float, float)

static vector\_magnitude(*v*)
:   The magnitude (length) of a vector.

    Parameters:
    :   **v** (*tuple*) – The vector.

    Return type:
    :   float

static vector\_normalize(*v*)
:   A vector scaled to a magnitude of 1.

    Parameters:
    :   **v** (*tuple*) – The vector.

    Return type:
    :   tuple(float, float, float)

    > **Note**
    >
    > Throws an exception if the vector is zero.

static vector\_distance(*u*, *v*)
:   The distance between two positions.

    Parameters:
    :   - **u** (*tuple*) – The first position.
        - **v** (*tuple*) – The second position.

    Return type:
    :   float

static vector\_angle(*u*, *v*)
:   The angle between two vectors, in radians, between 0 and pi.

    Parameters:
    :   - **u** (*tuple*) – The first vector.
        - **v** (*tuple*) – The second vector.

    Return type:
    :   float

    > **Note**
    >
    > Throws an exception if either vector is zero.

static vector\_lerp(*u*, *v*, *t*)
:   Linear interpolation between two vectors.

    Parameters:
    :   - **u** (*tuple*) – The vector when t is 0.
        - **v** (*tuple*) – The vector when t is 1.
        - **t** (*float*) – The interpolation parameter.

    Return type:
    :   tuple(float, float, float)

    > **Note**
    >
    > The parameter is not limited to the range 0 to 1, and a value outside it
    > extrapolates.

static quaternion\_multiply(*q*, *r*)
:   The composition of two rotations: the rotation r followed by the
    rotation q.

    Parameters:
    :   - **q** (*tuple*) – The second rotation.
        - **r** (*tuple*) – The first rotation.

    Return type:
    :   tuple(float, float, float, float)

    > **Note**
    >
    > Throws an exception if either quaternion is zero.

static quaternion\_inverse(*q*)
:   The inverse of a rotation.

    Parameters:
    :   **q** (*tuple*) – The rotation.

    Return type:
    :   tuple(float, float, float, float)

    > **Note**
    >
    > Throws an exception if the quaternion is zero.

static quaternion\_rotate\_vector(*q*, *v*)
:   A vector rotated by a rotation.

    Parameters:
    :   - **q** (*tuple*) – The rotation.
        - **v** (*tuple*) – The vector.

    Return type:
    :   tuple(float, float, float)

    > **Note**
    >
    > Throws an exception if the quaternion is zero.

static quaternion\_from\_axis\_angle(*axis*, *angle*)
:   The rotation about an axis by an angle.

    Parameters:
    :   - **axis** (*tuple*) – The axis of rotation.
        - **angle** (*float*) – The angle of rotation, in radians.

    Return type:
    :   tuple(float, float, float, float)

    > **Note**
    >
    > Throws an exception if the axis is zero.

static quaternion\_to\_axis\_angle(*q*)
:   The axis and angle of a rotation.

    Parameters:
    :   **q** (*tuple*) – The rotation.

    Return type:
    :   [`AxisAngle`](#StdLib.AxisAngle "StdLib.AxisAngle")

    > **Note**
    >
    > Throws an exception if the quaternion is zero.

static quaternion\_from\_to\_rotation(*u*, *v*)
:   The shortest rotation that takes one direction onto another.

    Parameters:
    :   - **u** (*tuple*) – The direction to rotate from.
        - **v** (*tuple*) – The direction to rotate to.

    Return type:
    :   tuple(float, float, float, float)

    > **Note**
    >
    > Opposite directions give a half turn about an arbitrary perpendicular axis.
    > Throws an exception if either vector is zero.

static quaternion\_look\_rotation(*forward*, *up*)
:   The rotation that takes the z axis onto a forward direction, and the y axis
    toward an up direction.

    Parameters:
    :   - **forward** (*tuple*) – The direction of the z axis.
        - **up** (*tuple*) – The direction the y axis points toward.

    Return type:
    :   tuple(float, float, float, float)

    > **Note**
    >
    > The up direction need not be perpendicular to the forward direction. Throws an
    > exception if either vector is zero, or if they are parallel.

static quaternion\_angle(*q*, *r*)
:   The angle of the rotation that takes one rotation to another,
    in radians, between 0 and pi.

    Parameters:
    :   - **q** (*tuple*) – The first rotation.
        - **r** (*tuple*) – The second rotation.

    Return type:
    :   float

    > **Note**
    >
    > Throws an exception if either quaternion is zero.

static quaternion\_slerp(*q*, *r*, *t*)
:   Spherical linear interpolation between two rotations.

    Parameters:
    :   - **q** (*tuple*) – The rotation when t is 0.
        - **r** (*tuple*) – The rotation when t is 1.
        - **t** (*float*) – The interpolation parameter.

    Return type:
    :   tuple(float, float, float, float)

    > **Note**
    >
    > The parameter is not limited to the range 0 to 1, and a value outside it
    > extrapolates. Throws an exception if either quaternion is zero.

static print(*message*)
:   Writes a message to the game’s log, for debugging.

    Parameters:
    :   **message** (*str*) – The message.

    > **Note**
    >
    > A function evaluated by a stream or an event prints on every update. The message is
    > written at the Info level, so a server log level of Warning or Error hides it.

class AxisAngle(*axis*, *angle*)
:   A rotation as an axis and an angle.

    > **Note**
    >
    > See [`quaternion_to_axis_angle()`](#StdLib.quaternion_to_axis_angle "StdLib.quaternion_to_axis_angle").

    axis
    :   The axis of the rotation, as a unit vector, or zero for no rotation.

        Return type:
        :   tuple(float, float, float)

    angle
    :   The angle of the rotation, in radians between 0 and pi.

        Return type:
        :   float

class ColorRGB(*r*, *g*, *b*)
:   A color, as red, green and blue components between 0 and 1.

    r
    :   The red component.

        Return type:
        :   float

    g
    :   The green component.

        Return type:
        :   float

    b
    :   The blue component.

        Return type:
        :   float

class ColorRGBA(*r*, *g*, *b*, *a*)
:   A color, as red, green, blue and alpha components between 0 and 1.

    > **Note**
    >
    > An alpha of 0 is fully transparent and 1 is fully opaque.

    r
    :   The red component.

        Return type:
        :   float

    g
    :   The green component.

        Return type:
        :   float

    b
    :   The blue component.

        Return type:
        :   float

    a
    :   The alpha component.

        Return type:
        :   float
