# ReferenceFrame

class ReferenceFrame
:   Represents a reference frame for positions, rotations and
    velocities. Contains:

    - The position of the origin.
    - The directions of the x, y and z axes.
    - The linear velocity of the frame.
    - The angular velocity of the frame.

    > **Note**
    >
    > Apart from being created and removed, a reference frame is only used as a
    > parameter to other functions.

    static create\_relative(*reference\_frame*[, *position=(0.0*, *0.0*, *0.0)*][, *rotation=(0.0*, *0.0*, *0.0*, *1.0)*][, *velocity=(0.0*, *0.0*, *0.0)*][, *angular\_velocity=(0.0*, *0.0*, *0.0)*])
    :   Create a relative reference frame. This is a custom reference frame
        whose components offset the components of a parent reference frame.

        Parameters:
        :   - **reference\_frame** ([*ReferenceFrame*](#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The parent reference frame on which to base this reference frame.
            - **position** (*tuple*) – The offset of the position of the origin, as a position vector. Defaults to \((0, 0, 0)\)
            - **rotation** (*tuple*) – The rotation to apply to the parent frames rotation, as a quaternion of the form \((x, y, z, w)\). Defaults to \((0, 0, 0, 1)\) (i.e. no rotation)
            - **velocity** (*tuple*) – The linear velocity to offset the parent frame by, as a vector pointing in the direction of travel, whose magnitude is the speed in meters per second. Defaults to \((0, 0, 0)\).
            - **angular\_velocity** (*tuple*) – The angular velocity to offset the parent frame by, as a vector. This vector points in the direction of the axis of rotation, and its magnitude is the speed of the rotation in radians per second. Defaults to \((0, 0, 0)\).

        Return type:
        :   [`ReferenceFrame`](#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        > **Note**
        >
        > Each call returns a new frame, which is kept until [`ReferenceFrame.remove()`](#SpaceCenter.ReferenceFrame.remove "SpaceCenter.ReferenceFrame.remove") is
        > called on it or the client that created it disconnects, so a script that
        > creates one repeatedly, for example once per update, should remove each one
        > when it is finished with it.

    static create\_hybrid(*position*[, *rotation=None*][, *velocity=None*][, *angular\_velocity=None*])
    :   Create a hybrid reference frame. This is a custom reference frame
        whose components inherited from other reference frames.

        Parameters:
        :   - **position** ([*ReferenceFrame*](#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame providing the position of the origin.
            - **rotation** ([*ReferenceFrame*](#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – The reference frame providing the rotation of the frame.
            - **velocity** ([*ReferenceFrame*](#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – The reference frame providing the linear velocity of the frame.
            - **angular\_velocity** ([*ReferenceFrame*](#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – The reference frame providing the angular velocity of the frame.

        Return type:
        :   [`ReferenceFrame`](#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        > **Note**
        >
        > The *position* reference frame is required but all other
        > reference frames are optional. If omitted, they are set to the
        > *position* reference frame.
        >
        > Each call returns a new frame, which is kept until [`ReferenceFrame.remove()`](#SpaceCenter.ReferenceFrame.remove "SpaceCenter.ReferenceFrame.remove") is
        > called on it or the client that created it disconnects, so a script that
        > creates one repeatedly, for example once per update, should remove each one
        > when it is finished with it.

    remove()
    :   Remove the reference frame, releasing the memory the server holds for it.

        > **Note**
        >
        > Only a frame created by [`ReferenceFrame.create_relative()`](#SpaceCenter.ReferenceFrame.create_relative "SpaceCenter.ReferenceFrame.create_relative") or
        > [`ReferenceFrame.create_hybrid()`](#SpaceCenter.ReferenceFrame.create_hybrid "SpaceCenter.ReferenceFrame.create_hybrid") can be removed. Every other frame belongs to
        > something in the game, and the server holds one of each.
        >
        > Any further use of this object throws an exception, as does use of a frame
        > built on it, which cannot be evaluated without it. A frame belongs to the
        > client that created it, and is removed when that client disconnects.
