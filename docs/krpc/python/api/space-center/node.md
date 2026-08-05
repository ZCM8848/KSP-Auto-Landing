# Node

class Node
:   Represents a maneuver node. Can be created using [`Control.add_node()`](./control.md#SpaceCenter.Control.add_node "SpaceCenter.Control.add_node").

    prograde
    :   The magnitude of the maneuver nodes delta-v in the prograde direction,
        in meters per second.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    normal
    :   The magnitude of the maneuver nodes delta-v in the normal direction,
        in meters per second.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    radial
    :   The magnitude of the maneuver nodes delta-v in the radial direction,
        in meters per second.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    delta\_v
    :   The delta-v of the maneuver node, in meters per second.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Does not change when executing the maneuver node. See [`Node.remaining_delta_v`](#SpaceCenter.Node.remaining_delta_v "SpaceCenter.Node.remaining_delta_v").

    remaining\_delta\_v
    :   Gets the remaining delta-v of the maneuver node, in meters per second. Changes as the
        node is executed. This is equivalent to the delta-v reported in-game.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    burn\_vector([*reference\_frame=None*])
    :   Returns the burn vector for the maneuver node.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned vector is in. Defaults to [`Vessel.orbital_reference_frame`](./vessel.md#SpaceCenter.Vessel.orbital_reference_frame "SpaceCenter.Vessel.orbital_reference_frame").

        Returns:
        :   A vector whose direction is the direction of the maneuver node burn, and magnitude is the delta-v of the burn in meters per second.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Does not change when executing the maneuver node. See [`Node.remaining_burn_vector()`](#SpaceCenter.Node.remaining_burn_vector "SpaceCenter.Node.remaining_burn_vector").

    remaining\_burn\_vector([*reference\_frame=None*])
    :   Returns the remaining burn vector for the maneuver node.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned vector is in. Defaults to [`Vessel.orbital_reference_frame`](./vessel.md#SpaceCenter.Vessel.orbital_reference_frame "SpaceCenter.Vessel.orbital_reference_frame").

        Returns:
        :   A vector whose direction is the direction of the maneuver node burn, and magnitude is the delta-v of the burn in meters per second.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Changes as the maneuver node is executed. See [`Node.burn_vector()`](#SpaceCenter.Node.burn_vector "SpaceCenter.Node.burn_vector").

    ut
    :   The universal time at which the maneuver will occur, in seconds.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    time\_to
    :   The time until the maneuver node will be encountered, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    orbit
    :   The orbit that results from executing the maneuver node.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Orbit`](./orbit.md#SpaceCenter.Orbit "SpaceCenter.Orbit")

        Game Scenes:
        :   Flight

    remove()
    :   Removes the maneuver node.

        Game Scenes:
        :   Flight

    reference\_frame
    :   The reference frame that is fixed relative to the maneuver node’s burn.

        - The origin is at the position of the maneuver node.
        - The y-axis points in the direction of the burn.
        - The x-axis and z-axis point in arbitrary but fixed directions.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Game Scenes:
        :   Flight

    orbital\_reference\_frame
    :   The reference frame that is fixed relative to the maneuver node, and
        orientated with the orbital prograde/normal/radial directions of the
        original orbit at the maneuver node’s position.

        - The origin is at the position of the maneuver node.
        - The x-axis points in the orbital anti-radial direction of the original
          orbit, at the position of the maneuver node.
        - The y-axis points in the orbital prograde direction of the original
          orbit, at the position of the maneuver node.
        - The z-axis points in the orbital normal direction of the original orbit,
          at the position of the maneuver node.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Game Scenes:
        :   Flight

    position(*reference\_frame*)
    :   The position vector of the maneuver node in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned position vector is in.

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    rotation(*reference\_frame*)
    :   The rotation of the maneuver nodes burn.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned rotation is in.

        Returns:
        :   The rotation as a quaternion of the form \((x, y, z, w)\).

        Return type:
        :   tuple(float, float, float, float)

        Game Scenes:
        :   Flight

    direction(*reference\_frame*)
    :   The direction of the maneuver nodes burn.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned direction is in.

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight
