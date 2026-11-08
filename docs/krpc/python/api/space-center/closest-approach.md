# Closest Approach

class ClosestApproach
:   A close approach between an orbit and a target orbit. Obtained by calling
    [`Orbit.next_closest_approach()`](./orbit.md#SpaceCenter.Orbit.next_closest_approach "SpaceCenter.Orbit.next_closest_approach") or [`Orbit.closest_approaches()`](./orbit.md#SpaceCenter.Orbit.closest_approaches "SpaceCenter.Orbit.closest_approaches").

    > **Note**
    >
    > The object names the approach rather than describing it once: each member
    > estimates the time of closest approach from where the two orbits are now, and
    > describes the state at that time. The estimate moves as the game runs, so
    > members read at different moments can differ a little; members read in one
    > physics tick all describe the same estimate. Relative quantities are the target
    > relative to the orbiting object (target minus self).
    >
    > The search runs forward over one orbital period from the current time. Once an
    > approach has passed and the two orbits are moving apart, the closest point in
    > that period is the current one, so the object reports the approach as now, at
    > the present separation, until the orbits bring another one within range.

    ut
    :   The universal time of the closest approach, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    time\_to
    :   The time until the closest approach, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    distance
    :   The distance between the objects at the closest approach, in meters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    relative\_speed
    :   The relative speed of the objects at the closest approach, in meters per
        second. This is the magnitude of [`ClosestApproach.relative_velocity()`](#SpaceCenter.ClosestApproach.relative_velocity "SpaceCenter.ClosestApproach.relative_velocity"), and does
        not depend on the choice of reference frame.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    vessel
    :   The vessel doing the approaching, or `None` if it is not a vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Vessel`](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel") or None

    body
    :   The celestial body doing the approaching, or `None` if it is not a
        celestial body.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`CelestialBody`](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody") or None

    target\_vessel
    :   The vessel being approached, or `None` if the target is not a vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Vessel`](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel") or None

    target\_body
    :   The celestial body being approached, or `None` if the target is not a
        celestial body.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`CelestialBody`](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody") or None

    position([*reference\_frame=None*])
    :   The position of the orbiting object at the closest approach.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – The reference frame that the returned position vector is in. Defaults to the orbital reference frame of the object the orbit belongs to.

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

    target\_position([*reference\_frame=None*])
    :   The position of the target object at the closest approach.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – The reference frame that the returned position vector is in. Defaults to the orbital reference frame of the object the orbit belongs to.

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

    velocity([*reference\_frame=None*])
    :   The velocity of the orbiting object at the closest approach.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – The reference frame that the returned velocity vector is in. Defaults to the orbital reference frame of the object the orbit belongs to.

        Returns:
        :   The velocity as a vector.

        Return type:
        :   tuple(float, float, float)

    target\_velocity([*reference\_frame=None*])
    :   The velocity of the target object at the closest approach.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – The reference frame that the returned velocity vector is in. Defaults to the orbital reference frame of the object the orbit belongs to.

        Returns:
        :   The velocity as a vector.

        Return type:
        :   tuple(float, float, float)

    relative\_position([*reference\_frame=None*])
    :   The position of the target relative to the orbiting object at the closest
        approach.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – The reference frame whose axes the returned vector is expressed in. Defaults to the orbital reference frame of the object the orbit belongs to.

        Returns:
        :   The relative position as a vector.

        Return type:
        :   tuple(float, float, float)

    relative\_velocity([*reference\_frame=None*])
    :   The velocity of the target relative to the orbiting object at the closest
        approach.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – The reference frame whose axes the returned vector is expressed in. Defaults to the orbital reference frame of the object the orbit belongs to.

        Returns:
        :   The relative velocity as a vector.

        Return type:
        :   tuple(float, float, float)
