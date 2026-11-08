# Orbit

class Orbit
:   Describes an orbit. For example, the orbit of a vessel, obtained by calling
    [`Vessel.orbit`](./vessel.md#SpaceCenter.Vessel.orbit "SpaceCenter.Vessel.orbit"), or a celestial body, obtained by calling
    [`CelestialBody.orbit`](./celestial-body.md#SpaceCenter.CelestialBody.orbit "SpaceCenter.CelestialBody.orbit").

    static create\_from\_position\_and\_velocity(*body*, *position*, *velocity*, *ut*[, *reference\_frame=None*])
    :   Create the orbit that passes through a given position at a given velocity.
        The orbit coasts freely under the gravity of the body being orbited, so it
        describes where an object left to fall from that position and velocity would
        be at any later time.

        Parameters:
        :   - **body** ([*CelestialBody*](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")) – The celestial body being orbited.
            - **position** (*tuple*) – The position, as a position vector.
            - **velocity** (*tuple*) – The velocity, as a vector pointing in the direction of travel, whose magnitude is the speed in meters per second.
            - **ut** (*float*) – The universal time, in seconds, at which the object is at *position* traveling at *velocity*.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – The reference frame that *position* and *velocity* are in. Defaults to [`CelestialBody.non_rotating_reference_frame`](./celestial-body.md#SpaceCenter.CelestialBody.non_rotating_reference_frame "SpaceCenter.CelestialBody.non_rotating_reference_frame") of *body*.

        Return type:
        :   [`Orbit`](#SpaceCenter.Orbit "SpaceCenter.Orbit")

        > **Note**
        >
        > The orbit is a single conic around *body*. It never changes
        > sphere of influence, so [`Orbit.next_orbit`](#SpaceCenter.Orbit.next_orbit "SpaceCenter.Orbit.next_orbit") is `None`, and no
        > atmosphere slows it. A vessel held on rails follows its own orbit exactly,
        > while one inside the physics bubble is simulated and drifts from it a little.
        >
        > [`Orbit.radius`](#SpaceCenter.Orbit.radius "SpaceCenter.Orbit.radius"), [`Orbit.speed`](#SpaceCenter.Orbit.speed "SpaceCenter.Orbit.speed"), [`Orbit.position()`](#SpaceCenter.Orbit.position "SpaceCenter.Orbit.position"),
        > [`Orbit.velocity()`](#SpaceCenter.Orbit.velocity "SpaceCenter.Orbit.velocity") and the like describe the orbit at
        > *ut* and stay there. Use [`Orbit.radius_at()`](#SpaceCenter.Orbit.radius_at "SpaceCenter.Orbit.radius_at"),
        > [`Orbit.speed_at()`](#SpaceCenter.Orbit.speed_at "SpaceCenter.Orbit.speed_at"), [`Orbit.position_at()`](#SpaceCenter.Orbit.position_at "SpaceCenter.Orbit.position_at"), [`Orbit.velocity_at()`](#SpaceCenter.Orbit.velocity_at "SpaceCenter.Orbit.velocity_at") and
        > [`Orbit.true_anomaly_at_ut()`](#SpaceCenter.Orbit.true_anomaly_at_ut "SpaceCenter.Orbit.true_anomaly_at_ut") for another time, and
        > [`Orbit.reference_frame`](#SpaceCenter.Orbit.reference_frame "SpaceCenter.Orbit.reference_frame") and [`Orbit.orbital_reference_frame`](#SpaceCenter.Orbit.orbital_reference_frame "SpaceCenter.Orbit.orbital_reference_frame") to follow
        > the orbit as time passes.
        >
        > The orbit is held until [`Orbit.remove()`](#SpaceCenter.Orbit.remove "SpaceCenter.Orbit.remove") is called on it or the client
        > that created it disconnects. Remove each one created by a script that creates
        > them repeatedly, for example once per update.

    static create\_from\_orbital\_elements(*body*, *semi\_major\_axis*, *eccentricity*, *inclination*, *longitude\_of\_ascending\_node*, *argument\_of\_periapsis*, *mean\_anomaly\_at\_epoch*, *epoch*)
    :   Create the orbit with the given
        [orbital elements](https://en.wikipedia.org/wiki/Orbital_elements)
        around a given body. The orbit coasts freely under the gravity of that body,
        so it describes where an object on it would be at any time.

        Parameters:
        :   - **body** ([*CelestialBody*](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")) – The celestial body being orbited.
            - **semi\_major\_axis** (*float*) – The semi-major axis of the orbit, in meters. Positive for an ellipse and negative for a hyperbola.
            - **eccentricity** (*float*) – The eccentricity of the orbit. Below one for an ellipse and above one for a hyperbola.
            - **inclination** (*float*) – The inclination of the orbit, in radians.
            - **longitude\_of\_ascending\_node** (*float*) – The longitude of the ascending node, in radians.
            - **argument\_of\_periapsis** (*float*) – The argument of periapsis, in radians.
            - **mean\_anomaly\_at\_epoch** (*float*) – The mean anomaly at *epoch*, in radians.
            - **epoch** (*float*) – The universal time, in seconds, that *mean\_anomaly\_at\_epoch* is measured at.

        Return type:
        :   [`Orbit`](#SpaceCenter.Orbit "SpaceCenter.Orbit")

        > **Note**
        >
        > The orbit is a single conic around *body*. It never changes
        > sphere of influence, so [`Orbit.next_orbit`](#SpaceCenter.Orbit.next_orbit "SpaceCenter.Orbit.next_orbit") is `None`, and no
        > atmosphere slows it.
        >
        > The angles are measured against the reference plane and direction that
        > [`Orbit.inclination`](#SpaceCenter.Orbit.inclination "SpaceCenter.Orbit.inclination"), [`Orbit.longitude_of_ascending_node`](#SpaceCenter.Orbit.longitude_of_ascending_node "SpaceCenter.Orbit.longitude_of_ascending_node") and
        > [`Orbit.argument_of_periapsis`](#SpaceCenter.Orbit.argument_of_periapsis "SpaceCenter.Orbit.argument_of_periapsis") report them against.
        > [`Orbit.reference_plane_normal()`](#SpaceCenter.Orbit.reference_plane_normal "SpaceCenter.Orbit.reference_plane_normal") and [`Orbit.reference_plane_direction()`](#SpaceCenter.Orbit.reference_plane_direction "SpaceCenter.Orbit.reference_plane_direction")
        > give these as vectors in a reference frame.
        >
        > [`Orbit.radius`](#SpaceCenter.Orbit.radius "SpaceCenter.Orbit.radius"), [`Orbit.speed`](#SpaceCenter.Orbit.speed "SpaceCenter.Orbit.speed"), [`Orbit.position()`](#SpaceCenter.Orbit.position "SpaceCenter.Orbit.position"),
        > [`Orbit.velocity()`](#SpaceCenter.Orbit.velocity "SpaceCenter.Orbit.velocity") and the like describe the orbit at
        > *epoch* and stay there. Use [`Orbit.radius_at()`](#SpaceCenter.Orbit.radius_at "SpaceCenter.Orbit.radius_at"),
        > [`Orbit.speed_at()`](#SpaceCenter.Orbit.speed_at "SpaceCenter.Orbit.speed_at"), [`Orbit.position_at()`](#SpaceCenter.Orbit.position_at "SpaceCenter.Orbit.position_at"), [`Orbit.velocity_at()`](#SpaceCenter.Orbit.velocity_at "SpaceCenter.Orbit.velocity_at") and
        > [`Orbit.true_anomaly_at_ut()`](#SpaceCenter.Orbit.true_anomaly_at_ut "SpaceCenter.Orbit.true_anomaly_at_ut") for another time, and
        > [`Orbit.reference_frame`](#SpaceCenter.Orbit.reference_frame "SpaceCenter.Orbit.reference_frame") and [`Orbit.orbital_reference_frame`](#SpaceCenter.Orbit.orbital_reference_frame "SpaceCenter.Orbit.orbital_reference_frame") to follow
        > the orbit as time passes.
        >
        > The orbit is held until [`Orbit.remove()`](#SpaceCenter.Orbit.remove "SpaceCenter.Orbit.remove") is called on it or the client
        > that created it disconnects. Remove each one created by a script that creates
        > them repeatedly, for example once per update.

    remove()
    :   Remove the orbit, releasing the memory the server holds for it.

        > **Note**
        >
        > Only an orbit created by [`Orbit.create_from_position_and_velocity()`](#SpaceCenter.Orbit.create_from_position_and_velocity "SpaceCenter.Orbit.create_from_position_and_velocity") or
        > [`Orbit.create_from_orbital_elements()`](#SpaceCenter.Orbit.create_from_orbital_elements "SpaceCenter.Orbit.create_from_orbital_elements") can be removed. Every other orbit
        > belongs to something in the game, and the server holds one of each.
        >
        > Any further use of this object throws an exception, as does use of a
        > reference frame defined against it. An orbit belongs to the client that
        > created it, and is removed when that client disconnects.

    body
    :   The celestial body (e.g. planet or moon) around which the object is orbiting.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`CelestialBody`](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")

    reference\_frame
    :   The reference frame that moves along the orbit, and is orientated in a fixed
        direction.

        - The origin is at the point the orbit has reached at the
            current time.
        - The axes do not rotate. They point in the same fixed
            directions as those of
            [`CelestialBody.non_rotating_reference_frame`](./celestial-body.md#SpaceCenter.CelestialBody.non_rotating_reference_frame "SpaceCenter.CelestialBody.non_rotating_reference_frame") of
            [`Orbit.body`](#SpaceCenter.Orbit.body "SpaceCenter.Orbit.body").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        > **Note**
        >
        > For the orbit of a vessel, the origin is the point the vessel’s orbit has
        > reached rather than the vessel’s own center of mass. The two are a few meters
        > apart, as a vessel inside the physics bubble is simulated rather than being
        > held on its orbit.

    orbital\_reference\_frame
    :   The reference frame that moves along the orbit, and is orientated with its
        prograde/normal/radial directions.

        - The origin is at the point the orbit has reached at the
            current time.
        - The axes rotate with the orbital prograde/normal/radial
            directions.
        - The x-axis points in the orbital anti-radial direction.
        - The y-axis points in the orbital prograde direction.
        - The z-axis points in the orbital normal direction.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        > **Note**
        >
        > For the orbit of a vessel, the origin is the point the vessel’s orbit has
        > reached rather than the vessel’s own center of mass, which is what
        > [`Vessel.orbital_reference_frame`](./vessel.md#SpaceCenter.Vessel.orbital_reference_frame "SpaceCenter.Vessel.orbital_reference_frame") is centered on.

    apoapsis
    :   Gets the apoapsis of the orbit, in meters, from the center of mass
        of the body being orbited.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        > **Note**
        >
        > For the apoapsis altitude reported on the in-game map view,
        > use [`Orbit.apoapsis_altitude`](#SpaceCenter.Orbit.apoapsis_altitude "SpaceCenter.Orbit.apoapsis_altitude").

    periapsis
    :   The periapsis of the orbit, in meters, from the center of mass
        of the body being orbited.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        > **Note**
        >
        > For the periapsis altitude reported on the in-game map view,
        > use [`Orbit.periapsis_altitude`](#SpaceCenter.Orbit.periapsis_altitude "SpaceCenter.Orbit.periapsis_altitude").

    apoapsis\_altitude
    :   The apoapsis of the orbit, in meters, above the sea level of the body being orbited.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        > **Note**
        >
        > This is equal to [`Orbit.apoapsis`](#SpaceCenter.Orbit.apoapsis "SpaceCenter.Orbit.apoapsis") minus the equatorial radius of the body.

    periapsis\_altitude
    :   The periapsis of the orbit, in meters, above the sea level of the body being orbited.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        > **Note**
        >
        > This is equal to [`Orbit.periapsis`](#SpaceCenter.Orbit.periapsis "SpaceCenter.Orbit.periapsis") minus the equatorial radius of the body.

    semi\_major\_axis
    :   The semi-major axis of the orbit, in meters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    semi\_minor\_axis
    :   The semi-minor axis of the orbit, in meters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    semi\_latus\_rectum
    :   The semi-latus rectum of the orbit, in meters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    radius
    :   The current radius of the orbit, in meters. This is the distance between the center
        of mass of the object in orbit, and the center of mass of the body around which it
        is orbiting.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        > **Note**
        >
        > This value will change over time if the orbit is elliptical.

    altitude
    :   The current altitude of the object, in meters above the sea level of the body
        being orbited.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        > **Note**
        >
        > This is equal to [`Orbit.radius`](#SpaceCenter.Orbit.radius "SpaceCenter.Orbit.radius") minus the equatorial radius of the body.

    radius\_at(*ut*)
    :   The orbital radius at the given time, in meters.

        Parameters:
        :   **ut** (*float*) – The universal time to measure the radius at.

        Return type:
        :   float

    radius\_at\_true\_anomaly(*true\_anomaly*)
    :   The orbital radius at the point in the orbit given by the true anomaly.

        Parameters:
        :   **true\_anomaly** (*float*) – The true anomaly.

        Return type:
        :   float

    position([*reference\_frame=None*])
    :   The position the orbit has reached at the current time, in the given reference
        frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – The reference frame that the returned position vector is in. Defaults to [`CelestialBody.non_rotating_reference_frame`](./celestial-body.md#SpaceCenter.CelestialBody.non_rotating_reference_frame "SpaceCenter.CelestialBody.non_rotating_reference_frame") of [`Orbit.body`](#SpaceCenter.Orbit.body "SpaceCenter.Orbit.body").

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

    position\_at(*ut*[, *reference\_frame=None*])
    :   The position at a given time, in the specified reference frame.

        Parameters:
        :   - **ut** (*float*) – The universal time to measure the position at.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – The reference frame that the returned position vector is in. Defaults to [`CelestialBody.non_rotating_reference_frame`](./celestial-body.md#SpaceCenter.CelestialBody.non_rotating_reference_frame "SpaceCenter.CelestialBody.non_rotating_reference_frame") of [`Orbit.body`](#SpaceCenter.Orbit.body "SpaceCenter.Orbit.body").

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

    position\_at\_true\_anomaly(*true\_anomaly*[, *reference\_frame=None*])
    :   The position at the point in the orbit given by the true anomaly, in the specified
        reference frame.

        Parameters:
        :   - **true\_anomaly** (*float*) – The true anomaly.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – The reference frame that the returned position vector is in. Defaults to [`CelestialBody.non_rotating_reference_frame`](./celestial-body.md#SpaceCenter.CelestialBody.non_rotating_reference_frame "SpaceCenter.CelestialBody.non_rotating_reference_frame") of [`Orbit.body`](#SpaceCenter.Orbit.body "SpaceCenter.Orbit.body").

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

    velocity([*reference\_frame=None*])
    :   The velocity the orbit has reached at the current time, in the given reference
        frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – The reference frame that the returned velocity vector is in. Defaults to [`CelestialBody.non_rotating_reference_frame`](./celestial-body.md#SpaceCenter.CelestialBody.non_rotating_reference_frame "SpaceCenter.CelestialBody.non_rotating_reference_frame") of [`Orbit.body`](#SpaceCenter.Orbit.body "SpaceCenter.Orbit.body").

        Returns:
        :   The velocity as a vector. The vector points in the direction of travel, and its magnitude is the speed in meters per second.

        Return type:
        :   tuple(float, float, float)

    velocity\_at(*ut*[, *reference\_frame=None*])
    :   The velocity at a given time, in the specified reference frame.

        Parameters:
        :   - **ut** (*float*) – The universal time to measure the velocity at.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – The reference frame that the returned velocity vector is in. Defaults to [`CelestialBody.non_rotating_reference_frame`](./celestial-body.md#SpaceCenter.CelestialBody.non_rotating_reference_frame "SpaceCenter.CelestialBody.non_rotating_reference_frame") of [`Orbit.body`](#SpaceCenter.Orbit.body "SpaceCenter.Orbit.body").

        Returns:
        :   The velocity as a vector. The vector points in the direction of travel, and its magnitude is the speed in meters per second.

        Return type:
        :   tuple(float, float, float)

    velocity\_at\_true\_anomaly(*true\_anomaly*[, *reference\_frame=None*])
    :   The velocity at the point in the orbit given by the true anomaly, in the specified
        reference frame.

        Parameters:
        :   - **true\_anomaly** (*float*) – The true anomaly.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – The reference frame that the returned velocity vector is in. Defaults to [`CelestialBody.non_rotating_reference_frame`](./celestial-body.md#SpaceCenter.CelestialBody.non_rotating_reference_frame "SpaceCenter.CelestialBody.non_rotating_reference_frame") of [`Orbit.body`](#SpaceCenter.Orbit.body "SpaceCenter.Orbit.body").

        Returns:
        :   The velocity as a vector. The vector points in the direction of travel, and its magnitude is the speed in meters per second.

        Return type:
        :   tuple(float, float, float)

    speed
    :   The current orbital speed of the object in meters per second.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        > **Note**
        >
        > This value will change over time if the orbit is elliptical.

    speed\_at(*ut*)
    :   The orbital speed at the given time, in meters per second.

        Parameters:
        :   **ut** (*float*) – The universal time to measure the speed at.

        Return type:
        :   float

    speed\_at\_radius(*radius*)
    :   The orbital speed at the given orbital radius, in meters per second.

        Parameters:
        :   **radius** (*float*) – The orbital radius in meters.

        Return type:
        :   float

        > **Note**
        >
        > This is the vis-viva speed. A radius the orbit never reaches is accepted. On
        > an ellipse the speed is `NaN` above twice the semi-major axis.

    speed\_at\_true\_anomaly(*true\_anomaly*)
    :   The orbital speed at the point in the orbit given by the true anomaly, in meters
        per second.

        Parameters:
        :   **true\_anomaly** (*float*) – The true anomaly.

        Return type:
        :   float

    specific\_energy
    :   The specific orbital energy, in Joules per kilogram (equivalently,
        meters squared per second squared).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        > **Note**
        >
        > This is the sum of the orbit’s specific kinetic and potential energy, and
        > is negative for a bound (elliptical) orbit.

    specific\_angular\_momentum
    :   The specific relative angular momentum, in meters squared per second.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    period
    :   The orbital period, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    mean\_motion
    :   The mean motion, in radians per second.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    time\_to\_apoapsis
    :   The time until the object reaches apoapsis, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    time\_to\_periapsis
    :   The time until the object reaches periapsis, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    eccentricity
    :   The [eccentricity](https://en.wikipedia.org/wiki/Orbital_eccentricity)
        of the orbit.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    inclination
    :   The [inclination](https://en.wikipedia.org/wiki/Orbital_inclination)
        of the orbit,
        in radians.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    longitude\_of\_ascending\_node
    :   The [longitude of the ascending node](https://en.wikipedia.org/wiki/Longitude_of_the_ascending_node), in radians.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        > **Note**
        >
        > For a near-equatorial orbit, the ascending node is ill-defined and
        > this value may vary erratically over time.

    argument\_of\_periapsis
    :   The [argument of periapsis](https://en.wikipedia.org/wiki/Argument_of_periapsis), in radians.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        > **Note**
        >
        > For a near-circular orbit, the periapsis is ill-defined and
        > this value may vary erratically over time.

    mean\_anomaly\_at\_epoch
    :   The [mean anomaly at epoch](https://en.wikipedia.org/wiki/Mean_anomaly).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    epoch
    :   The universal time, in seconds, at which the
        [mean anomaly at epoch](https://en.wikipedia.org/wiki/Mean_anomaly)
        is measured.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    mean\_anomaly
    :   The [mean anomaly](https://en.wikipedia.org/wiki/Mean_anomaly).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    mean\_anomaly\_at\_ut(*ut*)
    :   The mean anomaly at the given time.

        Parameters:
        :   **ut** (*float*) – The universal time in seconds.

        Return type:
        :   float

    eccentric\_anomaly
    :   The [eccentric anomaly](https://en.wikipedia.org/wiki/Eccentric_anomaly).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    eccentric\_anomaly\_at\_ut(*ut*)
    :   The eccentric anomaly at the given universal time.

        Parameters:
        :   **ut** (*float*) – The universal time, in seconds.

        Return type:
        :   float

    true\_anomaly
    :   The [true anomaly](https://en.wikipedia.org/wiki/True_anomaly).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    true\_anomaly\_at\_ut(*ut*)
    :   The true anomaly at the given time.

        Parameters:
        :   **ut** (*float*) – The universal time in seconds.

        Return type:
        :   float

    true\_anomaly\_at\_radius(*radius*)
    :   The true anomaly at the given orbital radius.

        Parameters:
        :   **radius** (*float*) – The orbital radius in meters.

        Return type:
        :   float

    ut\_at\_true\_anomaly(*true\_anomaly*)
    :   The universal time, in seconds, corresponding to the given true anomaly.

        Parameters:
        :   **true\_anomaly** (*float*) – True anomaly.

        Return type:
        :   float

    true\_anomaly\_at\_an(*target*)
    :   The true anomaly of the ascending node with the given target orbit.

        Parameters:
        :   **target** ([*Orbit*](#SpaceCenter.Orbit "SpaceCenter.Orbit")) – Target orbit.

        Return type:
        :   float

    true\_anomaly\_at\_dn(*target*)
    :   The true anomaly of the descending node with the given target orbit.

        Parameters:
        :   **target** ([*Orbit*](#SpaceCenter.Orbit "SpaceCenter.Orbit")) – Target orbit.

        Return type:
        :   float

    orbital\_speed
    :   > **Warning**
        >
        > Deprecated. Use [`Orbit.speed`](#SpaceCenter.Orbit.speed "SpaceCenter.Orbit.speed") instead.

        The current orbital speed in meters per second.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    orbital\_speed\_at(*ut*)
    :   > **Warning**
        >
        > Deprecated. Use [`Orbit.speed_at()`](#SpaceCenter.Orbit.speed_at "SpaceCenter.Orbit.speed_at") instead.

        The orbital speed at the given time, in meters per second.

        Parameters:
        :   **ut** (*float*) – The universal time to measure the speed at.

        Return type:
        :   float

    orbital\_energy
    :   > **Warning**
        >
        > Deprecated. Use [`Orbit.specific_energy`](#SpaceCenter.Orbit.specific_energy "SpaceCenter.Orbit.specific_energy") instead.

        The specific orbital energy of the orbit, in Joules per kilogram.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    static reference\_plane\_normal(*reference\_frame*)
    :   The direction that is normal to the orbits reference plane,
        in the given reference frame.
        The reference plane is the plane from which the orbits inclination is measured.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned direction is in.

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

    static reference\_plane\_direction(*reference\_frame*)
    :   The direction from which the orbits longitude of ascending node is measured,
        in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned direction is in.

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

    relative\_inclination(*target*)
    :   Relative inclination of this orbit and the target orbit, in radians.

        Parameters:
        :   **target** ([*Orbit*](#SpaceCenter.Orbit "SpaceCenter.Orbit")) – Target orbit.

        Return type:
        :   float

    time\_to\_soi\_change
    :   The time until the object changes sphere of influence, in seconds. Returns `NaN`
        if the object is not going to change sphere of influence.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    next\_orbit
    :   If the object is going to change sphere of influence in the future, returns the new
        orbit after the change. Otherwise returns `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Orbit`](#SpaceCenter.Orbit "SpaceCenter.Orbit") or None

    next\_closest\_approach(*target*)
    :   The next closest approach to a target orbit.

        Parameters:
        :   **target** ([*Orbit*](#SpaceCenter.Orbit "SpaceCenter.Orbit")) – Target orbit.

        Return type:
        :   [`ClosestApproach`](./closest-approach.md#SpaceCenter.ClosestApproach "SpaceCenter.ClosestApproach")

    closest\_approaches(*target*, *orbits*)
    :   A list of the closest approaches to a target orbit, one for each of the next
        *orbits* orbital periods.

        Parameters:
        :   - **target** ([*Orbit*](#SpaceCenter.Orbit "SpaceCenter.Orbit")) – Target orbit.
            - **orbits** (*int*) – The number of future orbits to search.

        Return type:
        :   list([`ClosestApproach`](./closest-approach.md#SpaceCenter.ClosestApproach "SpaceCenter.ClosestApproach"))
