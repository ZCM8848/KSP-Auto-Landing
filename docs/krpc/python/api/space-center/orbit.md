# Orbit

class Orbit
:   Describes an orbit. For example, the orbit of a vessel, obtained by calling
    [`Vessel.orbit`](./vessel.md#SpaceCenter.Vessel.orbit "SpaceCenter.Vessel.orbit"), or a celestial body, obtained by calling
    [`CelestialBody.orbit`](./celestial-body.md#SpaceCenter.CelestialBody.orbit "SpaceCenter.CelestialBody.orbit").

    body
    :   The celestial body (e.g. planet or moon) around which the object is orbiting.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`CelestialBody`](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")

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

    radius\_at(*ut*)
    :   The orbital radius at the given time, in meters.

        Parameters:
        :   **ut** (*float*) – The universal time to measure the radius at.

        Return type:
        :   float

    position\_at(*ut*, *reference\_frame*)
    :   The position at a given time, in the specified reference frame.

        Parameters:
        :   - **ut** (*float*) – The universal time to measure the position at.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned position vector is in.

        Returns:
        :   The position as a vector.

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

    period
    :   The orbital period, in seconds.

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
    :   The time since the epoch (the point at which the
        [mean anomaly at epoch](https://en.wikipedia.org/wiki/Mean_anomaly)
        was measured, in seconds.

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

    radius\_at\_true\_anomaly(*true\_anomaly*)
    :   The orbital radius at the point in the orbit given by the true anomaly.

        Parameters:
        :   **true\_anomaly** (*float*) – The true anomaly.

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
    :   The current orbital speed in meters per second.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    orbital\_speed\_at(*time*)
    :   The orbital speed at the given time, in meters per second.

        Parameters:
        :   **time** (*float*) – Time from now, in seconds.

        Return type:
        :   float

    orbital\_energy
    :   The specific orbital energy of the orbit, in Joules per kilogram
        (equivalently, meters squared per second squared). This is the sum
        of the orbit’s specific kinetic and potential energy, and is
        negative for a bound (elliptical) orbit.

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
        :   [`Orbit`](#SpaceCenter.Orbit "SpaceCenter.Orbit")

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

    time\_of\_closest\_approach(*target*)
    :   > **Warning**
        >
        > Deprecated. Use [`Orbit.next_closest_approach()`](#SpaceCenter.Orbit.next_closest_approach "SpaceCenter.Orbit.next_closest_approach") and read [`ClosestApproach.ut`](./closest-approach.md#SpaceCenter.ClosestApproach.ut "SpaceCenter.ClosestApproach.ut") instead.

        Estimates and returns the time at closest approach to a target orbit.

        Parameters:
        :   **target** ([*Orbit*](#SpaceCenter.Orbit "SpaceCenter.Orbit")) – Target orbit.

        Returns:
        :   The universal time at closest approach, in seconds.

        Return type:
        :   float

    distance\_at\_closest\_approach(*target*)
    :   > **Warning**
        >
        > Deprecated. Use [`Orbit.next_closest_approach()`](#SpaceCenter.Orbit.next_closest_approach "SpaceCenter.Orbit.next_closest_approach") and read [`ClosestApproach.distance`](./closest-approach.md#SpaceCenter.ClosestApproach.distance "SpaceCenter.ClosestApproach.distance") instead.

        Estimates and returns the distance at closest approach to a target orbit, in meters.

        Parameters:
        :   **target** ([*Orbit*](#SpaceCenter.Orbit "SpaceCenter.Orbit")) – Target orbit.

        Return type:
        :   float

    list\_closest\_approaches(*target*, *orbits*)
    :   > **Warning**
        >
        > Deprecated. Use [`Orbit.closest_approaches()`](#SpaceCenter.Orbit.closest_approaches "SpaceCenter.Orbit.closest_approaches") instead.

        Returns the times at closest approach and corresponding distances, to a target orbit.

        Parameters:
        :   - **target** ([*Orbit*](#SpaceCenter.Orbit "SpaceCenter.Orbit")) – Target orbit.
            - **orbits** (*int*) – The number of future orbits to search.

        Returns:
        :   A list of two lists. The first is a list of times at closest approach, as universal times in seconds. The second is a list of corresponding distances at closest approach, in meters.

        Return type:
        :   list(list(float))
