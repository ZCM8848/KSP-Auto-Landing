# CelestialBody

class CelestialBody
:   Represents a celestial body (such as a planet or moon).
    See [`bodies`](./space-center.md#SpaceCenter.bodies "SpaceCenter.bodies").

    name
    :   The name of the body.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    satellites
    :   A list of celestial bodies that are in orbit around this celestial body.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`CelestialBody`](#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody"))

    orbit
    :   The orbit of the body.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Orbit`](./orbit.md#SpaceCenter.Orbit "SpaceCenter.Orbit")

    mass
    :   The mass of the body, in kilograms.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    gravitational\_parameter
    :   The [standard gravitational parameter](https://en.wikipedia.org/wiki/Standard_gravitational_parameter) of the body in \(m^3s^{-2}\).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    surface\_gravity
    :   The acceleration due to gravity at sea level (mean altitude) on the body,
        in \(m/s^2\).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    rotational\_period
    :   The sidereal rotational period of the body, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    rotational\_speed
    :   The rotational speed of the body, in radians per second.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    rotation\_angle
    :   The current rotation angle of the body, in radians.
        A value between 0 and 2·pi

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    initial\_rotation
    :   The initial rotation angle of the body (at UT 0), in radians.
        A value between 0 and 2·pi

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    equatorial\_radius
    :   The equatorial radius of the body, in meters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    surface\_height(*latitude*, *longitude*)
    :   The height of the surface relative to mean sea level, in meters,
        at the given position. When over water this is equal to 0.

        Parameters:
        :   - **latitude** (*float*) – Latitude in degrees.
            - **longitude** (*float*) – Longitude in degrees.

        Return type:
        :   float

    bedrock\_height(*latitude*, *longitude*)
    :   The height of the surface relative to mean sea level, in meters,
        at the given position. When over water, this is the height
        of the sea-bed and is therefore negative value.

        Parameters:
        :   - **latitude** (*float*) – Latitude in degrees.
            - **longitude** (*float*) – Longitude in degrees.

        Return type:
        :   float

    msl\_position(*latitude*, *longitude*, *reference\_frame*)
    :   The position at mean sea level at the given latitude and longitude,
        in the given reference frame.

        Parameters:
        :   - **latitude** (*float*) – Latitude in degrees.
            - **longitude** (*float*) – Longitude in degrees.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame for the returned position vector.

        Returns:
        :   Position as a vector.

        Return type:
        :   tuple(float, float, float)

    surface\_position(*latitude*, *longitude*, *reference\_frame*)
    :   The position of the surface at the given latitude and longitude, in the given
        reference frame. When over water, this is the position of the surface of the water.

        Parameters:
        :   - **latitude** (*float*) – Latitude in degrees.
            - **longitude** (*float*) – Longitude in degrees.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame for the returned position vector.

        Returns:
        :   Position as a vector.

        Return type:
        :   tuple(float, float, float)

    bedrock\_position(*latitude*, *longitude*, *reference\_frame*)
    :   The position of the surface at the given latitude and longitude, in the given
        reference frame. When over water, this is the position at the bottom of the sea-bed.

        Parameters:
        :   - **latitude** (*float*) – Latitude in degrees.
            - **longitude** (*float*) – Longitude in degrees.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame for the returned position vector.

        Returns:
        :   Position as a vector.

        Return type:
        :   tuple(float, float, float)

    position\_at\_altitude(*latitude*, *longitude*, *altitude*, *reference\_frame*)
    :   The position at the given latitude, longitude and altitude, in the given reference frame.

        Parameters:
        :   - **latitude** (*float*) – Latitude in degrees.
            - **longitude** (*float*) – Longitude in degrees.
            - **altitude** (*float*) – Altitude in meters above sea level.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame for the returned position vector.

        Returns:
        :   Position as a vector.

        Return type:
        :   tuple(float, float, float)

    altitude\_at\_position(*position*, *reference\_frame*)
    :   The altitude, in meters, of the given position in the given reference frame.

        Parameters:
        :   - **position** (*tuple*) – Position as a vector.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame for the position vector.

        Return type:
        :   float

    latitude\_at\_position(*position*, *reference\_frame*)
    :   The latitude of the given position, in the given reference frame.

        Parameters:
        :   - **position** (*tuple*) – Position as a vector.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame for the position vector.

        Return type:
        :   float

    longitude\_at\_position(*position*, *reference\_frame*)
    :   The longitude of the given position, in the given reference frame.

        Parameters:
        :   - **position** (*tuple*) – Position as a vector.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame for the position vector.

        Return type:
        :   float

    sphere\_of\_influence
    :   The radius of the sphere of influence of the body, in meters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    is\_star
    :   Whether or not the body is a star.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    has\_solid\_surface
    :   Whether or not the body has a solid surface.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    has\_atmosphere
    :   `True` if the body has an atmosphere.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    atmosphere\_depth
    :   The depth of the atmosphere, in meters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    atmospheric\_density\_at\_position(*position*, *reference\_frame*)
    :   The atmospheric density at the given position, in \(kg/m^3\),
        in the given reference frame.

        Parameters:
        :   - **position** (*tuple*) – The position vector at which to measure the density.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – Reference frame that the position vector is in.

        Return type:
        :   float

    has\_atmospheric\_oxygen
    :   `True` if there is oxygen in the atmosphere, required for air-breathing engines.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    temperature\_at(*position*, *reference\_frame*)
    :   The temperature on the body at the given position, in the given reference frame.

        Parameters:
        :   - **position** (*tuple*) – Position as a vector.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the position is in.

        Return type:
        :   float

        > **Note**
        >
        > The atmospheric ephemeris is evaluated at the current universal time.

    density\_at(*altitude*)
    :   Gets the air density, in \(kg/m^3\), for the specified
        altitude above sea level, in meters.

        Parameters:
        :   **altitude** (*float*)

        Return type:
        :   float

        > **Note**
        >
        > This is an approximation, because actual calculations, taking sun exposure into account
        > to compute air temperature, require us to know the exact point on the body where the
        > density is to be computed (knowing the altitude is not enough).
        > However, the difference is small for high altitudes, so it makes very little difference
        > for trajectory prediction.

    pressure\_at(*altitude*)
    :   Gets the air pressure, in Pascals, for the specified
        altitude above sea level, in meters.

        Parameters:
        :   **altitude** (*float*)

        Return type:
        :   float

    biomes
    :   The biomes present on this body.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   set(str)

    biome\_at(*latitude*, *longitude*)
    :   The biome at the given latitude and longitude, in degrees.

        Parameters:
        :   - **latitude** (*float*)
            - **longitude** (*float*)

        Return type:
        :   str

    flying\_high\_altitude\_threshold
    :   The altitude, in meters, above which a vessel is considered to be
        flying “high” when doing science.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    space\_high\_altitude\_threshold
    :   The altitude, in meters, above which a vessel is considered to be
        in “high” space when doing science.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    reference\_frame
    :   The reference frame that is fixed relative to the celestial body.

        - The origin is at the center of the body.
        - The axes rotate with the body.
        - The x-axis points from the center of the body
          towards the intersection of the prime meridian and equator (the
          position at 0° longitude, 0° latitude).
        - The y-axis points from the center of the body
          towards the north pole.
        - The z-axis points from the center of the body
          towards the equator at 90°E longitude.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        ![../../../_images/celestial-body.png](../../../_images/celestial-body.png)

        Celestial body reference frame origin and axes. The equator is shown in
        blue, and the prime meridian in red.

    non\_rotating\_reference\_frame
    :   The reference frame that is fixed relative to this celestial body, and
        orientated in a fixed direction (it does not rotate with the body).

        - The origin is at the center of the body.
        - The axes do not rotate.
        - The x-axis points in an arbitrary direction through the
          equator.
        - The y-axis points from the center of the body towards
          the north pole.
        - The z-axis points in an arbitrary direction through the
          equator.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

    orbital\_reference\_frame
    :   The reference frame that is fixed relative to this celestial body, but
        orientated with the body’s orbital prograde/normal/radial directions.

        - The origin is at the center of the body.
        - The axes rotate with the orbital prograde/normal/radial
          directions.
        - The x-axis points in the orbital anti-radial direction.
        - The y-axis points in the orbital prograde direction.
        - The z-axis points in the orbital normal direction.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

    position(*reference\_frame*)
    :   The position of the center of the body, in the specified reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned position vector is in.

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

    velocity(*reference\_frame*)
    :   The linear velocity of the body, in the specified reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned velocity vector is in.

        Returns:
        :   The velocity as a vector. The vector points in the direction of travel, and its magnitude is the speed of the body in meters per second.

        Return type:
        :   tuple(float, float, float)

    rotation(*reference\_frame*)
    :   The rotation of the body, in the specified reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned rotation is in.

        Returns:
        :   The rotation as a quaternion of the form \((x, y, z, w)\).

        Return type:
        :   tuple(float, float, float, float)

    direction(*reference\_frame*)
    :   The direction in which the north pole of the celestial body is pointing,
        in the specified reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned direction is in.

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

    angular\_velocity(*reference\_frame*)
    :   The angular velocity of the body in the specified reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame the returned angular velocity is in.

        Returns:
        :   The angular velocity as a vector. The magnitude of the vector is the rotational speed of the body, in radians per second. The direction of the vector indicates the axis of rotation, using the right-hand rule.

        Return type:
        :   tuple(float, float, float)
