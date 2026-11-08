# Flight

class Flight
:   Used to get flight telemetry for a vessel, by calling [`Vessel.flight()`](./vessel.md#SpaceCenter.Vessel.flight "SpaceCenter.Vessel.flight").
    All of the information returned by this class is given in the reference frame
    passed to that method.
    Obtained by calling [`Vessel.flight()`](./vessel.md#SpaceCenter.Vessel.flight "SpaceCenter.Vessel.flight").

    > **Note**
    >
    > To get orbital information, such as the apoapsis or inclination, see [`Orbit`](./orbit.md#SpaceCenter.Orbit "SpaceCenter.Orbit").

    g\_force
    :   The current G force acting on the vessel in \(g\).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    mean\_altitude
    :   The altitude above sea level, in meters.
        Measured from the center of mass of the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    surface\_altitude
    :   The altitude above the surface of the body or sea level, whichever is closer, in meters.
        Measured from the center of mass of the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    bedrock\_altitude
    :   The altitude above the surface of the body, in meters. When over water, this is the altitude above the sea floor.
        Measured from the center of mass of the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    elevation
    :   The elevation of the terrain under the vessel, in meters. This is the height of the terrain above sea level,
        and is negative when the vessel is over the sea.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    latitude
    :   The [latitude](https://en.wikipedia.org/wiki/Latitude) of the vessel for the body being orbited, in degrees.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    longitude
    :   The [longitude](https://en.wikipedia.org/wiki/Longitude) of the vessel for the body being orbited, in degrees.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    surface\_normal
    :   The normal to the surface under the vessel, in the reference frame
        [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame"). This describes the slope of the terrain,
        and points straight up when the vessel is over water.

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The normal as a unit vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    velocity
    :   The velocity of the vessel, in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The velocity as a vector. The vector points in the direction of travel, and its magnitude is the speed of the vessel in meters per second.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    speed
    :   The speed of the vessel in meters per second,
        in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    horizontal\_speed
    :   The horizontal speed of the vessel in meters per second,
        in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    vertical\_speed
    :   The vertical speed of the vessel in meters per second,
        in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    acceleration
    :   The acceleration of the vessel, in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").
        This is the total acceleration, including the acceleration due to gravity, and is the
        time derivative of [`Flight.velocity`](#SpaceCenter.Flight.velocity "SpaceCenter.Flight.velocity").

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The acceleration as a vector. The vector points in the direction of the acceleration, and its magnitude is the acceleration of the vessel in \(m/s^2\).

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    center\_of\_mass
    :   The position of the center of mass of the vessel,
        in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    rotation
    :   The rotation of the vessel, in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The rotation as a quaternion of the form \((x, y, z, w)\).

        Return type:
        :   tuple(float, float, float, float)

        Game Scenes:
        :   Flight

    direction
    :   The direction that the vessel is pointing in,
        in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    pitch
    :   The pitch of the vessel relative to the horizon, in degrees.
        A value between -90° and +90°.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > An absolute Euler angle, ill-conditioned when the vessel points near vertical (a pitch
        > near -90° or +90°), where heading and roll become ambiguous. For an always-defined
        > attitude use
        > [`Flight.rotation`](#SpaceCenter.Flight.rotation "SpaceCenter.Flight.rotation") or [`Flight.direction`](#SpaceCenter.Flight.direction "SpaceCenter.Flight.direction").

    heading
    :   The heading of the vessel (its angle relative to north), in degrees.
        A value between 0° and 360°.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > An absolute Euler angle, undefined when the vessel points near vertical (a pitch near
        > -90° or +90°). For an always-defined attitude use [`Flight.rotation`](#SpaceCenter.Flight.rotation "SpaceCenter.Flight.rotation") or
        > [`Flight.direction`](#SpaceCenter.Flight.direction "SpaceCenter.Flight.direction").

    roll
    :   The roll of the vessel relative to the horizon, in degrees.
        A value between -180° and +180°.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > An absolute Euler angle, ill-conditioned when the vessel points near vertical (a pitch
        > near -90° or +90°), where the vertical-plane reference vanishes. For an always-defined
        > attitude use
        > [`Flight.rotation`](#SpaceCenter.Flight.rotation "SpaceCenter.Flight.rotation"); for a well-defined roll use the auto-pilot’s
        > `TargetRoll` / `RollError` against a chosen up reference.

    prograde
    :   The prograde direction of the vessels orbit,
        in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    retrograde
    :   The retrograde direction of the vessels orbit,
        in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    surface\_prograde
    :   The direction of the vessels surface velocity,
        in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").
        This is the prograde direction as shown on the navball when in surface mode.

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Singular when surface speed is approximately zero.

    surface\_retrograde
    :   The direction opposite to the vessels surface velocity,
        in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").
        This is the retrograde direction as shown on the navball when in surface mode.

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Singular when surface speed is approximately zero.

    normal
    :   The direction normal to the vessels orbit,
        in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    anti\_normal
    :   The direction opposite to the normal of the vessels orbit,
        in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    radial
    :   The radial direction of the vessels orbit,
        in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    anti\_radial
    :   The direction opposite to the radial direction of the vessels orbit,
        in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    atmosphere\_density
    :   The current density of the atmosphere around the vessel, in \(kg/m^3\).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    dynamic\_pressure
    :   The dynamic pressure acting on the vessel, in Pascals. This is a measure of the
        strength of the aerodynamic forces. It is equal to
        \(\frac{1}{2} . \mbox{air density} . \mbox{velocity}^2\).
        It is commonly denoted \(Q\).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    static\_pressure
    :   The static pressure acting on the vessel, in Pascals.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Below the waterline this includes the pressure of the water above the vessel’s
        > center of mass.

    static\_pressure\_at\_msl
    :   The static atmospheric pressure at mean sea level, in Pascals.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    displacement
    :   The volume of water the vessel displaces when fully submerged, in
        \(m^3\).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    displaced\_volume
    :   The volume of water the vessel is displacing, in \(m^3\).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    submerged\_portion
    :   The fraction of the vessel below the waterline, between 0 and 1. This is
        [`Flight.displaced_volume`](#SpaceCenter.Flight.displaced_volume "SpaceCenter.Flight.displaced_volume") over [`Flight.displacement`](#SpaceCenter.Flight.displacement "SpaceCenter.Flight.displacement").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    center\_of\_buoyancy
    :   The point the total buoyant force on the vessel acts through, in reference frame
        [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame"). Returns `None` if no part of the vessel is in
        the water.

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float) or None

        Game Scenes:
        :   Flight

        > **Note**
        >
        > A hull at rest carries this on the same vertical line as
        > [`Flight.center_of_mass`](#SpaceCenter.Flight.center_of_mass "SpaceCenter.Flight.center_of_mass"). A horizontal offset between the two turns the weight
        > and the buoyancy into a couple that rolls or pitches the vessel.

    buoyant\_force
    :   The total buoyant force acting on the vessel, in reference frame
        [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   A vector pointing in the direction that the force acts, with its magnitude equal to the strength of the force in Newtons.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > A vessel floating at rest carries a buoyant force equal to its weight.

    buoyant\_acceleration
    :   The acceleration of the vessel due to the buoyant force acting on it
        ([`Flight.buoyant_force`](#SpaceCenter.Flight.buoyant_force "SpaceCenter.Flight.buoyant_force") divided by the vessel’s mass),
        in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   A vector pointing in the direction that the vessel is accelerated, with its magnitude equal to the acceleration in \(m/s^2\).

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    buoyant\_torque
    :   The torque the buoyant force exerts on the vessel about its center of mass,
        in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   A vector pointing along the axis of the torque, with its magnitude equal to the strength of the torque in newton-meters.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    fully\_submerged\_center\_of\_buoyancy
    :   The point the buoyant force acts through with the vessel fully submerged, in
        reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame"). Returns `None` if no part of
        the vessel displaces any water.

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float) or None

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The point is fixed relative to the hull. [`Flight.center_of_buoyancy`](#SpaceCenter.Flight.center_of_buoyancy "SpaceCenter.Flight.center_of_buoyancy") moves as
        > the vessel rides the waves.

    buoyant\_force\_at(*density*, *gravity*)
    :   The buoyant force on the vessel when fully submerged in a fluid of the given
        density, in Newtons.

        Parameters:
        :   - **density** (*float*) – The density of the fluid, in \(kg/m^3\).
            - **gravity** (*float*) – The acceleration due to gravity, in \(m/s^2\).

        Return type:
        :   float

        Game Scenes:
        :   Flight

    buoyant\_torque\_at(*density*, *gravity*)
    :   The torque the buoyant force exerts about the vessel’s center of mass when fully
        submerged in a fluid of the given density, in reference frame
        [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame"). Gravity acts towards the center of the body.

        Parameters:
        :   - **density** (*float*) – The density of the fluid, in \(kg/m^3\).
            - **gravity** (*float*) – The acceleration due to gravity, in \(m/s^2\).

        Returns:
        :   A vector pointing along the axis of the torque, with its magnitude equal to the strength of the torque in newton-meters.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    depth
    :   The depth of the center of mass of the vessel below the waterline, in meters.
        Negative above the waterline.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    submerged\_dynamic\_pressure
    :   The dynamic pressure of the water acting on the vessel, in Pascals.
        Returns zero if no part of the vessel is below the waterline.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    aerodynamic\_force
    :   The total aerodynamic forces acting on the vessel,
        in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   A vector pointing in the direction that the force acts, with its magnitude equal to the strength of the force in Newtons.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > This is the force the aerodynamics model actually applied to the vessel on the
        > last physics frame, summed over its parts, so it points where the air pushes the
        > vessel whatever the vessel’s orientation. It equals [`Flight.lift`](#SpaceCenter.Flight.lift "SpaceCenter.Flight.lift") plus
        > [`Flight.side_force`](#SpaceCenter.Flight.side_force "SpaceCenter.Flight.side_force") plus [`Flight.drag`](#SpaceCenter.Flight.drag "SpaceCenter.Flight.drag").

    aerodynamic\_torque
    :   The net aerodynamic torque currently acting on the vessel about its center of
        mass, in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame"). The magnitude is in
        newton-meters.

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   A vector pointing along the axis of the torque, with its magnitude equal to the strength of the torque in newton-meters.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > This is the live counterpart to [`Flight.aerodynamic_force`](#SpaceCenter.Flight.aerodynamic_force "SpaceCenter.Flight.aerodynamic_force"): it is the torque
        > the aerodynamics model applied on the current physics frame, taken about the
        > center of mass, rather than one re-simulated for hypothetical conditions the way
        > [`Flight.simulate_aerodynamic_torque_at()`](#SpaceCenter.Flight.simulate_aerodynamic_torque_at "SpaceCenter.Flight.simulate_aerodynamic_torque_at") does. It is intended for validating the
        > simulator against the live game state.

    simulate\_aerodynamic\_force\_at(*body*, *position*, *velocity*, *rotation*)
    :   Simulate and return the total aerodynamic forces acting on the vessel,
        if it were traveling with the given velocity, at the given position and
        orientation, in the atmosphere of the given celestial body.

        Parameters:
        :   - **body** ([*CelestialBody*](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")) – The celestial body whose atmosphere the forces are simulated in.
            - **position** (*tuple*) – The position of the vessel, in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").
            - **velocity** (*tuple*) – The velocity of the vessel, in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").
            - **rotation** (*tuple*) – The orientation of the vessel, in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame"), in the same form as [`Vessel.rotation()`](./vessel.md#SpaceCenter.Vessel.rotation "SpaceCenter.Vessel.rotation"). The angle of attack and sideslip follow from this orientation relative to the velocity; the roll component sets the direction of any aerodynamic lift. Pass the vessel’s current rotation to evaluate the force at its current orientation.

        Returns:
        :   A vector pointing in the direction that the force acts, with its magnitude equal to the strength of the force in Newtons, in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The position, velocity and rotation arguments, and the returned force, are all
        > expressed in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame"). The result is the
        > force the vessel would experience if it were placed at that position and
        > orientation with the air flowing past it at that velocity; it is the force at
        > the requested orientation, not the force in the vessel’s current orientation.
        > Atmospheric temperature and density are evaluated at the current universal time.

    simulate\_aerodynamic\_torque\_at(*body*, *position*, *velocity*, *rotation*, *angular\_velocity*)
    :   Simulate and return the total aerodynamic torque acting on the vessel about its
        center of mass, if it were traveling with the given velocity, at the given
        position, orientation and angular velocity, in the atmosphere of the given
        celestial body.

        Parameters:
        :   - **body** ([*CelestialBody*](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")) – The celestial body whose atmosphere the torque is simulated in.
            - **position** (*tuple*) – The position of the vessel, in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").
            - **velocity** (*tuple*) – The velocity of the vessel, in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").
            - **rotation** (*tuple*) – The orientation of the vessel, in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame"), in the same form as [`Vessel.rotation()`](./vessel.md#SpaceCenter.Vessel.rotation "SpaceCenter.Vessel.rotation"). Pass the vessel’s current rotation to evaluate the torque at its current orientation.
            - **angular\_velocity** (*tuple*) – The angular velocity of the vessel, in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame"). This adds the solid-body rotation term to each part’s local airflow and the per-part rigid-body angular drag the game applies, together giving the aerodynamic damping torque. Pass a zero vector to evaluate the static torque.

        Returns:
        :   A vector pointing along the axis of the torque, with its magnitude equal to the strength of the torque in newton-meters, in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The position, velocity, rotation and angular velocity arguments, and the returned
        > torque, are all expressed in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame"). When
        > [Ferram Aerospace Research](https://forum.kerbalspaceprogram.com/topic/19321-131-ferram-aerospace-research-v01591-liepmann-4218/)
        > is installed the angular velocity argument is ignored.
        > Atmospheric temperature and density are evaluated at the current universal time.
        >
        > This is the ideal rigid-body aerodynamic torque, summed from the per-part forces
        > about the center of mass. A vessel may not visibly rotate by the full amount when
        > a large aerodynamic force acts on a small part far from the center of mass, because
        > the game applies each part’s force to that part and propagates it through the joints
        > rather than to the vessel as a rigid body.

    simulate\_aerodynamic\_wrench\_at(*body*, *position*, *velocity*, *rotation*, *angular\_velocity*, *ut*)
    :   Simulate and return the total aerodynamic force and torque acting on the vessel,
        if its center of mass were traveling with the given velocity, at the given position,
        orientation and angular velocity, in the atmosphere of the given celestial body.

        Parameters:
        :   - **body** ([*CelestialBody*](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")) – The celestial body whose atmosphere the wrench is simulated in.
            - **position** (*tuple*) – The position of the vessel’s center of mass, in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").
            - **velocity** (*tuple*) – The velocity of the vessel’s center of mass, in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").
            - **rotation** (*tuple*) – The orientation of the vessel, in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame"), in the same form as [`Vessel.rotation()`](./vessel.md#SpaceCenter.Vessel.rotation "SpaceCenter.Vessel.rotation"). Pass the vessel’s current rotation to evaluate the wrench at its current orientation.
            - **angular\_velocity** (*tuple*) – The angular velocity of the vessel, in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame"). This adds the solid-body rotation term to each part’s local airflow and the per-part rigid-body angular drag the game applies, together giving the aerodynamic damping force and torque. Pass a zero vector to evaluate the static wrench relative to the reference frame.
            - **ut** (*float*) – The universal time used for the atmospheric ephemeris. It selects the body/Sun geometry used for temperature and density, but does not change or propagate [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") or any of the state arguments.

        Returns:
        :   The aerodynamic force, and the aerodynamic torque about the vessel’s center of mass. Both are vectors in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Return type:
        :   [`Wrench`](#SpaceCenter.Wrench "SpaceCenter.Wrench")

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The position and velocity describe the hypothetical center-of-mass state. The
        > position, velocity, rotation and angular velocity arguments, and both returned
        > vectors, are expressed in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").
        > For future-state prediction, [`CelestialBody.non_rotating_reference_frame`](./celestial-body.md#SpaceCenter.CelestialBody.non_rotating_reference_frame "SpaceCenter.CelestialBody.non_rotating_reference_frame")
        > is recommended so that the spatial state has unambiguous inertial semantics.
        >
        > This is an instantaneous rigid-body result based on the vessel’s current parts,
        > drag cubes and control-surface state. When
        > [Ferram Aerospace Research](https://forum.kerbalspaceprogram.com/topic/19321-131-ferram-aerospace-research-v01591-liepmann-4218/)
        > is installed the angular velocity and *ut* arguments are ignored.

    lift
    :   The [aerodynamic lift](https://en.wikipedia.org/wiki/Aerodynamic_force)
        currently acting on the vessel.

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   A vector pointing in the direction that the force acts, with its magnitude equal to the strength of the force in Newtons.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The part of [`Flight.aerodynamic_force`](#SpaceCenter.Flight.aerodynamic_force "SpaceCenter.Flight.aerodynamic_force") that acts across the air stream and
        > towards the top of the vessel: perpendicular to the air stream, in the plane the
        > vessel’s nose and its top sit in. A vessel flying with sideslip is also pushed out
        > of that plane, and that part of the force is [`Flight.side_force`](#SpaceCenter.Flight.side_force "SpaceCenter.Flight.side_force"). Zero for a
        > vessel flying exactly sideways, which has no such plane to speak of.

    side\_force
    :   The aerodynamic side force currently acting on the vessel.

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   A vector pointing in the direction that the force acts, with its magnitude equal to the strength of the force in Newtons.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The part of [`Flight.aerodynamic_force`](#SpaceCenter.Flight.aerodynamic_force "SpaceCenter.Flight.aerodynamic_force") that acts across the air stream and out
        > of the vessel’s side, which is what the air does to a vessel flying with sideslip.
        > It is perpendicular to both [`Flight.lift`](#SpaceCenter.Flight.lift "SpaceCenter.Flight.lift") and [`Flight.drag`](#SpaceCenter.Flight.drag "SpaceCenter.Flight.drag"), and the
        > three of them sum to [`Flight.aerodynamic_force`](#SpaceCenter.Flight.aerodynamic_force "SpaceCenter.Flight.aerodynamic_force").

    drag
    :   The [aerodynamic drag](https://en.wikipedia.org/wiki/Aerodynamic_force) currently acting on the vessel.

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   A vector pointing in the direction of the force, with its magnitude equal to the strength of the force in Newtons.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The part of [`Flight.aerodynamic_force`](#SpaceCenter.Flight.aerodynamic_force "SpaceCenter.Flight.aerodynamic_force") along the air stream, opposing the
        > vessel’s motion through the air.

    aerodynamic\_acceleration
    :   The acceleration of the vessel due to the total aerodynamic forces acting on it
        ([`Flight.aerodynamic_force`](#SpaceCenter.Flight.aerodynamic_force "SpaceCenter.Flight.aerodynamic_force") divided by the vessel’s mass),
        in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   A vector pointing in the direction that the vessel is accelerated, with its magnitude equal to the acceleration in \(m/s^2\).

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    lift\_acceleration
    :   The acceleration of the vessel due to [`Flight.lift`](#SpaceCenter.Flight.lift "SpaceCenter.Flight.lift")
        (the aerodynamic lift divided by the vessel’s mass),
        in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   A vector pointing in the direction that the vessel is accelerated, with its magnitude equal to the acceleration in \(m/s^2\).

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    drag\_acceleration
    :   The acceleration of the vessel due to [`Flight.drag`](#SpaceCenter.Flight.drag "SpaceCenter.Flight.drag")
        (the aerodynamic drag divided by the vessel’s mass),
        in reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame").

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   A vector pointing in the direction that the vessel is accelerated, with its magnitude equal to the acceleration in \(m/s^2\).

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    speed\_of\_sound
    :   The speed of sound, in the atmosphere around the vessel, in \(m/s\).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    mach
    :   The speed of the vessel, in multiples of the speed of sound.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    reynolds\_number
    :   The vessels Reynolds number.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Requires [Ferram Aerospace Research](https://forum.kerbalspaceprogram.com/topic/19321-131-ferram-aerospace-research-v01591-liepmann-4218/).

    true\_air\_speed
    :   The [true air speed](https://en.wikipedia.org/wiki/True_airspeed)
        of the vessel, in meters per second.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    equivalent\_air\_speed
    :   The [equivalent air speed](https://en.wikipedia.org/wiki/Equivalent_airspeed)
        of the vessel, in meters per second.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    terminal\_velocity
    :   An estimate of the current terminal velocity of the vessel, in meters per second.
        This is the speed at which the drag forces cancel out the force of gravity.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    angle\_of\_attack
    :   The pitch angle between the orientation of the vessel and its velocity vector,
        in degrees.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    sideslip\_angle
    :   The yaw angle between the orientation of the vessel and its velocity vector, in degrees.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    total\_air\_temperature
    :   The [total air temperature](https://en.wikipedia.org/wiki/Total_air_temperature)
        of the atmosphere around the vessel, in Kelvin.
        This includes the [`Flight.static_air_temperature`](#SpaceCenter.Flight.static_air_temperature "SpaceCenter.Flight.static_air_temperature") and the vessel’s kinetic energy.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    static\_air\_temperature
    :   The [static (ambient) temperature](https://en.wikipedia.org/wiki/Total_air_temperature) of the atmosphere around the vessel, in Kelvin.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    stall\_fraction
    :   The current amount of stall, between 0 and 1. A value greater than 0.005 indicates
        a minor stall and a value greater than 0.5 indicates a large-scale stall.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Requires [Ferram Aerospace Research](https://forum.kerbalspaceprogram.com/topic/19321-131-ferram-aerospace-research-v01591-liepmann-4218/).

    drag\_coefficient
    :   The coefficient of drag. This is the amount of drag produced by the vessel.
        It depends on air speed, air density and wing area.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Requires [Ferram Aerospace Research](https://forum.kerbalspaceprogram.com/topic/19321-131-ferram-aerospace-research-v01591-liepmann-4218/).

    lift\_coefficient
    :   The coefficient of lift. This is the amount of lift produced by the vessel, and
        depends on air speed, air density and wing area.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Requires [Ferram Aerospace Research](https://forum.kerbalspaceprogram.com/topic/19321-131-ferram-aerospace-research-v01591-liepmann-4218/).

    ballistic\_coefficient
    :   The [ballistic coefficient](https://en.wikipedia.org/wiki/Ballistic_coefficient).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Requires [Ferram Aerospace Research](https://forum.kerbalspaceprogram.com/topic/19321-131-ferram-aerospace-research-v01591-liepmann-4218/).

    thrust\_specific\_fuel\_consumption
    :   The thrust specific fuel consumption for the jet engines on the vessel. This is a
        measure of the efficiency of the engines, with a lower value indicating a more
        efficient vessel. This value is the number of Newtons of fuel that are burned,
        per hour, to produce one newton of thrust.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Requires [Ferram Aerospace Research](https://forum.kerbalspaceprogram.com/topic/19321-131-ferram-aerospace-research-v01591-liepmann-4218/).

class Wrench(*force*, *torque*)
:   A force and a torque that act together.

    force
    :   The force, in Newtons.

        Return type:
        :   tuple(float, float, float)

    torque
    :   The torque, in Newton meters.

        Return type:
        :   tuple(float, float, float)
