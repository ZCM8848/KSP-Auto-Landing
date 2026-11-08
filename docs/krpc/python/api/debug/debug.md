# Debug

Provides functionality to modify the state of the simulation directly, in ways
the game normally does not allow: teleporting a vessel, setting its attitude,
applying forces to its parts, refilling its tanks and enabling the game’s cheat
options.

> **Note**
>
> Everything in this service is cheating. It has the same effect on a save as the
> game’s own debug menu (Alt+F12), and the same consequences: contracts, records and
> career progress made with it are not earned. It is intended for setting up a
> scenario to fly or a script to test, without flying there first.

static set\_circular\_orbit(*body*, *altitude*)
:   Move the active vessel onto a circular orbit around a body.

    Parameters:
    :   - **body** ([*SpaceCenter.CelestialBody*](../space-center/celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")) – Body to orbit.
        - **altitude** (*float*) – Altitude of the orbit, in meters above sea level.

    Game Scenes:
    :   Flight

static set\_orbit(*body*, *semi\_major\_axis*, *eccentricity*, *inclination*, *longitude\_of\_ascending\_node*, *argument\_of\_periapsis*, *mean\_anomaly\_at\_epoch*, *epoch*)
:   Move the active vessel onto the orbit with the given orbital elements. The angles use
    the same units and conventions as [`SpaceCenter.Orbit`](../space-center/orbit.md#SpaceCenter.Orbit "SpaceCenter.Orbit").

    Parameters:
    :   - **body** ([*SpaceCenter.CelestialBody*](../space-center/celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")) – Body to orbit.
        - **semi\_major\_axis** (*float*) – Semi-major axis of the orbit, in meters.
        - **eccentricity** (*float*) – Eccentricity of the orbit.
        - **inclination** (*float*) – Inclination of the orbit, in radians.
        - **longitude\_of\_ascending\_node** (*float*) – Longitude of the ascending node, in radians.
        - **argument\_of\_periapsis** (*float*) – Argument of periapsis, in radians.
        - **mean\_anomaly\_at\_epoch** (*float*) – Mean anomaly at epoch, in radians.
        - **epoch** (*float*) – Universal time of the epoch, in seconds.

    Game Scenes:
    :   Flight

static set\_landed(*body*, *latitude*, *longitude*[, *altitude=0.0*])
:   Place the active vessel on the surface of a body at the given latitude and longitude,
    and wait for it to come to rest.

    Parameters:
    :   - **body** ([*SpaceCenter.CelestialBody*](../space-center/celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")) – Body to land on.
        - **latitude** (*float*) – Latitude in degrees.
        - **longitude** (*float*) – Longitude in degrees.
        - **altitude** (*float*) – Height above the terrain to settle at, in meters. Defaults to 0, resting on the surface.

    Game Scenes:
    :   Flight

static set\_flight(*body*, *latitude*, *longitude*, *altitude*, *speed*, *heading*[, *pitch=0.0*][, *roll=0.0*][, *angle\_of\_attack=0.0*])
:   Place the active vessel in atmospheric flight over the given point: at the given altitude
    and airspeed, pointing along the given heading at the given pitch and roll, and
    let physics resume so that it is flying. Use this to set up an in-air scenario
    without flying the craft up from the runway.

    Parameters:
    :   - **body** ([*SpaceCenter.CelestialBody*](../space-center/celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")) – Body to fly over.
        - **latitude** (*float*) – Latitude in degrees.
        - **longitude** (*float*) – Longitude in degrees.
        - **altitude** (*float*) – Altitude in meters above sea level.
        - **speed** (*float*) – Airspeed in meters per second, relative to the surface.
        - **heading** (*float*) – Compass heading to point along, in degrees (90 is east).
        - **pitch** (*float*) – Pitch above the horizon, in degrees. Defaults to 0 (level).
        - **roll** (*float*) – Roll, in degrees. Defaults to 0 (wings level).
        - **angle\_of\_attack** (*float*) – Angle of attack in degrees: how far the airspeed vector sits below the nose, in the pitch plane. 0 (the default) puts the airspeed along the nose; a positive value gives a nose-up attitude relative to the flight path, so the flight-path angle is *pitch* minus this.

    Game Scenes:
    :   Flight

    > **Note**
    >
    > The pitch, heading and roll match those reported by the vessel’s
    > [`SpaceCenter.Flight`](../space-center/flight.md#SpaceCenter.Flight "SpaceCenter.Flight").

static set\_position(*position*[, *reference\_frame=None*])
:   Move the active vessel to the given position, leaving its velocity as it is. To put it
    alongside another vessel, pass a position in that vessel’s reference frame.

    Parameters:
    :   - **position** (*tuple*) – Position of the vessel’s center of mass, in meters.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – Reference frame the position is in. Defaults to the reference frame of the body being orbited.

    Game Scenes:
    :   Flight

    > **Note**
    >
    > This is the inverse of [`SpaceCenter.Vessel.position()`](../space-center/vessel.md#SpaceCenter.Vessel.position "SpaceCenter.Vessel.position") in the same
    > reference frame.

static set\_velocity(*velocity*[, *reference\_frame=None*])
:   Set the velocity of the active vessel, leaving it where it is. The vessel is moved onto the
    orbit that passes through its current position at the new velocity, so this works
    in orbit and in the atmosphere alike.

    Parameters:
    :   - **velocity** (*tuple*) – Velocity in meters per second.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – Reference frame the velocity is in. Defaults to the reference frame of the body being orbited, which rotates with it, so a zero velocity leaves the vessel at rest relative to the ground.

    Game Scenes:
    :   Flight

    > **Note**
    >
    > This is the inverse of [`SpaceCenter.Vessel.velocity()`](../space-center/vessel.md#SpaceCenter.Vessel.velocity "SpaceCenter.Vessel.velocity") in the same
    > reference frame.

static set\_pitch\_heading\_roll(*pitch*, *heading*, *roll*[, *reference\_frame=None*][, *vessel=None*])
:   Set the attitude of a vessel to the given pitch, heading and roll in the given
    reference frame, and stop it rotating. Lets a script start from a fixed, still
    pose without first flying the autopilot there.

    Parameters:
    :   - **pitch** (*float*) – Target pitch, in degrees.
        - **heading** (*float*) – Target heading, in degrees.
        - **roll** (*float*) – Target roll, in degrees.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – Reference frame the angles are in. Defaults to the vessel’s surface reference frame.
        - **vessel** ([*SpaceCenter.Vessel*](../space-center/vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel") *or* *None*) – Vessel to rotate. Defaults to the active vessel.

    Game Scenes:
    :   Flight

    > **Note**
    >
    > The angles match those reported by the vessel’s
    > [`SpaceCenter.Flight`](../space-center/flight.md#SpaceCenter.Flight "SpaceCenter.Flight").

static set\_direction(*direction*, *roll*[, *reference\_frame=None*][, *vessel=None*])
:   Point a vessel along the given direction in the given reference frame, and stop it
    rotating. This is [`set_pitch_heading_roll()`](#Debug.set_pitch_heading_roll "Debug.set_pitch_heading_roll") with a pointing vector instead
    of a pitch and heading, matching the attitude the autopilot holds for the same
    target direction and roll.

    Parameters:
    :   - **direction** (*tuple*) – Direction to point the nose of the vessel in.
        - **roll** (*float*) – Target roll, in degrees, or `NaN` to leave the roll uncontrolled.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – Reference frame the direction and roll are in. Defaults to the vessel’s surface reference frame.
        - **vessel** ([*SpaceCenter.Vessel*](../space-center/vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel") *or* *None*) – Vessel to rotate. Defaults to the active vessel.

    Game Scenes:
    :   Flight

static apply\_rotation(*angle*, *axis*[, *vessel=None*])
:   Rotate a vessel about the given axis, in its own reference frame, by the given angle.

    Parameters:
    :   - **angle** (*float*) – Angle to rotate by, in degrees.
        - **axis** (*tuple*) – Axis to rotate about.
        - **vessel** ([*SpaceCenter.Vessel*](../space-center/vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel") *or* *None*) – Vessel to rotate. Defaults to the active vessel.

    Game Scenes:
    :   Flight

static set\_angular\_velocity(*angular\_velocity*[, *reference\_frame=None*][, *vessel=None*])
:   Set the angular velocity of a vessel, expressed in the given reference frame. Pass a
    zero vector to stop the vessel rotating.

    Parameters:
    :   - **angular\_velocity** (*tuple*) – Angular velocity, in radians per second. The direction is the axis of rotation and the magnitude is the rate.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – Reference frame the angular velocity is in. Defaults to the vessel’s surface reference frame.
        - **vessel** ([*SpaceCenter.Vessel*](../space-center/vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel") *or* *None*) – Vessel to rotate. Defaults to the active vessel.

    Game Scenes:
    :   Flight

    > **Note**
    >
    > The whole vessel is put into a rigid rotation about its center of mass, so it spins in
    > place rather than shearing apart or translating.

static add\_force(*part*, *force*, *position*, *reference\_frame*)
:   Exert a constant force on a part, acting at the given position.

    Parameters:
    :   - **part** ([*SpaceCenter.Part*](../space-center/parts.md#SpaceCenter.Part "SpaceCenter.Part")) – The part to apply the force to.
        - **force** (*tuple*) – A vector pointing in the direction that the force acts, with its magnitude equal to the strength of the force in Newtons.
        - **position** (*tuple*) – The position at which the force acts, as a vector.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the force and position are in.

    Returns:
    :   An object that can be used to remove or modify the force.

    Return type:
    :   [`SpaceCenter.Force`](../space-center/parts.md#SpaceCenter.Force "SpaceCenter.Force")

    Game Scenes:
    :   Flight

    > **Note**
    >
    > The force is removed when the client that added it disconnects.

static instantaneous\_force(*part*, *force*, *position*, *reference\_frame*)
:   Exert an instantaneous force on a part, acting at the given position.

    Parameters:
    :   - **part** ([*SpaceCenter.Part*](../space-center/parts.md#SpaceCenter.Part "SpaceCenter.Part")) – The part to apply the force to.
        - **force** (*tuple*) – A vector pointing in the direction that the force acts, with its magnitude equal to the strength of the force in Newtons.
        - **position** (*tuple*) – The position at which the force acts, as a vector.
        - **reference\_frame** ([*SpaceCenter.ReferenceFrame*](../space-center/reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the force and position are in.

    Game Scenes:
    :   Flight

    > **Note**
    >
    > The force is applied instantaneously in a single physics update.

static fill\_all\_resources([*vessel=None*])
:   Fill every resource tank on a vessel to its maximum capacity.

    Parameters:
    :   **vessel** ([*SpaceCenter.Vessel*](../space-center/vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel") *or* *None*) – Vessel to fill. Defaults to the active vessel.

    Game Scenes:
    :   Flight

static fill\_resources(*resource\_name*[, *vessel=None*])
:   Fill the tanks of a single resource on a vessel to their maximum capacity.

    Parameters:
    :   - **resource\_name** (*str*) – Name of the resource to fill, for example `"LiquidFuel"`.
        - **vessel** ([*SpaceCenter.Vessel*](../space-center/vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel") *or* *None*) – Vessel to fill. Defaults to the active vessel.

    Game Scenes:
    :   Flight

infinite\_propellant
:   Whether engines and RCS thrusters draw no propellant.

    Attribute:
    :   Can be read or written

    Return type:
    :   bool

infinite\_electricity
:   Whether parts draw no electric charge.

    Attribute:
    :   Can be read or written

    Return type:
    :   bool

no\_crash\_damage
:   Whether parts survive collisions that would otherwise destroy them.

    Attribute:
    :   Can be read or written

    Return type:
    :   bool

unbreakable\_joints
:   Whether the joints between parts are rigid and cannot break.

    Attribute:
    :   Can be read or written

    Return type:
    :   bool

ignore\_max\_temperature
:   Whether parts survive temperatures above their maximum, instead of exploding.

    Attribute:
    :   Can be read or written

    Return type:
    :   bool

allow\_part\_clipping
:   Whether parts may be placed so that they intersect one another in the editor.

    Attribute:
    :   Can be read or written

    Return type:
    :   bool

non\_strict\_attachment\_orientation
:   Whether parts may be attached in any orientation, rather than only the ones their
    attachment nodes allow.

    Attribute:
    :   Can be read or written

    Return type:
    :   bool

ignore\_kerbal\_inventory\_limits
:   Whether a kerbal’s inventory may hold more than its slot and mass limits allow.

    Attribute:
    :   Can be read or written

    Return type:
    :   bool

ignore\_eva\_construction\_mass\_limit
:   Whether EVA construction may move parts heavier than a kerbal can normally carry.

    Attribute:
    :   Can be read or written

    Return type:
    :   bool

ignore\_agency\_mindset\_on\_contracts
:   Whether contracts are offered regardless of the agency mindset that normally
    restricts which ones appear.

    Attribute:
    :   Can be read or written

    Return type:
    :   bool

pause\_on\_vessel\_unpack
:   Whether the game pauses whenever a vessel comes off rails.

    Attribute:
    :   Can be read or written

    Return type:
    :   bool

biomes\_visible
:   Whether biome boundaries are drawn on the surface of bodies.

    Attribute:
    :   Can be read or written

    Return type:
    :   bool

gravity\_multiplier
:   Multiplier applied to the force of gravity on every vessel. 1 is normal gravity,
    0 is weightlessness.

    Attribute:
    :   Can be read or written

    Return type:
    :   float

funds
:   The number of funds the player has. Only available in career mode.

    Attribute:
    :   Can be read or written

    Return type:
    :   float

science
:   The amount of science the player has. Only available in career and science mode.

    Attribute:
    :   Can be read or written

    Return type:
    :   float

reputation
:   The player’s reputation. Only available in career mode.

    Attribute:
    :   Can be read or written

    Return type:
    :   float

static unlock\_technology\_tree()
:   Research every node in the technology tree and purchase every part in it.
    Only available in career and science mode.

static upgrade\_facilities()
:   Upgrade every building at the space center to its highest level.
    Only available in career mode.

static max\_kerbal\_experience()
:   Raise every kerbal in the roster to the highest experience level.
