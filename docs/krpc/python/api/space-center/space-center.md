# SpaceCenter

Provides functionality to interact with Kerbal Space Program. This includes controlling
the active vessel, managing its resources, planning maneuver nodes and auto-piloting.

expansions
:   The names of the installed expansions, for example `"Serenity"` (Breaking Ground)
    or `"MakingHistory"` (Making History).

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   list(str)

science
:   The current amount of science.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   float

funds
:   The current amount of funds.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   float

reputation
:   The current amount of reputation.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   float

active\_vessel
:   The currently active vessel.

    Attribute:
    :   Can be read or written

    Return type:
    :   [`Vessel`](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")

vessels
:   A list of all the vessels in the game.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   list([`Vessel`](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel"))

launch\_sites
:   A list of available launch sites.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   list([`LaunchSite`](#SpaceCenter.LaunchSite "SpaceCenter.LaunchSite"))

bodies
:   A dictionary of all celestial bodies (planets, moons, etc.) in the game,
    keyed by the name of the body.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   dict(str, [`CelestialBody`](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody"))

target\_body
:   The currently targeted celestial body.

    Attribute:
    :   Can be read or written

    Return type:
    :   [`CelestialBody`](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")

    Game Scenes:
    :   Flight

target\_vessel
:   The currently targeted vessel.

    Attribute:
    :   Can be read or written

    Return type:
    :   [`Vessel`](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")

    Game Scenes:
    :   Flight

target\_docking\_port
:   The currently targeted docking port.

    Attribute:
    :   Can be read or written

    Return type:
    :   [`DockingPort`](./parts.md#SpaceCenter.DockingPort "SpaceCenter.DockingPort")

    Game Scenes:
    :   Flight

static clear\_target()
:   Clears the current target.

    Game Scenes:
    :   Flight

static launchable\_vessels(*craft\_directory*)
:   Returns a list of vessels from the given *craft\_directory*
    that can be launched.

    Parameters:
    :   **craft\_directory** (*str*) – Name of the directory in the current saves “Ships” directory. For example `"VAB"` or `"SPH"`.

    Return type:
    :   list(str)

static launch\_vessel(*craft\_directory*, *name*, *launch\_site*, *crew*[, *recover=True*][, *flag\_url=''*])
:   Launch a vessel.

    Parameters:
    :   - **craft\_directory** (*str*) – Name of the directory in the current saves “Ships” directory, that contains the craft file. For example `"VAB"` or `"SPH"`.
        - **name** (*str*) – Name of the vessel to launch. This is the name of the “.craft” file in the save directory, without the “.craft” file extension.
        - **launch\_site** (*str*) – Name of the launch site. For example `"LaunchPad"` or `"Runway"`.
        - **crew** (*list*) – A list of names of Kerbals to place in the craft. Pass an empty list to use default crew assignments.
        - **recover** (*bool*) – If true and there is a vessel on the launch site, recover it before launching.
        - **flag\_url** (*str*) – If not `None`, the asset URL of the mission flag to use for the launch.

    > **Note**
    >
    > Throws an exception if any of the games pre-flight checks fail.

static launch\_vessel\_from\_vab(*name*[, *recover=True*])
:   Launch a new vessel from the VAB onto the launchpad.

    Parameters:
    :   - **name** (*str*) – Name of the vessel to launch.
        - **recover** (*bool*) – If true and there is a vessel on the launch pad, recover it before launching.

    > **Note**
    >
    > This is equivalent to calling [`launch_vessel()`](#SpaceCenter.launch_vessel "SpaceCenter.launch_vessel") with the craft directory
    > set to “VAB” and the launch site set to “LaunchPad”.
    > Throws an exception if any of the games pre-flight checks fail.

static launch\_vessel\_from\_sph(*name*[, *recover=True*])
:   Launch a new vessel from the SPH onto the runway.

    Parameters:
    :   - **name** (*str*) – Name of the vessel to launch.
        - **recover** (*bool*) – If true and there is a vessel on the runway, recover it before launching.

    > **Note**
    >
    > This is equivalent to calling [`launch_vessel()`](#SpaceCenter.launch_vessel "SpaceCenter.launch_vessel") with the craft directory
    > set to “SPH” and the launch site set to “Runway”.
    > Throws an exception if any of the games pre-flight checks fail.

static save(*name*)
:   Save the game with a given name.
    This will create a save file called `name.sfs` in the folder of the
    current save game.

    Parameters:
    :   **name** (*str*) – Name of the save.

static load(*name*)
:   Load the game with the given name.
    This will create a load a save file called `name.sfs` from the folder of the
    current save game.

    Parameters:
    :   **name** (*str*) – Name of the save.

static quicksave()
:   Save a quicksave.

    > **Note**
    >
    > This is the same as calling [`save()`](#SpaceCenter.save "SpaceCenter.save") with the name “quicksave”.

static quickload()
:   Load a quicksave.

    > **Note**
    >
    > This is the same as calling [`load()`](#SpaceCenter.load "SpaceCenter.load") with the name “quicksave”.

static can\_revert\_to\_launch()
:   Whether the current flight can be reverted to launch.

    Return type:
    :   bool

static revert\_to\_launch()
:   Revert the current flight to launch.

static transfer\_crew(*crew\_member*, *target\_part*)
:   Transfers a crew member to a different part.

    Parameters:
    :   - **crew\_member** ([*CrewMember*](./vessel.md#SpaceCenter.CrewMember "SpaceCenter.CrewMember")) – The crew member to transfer.
        - **target\_part** ([*Part*](./parts.md#SpaceCenter.Part "SpaceCenter.Part")) – The part to move them to.

    Game Scenes:
    :   Flight

ui\_visible
:   Whether the UI is visible.

    Attribute:
    :   Can be read or written

    Return type:
    :   bool

    Game Scenes:
    :   Flight

navball
:   Whether the navball is visible.

    Attribute:
    :   Can be read or written

    Return type:
    :   bool

    Game Scenes:
    :   Flight

altimeter\_mode
:   The current mode of the altimeter.

    Attribute:
    :   Can be read or written

    Return type:
    :   [`AltimeterMode`](#SpaceCenter.AltimeterMode "SpaceCenter.AltimeterMode")

    Game Scenes:
    :   Flight

ut
:   The current universal time in seconds.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   float

g
:   The value of the [gravitational constant](https://en.wikipedia.org/wiki/Gravitational_constant) G in \(N(m/kg)^2\).

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   float

warp\_rate
:   The current warp rate. This is the rate at which time is passing for
    either on-rails or physical time warp. For example, a value of 10 means
    time is passing 10x faster than normal. Returns 1 if time warp is not
    active.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   float

    Game Scenes:
    :   Flight

warp\_factor
:   The current warp factor. This is the index of the rate at which time
    is passing for either regular “on-rails” or physical time warp. Returns 0
    if time warp is not active. When in on-rails time warp, this is equal to
    [`rails_warp_factor`](#SpaceCenter.rails_warp_factor "SpaceCenter.rails_warp_factor"), and in physics time warp, this is equal to
    [`physics_warp_factor`](#SpaceCenter.physics_warp_factor "SpaceCenter.physics_warp_factor").

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   float

    Game Scenes:
    :   Flight

rails\_warp\_factor
:   The time warp rate, using regular “on-rails” time warp. A value between
    0 and 7 inclusive. 0 means no time warp. Returns 0 if physical time warp
    is active.

    If requested time warp factor cannot be set, it will be set to the next
    lowest possible value. For example, if the vessel is too close to a
    planet. See [the KSP wiki](https://wiki.kerbalspaceprogram.com/wiki/Time_warp) for details.

    Attribute:
    :   Can be read or written

    Return type:
    :   int

    Game Scenes:
    :   Flight

physics\_warp\_factor
:   The physical time warp rate. A value between 0 and 3 inclusive. 0 means
    no time warp. Returns 0 if regular “on-rails” time warp is active.

    Attribute:
    :   Can be read or written

    Return type:
    :   int

    Game Scenes:
    :   Flight

static can\_rails\_warp\_at([*factor=1*])
:   Returns `True` if regular “on-rails” time warp can be used, at the specified warp
    *factor*. The maximum time warp rate is limited by various things,
    including how close the active vessel is to a planet. See
    [the KSP wiki](https://wiki.kerbalspaceprogram.com/wiki/Time_warp)
    for details.

    Parameters:
    :   **factor** (*int*) – The warp factor to check.

    Return type:
    :   bool

    Game Scenes:
    :   Flight

maximum\_rails\_warp\_factor
:   The current maximum regular “on-rails” warp factor that can be set.
    A value between 0 and 7 inclusive. See
    [the KSP wiki](https://wiki.kerbalspaceprogram.com/wiki/Time_warp)
    for details.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   int

    Game Scenes:
    :   Flight

static warp\_to(*ut*[, *max\_rails\_rate=100000.0*][, *max\_physics\_rate=2.0*])
:   Uses time acceleration to warp forward to a time in the future, specified
    by universal time *ut*. This call blocks until the desired
    time is reached. Uses regular “on-rails” or physical time warp as appropriate.
    For example, physical time warp is used when the active vessel is traveling
    through an atmosphere. When using regular “on-rails” time warp, the warp
    rate is limited by *max\_rails\_rate*, and when using physical
    time warp, the warp rate is limited by *max\_physics\_rate*.

    Parameters:
    :   - **ut** (*float*) – The universal time to warp to, in seconds.
        - **max\_rails\_rate** (*float*) – The maximum warp rate in regular “on-rails” time warp.
        - **max\_physics\_rate** (*float*) – The maximum warp rate in physical time warp.

    Returns:
    :   When the time warp is complete.

    Game Scenes:
    :   Flight

static transform\_position(*position*, *from*, *to*)
:   Converts a position from one reference frame to another.

    Parameters:
    :   - **position** (*tuple*) – Position, as a vector, in reference frame *from*.
        - **from** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the position is in.
        - **to** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame to covert the position to.

    Returns:
    :   The corresponding position, as a vector, in reference frame *to*.

    Return type:
    :   tuple(float, float, float)

static transform\_direction(*direction*, *from*, *to*)
:   Converts a direction from one reference frame to another.

    Parameters:
    :   - **direction** (*tuple*) – Direction, as a vector, in reference frame *from*.
        - **from** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the direction is in.
        - **to** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame to covert the direction to.

    Returns:
    :   The corresponding direction, as a vector, in reference frame *to*.

    Return type:
    :   tuple(float, float, float)

static transform\_rotation(*rotation*, *from*, *to*)
:   Converts a rotation from one reference frame to another.

    Parameters:
    :   - **rotation** (*tuple*) – Rotation, as a quaternion of the form \((x, y, z, w)\), in reference frame *from*.
        - **from** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the rotation is in.
        - **to** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame to covert the rotation to.

    Returns:
    :   The corresponding rotation, as a quaternion of the form \((x, y, z, w)\), in reference frame *to*.

    Return type:
    :   tuple(float, float, float, float)

static transform\_velocity(*position*, *velocity*, *from*, *to*)
:   Converts a velocity (acting at the specified position) from one reference frame
    to another. The position is required to take the relative angular velocity of the
    reference frames into account.

    Parameters:
    :   - **position** (*tuple*) – Position, as a vector, in reference frame *from*.
        - **velocity** (*tuple*) – Velocity, as a vector that points in the direction of travel and whose magnitude is the speed in meters per second, in reference frame *from*.
        - **from** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the position and velocity are in.
        - **to** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame to covert the velocity to.

    Returns:
    :   The corresponding velocity, as a vector, in reference frame *to*.

    Return type:
    :   tuple(float, float, float)

static raycast\_distance(*position*, *direction*, *reference\_frame*)
:   Cast a ray from a given position in a given direction, and return the distance to the hit point.
    If no hit occurs, returns infinity.

    Parameters:
    :   - **position** (*tuple*) – Position, as a vector, of the origin of the ray.
        - **direction** (*tuple*) – Direction of the ray, as a unit vector.
        - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the position and direction are in.

    Returns:
    :   The distance to the hit, in meters, or infinity if there was no hit.

    Return type:
    :   float

static raycast\_part(*position*, *direction*, *reference\_frame*)
:   Cast a ray from a given position in a given direction, and return the part that it hits.
    If no hit occurs, returns `None`.

    Parameters:
    :   - **position** (*tuple*) – Position, as a vector, of the origin of the ray.
        - **direction** (*tuple*) – Direction of the ray, as a unit vector.
        - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the position and direction are in.

    Returns:
    :   The part that was hit or `None` if there was no hit.

    Return type:
    :   [`Part`](./parts.md#SpaceCenter.Part "SpaceCenter.Part")

    Game Scenes:
    :   Flight

far\_available
:   Whether [Ferram Aerospace Research](https://forum.kerbalspaceprogram.com/index.php?/topic/19321-130-ferram-aerospace-research-v0159-liebe-82117/) is installed.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   bool

static create\_kerbal(*name*, *job*, *male*)
:   Creates a Kerbal.

    Parameters:
    :   - **name** (*str*)
        - **job** (*str*)
        - **male** (*bool*)

static get\_kerbal(*name*)
:   Find a Kerbal by name.

    Parameters:
    :   **name** (*str*)

    Return type:
    :   [`CrewMember`](./vessel.md#SpaceCenter.CrewMember "SpaceCenter.CrewMember")

static load\_space\_center()
:   > **Warning**
    >
    > Deprecated. Set [`KRPC.game_scene`](../krpc/krpc.md#KRPC.game_scene "KRPC.game_scene") instead.

    Switch to the space center view.

map\_filter
:   The visible objects in map mode.

    Attribute:
    :   Can be read or written

    Return type:
    :   [`MapFilterType`](#SpaceCenter.MapFilterType "SpaceCenter.MapFilterType")

static screenshot(*file\_path*[, *scale=1*])
:   Saves a screenshot.

    Parameters:
    :   - **file\_path** (*str*) – The path of the file to save.
        - **scale** (*int*) – Resolution scaling factor

    Game Scenes:
    :   Flight

game\_mode
:   The current mode the game is in.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   [`GameMode`](#SpaceCenter.GameMode "SpaceCenter.GameMode")

warp\_mode
:   The current time warp mode. Returns [`WarpMode.none`](#SpaceCenter.WarpMode.none "SpaceCenter.WarpMode.none") if time
    warp is not active, [`WarpMode.rails`](#SpaceCenter.WarpMode.rails "SpaceCenter.WarpMode.rails") if regular “on-rails” time warp
    is active, or [`WarpMode.physics`](#SpaceCenter.WarpMode.physics "SpaceCenter.WarpMode.physics") if physical time warp is active.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   [`WarpMode`](#SpaceCenter.WarpMode "SpaceCenter.WarpMode")

    Game Scenes:
    :   Flight

camera
:   An object that can be used to control the camera.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   [`Camera`](./camera.md#SpaceCenter.Camera "SpaceCenter.Camera")

    Game Scenes:
    :   Flight

waypoint\_manager
:   The waypoint manager.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   [`WaypointManager`](./waypoints.md#SpaceCenter.WaypointManager "SpaceCenter.WaypointManager")

    Game Scenes:
    :   Flight

contract\_manager
:   The contract manager.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   [`ContractManager`](./contracts.md#SpaceCenter.ContractManager "SpaceCenter.ContractManager")

    Game Scenes:
    :   Space Center, Flight, Tracking Station, Editor Vab, Editor Sph

alarm\_manager
:   The alarm manager.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   [`AlarmManager`](./alarms.md#SpaceCenter.AlarmManager "SpaceCenter.AlarmManager")

class GameMode
:   The game mode.
    Returned by [`GameMode`](#SpaceCenter.GameMode "SpaceCenter.GameMode")

    sandbox
    :   Sandbox mode.

    career
    :   Career mode.

    science
    :   Science career mode.

    science\_sandbox
    :   Science sandbox mode.

    mission
    :   Mission mode.

    mission\_builder
    :   Mission builder mode.

    scenario
    :   Scenario mode.

    scenario\_non\_resumable
    :   Scenario mode that cannot be resumed.

class AltimeterMode
:   The mode of the altitude reported on the altimeter.
    See [`altimeter_mode`](#SpaceCenter.altimeter_mode "SpaceCenter.altimeter_mode").

    default
    :   The altimeter is in the default mode.

    asl
    :   The altimeter is in the “Above Sea Level” mode.

    agl
    :   The altimeter is in the “Above Ground Level” mode.

class WarpMode
:   The time warp mode.
    Returned by [`WarpMode`](#SpaceCenter.WarpMode "SpaceCenter.WarpMode")

    rails
    :   Time warp is active, and in regular “on-rails” mode.

    physics
    :   Time warp is active, and in physical time warp mode.

    none
    :   Time warp is not active.

class MapFilterType
:   The set of things that are visible in map mode.
    These may be combined with bitwise logic.

    all
    :   Everything.

    none
    :   Nothing.

    debris
    :   Debris.

    unknown
    :   Unknown.

    space\_objects
    :   SpaceObjects.

    probes
    :   Probes.

    rovers
    :   Rovers.

    landers
    :   Landers.

    ships
    :   Ships.

    stations
    :   Stations.

    bases
    :   Bases.

    ev\_as
    :   EVAs.

    flags
    :   Flags.

    plane
    :   Planes.

    relay
    :   Relays.

    site
    :   Launch Sites.

    deployed\_science\_controller
    :   Deployed Science Controllers.

class LaunchSite
:   A place where craft can be launched from.
    More of these can be added with mods like Kerbal Konstructs.

    name
    :   The name of the launch site.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    body
    :   The celestial body the launch site is on.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`CelestialBody`](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")

    editor\_facility
    :   Which editor is normally used for this launch site.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`EditorFacility`](#SpaceCenter.EditorFacility "SpaceCenter.EditorFacility")

class EditorFacility
:   Editor facility.
    See [`LaunchSite.editor_facility`](#SpaceCenter.LaunchSite.editor_facility "SpaceCenter.LaunchSite.editor_facility").

    vab
    :   Vehicle Assembly Building.

    sph
    :   Space Plane Hanger.

    none
    :   None.
