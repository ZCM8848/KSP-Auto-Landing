# Control

class Control
:   Used to manipulate the controls of a vessel. This includes adjusting the
    throttle, enabling/disabling systems such as SAS and RCS, or altering the
    direction in which the vessel is pointing.
    Obtained by calling [`Vessel.control`](./vessel.md#SpaceCenter.Vessel.control "SpaceCenter.Vessel.control").

    > **Note**
    >
    > Control inputs (pitch, yaw, roll, translation, wheel throttle, wheel steering
    > and the custom axes) are zeroed when all clients that have set one or more of
    > these inputs are no longer connected. The throttle is an exception: it keeps
    > its value when clients disconnect.

    source
    :   The source of the vessels control, for example by a kerbal or a probe core.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ControlSource`](#SpaceCenter.ControlSource "SpaceCenter.ControlSource")

        Game Scenes:
        :   Flight

    state
    :   The control state of the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ControlState`](#SpaceCenter.ControlState "SpaceCenter.ControlState")

        Game Scenes:
        :   Flight

    sas
    :   The state of SAS.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Equivalent to [`AutoPilot.sas`](./auto-pilot.md#SpaceCenter.AutoPilot.sas "SpaceCenter.AutoPilot.sas").
        > Throws an exception if set to `True` while the auto-pilot is engaged, as the
        > auto-pilot holds SAS off for as long as it is flying the vessel.

    sas\_mode
    :   The current [`Control.sas_mode`](#SpaceCenter.Control.sas_mode "SpaceCenter.Control.sas_mode").
        These modes are equivalent to the mode buttons to
        the left of the navball that appear when SAS is enabled.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`SASMode`](#SpaceCenter.SASMode "SpaceCenter.SASMode")

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Equivalent to [`AutoPilot.sas_mode`](./auto-pilot.md#SpaceCenter.AutoPilot.sas_mode "SpaceCenter.AutoPilot.sas_mode")

    speed\_mode
    :   The current [`Control.speed_mode`](#SpaceCenter.Control.speed_mode "SpaceCenter.Control.speed_mode") of the navball.
        This is the mode displayed next to the speed at the top of the navball.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`SpeedMode`](#SpaceCenter.SpeedMode "SpaceCenter.SpeedMode")

        Game Scenes:
        :   Flight

    engine\_gimbals
    :   Returns whether all gimballed engines on the vessel have gimbal enabled,
        and sets the gimbal enabled state of all gimballed engines.
        See [`Engine.gimbal_locked`](./parts.md#SpaceCenter.Engine.gimbal_locked "SpaceCenter.Engine.gimbal_locked").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    thrust\_reversers
    :   Returns whether all engines with a thrust reverser on the vessel have
        their thrust reverser engaged, and sets the thrust reverser state of all
        such engines.
        See [`Engine.thrust_reversed`](./parts.md#SpaceCenter.Engine.thrust_reversed "SpaceCenter.Engine.thrust_reversed").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    aero\_surfaces
    :   Returns whether all control surfaces on the vessel have pitch, yaw and roll enabled,
        and sets the pitch, yaw and roll enabled state of all control surfaces.
        See [`ControlSurface.pitch_enabled`](./parts.md#SpaceCenter.ControlSurface.pitch_enabled "SpaceCenter.ControlSurface.pitch_enabled"),
        [`ControlSurface.yaw_enabled`](./parts.md#SpaceCenter.ControlSurface.yaw_enabled "SpaceCenter.ControlSurface.yaw_enabled") and
        [`ControlSurface.roll_enabled`](./parts.md#SpaceCenter.ControlSurface.roll_enabled "SpaceCenter.ControlSurface.roll_enabled").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    rcs
    :   The state of RCS.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    reaction\_wheels
    :   Returns whether all reactive wheels on the vessel are active,
        and sets the active state of all reaction wheels.
        See [`ReactionWheel.active`](./parts.md#SpaceCenter.ReactionWheel.active "SpaceCenter.ReactionWheel.active").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    gear
    :   The state of the landing gear/legs.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    legs
    :   Returns whether all landing legs on the vessel are deployed,
        and sets the deployment state of all landing legs.
        Does not include wheels (for example landing gear).
        See [`Leg.deployed`](./parts.md#SpaceCenter.Leg.deployed "SpaceCenter.Leg.deployed").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    wheels
    :   Returns whether all wheels on the vessel are deployed,
        and sets the deployment state of all wheels.
        Does not include landing legs.
        See [`Wheel.deployed`](./parts.md#SpaceCenter.Wheel.deployed "SpaceCenter.Wheel.deployed").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    lights
    :   The state of the lights.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    brakes
    :   The state of the wheel brakes.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    antennas
    :   Returns whether all antennas on the vessel are deployed,
        and sets the deployment state of all antennas.
        See [`Antenna.deployed`](./parts.md#SpaceCenter.Antenna.deployed "SpaceCenter.Antenna.deployed").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    cargo\_bays
    :   Returns whether any of the cargo bays on the vessel are open,
        and sets the open state of all cargo bays.
        See [`CargoBay.open`](./parts.md#SpaceCenter.CargoBay.open "SpaceCenter.CargoBay.open").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    intakes
    :   Returns whether all of the air intakes on the vessel are open,
        and sets the open state of all air intakes.
        See [`Intake.open`](./parts.md#SpaceCenter.Intake.open "SpaceCenter.Intake.open").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    parachutes
    :   Returns whether all parachutes on the vessel are deployed,
        and sets the deployment state of all parachutes.
        Cannot be set to `False`.
        See [`Parachute.deployed`](./parts.md#SpaceCenter.Parachute.deployed "SpaceCenter.Parachute.deployed").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    radiators
    :   Returns whether all radiators on the vessel are deployed,
        and sets the deployment state of all radiators.
        See [`Radiator.deployed`](./parts.md#SpaceCenter.Radiator.deployed "SpaceCenter.Radiator.deployed").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    resource\_harvesters
    :   Returns whether all of the resource harvesters on the vessel are deployed,
        and sets the deployment state of all resource harvesters.
        See [`ResourceHarvester.deployed`](./parts.md#SpaceCenter.ResourceHarvester.deployed "SpaceCenter.ResourceHarvester.deployed").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    resource\_harvesters\_active
    :   Returns whether any of the resource harvesters on the vessel are active,
        and sets the active state of all resource harvesters.
        See [`ResourceHarvester.active`](./parts.md#SpaceCenter.ResourceHarvester.active "SpaceCenter.ResourceHarvester.active").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    solar\_panels
    :   Returns whether all solar panels on the vessel are deployed,
        and sets the deployment state of all solar panels.
        See [`SolarPanel.deployed`](./parts.md#SpaceCenter.SolarPanel.deployed "SpaceCenter.SolarPanel.deployed").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    abort
    :   The state of the abort action group.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    throttle
    :   The state of the throttle. A value between 0 and 1.
        Unlike the other control inputs, the throttle is not zeroed when the
        clients that set it disconnect.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The getter returns the input currently applied to the vessel, which
        > combines the input set by kRPC with any keyboard, SAS and trim input.
        > It refreshes one physics tick after a set, so a read immediately after
        > a set returns the previously applied value rather than the value just set.

    input\_mode
    :   Sets the behavior of the pitch, yaw, roll and translation control inputs.
        When set to additive, these inputs are added to the vessels current inputs.
        This mode is the default.
        When set to override, these inputs (if non-zero) override the vessels inputs.
        This mode prevents keyboard control, or SAS, from interfering with the controls when
        they are set.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`ControlInputMode`](#SpaceCenter.ControlInputMode "SpaceCenter.ControlInputMode")

        Game Scenes:
        :   Flight

    pitch
    :   The state of the pitch control.
        A value between -1 and 1.
        Equivalent to the w and s keys.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The getter returns the input currently applied to the vessel, which
        > combines the input set by kRPC with any keyboard, SAS and trim input.
        > It refreshes one physics tick after a set, so a read immediately after
        > a set returns the previously applied value rather than the value just set.

    yaw
    :   The state of the yaw control.
        A value between -1 and 1.
        Equivalent to the a and d keys.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The getter returns the input currently applied to the vessel, which
        > combines the input set by kRPC with any keyboard, SAS and trim input.
        > It refreshes one physics tick after a set, so a read immediately after
        > a set returns the previously applied value rather than the value just set.

    roll
    :   The state of the roll control.
        A value between -1 and 1.
        Equivalent to the q and e keys.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The getter returns the input currently applied to the vessel, which
        > combines the input set by kRPC with any keyboard, SAS and trim input.
        > It refreshes one physics tick after a set, so a read immediately after
        > a set returns the previously applied value rather than the value just set.

    pitch\_trim
    :   The state of the pitch trim.
        A value between -1 and 1.
        Equivalent to the Alt+W and Alt+S keys.
        This is a persistent trim that remains latched until changed or reset,
        and can only be accessed for the active vessel.
        Unlike the pitch, yaw and roll control inputs, the trim is not cleared
        when clients disconnect.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    yaw\_trim
    :   The state of the yaw trim.
        A value between -1 and 1.
        Equivalent to the Alt+A and Alt+D keys.
        This is a persistent trim that remains latched until changed or reset,
        and can only be accessed for the active vessel.
        Unlike the pitch, yaw and roll control inputs, the trim is not cleared
        when clients disconnect.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    roll\_trim
    :   The state of the roll trim.
        A value between -1 and 1.
        Equivalent to the Alt+Q and Alt+E keys.
        This is a persistent trim that remains latched until changed or reset,
        and can only be accessed for the active vessel.
        Unlike the pitch, yaw and roll control inputs, the trim is not cleared
        when clients disconnect.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    forward
    :   The state of the forward translational control.
        A value between -1 and 1.
        Equivalent to the h and n keys.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The getter returns the input currently applied to the vessel, which
        > combines the input set by kRPC with any keyboard, SAS and trim input.
        > It refreshes one physics tick after a set, so a read immediately after
        > a set returns the previously applied value rather than the value just set.

    up
    :   The state of the up translational control.
        A value between -1 and 1.
        Equivalent to the i and k keys.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The getter returns the input currently applied to the vessel, which
        > combines the input set by kRPC with any keyboard, SAS and trim input.
        > It refreshes one physics tick after a set, so a read immediately after
        > a set returns the previously applied value rather than the value just set.

    right
    :   The state of the right translational control.
        A value between -1 and 1.
        Equivalent to the j and l keys.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The getter returns the input currently applied to the vessel, which
        > combines the input set by kRPC with any keyboard, SAS and trim input.
        > It refreshes one physics tick after a set, so a read immediately after
        > a set returns the previously applied value rather than the value just set.

    wheel\_throttle
    :   The state of the wheel throttle.
        A value between -1 and 1.
        A value of 1 rotates the wheels forwards, a value of -1 rotates
        the wheels backwards.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The getter returns the input currently applied to the vessel, which
        > combines the input set by kRPC with any keyboard, SAS and trim input.
        > It refreshes one physics tick after a set, so a read immediately after
        > a set returns the previously applied value rather than the value just set.

    wheel\_steering
    :   The state of the wheel steering.
        A value between -1 and 1.
        A value of 1 steers to the left, and a value of -1 steers to the right.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The getter returns the input currently applied to the vessel, which
        > combines the input set by kRPC with any keyboard, SAS and trim input.
        > It refreshes one physics tick after a set, so a read immediately after
        > a set returns the previously applied value rather than the value just set.

    custom\_axis01
    :   The state of CustomAxis01.
        A value between -1 and 1.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    custom\_axis02
    :   The state of CustomAxis02.
        A value between -1 and 1.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    custom\_axis03
    :   The state of CustomAxis03.
        A value between -1 and 1.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    custom\_axis04
    :   The state of CustomAxis04.
        A value between -1 and 1.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    current\_stage
    :   The current stage of the vessel. Corresponds to the stage number in
        the in-game UI.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   int

        Game Scenes:
        :   Flight

    activate\_next\_stage()
    :   Activates the next stage. Equivalent to pressing the space bar in-game.

        Returns:
        :   A list of vessel objects that are jettisoned from the active vessel.

        Return type:
        :   list([`Vessel`](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel"))

        Game Scenes:
        :   Flight

        > **Note**
        >
        > When called, the active vessel may change. It is therefore possible that,
        > after calling this function, the object(s) returned by previous call(s) to
        > [`active_vessel`](./space-center.md#SpaceCenter.active_vessel "SpaceCenter.active_vessel") no longer refer to the active vessel.
        > Throws an exception if staging is locked.

    stage\_lock
    :   Whether staging is locked on the vessel.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

        > **Note**
        >
        > This is equivalent to locking the staging using Alt+L

    get\_action\_group(*group*)
    :   Returns `True` if the given action group is enabled.

        Parameters:
        :   **group** (*int*) – A number between 0 and 9 inclusive, or between 0 and 250 inclusive when the [Extended Action Groups mod](https://forum.kerbalspaceprogram.com/index.php?/topic/67235-122dec1016-action-groups-extended-250-action-groups-in-flight-editing-now-kosremotetech/) is installed.

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    set\_action\_group(*group*, *state*)
    :   Sets the state of the given action group.

        Parameters:
        :   - **group** (*int*) –

              A number between 0 and 9 inclusive, or between 0 and 250 inclusive when the [Extended Action Groups mod](https://forum.kerbalspaceprogram.com/index.php?/topic/67235-122dec1016-action-groups-extended-250-action-groups-in-flight-editing-now-kosremotetech/) is installed.
            - **state** (*bool*)

        Game Scenes:
        :   Flight

    toggle\_action\_group(*group*)
    :   Toggles the state of the given action group.

        Parameters:
        :   **group** (*int*) –

            A number between 0 and 9 inclusive, or between 0 and 250 inclusive when the [Extended Action Groups mod](https://forum.kerbalspaceprogram.com/index.php?/topic/67235-122dec1016-action-groups-extended-250-action-groups-in-flight-editing-now-kosremotetech/) is installed.

        Game Scenes:
        :   Flight

    get\_action\_group\_actions(*group*)
    :   Returns a list of all the part actions that are assigned to the given action group.
        Each entry identifies the part, the part module, and the action’s name and identifier.
        Returns an empty list if no actions are assigned to the group.

        Parameters:
        :   **group** (*int*) –

            A number between 0 and 9 inclusive, or between 0 and 250 inclusive when the [Extended Action Groups mod](https://forum.kerbalspaceprogram.com/index.php?/topic/67235-122dec1016-action-groups-extended-250-action-groups-in-flight-editing-now-kosremotetech/) is installed.

        Return type:
        :   list([`ActionGroupAction`](#SpaceCenter.ActionGroupAction "SpaceCenter.ActionGroupAction"))

        Game Scenes:
        :   Flight

        > **Note**
        >
        > For stock action groups, the assignments are read from each part action directly.
        > When the Extended Action Groups mod is installed, the assignments are queried from
        > the mod instead, so that actions assigned to its additional groups are included.

    add\_node(*ut*[, *prograde=0.0*][, *normal=0.0*][, *radial=0.0*])
    :   Creates a maneuver node at the given universal time, and returns a
        [`Node`](./node.md#SpaceCenter.Node "SpaceCenter.Node") object that can be used to modify it.
        Optionally sets the magnitude of the delta-v for the maneuver node
        in the prograde, normal and radial directions.

        Parameters:
        :   - **ut** (*float*) – Universal time of the maneuver node.
            - **prograde** (*float*) – Delta-v in the prograde direction.
            - **normal** (*float*) – Delta-v in the normal direction.
            - **radial** (*float*) – Delta-v in the radial direction.

        Return type:
        :   [`Node`](./node.md#SpaceCenter.Node "SpaceCenter.Node")

        Game Scenes:
        :   Flight

    nodes
    :   Returns a list of all existing maneuver nodes, ordered by time from first to last.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Node`](./node.md#SpaceCenter.Node "SpaceCenter.Node"))

        Game Scenes:
        :   Flight

    remove\_nodes()
    :   Remove all maneuver nodes.

        Game Scenes:
        :   Flight

class ActionGroupAction
:   An action, belonging to a part module, that is assigned to an action group.
    Obtained by calling [`Control.get_action_group_actions()`](#SpaceCenter.Control.get_action_group_actions "SpaceCenter.Control.get_action_group_actions").

    part
    :   The part that the action acts on.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](./parts.md#SpaceCenter.Part "SpaceCenter.Part")

    module
    :   The part module that the action belongs to. Returns `None` for a
        part-level action that is not associated with a module. This only occurs when
        the Extended Action Groups mod is installed, as it can assign actions defined
        directly on a part, rather than on one of its modules, to an action group.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Module`](./parts.md#SpaceCenter.Module "SpaceCenter.Module")

    name
    :   The human-readable name of the action, as shown in the action group editor.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    id
    :   The non-localized identifier for the action.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

class ControlState
:   The control state of a vessel.
    See [`Control.state`](#SpaceCenter.Control.state "SpaceCenter.Control.state").

    full
    :   Full controllable.

    partial
    :   Partially controllable.

    none
    :   Not controllable.

class ControlSource
:   The control source of a vessel.
    See [`Control.source`](#SpaceCenter.Control.source "SpaceCenter.Control.source").

    kerbal
    :   Vessel is controlled by a Kerbal.

    probe
    :   Vessel is controlled by a probe core.

    none
    :   Vessel is not controlled.

class SASMode
:   The behavior of the SAS auto-pilot. See [`AutoPilot.sas_mode`](./auto-pilot.md#SpaceCenter.AutoPilot.sas_mode "SpaceCenter.AutoPilot.sas_mode").

    stability\_assist
    :   Stability assist mode. Dampen out any rotation.

    maneuver
    :   Point in the burn direction of the next maneuver node.

    prograde
    :   Point in the prograde direction.

    retrograde
    :   Point in the retrograde direction.

    normal
    :   Point in the orbit normal direction.

    anti\_normal
    :   Point in the orbit anti-normal direction.

    radial
    :   Point in the orbit radial direction.

    anti\_radial
    :   Point in the orbit anti-radial direction.

    target
    :   Point in the direction of the current target.

    anti\_target
    :   Point away from the current target.

class SpeedMode
:   The mode of the speed reported in the navball.
    See [`Control.speed_mode`](#SpaceCenter.Control.speed_mode "SpaceCenter.Control.speed_mode").

    orbit
    :   Speed is relative to the vessel’s orbit.

    surface
    :   Speed is relative to the surface of the body being orbited.

    target
    :   Speed is relative to the current target.

class ControlInputMode
:   See [`Control.input_mode`](#SpaceCenter.Control.input_mode "SpaceCenter.Control.input_mode").

    additive
    :   Control inputs are added to the vessels current control inputs.

    override
    :   Control inputs (when they are non-zero) override the vessels current control inputs.
