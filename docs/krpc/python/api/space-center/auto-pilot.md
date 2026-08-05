# AutoPilot

class AutoPilot
:   Provides basic auto-piloting utilities for a vessel.
    Created by calling [`Vessel.auto_pilot`](./vessel.md#SpaceCenter.Vessel.auto_pilot "SpaceCenter.Vessel.auto_pilot").

    > **Note**
    >
    > If a client engages the auto-pilot and then closes its connection to the server,
    > the auto-pilot will be disengaged. Its configuration and target are left unchanged.

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
        > Equivalent to [`Control.sas`](./control.md#SpaceCenter.Control.sas "SpaceCenter.Control.sas").
        > Throws an exception if set to `True` while the auto-pilot is engaged, as the
        > auto-pilot holds SAS off for as long as it is flying the vessel.

    sas\_mode
    :   The current [`AutoPilot.sas_mode`](#SpaceCenter.AutoPilot.sas_mode "SpaceCenter.AutoPilot.sas_mode").
        These modes are equivalent to the mode buttons to the left of the navball that appear
        when SAS is enabled.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`SASMode`](./control.md#SpaceCenter.SASMode "SpaceCenter.SASMode")

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Equivalent to [`Control.sas_mode`](./control.md#SpaceCenter.Control.sas_mode "SpaceCenter.Control.sas_mode")

    engaged
    :   Whether the auto-pilot is engaged.
        Setting to `True` engages the auto-pilot; setting to `False` disengages it.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    show\_info\_ui
    :   Whether an in-game window showing the auto-pilot’s state (engagement, attitude error,
        target, angular rate, inner-loop PID gains and oscillation suppression) is displayed for
        this vessel. Defaults to `False`. This is a debugging aid; the window is reset to
        hidden when the game is restarted.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    reset()
    :   Disengages the auto-pilot and resets all configuration parameters to their defaults.
        Also resets the target pitch, heading and roll, and clears all internal controller
        state, including the oscillation detector’s structural level — which otherwise
        persists across engagements so that a craft known to be flexible re-latches quickly.

        Game Scenes:
        :   Flight

    reference\_frame
    :   The reference frame for the target direction ([`AutoPilot.target_direction`](#SpaceCenter.AutoPilot.target_direction "SpaceCenter.AutoPilot.target_direction")).

        Attribute:
        :   Can be read or written

        Return type:
        :   [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Game Scenes:
        :   Flight

        > **Note**
        >
        > An error will be thrown if this property is set to a reference frame that rotates with
        > the vessel being controlled, as it is impossible to rotate the vessel in such a
        > reference frame.

    target\_pitch
    :   The target pitch, in degrees, between -90° and +90°.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > A convenience for aiming the nose by angle. Heading (and hence roll) is ill-defined when
        > the nose is near vertical (pitch → ±90°); near the vertical prefer
        > [`AutoPilot.target_direction`](#SpaceCenter.AutoPilot.target_direction "SpaceCenter.AutoPilot.target_direction") or [`AutoPilot.set_direction_and_up()`](#SpaceCenter.AutoPilot.set_direction_and_up "SpaceCenter.AutoPilot.set_direction_and_up"). The setter preserves the
        > current roll relative to [`AutoPilot.up_reference`](#SpaceCenter.AutoPilot.up_reference "SpaceCenter.AutoPilot.up_reference").

    target\_heading
    :   The target heading, in degrees, between 0° and 360°.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > A convenience for aiming the nose by angle, ill-defined when the nose is near vertical
        > (pitch → ±90°) — see [`AutoPilot.target_pitch`](#SpaceCenter.AutoPilot.target_pitch "SpaceCenter.AutoPilot.target_pitch"). The setter preserves the current roll
        > relative to [`AutoPilot.up_reference`](#SpaceCenter.AutoPilot.up_reference "SpaceCenter.AutoPilot.up_reference").

    target\_roll
    :   The target roll, in degrees, measured about the vessel’s nose relative to the
        [`AutoPilot.up_reference`](#SpaceCenter.AutoPilot.up_reference "SpaceCenter.AutoPilot.up_reference") (roll 0 aligns the vessel’s dorsal/roof axis with the reference;
        positive roll banks right). `NaN` if no target roll is set.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > When left unset (`NaN`) the auto-pilot suppresses roll rotation — it drives the roll
        > rate to zero rather than holding a specific roll angle. Setting a value re-rolls the
        > current target to that angle relative to the up reference while keeping the nose direction.
        > With the default reference (the frame’s up) this reproduces the historical roll away from
        > the vertical, and is ill-defined only when the nose points along the reference (near
        > straight up or down). To hold a well-defined roll through the vertical — for example a
        > gravity turn — set the up reference off the flight path (see
        > [`AutoPilot.set_direction_and_up()`](#SpaceCenter.AutoPilot.set_direction_and_up "SpaceCenter.AutoPilot.set_direction_and_up") / [`AutoPilot.up_reference`](#SpaceCenter.AutoPilot.up_reference "SpaceCenter.AutoPilot.up_reference")).

    up\_reference
    :   The reference direction, in the reference frame specified by [`AutoPilot.reference_frame`](#SpaceCenter.AutoPilot.reference_frame "SpaceCenter.AutoPilot.reference_frame"),
        that [`AutoPilot.target_roll`](#SpaceCenter.AutoPilot.target_roll "SpaceCenter.AutoPilot.target_roll") is measured against: at roll 0 the vessel’s dorsal (roof)
        axis is aligned with this vector’s component perpendicular to the nose. Defaults to the
        frame’s up (the zenith / radial-out direction).

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Setting this re-anchors how roll is measured without moving the current target, so
        > the reference can be set once and then rolls commanded against it with
        > [`AutoPilot.target_roll`](#SpaceCenter.AutoPilot.target_roll "SpaceCenter.AutoPilot.target_roll") while the nose direction changes freely. It is also set as a side
        > effect of [`AutoPilot.set_direction_and_up()`](#SpaceCenter.AutoPilot.set_direction_and_up "SpaceCenter.AutoPilot.set_direction_and_up"). Setting the target rotation, target direction,
        > or the scalar pitch/heading leaves it unchanged. Choosing a reference off the flight path
        > keeps roll well-defined through the vertical.

    target\_pitch\_and\_heading(*pitch*, *heading*)
    :   Set target pitch and heading angles.

        Parameters:
        :   - **pitch** (*float*) – Target pitch angle, in degrees between -90° and +90°.
            - **heading** (*float*) – Target heading angle, in degrees between 0° and 360°.

        Game Scenes:
        :   Flight

        > **Note**
        >
        > A convenience for aiming the nose by angle; heading is ill-defined when the nose is near
        > vertical (pitch → ±90°), so near the vertical prefer [`AutoPilot.target_direction`](#SpaceCenter.AutoPilot.target_direction "SpaceCenter.AutoPilot.target_direction") or
        > [`AutoPilot.set_direction_and_up()`](#SpaceCenter.AutoPilot.set_direction_and_up "SpaceCenter.AutoPilot.set_direction_and_up"). Preserves the current roll relative to
        > [`AutoPilot.up_reference`](#SpaceCenter.AutoPilot.up_reference "SpaceCenter.AutoPilot.up_reference").

    target\_direction
    :   Direction vector corresponding to the target pitch and heading.
        This is in the reference frame specified by [`AutoPilot.reference_frame`](#SpaceCenter.AutoPilot.reference_frame "SpaceCenter.AutoPilot.reference_frame").

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    target\_rotation
    :   The target rotation quaternion. Setting this also sets the target roll.
        This is in the reference frame specified by [`AutoPilot.reference_frame`](#SpaceCenter.AutoPilot.reference_frame "SpaceCenter.AutoPilot.reference_frame").

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float, float)

        Game Scenes:
        :   Flight

    set\_direction\_and\_up(*direction*, *up*[, *roll=0.0*])
    :   Set the target attitude from a nose direction and an up vector: point the nose along
        *direction* and roll so the vessel’s dorsal (roof) axis aligns with
        *up* (its component perpendicular to the nose), then apply an optional
        *roll* offset about the nose. Both vectors are in the reference frame
        specified by [`AutoPilot.reference_frame`](#SpaceCenter.AutoPilot.reference_frame "SpaceCenter.AutoPilot.reference_frame").

        Parameters:
        :   - **direction** (*tuple*) – The direction to point the nose in.
            - **up** (*tuple*) – The reference direction the roof is rolled towards. Need not be normalized or perpendicular to *direction* — its component perpendicular to the nose is used. Stored as the [`AutoPilot.up_reference`](#SpaceCenter.AutoPilot.up_reference "SpaceCenter.AutoPilot.up_reference").
            - **roll** (*float*) – An additional roll about the nose, in degrees (positive banks right). Defaults to 0.

        Game Scenes:
        :   Flight

        > **Note**
        >
        > This is the way to hold a well-defined orientation through a maneuver — for example a
        > gravity turn: pass a fixed *up* (say north) and the roll stays defined the
        > whole way, with no singularity at the vertical. It is well-defined for every nose direction
        > except *up* parallel to *direction* (asking the roof to
        > point where the nose already points), where it falls back to pointing the nose only.
        > Equivalent to setting [`AutoPilot.up_reference`](#SpaceCenter.AutoPilot.up_reference "SpaceCenter.AutoPilot.up_reference") to *up*, aiming at
        > *direction* and setting [`AutoPilot.target_roll`](#SpaceCenter.AutoPilot.target_roll "SpaceCenter.AutoPilot.target_roll") to
        > *roll*.

    current\_target\_pitch
    :   The current target pitch the auto-pilot is tracking, in degrees. When
        [`AutoPilot.target_smoothing_time`](#SpaceCenter.AutoPilot.target_smoothing_time "SpaceCenter.AutoPilot.target_smoothing_time") is non-zero this lags the commanded
        [`AutoPilot.target_pitch`](#SpaceCenter.AutoPilot.target_pitch "SpaceCenter.AutoPilot.target_pitch") while a change is slewed in; otherwise the two are equal.
        A convenience scalar, ill-defined near the vertical — see [`AutoPilot.target_pitch`](#SpaceCenter.AutoPilot.target_pitch "SpaceCenter.AutoPilot.target_pitch").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    current\_target\_heading
    :   The current target heading the auto-pilot is tracking, in degrees. When
        [`AutoPilot.target_smoothing_time`](#SpaceCenter.AutoPilot.target_smoothing_time "SpaceCenter.AutoPilot.target_smoothing_time") is non-zero this lags the commanded
        [`AutoPilot.target_heading`](#SpaceCenter.AutoPilot.target_heading "SpaceCenter.AutoPilot.target_heading") while a change is slewed in; otherwise the two are equal.
        A convenience scalar, ill-defined near the vertical — see [`AutoPilot.target_heading`](#SpaceCenter.AutoPilot.target_heading "SpaceCenter.AutoPilot.target_heading").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    current\_target\_roll
    :   The current target roll the auto-pilot is tracking, in degrees. When
        [`AutoPilot.target_smoothing_time`](#SpaceCenter.AutoPilot.target_smoothing_time "SpaceCenter.AutoPilot.target_smoothing_time") is non-zero this lags the commanded
        [`AutoPilot.target_roll`](#SpaceCenter.AutoPilot.target_roll "SpaceCenter.AutoPilot.target_roll") while a change is slewed in; otherwise the two are equal.
        `NaN` if no target roll is set.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    current\_target\_direction
    :   Direction vector corresponding to the current target pitch and heading
        (see [`AutoPilot.current_target_pitch`](#SpaceCenter.AutoPilot.current_target_pitch "SpaceCenter.AutoPilot.current_target_pitch")), in the reference frame specified by
        [`AutoPilot.reference_frame`](#SpaceCenter.AutoPilot.reference_frame "SpaceCenter.AutoPilot.reference_frame"). Lags [`AutoPilot.target_direction`](#SpaceCenter.AutoPilot.target_direction "SpaceCenter.AutoPilot.target_direction") while a change is
        slewed in when [`AutoPilot.target_smoothing_time`](#SpaceCenter.AutoPilot.target_smoothing_time "SpaceCenter.AutoPilot.target_smoothing_time") is non-zero.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    current\_target\_rotation
    :   The current target rotation quaternion the auto-pilot is tracking, in the reference frame
        specified by [`AutoPilot.reference_frame`](#SpaceCenter.AutoPilot.reference_frame "SpaceCenter.AutoPilot.reference_frame"). Lags [`AutoPilot.target_rotation`](#SpaceCenter.AutoPilot.target_rotation "SpaceCenter.AutoPilot.target_rotation") while a
        change is slewed in when [`AutoPilot.target_smoothing_time`](#SpaceCenter.AutoPilot.target_smoothing_time "SpaceCenter.AutoPilot.target_smoothing_time") is non-zero.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   tuple(float, float, float, float)

        Game Scenes:
        :   Flight

    wait([*timeout=-1.0*])
    :   Blocks until the vessel is pointing in the target direction and has
        the target roll (if set). Throws an exception if the auto-pilot has not been engaged.

        Parameters:
        :   **timeout** (*float*) – Maximum time to wait in seconds. If not specified, waits indefinitely.

        Game Scenes:
        :   Flight

    stopping\_angle\_threshold
    :   The threshold, in degrees, below which the pointing error must fall for
        [`AutoPilot.wait()`](#SpaceCenter.AutoPilot.wait "SpaceCenter.AutoPilot.wait") to return. Defaults to 1 degree.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    stopping\_velocity\_threshold
    :   The threshold angular velocity, in rad/s, below which the vessel’s angular
        velocity magnitude must fall for [`AutoPilot.wait()`](#SpaceCenter.AutoPilot.wait "SpaceCenter.AutoPilot.wait") to return.
        Defaults to 0.05 rad/s.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    error
    :   The error, in degrees, between the direction the ship has been asked
        to point in and the direction it is pointing in. Throws an exception if the auto-pilot
        has not been engaged and SAS is not enabled or is in stability assist mode.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > This is the error relative to the commanded target. While a change is being slewed in
        > (see [`AutoPilot.target_smoothing_time`](#SpaceCenter.AutoPilot.target_smoothing_time "SpaceCenter.AutoPilot.target_smoothing_time")) it differs from [`AutoPilot.current_error`](#SpaceCenter.AutoPilot.current_error "SpaceCenter.AutoPilot.current_error"), the
        > error relative to the target the auto-pilot is currently tracking.

    attitude\_error
    :   The per-axis attitude error (pitch, yaw, roll), in degrees, between the vessel’s current
        attitude and the commanded target. All three components come from one singularity-free
        residual decomposition, so they stay well-defined near the vertical (unlike a subtraction
        of pitch/heading/roll angles). The scalar [`AutoPilot.pitch_error`](#SpaceCenter.AutoPilot.pitch_error "SpaceCenter.AutoPilot.pitch_error"),
        [`AutoPilot.heading_error`](#SpaceCenter.AutoPilot.heading_error "SpaceCenter.AutoPilot.heading_error") and [`AutoPilot.roll_error`](#SpaceCenter.AutoPilot.roll_error "SpaceCenter.AutoPilot.roll_error") are the magnitudes of the pitch, yaw
        and roll components respectively. Throws an exception if the auto-pilot has not been
        engaged.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    pitch\_error
    :   The error, in degrees, between the vessels current and target pitch.
        Throws an exception if the auto-pilot has not been engaged.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The pitch component of [`AutoPilot.attitude_error`](#SpaceCenter.AutoPilot.attitude_error "SpaceCenter.AutoPilot.attitude_error") — the pitch part of the direction error
        > resolved in the roll-invariant frame, well-defined near the vertical.

    heading\_error
    :   The error, in degrees, between the vessels current and target heading.
        Throws an exception if the auto-pilot has not been engaged.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The yaw component of [`AutoPilot.attitude_error`](#SpaceCenter.AutoPilot.attitude_error "SpaceCenter.AutoPilot.attitude_error") — the yaw part of the direction error
        > resolved in the roll-invariant frame, well-defined near the vertical (unlike the absolute
        > heading, which is undefined at the pole).

    roll\_error
    :   The error, in degrees, between the vessels current and target roll.
        Throws an exception if the auto-pilot has not been engaged or no target roll is set.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Measured about the vessel’s nose axis, so it stays well-defined near the vertical
        > singularity — unlike a subtraction of pitch/heading/roll angles, whose roll term is
        > ill-conditioned when the vessel points close to straight up or down.

    current\_error
    :   The error, in degrees, between the direction the auto-pilot is currently tracking and the
        direction the ship is pointing in. Unlike [`AutoPilot.error`](#SpaceCenter.AutoPilot.error "SpaceCenter.AutoPilot.error") (which is relative to the
        commanded target), this is relative to the slewed target the auto-pilot is currently
        holding, so it stays small while a smoothed change (see [`AutoPilot.target_smoothing_time`](#SpaceCenter.AutoPilot.target_smoothing_time "SpaceCenter.AutoPilot.target_smoothing_time"))
        is fed in. Equal to [`AutoPilot.error`](#SpaceCenter.AutoPilot.error "SpaceCenter.AutoPilot.error") when smoothing is off. Throws an exception if the
        auto-pilot has not been engaged.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    current\_attitude\_error
    :   The per-axis attitude error (pitch, yaw, roll), in degrees, between the vessel’s current
        attitude and the target the auto-pilot is currently tracking (the slewed target — see
        [`AutoPilot.current_target_rotation`](#SpaceCenter.AutoPilot.current_target_rotation "SpaceCenter.AutoPilot.current_target_rotation")). Like [`AutoPilot.attitude_error`](#SpaceCenter.AutoPilot.attitude_error "SpaceCenter.AutoPilot.attitude_error") but relative to the
        current target, so it stays small while a smoothed change (see
        [`AutoPilot.target_smoothing_time`](#SpaceCenter.AutoPilot.target_smoothing_time "SpaceCenter.AutoPilot.target_smoothing_time")) is fed in; equal to [`AutoPilot.attitude_error`](#SpaceCenter.AutoPilot.attitude_error "SpaceCenter.AutoPilot.attitude_error") when
        smoothing is off. Throws an exception if the auto-pilot has not been engaged.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    current\_pitch\_error
    :   The error, in degrees, between the vessels current pitch and the pitch the auto-pilot is
        currently tracking (see [`AutoPilot.current_target_pitch`](#SpaceCenter.AutoPilot.current_target_pitch "SpaceCenter.AutoPilot.current_target_pitch")). Throws an exception if the
        auto-pilot has not been engaged.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The pitch component of [`AutoPilot.current_attitude_error`](#SpaceCenter.AutoPilot.current_attitude_error "SpaceCenter.AutoPilot.current_attitude_error"), well-defined near the vertical.

    current\_heading\_error
    :   The error, in degrees, between the vessels current heading and the heading the auto-pilot
        is currently tracking (see [`AutoPilot.current_target_heading`](#SpaceCenter.AutoPilot.current_target_heading "SpaceCenter.AutoPilot.current_target_heading")). Throws an exception if the
        auto-pilot has not been engaged.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The yaw component of [`AutoPilot.current_attitude_error`](#SpaceCenter.AutoPilot.current_attitude_error "SpaceCenter.AutoPilot.current_attitude_error"), well-defined near the vertical.

    current\_roll\_error
    :   The error, in degrees, between the vessels current roll and the roll the auto-pilot is
        currently tracking (see [`AutoPilot.current_target_roll`](#SpaceCenter.AutoPilot.current_target_roll "SpaceCenter.AutoPilot.current_target_roll")). Throws an exception if the
        auto-pilot has not been engaged or no target roll is set.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Measured about the vessel’s nose axis, so it stays well-defined near the vertical
        > singularity — see [`AutoPilot.roll_error`](#SpaceCenter.AutoPilot.roll_error "SpaceCenter.AutoPilot.roll_error").

    roll\_start\_angle
    :   The direction error, in degrees, above which roll blending is fully suppressed.
        Defaults to 20 degrees.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    roll\_engage\_angle
    :   The direction error, in degrees, below which roll is fully engaged.
        Roll blends linearly between [`AutoPilot.roll_start_angle`](#SpaceCenter.AutoPilot.roll_start_angle "SpaceCenter.AutoPilot.roll_start_angle") and this value.
        Defaults to 15 degrees.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_angular\_velocity
    :   The maximum angular velocity of the vessel, in rad/s, for each of the pitch, roll
        and yaw axes. Limits the target angular velocity computed by the bang-bang profile so
        that vessels with very high torque availability do not spin faster than desired.
        Defaults to 1 rad/s for each axis.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    pitch\_yaw\_attenuation\_angle
    :   The angle, in degrees, at which the autopilot considers the vessel to be pointing close
        to the target direction. This sets the high angle of the pitch/yaw pointing deadband: at
        or above this error the target velocity is at full, and below it the target velocity
        ramps linearly to zero at half this angle, so the vessel coasts to a stop. Pitch and yaw
        are controlled jointly, so a single angle applies to both. Defaults to 1°.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    roll\_attenuation\_angle
    :   The angle, in degrees, at which the autopilot considers the vessel to be pointing close
        to the target roll. This sets the high angle of the roll-axis pointing deadband: at or
        above this error the target velocity is at full, and below it the target velocity ramps
        linearly to zero at half this angle, so the roll coasts to a stop. Defaults to 1°.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    auto\_tune
    :   Whether the rotation rate controllers PID parameters should be automatically tuned
        using the vessels moment of inertia and available torque. Defaults to `True`.
        See [`AutoPilot.time_to_peak`](#SpaceCenter.AutoPilot.time_to_peak "SpaceCenter.AutoPilot.time_to_peak") and [`AutoPilot.overshoot`](#SpaceCenter.AutoPilot.overshoot "SpaceCenter.AutoPilot.overshoot").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    time\_to\_peak
    :   The target time to peak used to autotune the PID controllers.
        A vector of three times, in seconds, for each of the pitch, roll and yaw axes.
        Defaults to 1 second for each axis.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    soft\_start\_time
    :   The duration, in seconds, over which the control output is faded in when the
        autopilot is engaged. This soft-start spreads the engagement transient over many
        physics ticks so engaging (on the pad or mid-flight) does not command a near-maximum
        control deflection that can excite an oscillation. Defaults to 0.5 seconds.
        Set to 0 to disable the fade-in.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    target\_smoothing\_time
    :   The duration, in seconds, over which a change to the target attitude is applied to the
        control target. When set above zero, changing the target pitch, heading, roll, direction
        or rotation makes the effective control target ramp smoothly (a constant-rate rotation)
        from its current value to the new value over this many seconds, rather than jumping
        instantly. This lets a slow control loop drive a smooth maneuver (for example a gravity
        turn) without inducing oscillation from stepwise target changes. Defaults to 0
        (instantaneous).

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    overshoot
    :   The target overshoot percentage used to autotune the PID controllers.
        A vector of three values, between 0 and 1, for each of the pitch, roll and yaw axes.
        Defaults to 0.01 for each axis.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    pitch\_pid\_gains
    :   Gains for the pitch PID controller.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > When [`AutoPilot.auto_tune`](#SpaceCenter.AutoPilot.auto_tune "SpaceCenter.AutoPilot.auto_tune") is true, these values are updated automatically,
        > which will overwrite any manual changes.

    roll\_pid\_gains
    :   Gains for the roll PID controller.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > When [`AutoPilot.auto_tune`](#SpaceCenter.AutoPilot.auto_tune "SpaceCenter.AutoPilot.auto_tune") is true, these values are updated automatically,
        > which will overwrite any manual changes.

    yaw\_pid\_gains
    :   Gains for the yaw PID controller.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > When [`AutoPilot.auto_tune`](#SpaceCenter.AutoPilot.auto_tune "SpaceCenter.AutoPilot.auto_tune") is true, these values are updated automatically,
        > which will overwrite any manual changes.

    pitch\_yaw\_rate\_filter\_mode
    :   Controls the rate-feedback filtering (the wobble-suppression filter on the measured
        angular velocity) for the pitch and yaw axes of a structurally flexible vessel. When
        [`RateFilterMode.automatic`](#SpaceCenter.RateFilterMode.automatic "SpaceCenter.RateFilterMode.automatic") (the default) the auto-pilot detects the
        oscillation at runtime, estimates its frequency and routes it to the appropriate tool
        (a notch filter for a low-frequency mode near the control band, a low-pass for a
        high-frequency mode). [`RateFilterMode.off`](#SpaceCenter.RateFilterMode.off "SpaceCenter.RateFilterMode.off") disables rate filtering only —
        the other oscillation mitigations are unaffected. [`RateFilterMode.notch`](#SpaceCenter.RateFilterMode.notch "SpaceCenter.RateFilterMode.notch")
        and [`RateFilterMode.low_pass`](#SpaceCenter.RateFilterMode.low_pass "SpaceCenter.RateFilterMode.low_pass") force the respective tool unconditionally at
        [`AutoPilot.pitch_yaw_oscillation_frequency`](#SpaceCenter.AutoPilot.pitch_yaw_oscillation_frequency "SpaceCenter.AutoPilot.pitch_yaw_oscillation_frequency"), for a vessel known in advance to be
        flexible.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`RateFilterMode`](#SpaceCenter.RateFilterMode "SpaceCenter.RateFilterMode")

        Game Scenes:
        :   Flight

    roll\_rate\_filter\_mode
    :   Controls the rate-feedback filtering for the roll axis. Behaves as
        [`AutoPilot.pitch_yaw_rate_filter_mode`](#SpaceCenter.AutoPilot.pitch_yaw_rate_filter_mode "SpaceCenter.AutoPilot.pitch_yaw_rate_filter_mode") but for roll, using
        [`AutoPilot.roll_oscillation_frequency`](#SpaceCenter.AutoPilot.roll_oscillation_frequency "SpaceCenter.AutoPilot.roll_oscillation_frequency"). Defaults to
        [`RateFilterMode.automatic`](#SpaceCenter.RateFilterMode.automatic "SpaceCenter.RateFilterMode.automatic").

        Attribute:
        :   Can be read or written

        Return type:
        :   [`RateFilterMode`](#SpaceCenter.RateFilterMode "SpaceCenter.RateFilterMode")

        Game Scenes:
        :   Flight

    pitch\_yaw\_oscillation\_frequency
    :   The structural mode frequency, in Hz, for the pitch/yaw axis group. Used directly as the
        filter frequency in [`RateFilterMode.notch`](#SpaceCenter.RateFilterMode.notch "SpaceCenter.RateFilterMode.notch") / [`RateFilterMode.low_pass`](#SpaceCenter.RateFilterMode.low_pass "SpaceCenter.RateFilterMode.low_pass")
        mode, and as the seed for the automatic frequency estimator before it acquires. Defaults
        to 1.5 Hz.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    roll\_oscillation\_frequency
    :   The structural mode frequency, in Hz, for the roll axis. Behaves as
        [`AutoPilot.pitch_yaw_oscillation_frequency`](#SpaceCenter.AutoPilot.pitch_yaw_oscillation_frequency "SpaceCenter.AutoPilot.pitch_yaw_oscillation_frequency") but for roll. Defaults to 1.5 Hz.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    oscillation\_notch\_q
    :   The quality factor of the notch filter used to suppress a low-frequency structural mode.
        A higher value gives a narrower notch (less in-band control lag but less tolerance to the
        mode frequency drifting); a lower value gives a wider notch. Defaults to 2.5. This is an
        advanced tuning parameter.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    oscillation\_bandwidth\_floor\_mode
    :   Controls the bandwidth-floor mitigation: the reduction of the inner control loop
        bandwidth on a structurally flexible axis — the primary oscillation stabilizer. When
        [`MitigationMode.automatic`](#SpaceCenter.MitigationMode.automatic "SpaceCenter.MitigationMode.automatic") (the default) it engages on a latched axis
        while holding (and during a detected limit cycle). [`MitigationMode.off`](#SpaceCenter.MitigationMode.off "SpaceCenter.MitigationMode.off")
        never reduces the bandwidth; [`MitigationMode.forced`](#SpaceCenter.MitigationMode.forced "SpaceCenter.MitigationMode.forced") keeps it fully
        reduced at all times.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`MitigationMode`](#SpaceCenter.MitigationMode "SpaceCenter.MitigationMode")

        Game Scenes:
        :   Flight

    oscillation\_bandwidth\_floor
    :   The inner control loop bandwidth, in rad/s, that an axis is reduced towards while the
        bandwidth-floor mitigation is engaged on it. Lowering it suppresses oscillation more
        strongly; raising it keeps more control authority at the cost of allowing more wobble.
        Defaults to 1 rad/s. This is an advanced tuning parameter.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    oscillation\_feedforward\_mode
    :   Controls the feedforward-cut mitigation: removal of the acceleration feedforward on a
        structurally flexible axis while holding, so it cannot re-excite a residual mode at
        the reduced bandwidth. When [`MitigationMode.automatic`](#SpaceCenter.MitigationMode.automatic "SpaceCenter.MitigationMode.automatic") (the default) it
        follows the hold gate on a latched axis. [`MitigationMode.off`](#SpaceCenter.MitigationMode.off "SpaceCenter.MitigationMode.off") never cuts
        the feedforward; [`MitigationMode.forced`](#SpaceCenter.MitigationMode.forced "SpaceCenter.MitigationMode.forced") always cuts it fully.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`MitigationMode`](#SpaceCenter.MitigationMode "SpaceCenter.MitigationMode")

        Game Scenes:
        :   Flight

    oscillation\_output\_filter\_mode
    :   Controls the output-smoothing mitigation: a low-pass on the delivered actuator
        command that caps residual control chatter. When
        [`MitigationMode.automatic`](#SpaceCenter.MitigationMode.automatic "SpaceCenter.MitigationMode.automatic") (the default) it engages on a latched axis
        (and, lightly, while the oscillation detector is firing on an unlatched one).
        [`MitigationMode.off`](#SpaceCenter.MitigationMode.off "SpaceCenter.MitigationMode.off") never smooths; [`MitigationMode.forced`](#SpaceCenter.MitigationMode.forced "SpaceCenter.MitigationMode.forced")
        smooths fully at all times.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`MitigationMode`](#SpaceCenter.MitigationMode "SpaceCenter.MitigationMode")

        Game Scenes:
        :   Flight

    pitch\_yaw\_control\_oscillation
    :   The current amplitude of control-output oscillation on the pitch/yaw axis group, measured as
        the deviation of the delivered control about its slowly-varying trim. A settled hold sits
        near zero; a sustained limit cycle drives it toward 1. Read-only.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    roll\_control\_oscillation
    :   The current amplitude of control-output oscillation on the roll axis, measured as the
        deviation of the delivered control about its slowly-varying trim. Read-only. See
        [`AutoPilot.pitch_yaw_control_oscillation`](#SpaceCenter.AutoPilot.pitch_yaw_control_oscillation "SpaceCenter.AutoPilot.pitch_yaw_control_oscillation").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    oscillation\_level
    :   A measure, between 0 and 1 for each of the pitch, roll and yaw axes, of how strongly the
        auto-pilot currently detects structural oscillation (wobble) on that axis. 0 means none
        detected; values approaching 1 mean a sustained structural oscillation. Read-only.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    pitch\_yaw\_oscillation\_latched
    :   Whether the auto-pilot has confirmed the pitch/yaw axes to be structurally flexible and
        latched oscillation suppression on for them. Read-only. See
        [`AutoPilot.pitch_yaw_rate_filter_mode`](#SpaceCenter.AutoPilot.pitch_yaw_rate_filter_mode "SpaceCenter.AutoPilot.pitch_yaw_rate_filter_mode").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    roll\_oscillation\_latched
    :   Whether the auto-pilot has confirmed the roll axis to be structurally flexible and latched
        oscillation suppression on for it. Read-only. See [`AutoPilot.roll_rate_filter_mode`](#SpaceCenter.AutoPilot.roll_rate_filter_mode "SpaceCenter.AutoPilot.roll_rate_filter_mode").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    pitch\_yaw\_oscillation\_detected\_frequency
    :   The structural oscillation frequency, in Hz, estimated by the automatic detector for the
        pitch/yaw axis group, or `NaN` until the estimator acquires. The estimator runs in
        all modes, so this is observable even when suppression is off or forced. Read-only.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    roll\_oscillation\_detected\_frequency
    :   The structural oscillation frequency, in Hz, estimated by the automatic detector for the
        roll axis, or `NaN` until the estimator acquires. Read-only. See
        [`AutoPilot.pitch_yaw_oscillation_detected_frequency`](#SpaceCenter.AutoPilot.pitch_yaw_oscillation_detected_frequency "SpaceCenter.AutoPilot.pitch_yaw_oscillation_detected_frequency").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    diagnostic\_logging
    :   When `True`, records one row of diagnostic data per physics tick to an
        in-memory buffer (see [`AutoPilot.diagnostic_log`](#SpaceCenter.AutoPilot.diagnostic_log "SpaceCenter.AutoPilot.diagnostic_log")), and echoes each row to
        Player.log prefixed with `[KRPC.AP]`. The data is CSV: the first row is a
        header naming every column, and each subsequent row records the auto-pilot’s full
        control-loop state for one tick (setpoints, errors, measured rates, gains,
        velocity-profile and feedforward internals, control outputs, and the oscillation
        detector/gate/mitigation state). The buffer is capped at 3000 data rows (one minute
        at the 50 Hz physics rate); when full, this property switches itself back to
        `False` and the buffer holds the minute following the enable. Setting to
        `True` clears the buffer. Defaults to `False`.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    diagnostic\_log
    :   The diagnostic log collected since [`AutoPilot.diagnostic_logging`](#SpaceCenter.AutoPilot.diagnostic_logging "SpaceCenter.AutoPilot.diagnostic_logging") was last set to
        `True`: CSV text whose first line is the column header and each subsequent
        line records one physics tick. Vector-valued channels use one column per component
        (suffixed `.p/.r/.y` for pitch, roll, yaw); pitch-yaw-group/roll channel pairs
        are suffixed `.py/.roll`. Returns an empty string if diagnostic logging has
        not been enabled or no ticks have occurred.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

        Game Scenes:
        :   Flight

class RateFilterMode
:   Controls the auto-pilot’s rate-feedback filtering for an axis group — the mitigation that
    removes a structural oscillation (wobble) from the measured angular velocity before the
    control loops consume it.
    See [`AutoPilot.pitch_yaw_rate_filter_mode`](#SpaceCenter.AutoPilot.pitch_yaw_rate_filter_mode "SpaceCenter.AutoPilot.pitch_yaw_rate_filter_mode") and [`AutoPilot.roll_rate_filter_mode`](#SpaceCenter.AutoPilot.roll_rate_filter_mode "SpaceCenter.AutoPilot.roll_rate_filter_mode").

    automatic
    :   The default. The auto-pilot detects structural oscillation at runtime, estimates its
        frequency and routes it to the appropriate filter: a notch for a low-frequency mode
        near the control band, a low-pass for a high-frequency mode, or a broadband low-pass
        while the frequency is not yet known. Rigid vessels are left untouched.

    off
    :   No rate filtering. The other oscillation mitigations are unaffected.

    notch
    :   Force a notch filter at the manually set frequency
        ([`AutoPilot.pitch_yaw_oscillation_frequency`](#SpaceCenter.AutoPilot.pitch_yaw_oscillation_frequency "SpaceCenter.AutoPilot.pitch_yaw_oscillation_frequency") /
        [`AutoPilot.roll_oscillation_frequency`](#SpaceCenter.AutoPilot.roll_oscillation_frequency "SpaceCenter.AutoPilot.roll_oscillation_frequency")), for a vessel whose structural mode
        is known in advance.

    low\_pass
    :   Force a low-pass filter derived from the manually set frequency.

class MitigationMode
:   Controls one of the auto-pilot’s individually-toggleable oscillation mitigations
    ([`AutoPilot.oscillation_bandwidth_floor_mode`](#SpaceCenter.AutoPilot.oscillation_bandwidth_floor_mode "SpaceCenter.AutoPilot.oscillation_bandwidth_floor_mode"),
    [`AutoPilot.oscillation_feedforward_mode`](#SpaceCenter.AutoPilot.oscillation_feedforward_mode "SpaceCenter.AutoPilot.oscillation_feedforward_mode"),
    [`AutoPilot.oscillation_output_filter_mode`](#SpaceCenter.AutoPilot.oscillation_output_filter_mode "SpaceCenter.AutoPilot.oscillation_output_filter_mode")).

    automatic
    :   The default: the mitigation engages automatically, driven by the runtime oscillation
        detector and the hold gate. Rigid vessels are left untouched.

    off
    :   The mitigation never engages. The other mitigations are unaffected.

    forced
    :   The mitigation is fully engaged at all times, regardless of what the oscillation
        detector reports.
