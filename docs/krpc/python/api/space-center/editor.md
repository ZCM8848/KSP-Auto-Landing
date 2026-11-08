# Editor

class Editor
:   Provides access to the vehicle assembly building and space plane hangar.
    Obtained by calling [`editor`](./space-center.md#SpaceCenter.editor "SpaceCenter.editor").

    facility
    :   The editor that is currently open.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`EditorFacility`](./space-center.md#SpaceCenter.EditorFacility "SpaceCenter.EditorFacility")

        Game Scenes:
        :   Editor Vab, Editor Sph

    vessel
    :   The vessel that is being constructed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`EditorVessel`](#SpaceCenter.EditorVessel "SpaceCenter.EditorVessel")

        Game Scenes:
        :   Editor Vab, Editor Sph

        > **Note**
        >
        > An editor that has not had a vessel loaded into it holds an empty vessel, with
        > no parts, rather than nothing at all.

    load\_vessel(*craft\_directory*, *name*)
    :   Load a vessel from a craft file into the editor, replacing the vessel that
        is currently being constructed.

        Parameters:
        :   - **craft\_directory** (*str*) – Name of the directory in the current saves “Ships” directory, that contains the craft file. For example `"VAB"` or `"SPH"`.
            - **name** (*str*) – Name of the vessel to load. This is the name of the “.craft” file in the save directory, without the “.craft” file extension.

        Game Scenes:
        :   Editor Vab, Editor Sph

        > **Note**
        >
        > The vessel is loaded into whichever editor is open, regardless of which
        > directory it came from. Its delta-v figures are recalculated in the
        > background afterwards; see [`EditorVessel.delta_v_ready`](#SpaceCenter.EditorVessel.delta_v_ready "SpaceCenter.EditorVessel.delta_v_ready").

    launch\_vessel(*launch\_site*[, *crew=None*][, *recover=True*][, *flag\_url=''*])
    :   Launch the vessel that is being constructed, from the given launch site.

        Parameters:
        :   - **launch\_site** (*str*) – Name of the launch site. For example `"LaunchPad"` or `"Runway"`.
            - **crew** (*list* *or* *None*) – The Kerbals to place in the vessel, by name. Controls how the vessel is crewed, as described in the remarks.
            - **recover** (*bool*) – If true and there is a vessel on the launch site, recover it before launching.
            - **flag\_url** (*str*) – If not `None`, the asset URL of the mission flag to use for the launch.

        Game Scenes:
        :   Editor Vab, Editor Sph

        > **Note**
        >
        > The vessel being constructed is not a craft file, so it is written to the editor’s
        > auto-saved craft file and that is launched, as the game’s own launch button does.
        > The vessel’s own craft file, if it was loaded from one, is left alone.
        > The *crew* parameter works as it does for
        > [`launch_vessel()`](./space-center.md#SpaceCenter.launch_vessel "SpaceCenter.launch_vessel"), and takes precedence over the crew assigned
        > in the editor.
        > Throws an exception if any of the games pre-flight checks fail.

    delta\_v\_body
    :   The body whose gravity and atmosphere the vessel’s delta-v, thrust, TWR and
        specific impulse figures assume.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`CelestialBody`](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody")

        Game Scenes:
        :   Editor Vab, Editor Sph

        > **Note**
        >
        > This and [`Editor.delta_v_altitude`](#SpaceCenter.Editor.delta_v_altitude "SpaceCenter.Editor.delta_v_altitude") are what [`EditorVessel.delta_v`](#SpaceCenter.EditorVessel.delta_v "SpaceCenter.EditorVessel.delta_v"),
        > [`Stage.delta_v`](./stage.md#SpaceCenter.Stage.delta_v "SpaceCenter.Stage.delta_v") and the other current-situation figures are
        > computed for. The body sets the gravity the thrust-to-weight ratios are
        > against, and, with the altitude, the atmospheric pressure and density the
        > specific impulses and thrusts are against.
        > This is global game state rather than editor state: the same setting backs
        > the game’s delta-v readout in flight, and it persists across scene changes.
        > Setting it asks the game to re-run the simulation and returns straight away;
        > the figures are out of date until [`EditorVessel.delta_v_ready`](#SpaceCenter.EditorVessel.delta_v_ready "SpaceCenter.EditorVessel.delta_v_ready") is
        > true again.

    delta\_v\_altitude
    :   The altitude above [`Editor.delta_v_body`](#SpaceCenter.Editor.delta_v_body "SpaceCenter.Editor.delta_v_body"), in meters, that the vessel’s
        delta-v, thrust, TWR and specific impulse figures assume. The atmospheric
        pressure and density at this altitude are what the figures are computed for,
        so raising it from zero to above the body’s atmosphere moves them from the
        vessel’s sea-level figures to its vacuum ones.
        Setting it asks the game to re-run the simulation and returns straight away;
        the figures are out of date until [`EditorVessel.delta_v_ready`](#SpaceCenter.EditorVessel.delta_v_ready "SpaceCenter.EditorVessel.delta_v_ready") is
        true again.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Editor Vab, Editor Sph

    delta\_v\_situation
    :   Which situation the game’s own delta-v readout shows figures for.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`DeltaVSituation`](#SpaceCenter.DeltaVSituation "SpaceCenter.DeltaVSituation")

        Game Scenes:
        :   Editor Vab, Editor Sph

        > **Note**
        >
        > This selects which column the game’s own delta-v app displays, and nothing
        > else. It does not affect anything kRPC reports: the current-situation
        > figures follow [`Editor.delta_v_body`](#SpaceCenter.Editor.delta_v_body "SpaceCenter.Editor.delta_v_body") and [`Editor.delta_v_altitude`](#SpaceCenter.Editor.delta_v_altitude "SpaceCenter.Editor.delta_v_altitude")
        > whatever this is set to, and the sea-level and vacuum figures are reported
        > directly as [`EditorVessel.sea_level_delta_v`](#SpaceCenter.EditorVessel.sea_level_delta_v "SpaceCenter.EditorVessel.sea_level_delta_v") and
        > [`EditorVessel.vacuum_delta_v`](#SpaceCenter.EditorVessel.vacuum_delta_v "SpaceCenter.EditorVessel.vacuum_delta_v"). It is here for completeness with the
        > app.

class DeltaVSituation
:   The situation the game’s own delta-v readout shows figures for.
    See [`Editor.delta_v_situation`](#SpaceCenter.Editor.delta_v_situation "SpaceCenter.Editor.delta_v_situation").

    sea\_level
    :   At sea level on the selected body.

    altitude
    :   At the selected altitude on the selected body.

    vacuum
    :   In vacuum.

class EditorVessel
:   The vessel that is being constructed in the editor.
    Obtained by calling [`Editor.vessel`](#SpaceCenter.Editor.vessel "SpaceCenter.Editor.vessel").

    name
    :   The name of the vessel.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Editor Vab, Editor Sph

    description
    :   The description of the vessel.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Editor Vab, Editor Sph

    facility
    :   The editor the vessel is in.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`EditorFacility`](./space-center.md#SpaceCenter.EditorFacility "SpaceCenter.EditorFacility")

        Game Scenes:
        :   Editor Vab, Editor Sph

        > **Note**
        >
        > This is the editor the vessel is currently loaded into, not the one it was
        > designed in. A vessel moved from the vehicle assembly building to the space
        > plane hangar reports the space plane hangar.

    mass
    :   The total mass of the vessel, including resources and crew, in kg.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Editor Vab, Editor Sph

    dry\_mass
    :   The total mass of the vessel, excluding resources, in kg.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Editor Vab, Editor Sph

    moment\_of\_inertia
    :   The moment of inertia of the vessel around its center of mass in \(kg.m^2\).
        The inertia values in the returned 3-tuple are around the
        pitch, roll and yaw directions respectively.
        This corresponds to the vessels reference frame ([`EditorVessel.reference_frame`](#SpaceCenter.EditorVessel.reference_frame "SpaceCenter.EditorVessel.reference_frame")).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Editor Vab, Editor Sph

    inertia\_tensor
    :   The inertia tensor of the vessel around its center of mass,
        in the vessels reference frame ([`EditorVessel.reference_frame`](#SpaceCenter.EditorVessel.reference_frame "SpaceCenter.EditorVessel.reference_frame")).
        Returns the 3x3 matrix as a list of elements, in row-major order.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list(float)

        Game Scenes:
        :   Editor Vab, Editor Sph

    reference\_frame
    :   The reference frame that is fixed relative to the vessel,
        and orientated with the vessel.

        - The origin is at the center of mass of the vessel.
        - The axes rotate with the vessel.
        - The x-axis points out to the right of the vessel.
        - The y-axis points in the forward direction of the vessel.
        - The z-axis points out of the bottom off the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Game Scenes:
        :   Editor Vab, Editor Sph

        > **Note**
        >
        > The axes are those of the root part. This is the frame
        > [`Vessel.reference_frame`](./vessel.md#SpaceCenter.Vessel.reference_frame "SpaceCenter.Vessel.reference_frame") gives at launch, with the root part as the
        > control point.

    surface\_reference\_frame
    :   The reference frame that is fixed relative to the vessel,
        and orientated with the vertical of the editor.

        - The origin is at the center of mass of the vessel.
        - The axes are fixed in the editor.
        - The x-axis points upwards.
        - The y-axis and z-axis are horizontal.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Game Scenes:
        :   Editor Vab, Editor Sph

        > **Note**
        >
        > This is the vessel as it stands in the editor, and as it stands at launch. The
        > editor has no north or east, so the horizontal axes are fixed in the editor.

    flight([*reference\_frame=None*])
    :   Returns a [`SimulatedFlight`](#SpaceCenter.SimulatedFlight "SpaceCenter.SimulatedFlight") object that can be used to simulate the
        vessel in flight, in the specified reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame") *or* *None*) – Reference frame. Defaults to the simulated surface reference frame ([`SimulatedFlight.surface_reference_frame`](#SpaceCenter.SimulatedFlight.surface_reference_frame "SpaceCenter.SimulatedFlight.surface_reference_frame")), and follows it when it is set.

        Return type:
        :   [`SimulatedFlight`](#SpaceCenter.SimulatedFlight "SpaceCenter.SimulatedFlight")

        Game Scenes:
        :   Editor Vab, Editor Sph

        > **Note**
        >
        > Each call returns a new object, which is kept until the game leaves the editor.

    position(*reference\_frame*)
    :   The position of the center of mass of the vessel, in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned position vector is in.

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Editor Vab, Editor Sph

    bounding\_box(*reference\_frame*)
    :   The axis-aligned bounding box of the vessel in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned position vectors are in.

        Returns:
        :   The positions of the minimum and maximum vertices of the box.

        Return type:
        :   [`BoundingBox`](./vessel.md#SpaceCenter.BoundingBox "SpaceCenter.BoundingBox")

        Game Scenes:
        :   Editor Vab, Editor Sph

    rotation(*reference\_frame*)
    :   The rotation of the vessel, in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned rotation is in.

        Returns:
        :   The rotation as a quaternion of the form \((x, y, z, w)\).

        Return type:
        :   tuple(float, float, float, float)

        Game Scenes:
        :   Editor Vab, Editor Sph

    direction(*reference\_frame*)
    :   The direction in which the vessel is pointing, in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned direction is in.

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Editor Vab, Editor Sph

    cost
    :   The total cost of the vessel, including resources, in funds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Editor Vab, Editor Sph

    crew\_capacity
    :   The total number of crew that the vessel can hold.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   int

        Game Scenes:
        :   Editor Vab, Editor Sph

    parts
    :   A [`Parts`](./parts.md#SpaceCenter.Parts "SpaceCenter.Parts") object, that can be used to interact with the parts
        that make up the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Parts`](./parts.md#SpaceCenter.Parts "SpaceCenter.Parts")

        Game Scenes:
        :   Editor Vab, Editor Sph

        > **Note**
        >
        > Only the parts of [`Part`](./parts.md#SpaceCenter.Part "SpaceCenter.Part") that describe the design are
        > available in the editor. Those that need a vessel in flight or running physics,
        > such as the thermal members, are restricted to the flight scene.
        > [`Part.position()`](./parts.md#SpaceCenter.Part.position "SpaceCenter.Part.position"), [`Part.center_of_mass()`](./parts.md#SpaceCenter.Part.center_of_mass "SpaceCenter.Part.center_of_mass"),
        > [`Part.direction()`](./parts.md#SpaceCenter.Part.direction "SpaceCenter.Part.direction"), [`Part.rotation()`](./parts.md#SpaceCenter.Part.rotation "SpaceCenter.Part.rotation"),
        > [`Part.bounding_box()`](./parts.md#SpaceCenter.Part.bounding_box "SpaceCenter.Part.bounding_box"), the part’s reference frames,
        > [`Part.moment_of_inertia`](./parts.md#SpaceCenter.Part.moment_of_inertia "SpaceCenter.Part.moment_of_inertia") and
        > [`Part.inertia_tensor`](./parts.md#SpaceCenter.Part.inertia_tensor "SpaceCenter.Part.inertia_tensor") are available.

    resources
    :   A [`EditorVessel.resources`](#SpaceCenter.EditorVessel.resources "SpaceCenter.EditorVessel.resources") object for the resources the vessel can hold.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Resources`](./resources.md#SpaceCenter.Resources "SpaceCenter.Resources")

        Game Scenes:
        :   Editor Vab, Editor Sph

        > **Note**
        >
        > This is a view of what the vessel can hold as designed. The flow-state
        > members work, as they are part of the design, but nothing here reports how
        > a resource would be drawn on in flight.

    stages
    :   Activation (burn) stages for the vessel, in ascending stage order. This does
        not include the -1 stage (parts with no staging icon), as it carries no
        delta-v; use [`EditorVessel.stage_at()`](#SpaceCenter.EditorVessel.stage_at "SpaceCenter.EditorVessel.stage_at") with -1 to get those parts.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Stage`](./stage.md#SpaceCenter.Stage "SpaceCenter.Stage"))

        Game Scenes:
        :   Editor Vab, Editor Sph

    stage\_at(*stage*)
    :   The activation stage with the given number. Pass -1 to get the parts that are
        never activated (those with no staging icon).

        Parameters:
        :   **stage** (*int*) – Get activation stage at this index.

        Return type:
        :   [`Stage`](./stage.md#SpaceCenter.Stage "SpaceCenter.Stage")

        Game Scenes:
        :   Editor Vab, Editor Sph

    decouple\_stages
    :   Decouple stages for the vessel, in ascending stage order. The -1 stage,
        containing the parts that are never decoupled and remain on the vessel, is
        included first when any such parts exist.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Stage`](./stage.md#SpaceCenter.Stage "SpaceCenter.Stage"))

        Game Scenes:
        :   Editor Vab, Editor Sph

    decouple\_stage\_at(*stage*)
    :   The decouple stage with the given number. Pass -1 to get the parts that are
        never decoupled and remain on the vessel after all stages have fired.

        Parameters:
        :   **stage** (*int*) – Get decouple stage at this index.

        Return type:
        :   [`Stage`](./stage.md#SpaceCenter.Stage "SpaceCenter.Stage")

        Game Scenes:
        :   Editor Vab, Editor Sph

    delta\_v\_ready
    :   Whether the stock delta-v figures for the vessel have been calculated.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Editor Vab, Editor Sph

        > **Note**
        >
        > The game recalculates them in the background whenever the vessel changes, and
        > whenever [`Editor.delta_v_body`](#SpaceCenter.Editor.delta_v_body "SpaceCenter.Editor.delta_v_body") or [`Editor.delta_v_altitude`](#SpaceCenter.Editor.delta_v_altitude "SpaceCenter.Editor.delta_v_altitude")
        > is set, and they are out of date until it has finished. Reading
        > [`EditorVessel.delta_v`](#SpaceCenter.EditorVessel.delta_v "SpaceCenter.EditorVessel.delta_v"), or any delta-v member of a [`Stage`](./stage.md#SpaceCenter.Stage "SpaceCenter.Stage"), before
        > then raises an error rather than returning a stale figure. Poll this after
        > changing anything the figures depend on.

    delta\_v
    :   Total delta-v for the vessel in the situation the game’s delta-v readout
        assumes, in m/s. See [`Editor.delta_v_body`](#SpaceCenter.Editor.delta_v_body "SpaceCenter.Editor.delta_v_body").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Editor Vab, Editor Sph

    vacuum\_delta\_v
    :   Total vacuum delta-v for the vessel, in m/s.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Editor Vab, Editor Sph

    sea\_level\_delta\_v
    :   Total sea-level delta-v for the vessel, in m/s.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Editor Vab, Editor Sph

    burn\_time
    :   Total burn time for the vessel, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Editor Vab, Editor Sph

class SimulatedFlight
:   Used to simulate the vessel in the editor in flight. The counterpart of
    [`Flight`](./flight.md#SpaceCenter.Flight "SpaceCenter.Flight"). All of the information returned by this class is given in
    the reference frame passed to [`EditorVessel.flight()`](#SpaceCenter.EditorVessel.flight "SpaceCenter.EditorVessel.flight"), or in
    [`SimulatedFlight.surface_reference_frame`](#SpaceCenter.SimulatedFlight.surface_reference_frame "SpaceCenter.SimulatedFlight.surface_reference_frame") if none was passed.
    Obtained by calling [`EditorVessel.flight()`](#SpaceCenter.EditorVessel.flight "SpaceCenter.EditorVessel.flight").

    > **Note**
    >
    > The simulated attitude is set by [`SimulatedFlight.surface_reference_frame`](#SpaceCenter.SimulatedFlight.surface_reference_frame "SpaceCenter.SimulatedFlight.surface_reference_frame"), and belongs to
    > this object. The vessel is read from the editor on each call, so the object follows
    > every edit.

    surface\_reference\_frame
    :   The reference frame of the surface the vessel is simulated over. Its x-axis
        points upwards, and gravity acts along the negative x-axis. Defaults to
        [`EditorVessel.surface_reference_frame`](#SpaceCenter.EditorVessel.surface_reference_frame "SpaceCenter.EditorVessel.surface_reference_frame").

        Attribute:
        :   Can be read or written

        Return type:
        :   [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Game Scenes:
        :   Editor Vab, Editor Sph

        > **Note**
        >
        > Only the rotation of the frame is used. A frame created relative to
        > [`EditorVessel.reference_frame`](#SpaceCenter.EditorVessel.reference_frame "SpaceCenter.EditorVessel.reference_frame") follows the vessel as it is edited.

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
        :   Editor Vab, Editor Sph

    rotation
    :   The rotation of the vessel, in the reference frame [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Attribute:
        :   Read-only, cannot be set

        Returns:
        :   The rotation as a quaternion of the form \((x, y, z, w)\).

        Return type:
        :   tuple(float, float, float, float)

        Game Scenes:
        :   Editor Vab, Editor Sph

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
        :   Editor Vab, Editor Sph

    pitch
    :   The pitch of the vessel relative to the horizon, in degrees.
        A value between -90° and +90°.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Editor Vab, Editor Sph

        > **Note**
        >
        > An absolute Euler angle, ill-conditioned when the vessel points near vertical (a pitch
        > near -90° or +90°). For an always-defined attitude use [`SimulatedFlight.rotation`](#SpaceCenter.SimulatedFlight.rotation "SpaceCenter.SimulatedFlight.rotation") or
        > [`SimulatedFlight.direction`](#SpaceCenter.SimulatedFlight.direction "SpaceCenter.SimulatedFlight.direction").

    roll
    :   The roll of the vessel relative to the horizon, in degrees.
        A value between -180° and +180°.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Editor Vab, Editor Sph

        > **Note**
        >
        > An absolute Euler angle, ill-conditioned when the vessel points near vertical (a pitch
        > near -90° or +90°). For an always-defined attitude use [`SimulatedFlight.rotation`](#SpaceCenter.SimulatedFlight.rotation "SpaceCenter.SimulatedFlight.rotation").

    displacement
    :   The volume of water the vessel displaces when fully submerged, in
        \(m^3\).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Editor Vab, Editor Sph

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
        :   Editor Vab, Editor Sph

        > **Note**
        >
        > A hull at rest carries this on the same vertical line as
        > [`SimulatedFlight.center_of_mass`](#SpaceCenter.SimulatedFlight.center_of_mass "SpaceCenter.SimulatedFlight.center_of_mass").

    buoyant\_force\_at(*density*, *gravity*)
    :   The buoyant force on the vessel when fully submerged in a fluid of the given
        density, in Newtons.

        Parameters:
        :   - **density** (*float*) – The density of the fluid, in \(kg/m^3\).
            - **gravity** (*float*) – The acceleration due to gravity, in \(m/s^2\).

        Return type:
        :   float

        Game Scenes:
        :   Editor Vab, Editor Sph

    buoyant\_torque\_at(*density*, *gravity*)
    :   The torque the buoyant force exerts about the vessel’s center of mass when fully
        submerged in a fluid of the given density, in reference frame
        [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame"). Gravity acts along the negative x-axis of
        [`SimulatedFlight.surface_reference_frame`](#SpaceCenter.SimulatedFlight.surface_reference_frame "SpaceCenter.SimulatedFlight.surface_reference_frame").

        Parameters:
        :   - **density** (*float*) – The density of the fluid, in \(kg/m^3\).
            - **gravity** (*float*) – The acceleration due to gravity, in \(m/s^2\).

        Returns:
        :   A vector pointing along the axis of the torque, with its magnitude equal to the strength of the torque in newton-meters.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Editor Vab, Editor Sph
