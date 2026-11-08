# Parts

The following classes allow interaction with a vessels individual parts.

- [Parts](#id1)
- [Part](#part)
- [Module](#module)
- [Config Node](#config-node)
- [Specific Types of Part](#specific-types-of-part)
    - [Antenna](#antenna)
    - [Cargo Bay](#cargo-bay)
    - [Control Surface](#control-surface)
    - [Decoupler](#decoupler)
    - [Deployable State](#deployable-state)
    - [Docking Port](#docking-port)
    - [Engine](#engine)
    - [Experiment](#experiment)
    - [Fairing](#fairing)
    - [Intake](#intake)
    - [Leg](#leg)
    - [Launch Clamp](#launch-clamp)
    - [Light](#light)
    - [Parachute](#parachute)
    - [Radiator](#radiator)
    - [Resource Converter](#resource-converter)
    - [Resource Harvester](#resource-harvester)
    - [Reaction Wheel](#reaction-wheel)
    - [Resource Drain](#resource-drain)
    - [Robotic Controller](#robotic-controller)
    - [Robotic Hinge](#robotic-hinge)
    - [Robotic Piston](#robotic-piston)
    - [Robotic Rotation](#robotic-rotation)
    - [Robotic Rotor](#robotic-rotor)
    - [RCS](#rcs)
    - [Sensor](#sensor)
    - [Solar Panel](#solar-panel)
    - [Thruster](#thruster)
    - [Wheel](#wheel)
- [Trees of Parts](#trees-of-parts)
    - [Traversing the Tree](#traversing-the-tree)
    - [Attachment Modes](#attachment-modes)
- [Fuel Lines](#fuel-lines)
- [Staging](#staging)

## [Parts](#id22)

class Parts
:   Instances of this class are used to interact with the parts of a vessel.
    An instance can be obtained by calling [`Vessel.parts`](./vessel.md#SpaceCenter.Vessel.parts "SpaceCenter.Vessel.parts").

    all
    :   A list of all of the vessels parts.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Part`](#SpaceCenter.Part "SpaceCenter.Part"))

    root
    :   The vessels root part.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        > **Note**
        >
        > See the discussion on [Trees of Parts](#python-api-parts-trees-of-parts).

    controlling
    :   The part from which the vessel is controlled.
        Returns `None` if the vessel has no control point, in which case it is
        oriented by its root part.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part") or None

        Game Scenes:
        :   Flight

        > **Note**
        >
        > A vessel gets no control point when it is created by a decoupler or an undocking
        > and none of its parts is a command module with crew aboard. A probe core does not
        > give it one either. Assign this property to choose the control point, which is
        > equivalent to “Control From Here” in game.

    with\_name(*name*)
    :   A list of parts whose [`Part.name`](#SpaceCenter.Part.name "SpaceCenter.Part.name") is *name*.

        Parameters:
        :   **name** (*str*)

        Return type:
        :   list([`Part`](#SpaceCenter.Part "SpaceCenter.Part"))

    with\_title(*title*)
    :   A list of all parts whose [`Part.title`](#SpaceCenter.Part.title "SpaceCenter.Part.title") is *title*.

        Parameters:
        :   **title** (*str*)

        Return type:
        :   list([`Part`](#SpaceCenter.Part "SpaceCenter.Part"))

    with\_tag(*tag*)
    :   A list of all parts whose [`Part.tag`](#SpaceCenter.Part.tag "SpaceCenter.Part.tag") is *tag*.

        Parameters:
        :   **tag** (*str*)

        Return type:
        :   list([`Part`](#SpaceCenter.Part "SpaceCenter.Part"))

    with\_module(*module\_name*)
    :   A list of all parts that contain a [`Module`](#SpaceCenter.Module "SpaceCenter.Module") whose
        [`Module.name`](#SpaceCenter.Module.name "SpaceCenter.Module.name") is *module\_name*.

        Parameters:
        :   **module\_name** (*str*)

        Return type:
        :   list([`Part`](#SpaceCenter.Part "SpaceCenter.Part"))

    modules\_with\_name(*module\_name*)
    :   A list of modules (combined across all parts in the vessel) whose
        [`Module.name`](#SpaceCenter.Module.name "SpaceCenter.Module.name") is *module\_name*.

        Parameters:
        :   **module\_name** (*str*)

        Return type:
        :   list([`Module`](#SpaceCenter.Module "SpaceCenter.Module"))

    antennas
    :   A list of all antennas in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Antenna`](#SpaceCenter.Antenna "SpaceCenter.Antenna"))

        > **Note**
        >
        > If RemoteTech is installed, this will always return an empty list.
        > To interact with RemoteTech antennas, use the RemoteTech service APIs.

    cargo\_bays
    :   A list of all cargo bays in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`CargoBay`](#SpaceCenter.CargoBay "SpaceCenter.CargoBay"))

    control\_surfaces
    :   A list of all control surfaces in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`ControlSurface`](#SpaceCenter.ControlSurface "SpaceCenter.ControlSurface"))

    decouplers
    :   A list of all decouplers in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Decoupler`](#SpaceCenter.Decoupler "SpaceCenter.Decoupler"))

    docking\_ports
    :   A list of all docking ports in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`DockingPort`](#SpaceCenter.DockingPort "SpaceCenter.DockingPort"))

    engines
    :   A list of all engines in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Engine`](#SpaceCenter.Engine "SpaceCenter.Engine"))

        > **Note**
        >
        > This includes any part that generates thrust. This covers many different types
        > of engine, including liquid fuel rockets, solid rocket boosters, jet engines and
        > RCS thrusters.

    experiments
    :   A list of all science experiments in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Experiment`](#SpaceCenter.Experiment "SpaceCenter.Experiment"))

    fairings
    :   A list of all fairings in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Fairing`](#SpaceCenter.Fairing "SpaceCenter.Fairing"))

    intakes
    :   A list of all intakes in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Intake`](#SpaceCenter.Intake "SpaceCenter.Intake"))

    legs
    :   A list of all landing legs attached to the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Leg`](#SpaceCenter.Leg "SpaceCenter.Leg"))

    launch\_clamps
    :   A list of all launch clamps attached to the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`LaunchClamp`](#SpaceCenter.LaunchClamp "SpaceCenter.LaunchClamp"))

    lights
    :   A list of all lights in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Light`](#SpaceCenter.Light "SpaceCenter.Light"))

    parachutes
    :   A list of all parachutes in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Parachute`](#SpaceCenter.Parachute "SpaceCenter.Parachute"))

    radiators
    :   A list of all radiators in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Radiator`](#SpaceCenter.Radiator "SpaceCenter.Radiator"))

    resource\_drains
    :   A list of all resource drains in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`ResourceDrain`](#SpaceCenter.ResourceDrain "SpaceCenter.ResourceDrain"))

    rcs
    :   A list of all RCS blocks/thrusters in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`RCS`](#SpaceCenter.RCS "SpaceCenter.RCS"))

    reaction\_wheels
    :   A list of all reaction wheels in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`ReactionWheel`](#SpaceCenter.ReactionWheel "SpaceCenter.ReactionWheel"))

    resource\_converters
    :   A list of all resource converters in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`ResourceConverter`](#SpaceCenter.ResourceConverter "SpaceCenter.ResourceConverter"))

    resource\_harvesters
    :   A list of all resource harvesters in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`ResourceHarvester`](#SpaceCenter.ResourceHarvester "SpaceCenter.ResourceHarvester"))

    robotic\_controllers
    :   A list of all robotic controllers in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`RoboticController`](#SpaceCenter.RoboticController "SpaceCenter.RoboticController"))

    robotic\_hinges
    :   A list of all robotic hinges in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`RoboticHinge`](#SpaceCenter.RoboticHinge "SpaceCenter.RoboticHinge"))

    robotic\_pistons
    :   A list of all robotic pistons in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`RoboticPiston`](#SpaceCenter.RoboticPiston "SpaceCenter.RoboticPiston"))

    robotic\_rotations
    :   A list of all robotic rotations in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`RoboticRotation`](#SpaceCenter.RoboticRotation "SpaceCenter.RoboticRotation"))

    robotic\_rotors
    :   A list of all robotic rotors in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`RoboticRotor`](#SpaceCenter.RoboticRotor "SpaceCenter.RoboticRotor"))

    sensors
    :   A list of all sensors in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Sensor`](#SpaceCenter.Sensor "SpaceCenter.Sensor"))

    solar\_panels
    :   A list of all solar panels in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`SolarPanel`](#SpaceCenter.SolarPanel "SpaceCenter.SolarPanel"))

    wheels
    :   A list of all wheels in the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Wheel`](#SpaceCenter.Wheel "SpaceCenter.Wheel"))

## [Part](#id23)

class Part
:   Represents an individual part. Vessels are made up of multiple parts.
    Instances of this class can be obtained by several methods in [`Parts`](#SpaceCenter.Parts "SpaceCenter.Parts").

    name
    :   Internal name of the part, as used in
        [part cfg files](https://wiki.kerbalspaceprogram.com/wiki/CFG_File_Documentation).
        For example “Mark1-2Pod”.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    title
    :   Title of the part, as shown when the part is right clicked in-game. For example “Mk1-2 Command Pod”.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    config
    :   The static configuration of the part, as found in its
        [part cfg file](https://wiki.kerbalspaceprogram.com/wiki/CFG_File_Documentation).
        This provides access to data that is not exposed elsewhere, such as the
        configuration of the part’s modules. Returns `None` if the part has no
        associated configuration node.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ConfigNode`](#SpaceCenter.ConfigNode "SpaceCenter.ConfigNode")

    tag
    :   The name tag for the part. Can be set to a custom string using the
        in-game user interface. Throws an exception if no name tag part module is installed.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        > **Note**
        >
        > The name tag part module is provided by
        > [KSPCommunityPartModules](https://github.com/KSPModdingLibs/KSPCommunityPartModules)
        > 0.5 or later, or by
        > [kOS](https://forum.kerbalspaceprogram.com/topic/165628-ksp-112x-kos-v1400-kos-scriptable-autopilot-system/),
        > which shares this string.

    flag\_url
    :   The asset URL for the part’s flag.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

    highlighted
    :   Whether the part is highlighted.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The highlighting is removed when the client that enabled it disconnects.

    highlight\_color
    :   The color used to highlight the part.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`StdLib.ColorRGB`](../stdlib.md#StdLib.ColorRGB "StdLib.ColorRGB")

        Game Scenes:
        :   Flight

    cost
    :   The cost of the part, in units of funds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    vessel
    :   The vessel that contains this part.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Vessel`](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")

        Game Scenes:
        :   Flight

    parent
    :   The parts parent. Returns `None` if the part does not have a parent.
        This, in combination with [`Part.children`](#SpaceCenter.Part.children "SpaceCenter.Part.children"), can be used to traverse the vessels
        parts tree.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part") or None

        > **Note**
        >
        > See the discussion on [Trees of Parts](#python-api-parts-trees-of-parts).

    children
    :   The parts children. Returns an empty list if the part has no children.
        This, in combination with [`Part.parent`](#SpaceCenter.Part.parent "SpaceCenter.Part.parent"), can be used to traverse the vessels
        parts tree.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Part`](#SpaceCenter.Part "SpaceCenter.Part"))

        > **Note**
        >
        > See the discussion on [Trees of Parts](#python-api-parts-trees-of-parts).

    axially\_attached
    :   Whether the part is axially attached to its parent, i.e. on the top
        or bottom of its parent. If the part has no parent, returns `False`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        > **Note**
        >
        > See the discussion on [Attachment Modes](#python-api-parts-attachment-modes).

    radially\_attached
    :   Whether the part is radially attached to its parent, i.e. on the side of its parent.
        If the part has no parent, returns `False`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        > **Note**
        >
        > See the discussion on [Attachment Modes](#python-api-parts-attachment-modes).

    stage
    :   The stage in which this part will be activated. Returns -1 if the part is not
        activated by staging.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   int

        > **Note**
        >
        > See the discussion on [Staging](#python-api-parts-staging).

    decouple\_stage
    :   The stage in which this part will be decoupled. Returns -1 if the part is never
        decoupled from the vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   int

        > **Note**
        >
        > See the discussion on [Staging](#python-api-parts-staging).

    massless
    :   Whether the part is
        [massless](https://wiki.kerbalspaceprogram.com/wiki/Massless_part).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    mass
    :   The current mass of the part, including resources it contains, in kilograms.
        Returns zero if the part is massless.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    dry\_mass
    :   The mass of the part, not including any resources it contains, in kilograms.
        Returns zero if the part is massless.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    shielded
    :   Whether the part is shielded from the exterior of the vessel, for example by a fairing.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    dynamic\_pressure
    :   The dynamic pressure acting on the part, in Pascals.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    static\_pressure
    :   The static pressure acting on the part, in Pascals. Below the waterline this
        includes the pressure of the water.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    displacement
    :   The volume of water the part displaces when fully submerged, in \(m^3\).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

        > **Note**
        >
        > See [`Part.displaced_volume`](#SpaceCenter.Part.displaced_volume "SpaceCenter.Part.displaced_volume") for the volume it is displacing now.

    buoyancy\_multiplier
    :   The multiplier applied to the buoyant force on the part. One for most parts.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    buoyant\_force\_at(*density*, *gravity*)
    :   The buoyant force on the part when fully submerged in a fluid of the given
        density, in Newtons.

        Parameters:
        :   - **density** (*float*) – The density of the fluid, in \(kg/m^3\).
            - **gravity** (*float*) – The acceleration due to gravity, in \(m/s^2\).

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

        > **Note**
        >
        > Applies [`buoyancy_scalar`](./space-center.md#SpaceCenter.buoyancy_scalar "SpaceCenter.buoyancy_scalar") and
        > [`Part.buoyancy_multiplier`](#SpaceCenter.Part.buoyancy_multiplier "SpaceCenter.Part.buoyancy_multiplier"), so the result is what the game would do to the
        > part rather than what
        > [Archimedes’ principle](https://en.wikipedia.org/wiki/Archimedes%27_principle) alone gives.

    splashed
    :   Whether the part is in contact with water.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    submerged\_portion
    :   The fraction of the part below the waterline, between 0 and 1.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    displaced\_volume
    :   The volume of water the part is displacing, in \(m^3\).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    depth
    :   The depth of the part below the waterline, in meters, measured at
        [`Part.center_of_displacement()`](#SpaceCenter.Part.center_of_displacement "SpaceCenter.Part.center_of_displacement"). Negative above the waterline. Returns zero
        if the body the vessel is at has no ocean.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    min\_depth
    :   The depth of the shallowest point of the part below the waterline, in meters.
        Returns zero if the body the vessel is at has no ocean.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_depth
    :   The depth of the deepest point of the part below the waterline, in meters.
        Returns zero if the body the vessel is at has no ocean.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The game ramps the buoyant force in over the first fraction of a meter of this
        > depth.

    submerged\_dynamic\_pressure
    :   The dynamic pressure of the water acting on the part, in Pascals.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    submerged\_drag\_multiplier
    :   The multiplier applied to the drag on the part while it is submerged.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The game sets this high on splashdown and eases it down as the part moves through
        > the water. It climbs back once the part slows to a crawl, so a hull under way
        > reads lower than one lying still.

    submerged\_lift\_multiplier
    :   The multiplier applied to the lift on the part while it is submerged.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    water\_angular\_drag\_multiplier
    :   The multiplier applied to the angular drag on the part while it is submerged.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

        > **Note**
        >
        > Part of the part’s configuration rather than a live readout, so it reads the same
        > in the editor and at a body with no ocean.

    impact\_tolerance
    :   The impact tolerance of the part, in meters per second.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    temperature
    :   Temperature of the part, in Kelvin.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    skin\_temperature
    :   Temperature of the skin of the part, in Kelvin.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_temperature
    :   Maximum temperature that the part can survive, in Kelvin.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    max\_skin\_temperature
    :   Maximum temperature that the skin of the part can survive, in Kelvin.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    max\_pressure
    :   Maximum pressure that the part can survive, in Pascals.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        > **Note**
        >
        > With part pressure limits enabled in the difficulty settings, the part is destroyed
        > when [`Part.static_pressure`](#SpaceCenter.Part.static_pressure "SpaceCenter.Part.static_pressure") plus [`Part.dynamic_pressure`](#SpaceCenter.Part.dynamic_pressure "SpaceCenter.Part.dynamic_pressure") exceeds this.

    thermal\_mass
    :   A measure of how much energy it takes to increase the internal temperature of the part,
        in Joules per Kelvin.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    thermal\_skin\_mass
    :   A measure of how much energy it takes to increase the skin temperature of the part,
        in Joules per Kelvin.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    thermal\_resource\_mass
    :   A measure of how much energy it takes to increase the temperature of the resources
        contained in the part, in Joules per Kelvin.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    thermal\_conduction\_flux
    :   The rate at which heat energy is conducting into or out of the part via contact with
        other parts. Measured in energy per unit time, or power, in Watts.
        A positive value means the part is gaining heat energy, and negative means it is
        losing heat energy.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    thermal\_convection\_flux
    :   The rate at which heat energy is convecting into or out of the part from the
        surrounding atmosphere. Measured in energy per unit time, or power, in Watts.
        A positive value means the part is gaining heat energy, and negative means it is
        losing heat energy.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    thermal\_radiation\_flux
    :   The rate at which heat energy is radiating into or out of the part from the surrounding
        environment. Measured in energy per unit time, or power, in Watts.
        A positive value means the part is gaining heat energy, and negative means it is
        losing heat energy.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    thermal\_internal\_flux
    :   The rate at which heat energy is begin generated by the part.
        For example, some engines generate heat by combusting fuel.
        Measured in energy per unit time, or power, in Watts.
        A positive value means the part is gaining heat energy, and negative means it is losing
        heat energy.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    thermal\_skin\_to\_internal\_flux
    :   The rate at which heat energy is transferring between the part’s skin and its internals.
        Measured in energy per unit time, or power, in Watts.
        A positive value means the part’s internals are gaining heat energy,
        and negative means its skin is gaining heat energy.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    crew\_capacity
    :   The number of crew members that can occupy the part.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   int

    crew
    :   The crew members occupying the part.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`CrewMember`](./vessel.md#SpaceCenter.CrewMember "SpaceCenter.CrewMember"))

        Game Scenes:
        :   Flight

    available\_seats
    :   How many open seats the part has.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   int

        Game Scenes:
        :   Flight

    resources
    :   A [`Part.resources`](#SpaceCenter.Part.resources "SpaceCenter.Part.resources") object for the part.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Resources`](./resources.md#SpaceCenter.Resources "SpaceCenter.Resources")

    crossfeed
    :   Whether this part is crossfeed capable.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    is\_fuel\_line
    :   Whether this part is a fuel line.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    fuel\_lines\_from
    :   The parts that are connected to this part via fuel lines, where the direction of the
        fuel line is into this part.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Part`](#SpaceCenter.Part "SpaceCenter.Part"))

        Game Scenes:
        :   Flight

        > **Note**
        >
        > See the discussion on [Fuel Lines](#python-api-parts-fuel-lines).

    fuel\_lines\_to
    :   The parts that are connected to this part via fuel lines, where the direction of the
        fuel line is out of this part.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Part`](#SpaceCenter.Part "SpaceCenter.Part"))

        Game Scenes:
        :   Flight

        > **Note**
        >
        > See the discussion on [Fuel Lines](#python-api-parts-fuel-lines).

    modules
    :   The modules for this part.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Module`](#SpaceCenter.Module "SpaceCenter.Module"))

    antenna
    :   An [`Part.antenna`](#SpaceCenter.Part.antenna "SpaceCenter.Part.antenna") if the part is an antenna, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Antenna`](#SpaceCenter.Antenna "SpaceCenter.Antenna") or None

        > **Note**
        >
        > If RemoteTech is installed, this will always return `None`.
        > To interact with RemoteTech antennas, use the RemoteTech service APIs.

    cargo\_bay
    :   A [`Part.cargo_bay`](#SpaceCenter.Part.cargo_bay "SpaceCenter.Part.cargo_bay") if the part is a cargo bay, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`CargoBay`](#SpaceCenter.CargoBay "SpaceCenter.CargoBay") or None

    control\_surface
    :   A [`Part.control_surface`](#SpaceCenter.Part.control_surface "SpaceCenter.Part.control_surface") if the part is an aerodynamic control surface,
        otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ControlSurface`](#SpaceCenter.ControlSurface "SpaceCenter.ControlSurface") or None

    decoupler
    :   A [`Part.decoupler`](#SpaceCenter.Part.decoupler "SpaceCenter.Part.decoupler") if the part is a decoupler, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Decoupler`](#SpaceCenter.Decoupler "SpaceCenter.Decoupler") or None

    docking\_port
    :   A [`Part.docking_port`](#SpaceCenter.Part.docking_port "SpaceCenter.Part.docking_port") if the part is a docking port, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`DockingPort`](#SpaceCenter.DockingPort "SpaceCenter.DockingPort") or None

    engine
    :   An [`Part.engine`](#SpaceCenter.Part.engine "SpaceCenter.Part.engine") if the part is an engine, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Engine`](#SpaceCenter.Engine "SpaceCenter.Engine") or None

    experiment
    :   An [`Part.experiment`](#SpaceCenter.Part.experiment "SpaceCenter.Part.experiment") if the part contains a
        single science experiment, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Experiment`](#SpaceCenter.Experiment "SpaceCenter.Experiment") or None

        > **Note**
        >
        > Throws an exception if the part contains more than one experiment.
        > In that case, use [`Part.experiments`](#SpaceCenter.Part.experiments "SpaceCenter.Part.experiments") to get the list of experiments in the part.

    experiments
    :   A list of [`Part.experiment`](#SpaceCenter.Part.experiment "SpaceCenter.Part.experiment") objects that the part contains.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Experiment`](#SpaceCenter.Experiment "SpaceCenter.Experiment")) or None

    fairing
    :   A [`Part.fairing`](#SpaceCenter.Part.fairing "SpaceCenter.Part.fairing") if the part is a fairing, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Fairing`](#SpaceCenter.Fairing "SpaceCenter.Fairing") or None

    intake
    :   An [`Part.intake`](#SpaceCenter.Part.intake "SpaceCenter.Part.intake") if the part is an intake, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Intake`](#SpaceCenter.Intake "SpaceCenter.Intake") or None

        > **Note**
        >
        > This includes any part that generates thrust. This covers many different types
        > of engine, including liquid fuel rockets, solid rocket boosters and jet engines.
        > For RCS thrusters see [`Part.rcs`](#SpaceCenter.Part.rcs "SpaceCenter.Part.rcs").

    leg
    :   A [`Part.leg`](#SpaceCenter.Part.leg "SpaceCenter.Part.leg") if the part is a landing leg, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Leg`](#SpaceCenter.Leg "SpaceCenter.Leg") or None

    launch\_clamp
    :   A [`Part.launch_clamp`](#SpaceCenter.Part.launch_clamp "SpaceCenter.Part.launch_clamp") if the part is a launch clamp, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`LaunchClamp`](#SpaceCenter.LaunchClamp "SpaceCenter.LaunchClamp") or None

    light
    :   A [`Part.light`](#SpaceCenter.Part.light "SpaceCenter.Part.light") if the part is a light, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Light`](#SpaceCenter.Light "SpaceCenter.Light") or None

    parachute
    :   A [`Part.parachute`](#SpaceCenter.Part.parachute "SpaceCenter.Part.parachute") if the part is a parachute, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Parachute`](#SpaceCenter.Parachute "SpaceCenter.Parachute") or None

    radiator
    :   A [`Part.radiator`](#SpaceCenter.Part.radiator "SpaceCenter.Part.radiator") if the part is a radiator, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Radiator`](#SpaceCenter.Radiator "SpaceCenter.Radiator") or None

    resource\_drain
    :   A [`Part.resource_drain`](#SpaceCenter.Part.resource_drain "SpaceCenter.Part.resource_drain") if the part is a resource drain, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ResourceDrain`](#SpaceCenter.ResourceDrain "SpaceCenter.ResourceDrain") or None

    rcs
    :   A [`Part.rcs`](#SpaceCenter.Part.rcs "SpaceCenter.Part.rcs") if the part is an RCS block/thruster, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`RCS`](#SpaceCenter.RCS "SpaceCenter.RCS") or None

    reaction\_wheel
    :   A [`Part.reaction_wheel`](#SpaceCenter.Part.reaction_wheel "SpaceCenter.Part.reaction_wheel") if the part is a reaction wheel, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ReactionWheel`](#SpaceCenter.ReactionWheel "SpaceCenter.ReactionWheel") or None

    resource\_converter
    :   A [`Part.resource_converter`](#SpaceCenter.Part.resource_converter "SpaceCenter.Part.resource_converter") if the part is a resource converter,
        otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ResourceConverter`](#SpaceCenter.ResourceConverter "SpaceCenter.ResourceConverter") or None

    resource\_harvester
    :   A [`Part.resource_harvester`](#SpaceCenter.Part.resource_harvester "SpaceCenter.Part.resource_harvester") if the part is a resource harvester,
        otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ResourceHarvester`](#SpaceCenter.ResourceHarvester "SpaceCenter.ResourceHarvester") or None

    robotic\_controller
    :   A [`Part.robotic_controller`](#SpaceCenter.Part.robotic_controller "SpaceCenter.Part.robotic_controller") if the part is a robotic controller,
        otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`RoboticController`](#SpaceCenter.RoboticController "SpaceCenter.RoboticController") or None

    robotic\_hinge
    :   A [`Part.robotic_hinge`](#SpaceCenter.Part.robotic_hinge "SpaceCenter.Part.robotic_hinge") if the part is a robotic hinge, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`RoboticHinge`](#SpaceCenter.RoboticHinge "SpaceCenter.RoboticHinge") or None

    robotic\_piston
    :   A [`Part.robotic_piston`](#SpaceCenter.Part.robotic_piston "SpaceCenter.Part.robotic_piston") if the part is a robotic piston, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`RoboticPiston`](#SpaceCenter.RoboticPiston "SpaceCenter.RoboticPiston") or None

    robotic\_rotation
    :   A [`Part.robotic_rotation`](#SpaceCenter.Part.robotic_rotation "SpaceCenter.Part.robotic_rotation") if the part is a robotic rotation servo, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`RoboticRotation`](#SpaceCenter.RoboticRotation "SpaceCenter.RoboticRotation") or None

    robotic\_rotor
    :   A [`Part.robotic_rotor`](#SpaceCenter.Part.robotic_rotor "SpaceCenter.Part.robotic_rotor") if the part is a robotic rotor, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`RoboticRotor`](#SpaceCenter.RoboticRotor "SpaceCenter.RoboticRotor") or None

    sensor
    :   A [`Part.sensor`](#SpaceCenter.Part.sensor "SpaceCenter.Part.sensor") if the part is a sensor, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Sensor`](#SpaceCenter.Sensor "SpaceCenter.Sensor") or None

    solar\_panel
    :   A [`Part.solar_panel`](#SpaceCenter.Part.solar_panel "SpaceCenter.Part.solar_panel") if the part is a solar panel, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`SolarPanel`](#SpaceCenter.SolarPanel "SpaceCenter.SolarPanel") or None

    wheel
    :   A [`Part.wheel`](#SpaceCenter.Part.wheel "SpaceCenter.Part.wheel") if the part is a wheel, otherwise `None`.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Wheel`](#SpaceCenter.Wheel "SpaceCenter.Wheel") or None

    position(*reference\_frame*)
    :   The position of the part in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned position vector is in.

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

        > **Note**
        >
        > This is a fixed position in the part, defined by the parts model.
        > It s not necessarily the same as the parts center of mass.
        > Use [`Part.center_of_mass()`](#SpaceCenter.Part.center_of_mass "SpaceCenter.Part.center_of_mass") to get the parts center of mass.

    center\_of\_mass(*reference\_frame*)
    :   The position of the parts center of mass in the given reference frame.
        If the part is physicsless in flight, this is equivalent to [`Part.position()`](#SpaceCenter.Part.position "SpaceCenter.Part.position").

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned position vector is in.

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    bounding\_box(*reference\_frame*)
    :   The axis-aligned bounding box of the part in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned position vectors are in.

        Returns:
        :   The positions of the minimum and maximum vertices of the box.

        Return type:
        :   [`BoundingBox`](./vessel.md#SpaceCenter.BoundingBox "SpaceCenter.BoundingBox")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

        > **Note**
        >
        > This is computed from the meshes that make up the parts model, and covers only
        > this part - the models of any child parts are excluded.
        > If the part has no visible model, the box has zero volume and is centered on
        > the [`Part.center_of_mass()`](#SpaceCenter.Part.center_of_mass "SpaceCenter.Part.center_of_mass") of the part.

    direction(*reference\_frame*)
    :   The direction the part points in, in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned direction is in.

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    velocity(*reference\_frame*)
    :   The linear velocity of the part in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned velocity vector is in.

        Returns:
        :   The velocity as a vector. The vector points in the direction of travel, and its magnitude is the speed of the body in meters per second.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    rotation(*reference\_frame*)
    :   The rotation of the part, in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned rotation is in.

        Returns:
        :   The rotation as a quaternion of the form \((x, y, z, w)\).

        Return type:
        :   tuple(float, float, float, float)

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    lift(*reference\_frame*)
    :   The aerodynamic [lift](https://en.wikipedia.org/wiki/Aerodynamic_force)
        currently acting on the part.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned vector is in.

        Returns:
        :   A vector pointing in the direction that the force acts, with its magnitude equal to the strength of the force in Newtons.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Not available when the Ferram Aerospace Research mod is installed.

    drag(*reference\_frame*)
    :   The aerodynamic [drag](https://en.wikipedia.org/wiki/Aerodynamic_force)
        currently acting on the part.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned vector is in.

        Returns:
        :   A vector pointing in the direction that the force acts, with its magnitude equal to the strength of the force in Newtons.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Not available when the Ferram Aerospace Research mod is installed.

    center\_of\_buoyancy(*reference\_frame*)
    :   The point the buoyant force on the part acts at, in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned position vector is in.

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

        > **Note**
        >
        > A submerged part reports the point the game applied the force at, which it shifts
        > towards the submerged part of the hull. Any other part reports its configured
        > center of buoyancy.

    center\_of\_displacement(*reference\_frame*)
    :   The point the depth of the part is measured at, in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned position vector is in.

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    buoyant\_force(*reference\_frame*)
    :   The buoyant force currently acting on the part.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned vector is in.

        Returns:
        :   A vector pointing in the direction that the force acts, with its magnitude equal to the strength of the force in Newtons.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The buoyant force on its own. The water drag and lift the game applies alongside
        > it are reported by [`Part.submerged_drag_multiplier`](#SpaceCenter.Part.submerged_drag_multiplier "SpaceCenter.Part.submerged_drag_multiplier") and
        > [`Part.submerged_lift_multiplier`](#SpaceCenter.Part.submerged_lift_multiplier "SpaceCenter.Part.submerged_lift_multiplier").

    moment\_of\_inertia
    :   The moment of inertia of the part in \(kg.m^2\) around its center of mass
        in the parts center of mass reference frame ([`Part.center_of_mass_reference_frame`](#SpaceCenter.Part.center_of_mass_reference_frame "SpaceCenter.Part.center_of_mass_reference_frame")).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    inertia\_tensor
    :   The inertia tensor of the part around its center of mass, in the parts center of
        mass reference frame ([`Part.center_of_mass_reference_frame`](#SpaceCenter.Part.center_of_mass_reference_frame "SpaceCenter.Part.center_of_mass_reference_frame")).
        Returns the 3x3 matrix as a list of elements, in row-major order.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list(float)

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    reference\_frame
    :   The reference frame that is fixed relative to this part, and centered on a fixed
        position within the part, defined by the parts model.

        - The origin is at the position of the part, as returned by
            [`Part.position()`](#SpaceCenter.Part.position "SpaceCenter.Part.position").
        - The axes rotate with the part.
        - The x, y and z axis directions depend on the design of the part.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

        > **Note**
        >
        > For docking port parts, this reference frame is not necessarily equivalent to the
        > reference frame for the docking port, returned by
        > [`DockingPort.reference_frame`](#SpaceCenter.DockingPort.reference_frame "SpaceCenter.DockingPort.reference_frame").

        ![../../../_images/part.png](../../../_images/part.png)

        Mk1 Command Pod reference frame origin and axes

    center\_of\_mass\_reference\_frame
    :   The reference frame that is fixed relative to this part, and centered on its
        center of mass.

        - The origin is at the center of mass of the part, as returned by
            [`Part.center_of_mass()`](#SpaceCenter.Part.center_of_mass "SpaceCenter.Part.center_of_mass").
        - The axes rotate with the part.
        - The x, y and z axis directions depend on the design of the part.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

        > **Note**
        >
        > For docking port parts, this reference frame is not necessarily equivalent to the
        > reference frame for the docking port, returned by
        > [`DockingPort.reference_frame`](#SpaceCenter.DockingPort.reference_frame "SpaceCenter.DockingPort.reference_frame").

    add\_force(*force*, *position*, *reference\_frame*)
    :   > **Warning**
        >
        > Deprecated. Use `Debug.AddForce` instead.

        Exert a constant force on the part, acting at the given position.

        Parameters:
        :   - **force** (*tuple*) – A vector pointing in the direction that the force acts, with its magnitude equal to the strength of the force in Newtons.
            - **position** (*tuple*) – The position at which the force acts, as a vector.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the force and position are in.

        Returns:
        :   An object that can be used to remove or modify the force.

        Return type:
        :   [`Force`](#SpaceCenter.Force "SpaceCenter.Force")

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The force is removed when the client that added it disconnects.

    instantaneous\_force(*force*, *position*, *reference\_frame*)
    :   > **Warning**
        >
        > Deprecated. Use `Debug.InstantaneousForce` instead.

        Exert an instantaneous force on the part, acting at the given position.

        Parameters:
        :   - **force** (*tuple*) – A vector pointing in the direction that the force acts, with its magnitude equal to the strength of the force in Newtons.
            - **position** (*tuple*) – The position at which the force acts, as a vector.
            - **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the force and position are in.

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The force is applied instantaneously in a single physics update.

    glow
    :   Whether the part is glowing.

        Attribute:
        :   Write-only, cannot be read

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    auto\_strut\_mode
    :   Auto-strut mode.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`AutoStrutMode`](#SpaceCenter.AutoStrutMode "SpaceCenter.AutoStrutMode")

    has\_real\_fuels\_tank
    :   Whether the part is a fuel tank managed by
        [RealFuels](https://forum.kerbalspaceprogram.com/topic/58236-18-real-fuels/).
        The remaining properties in this section are only available when it is.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    highly\_pressurized
    :   Whether the tank is highly pressurized, which is what a pressure-fed engine needs to
        draw from (see [`Engine.pressure_fed`](#SpaceCenter.Engine.pressure_fed "SpaceCenter.Engine.pressure_fed")). Throws an exception if the part is
        not a RealFuels fuel tank (see [`Part.has_real_fuels_tank`](#SpaceCenter.Part.has_real_fuels_tank "SpaceCenter.Part.has_real_fuels_tank")).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    boiloff\_rate
    :   The rate at which the tank’s cryogenic contents are boiling off, in kilograms per
        second of in-game time, measured over the last update RealFuels computed it in.
        The contents keep boiling off under time warp, where that update covers a much
        longer span of in-game time than a physics frame does. The rate is zero for a
        tank holding nothing cryogenic, for one still being fed by a launch clamp, whose
        temperature RealFuels holds down instead, and in the editor. Throws an exception
        if the part is not a RealFuels fuel tank (see [`Part.has_real_fuels_tank`](#SpaceCenter.Part.has_real_fuels_tank "SpaceCenter.Part.has_real_fuels_tank")).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

class AutoStrutMode
:   The state of an auto-strut. [`Part.auto_strut_mode`](#SpaceCenter.Part.auto_strut_mode "SpaceCenter.Part.auto_strut_mode")

    off
    :   Off

    root
    :   Root

    heaviest
    :   Heaviest

    grandparent
    :   Grandparent

    force\_root
    :   ForceRoot

    force\_heaviest
    :   ForceHeaviest

    force\_grandparent
    :   ForceGrandparent

class Force
:   Obtained by calling `Debug.AddForce`.

    part
    :   The part that this force is applied to.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight

    force\_vector
    :   The force vector, in Newtons.

        Attribute:
        :   Can be read or written

        Returns:
        :   A vector pointing in the direction that the force acts, with its magnitude equal to the strength of the force in Newtons.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    position
    :   The position at which the force acts, in reference frame [`Force.reference_frame`](#SpaceCenter.Force.reference_frame "SpaceCenter.Force.reference_frame").

        Attribute:
        :   Can be read or written

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    reference\_frame
    :   The reference frame of the force vector and position.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Game Scenes:
        :   Flight

    remove()
    :   Remove the force.

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Any further use of this object throws an exception.

## [Module](#id24)

class Module
:   This can be used to interact with a specific part module. This includes part modules in
    stock KSP, and those added by mods.

    In KSP, each part has zero or more
    [PartModules](https://wiki.kerbalspaceprogram.com/wiki/CFG_File_Documentation#MODULES)
    associated with it. Each one contains some of the functionality of the part.
    For example, an engine has a “ModuleEngines” part module that contains all the
    functionality of an engine.

    name
    :   Name of the PartModule. For example, “ModuleEngines”.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    part
    :   The part that contains this module.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

    config
    :   The static configuration of the module, as found in the part’s
        [cfg file](https://wiki.kerbalspaceprogram.com/wiki/CFG_File_Documentation#MODULES).
        This provides access to data that is not exposed as a field, such as the
        resources produced by a generator. Returns `None` if the module’s
        configuration node cannot be found.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ConfigNode`](#SpaceCenter.ConfigNode "SpaceCenter.ConfigNode")

    field\_list
    :   A list of all the fields of the module, including those not visible in the right-click
        menu of the part. Filter by [`PartField.visible`](#SpaceCenter.PartField.visible "SpaceCenter.PartField.visible") to get just the visible ones.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`PartField`](#SpaceCenter.PartField "SpaceCenter.PartField"))

    event\_list
    :   A list of all the events of the module, including those not currently visible or active.
        Events are the clickable buttons visible in the right-click menu of the part. Filter by
        [`PartEvent.visible`](#SpaceCenter.PartEvent.visible "SpaceCenter.PartEvent.visible") and [`PartEvent.active`](#SpaceCenter.PartEvent.active "SpaceCenter.PartEvent.active") to get just the ones
        shown in the menu.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`PartEvent`](#SpaceCenter.PartEvent "SpaceCenter.PartEvent"))

    action\_list
    :   A list of all the actions of the module. These are the parts actions that can be assigned
        to action groups in the in-game editor.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`PartAction`](#SpaceCenter.PartAction "SpaceCenter.PartAction"))

class PartField
:   A field of a part module. Obtained by calling [`Module.field_list`](#SpaceCenter.Module.field_list "SpaceCenter.Module.field_list").

    module
    :   The part module that contains this field.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Module`](#SpaceCenter.Module "SpaceCenter.Module")

    name
    :   The identifier of the field. This is stable and does not change between game versions,
        unlike [`PartField.gui_name`](#SpaceCenter.PartField.gui_name "SpaceCenter.PartField.gui_name").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    gui\_name
    :   The name of the field, as displayed in the right-click menu of the part. This may be
        empty for fields that are not visible in the menu.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    visible
    :   Whether the field is visible in the right-click menu of the part, in the current scene
        (flight or editor).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    type
    :   The type of the field.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`FieldType`](#SpaceCenter.FieldType "SpaceCenter.FieldType")

    value
    :   The value of the field, as a string. This works for fields of any type, and returns the
        same string that is shown in the right-click menu of the part.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        > **Note**
        >
        > Setting the value using this property is only permitted for string fields. Use the typed
        > properties ([`PartField.bool_value`](#SpaceCenter.PartField.bool_value "SpaceCenter.PartField.bool_value"), [`PartField.int_value`](#SpaceCenter.PartField.int_value "SpaceCenter.PartField.int_value"), [`PartField.float_value`](#SpaceCenter.PartField.float_value "SpaceCenter.PartField.float_value"),
        > [`PartField.double_value`](#SpaceCenter.PartField.double_value "SpaceCenter.PartField.double_value")) to set fields of other types.

    bool\_value
    :   The value of a boolean field.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        > **Note**
        >
        > The getter throws an exception if the field is not a boolean field
        > (see [`PartField.type`](#SpaceCenter.PartField.type "SpaceCenter.PartField.type")).

    int\_value
    :   The value of an integer field.

        Attribute:
        :   Can be read or written

        Return type:
        :   int

        > **Note**
        >
        > The getter throws an exception if the field is not an integer field
        > (see [`PartField.type`](#SpaceCenter.PartField.type "SpaceCenter.PartField.type")).

    float\_value
    :   The value of a single precision floating point field.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        > **Note**
        >
        > The getter throws an exception if the field is not a single precision floating point
        > field (see [`PartField.type`](#SpaceCenter.PartField.type "SpaceCenter.PartField.type")).

    double\_value
    :   The value of a double precision floating point field.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        > **Note**
        >
        > The getter throws an exception if the field is not a double precision floating point
        > field (see [`PartField.type`](#SpaceCenter.PartField.type "SpaceCenter.PartField.type")).

    reset()
    :   Set the value of the field to its original value.

        > **Note**
        >
        > The original value is the value the field had when the part was loaded.
        > Works for any field, including those not visible in the right-click menu.

class FieldType
:   The type of a part module field. See [`PartField.type`](#SpaceCenter.PartField.type "SpaceCenter.PartField.type").

    boolean
    :   A boolean field. Access using [`PartField.bool_value`](#SpaceCenter.PartField.bool_value "SpaceCenter.PartField.bool_value").

    integer
    :   An integer field. Access using [`PartField.int_value`](#SpaceCenter.PartField.int_value "SpaceCenter.PartField.int_value").

    float
    :   A single precision floating point field. Access using [`PartField.float_value`](#SpaceCenter.PartField.float_value "SpaceCenter.PartField.float_value").

    double
    :   A double precision floating point field. Access using [`PartField.double_value`](#SpaceCenter.PartField.double_value "SpaceCenter.PartField.double_value").

    string
    :   A string field. Access using [`PartField.value`](#SpaceCenter.PartField.value "SpaceCenter.PartField.value").

    unknown
    :   A field whose type is not one of the above. Its value can still be read as a
        string using [`PartField.value`](#SpaceCenter.PartField.value "SpaceCenter.PartField.value").

class PartEvent
:   An event of a part module. Events are the clickable buttons visible in the right-click menu
    of the part. Obtained by calling [`Module.event_list`](#SpaceCenter.Module.event_list "SpaceCenter.Module.event_list").

    module
    :   The part module that contains this event.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Module`](#SpaceCenter.Module "SpaceCenter.Module")

    name
    :   The identifier of the event. This is stable and does not change between game versions,
        unlike [`PartEvent.gui_name`](#SpaceCenter.PartEvent.gui_name "SpaceCenter.PartEvent.gui_name").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    gui\_name
    :   The name of the event, as displayed in the right-click menu of the part.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    visible
    :   Whether the event is visible in the right-click menu of the part, in the current scene
        (flight or editor).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    active
    :   Whether the event is currently active.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    trigger()
    :   Trigger the event. Equivalent to clicking the button in the right-click menu of the part.

class PartAction
:   An action of a part module. These are the part actions that can be assigned to action groups
    in the in-game editor. Obtained by calling [`Module.action_list`](#SpaceCenter.Module.action_list "SpaceCenter.Module.action_list").

    module
    :   The part module that contains this action.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Module`](#SpaceCenter.Module "SpaceCenter.Module")

    name
    :   The identifier of the action. This is stable and does not change between game versions,
        unlike [`PartAction.gui_name`](#SpaceCenter.PartAction.gui_name "SpaceCenter.PartAction.gui_name").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    gui\_name
    :   The name of the action, as displayed in the in-game editor.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    activated
    :   Activate or deactivate the action. Equivalent to triggering the action from an action
        group. Set to `True` to activate the action, or `False` to deactivate it.

        Attribute:
        :   Write-only, cannot be read

        Return type:
        :   bool

## [Config Node](#id25)

class ConfigNode
:   Represents a configuration node, as found in a part’s configuration file. A node has a
    name, a set of named values and a set of child nodes. This is used to access the
    static configuration of a part or part module, for example via
    [`Part.config`](#SpaceCenter.Part.config "SpaceCenter.Part.config") and [`Module.config`](#SpaceCenter.Module.config "SpaceCenter.Module.config").

    name
    :   The name of the configuration node. For example “MODULE”.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    values
    :   The values stored in the node, as a dictionary mapping names to values.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   dict(str, str)

        > **Note**
        >
        > If a name appears more than once, only the first value is included. Use
        > [`ConfigNode.get_values()`](#SpaceCenter.ConfigNode.get_values "SpaceCenter.ConfigNode.get_values") to get all of the values with a given name.

    has\_value(*name*)
    :   Returns `True` if the node has a value with the given name.

        Parameters:
        :   **name** (*str*) – Name of the value.

        Return type:
        :   bool

    get\_value(*name*)
    :   Returns the value with the given name. If there is more than one value with
        the given name, the first is returned. Throws an exception if there is no
        value with the given name.

        Parameters:
        :   **name** (*str*) – Name of the value.

        Return type:
        :   str

    get\_values(*name*)
    :   Returns all of the values with the given name, as a list.

        Parameters:
        :   **name** (*str*) – Name of the values.

        Return type:
        :   list(str)

    nodes
    :   The child nodes contained in this node.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`ConfigNode`](#SpaceCenter.ConfigNode "SpaceCenter.ConfigNode"))

    has\_node(*name*)
    :   Returns `True` if the node has a child node with the given name.

        Parameters:
        :   **name** (*str*) – Name of the child node.

        Return type:
        :   bool

    get\_node(*name*)
    :   Returns the child node with the given name. If there is more than one child
        node with the given name, the first is returned. Throws an exception if there
        is no child node with the given name.

        Parameters:
        :   **name** (*str*) – Name of the child node.

        Return type:
        :   [`ConfigNode`](#SpaceCenter.ConfigNode "SpaceCenter.ConfigNode")

    get\_nodes(*name*)
    :   Returns all of the child nodes with the given name, as a list.

        Parameters:
        :   **name** (*str*) – Name of the child nodes.

        Return type:
        :   list([`ConfigNode`](#SpaceCenter.ConfigNode "SpaceCenter.ConfigNode"))

## [Specific Types of Part](#id26)

The following classes provide functionality for specific types of part.

- [Antenna](#antenna)
- [Cargo Bay](#cargo-bay)
- [Control Surface](#control-surface)
- [Decoupler](#decoupler)
- [Deployable State](#deployable-state)
- [Docking Port](#docking-port)
- [Engine](#engine)
- [Experiment](#experiment)
- [Fairing](#fairing)
- [Intake](#intake)
- [Leg](#leg)
- [Launch Clamp](#launch-clamp)
- [Light](#light)
- [Parachute](#parachute)
- [Radiator](#radiator)
- [Resource Converter](#resource-converter)
- [Resource Harvester](#resource-harvester)
- [Reaction Wheel](#reaction-wheel)
- [Resource Drain](#resource-drain)
- [Robotic Controller](#robotic-controller)
- [Robotic Hinge](#robotic-hinge)
- [Robotic Piston](#robotic-piston)
- [Robotic Rotation](#robotic-rotation)
- [Robotic Rotor](#robotic-rotor)
- [RCS](#rcs)
- [Sensor](#sensor)
- [Solar Panel](#solar-panel)
- [Thruster](#thruster)
- [Wheel](#wheel)

### [Antenna](#id61)

> **Note**
>
> If RemoteTech is installed, use the RemoteTech service APIs to interact with antennas.
> This class is only for stock KSP antennas.

class Antenna
:   An antenna. Obtained by calling [`Part.antenna`](#SpaceCenter.Part.antenna "SpaceCenter.Part.antenna").

    part
    :   The part object for this antenna.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    state
    :   The current state of the antenna.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`DeployableState`](#SpaceCenter.DeployableState "SpaceCenter.DeployableState")

        Game Scenes:
        :   Flight

    deployable
    :   Whether the antenna is deployable.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    deployed
    :   Whether the antenna is deployed.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Fixed antennas are always deployed.
        > Returns an error if you try to deploy a fixed antenna.

    can\_transmit
    :   Whether data can be transmitted by this antenna.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    transmit()
    :   Transmit data.

        Game Scenes:
        :   Flight

    cancel()
    :   Cancel current transmission of data.

        Game Scenes:
        :   Flight

    allow\_partial
    :   Whether partial data transmission is permitted.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    power
    :   The power of the antenna.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    combinable
    :   Whether the antenna can be combined with other antennae on the vessel
        to boost the power.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    combinable\_exponent
    :   Exponent used to calculate the combined power of multiple antennae on a vessel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    packet\_interval
    :   Interval between sending packets in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    packet\_size
    :   Amount of data sent per packet in Mits.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    packet\_resource\_cost
    :   Units of electric charge consumed per packet sent.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

### [Cargo Bay](#id62)

class CargoBay
:   A cargo bay. Obtained by calling [`Part.cargo_bay`](#SpaceCenter.Part.cargo_bay "SpaceCenter.Part.cargo_bay").

    part
    :   The part object for this cargo bay.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    state
    :   The state of the cargo bay.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`DeployableState`](#SpaceCenter.DeployableState "SpaceCenter.DeployableState")

        Game Scenes:
        :   Flight

        > **Note**
        >
        > This describes where the bay’s doors are, which is not the same as whether the parts
        > inside are sheltered: a bay whose open end has nothing attached to it never shelters
        > anything, however tightly shut it is. Use [`Part.shielded`](#SpaceCenter.Part.shielded "SpaceCenter.Part.shielded") for that.
        >
        > A cargo bay is never [`DeployableState.broken`](#SpaceCenter.DeployableState.broken "SpaceCenter.DeployableState.broken"), as the game
        > does not track damage for them.

    open
    :   Whether the cargo bay is open.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

### [Control Surface](#id63)

class ControlSurface
:   An aerodynamic control surface. Obtained by calling [`Part.control_surface`](#SpaceCenter.Part.control_surface "SpaceCenter.Part.control_surface").

    part
    :   The part object for this control surface.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    pitch\_enabled
    :   Whether the control surface has pitch control enabled.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    yaw\_enabled
    :   Whether the control surface has yaw control enabled.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    roll\_enabled
    :   Whether the control surface has roll control enabled.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    authority\_limiter
    :   The authority limiter for the control surface, which controls how far the
        control surface will move.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    inverted
    :   Whether the control surface movement is inverted.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    deployed
    :   Whether the control surface has been fully deployed. Throws an exception if set
        while [`ControlSurface.deflection_override`](#SpaceCenter.ControlSurface.deflection_override "SpaceCenter.ControlSurface.deflection_override") is enabled, which holds the surface deployed.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    deploy\_angle
    :   The deploy angle of the control surface, in degrees.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

        > **Note**
        >
        > Applied when [`ControlSurface.deployed`](#SpaceCenter.ControlSurface.deployed "SpaceCenter.ControlSurface.deployed") is true, and clamped to the range given by
        > [`ControlSurface.min_deploy_angle`](#SpaceCenter.ControlSurface.min_deploy_angle "SpaceCenter.ControlSurface.min_deploy_angle") and [`ControlSurface.max_deploy_angle`](#SpaceCenter.ControlSurface.max_deploy_angle "SpaceCenter.ControlSurface.max_deploy_angle"). While
        > [`ControlSurface.deflection_override`](#SpaceCenter.ControlSurface.deflection_override "SpaceCenter.ControlSurface.deflection_override") is enabled, [`ControlSurface.deflection`](#SpaceCenter.ControlSurface.deflection "SpaceCenter.ControlSurface.deflection") drives the
        > angle. A value set here takes effect when the override is released.

    min\_deploy\_angle
    :   The minimum [`ControlSurface.deploy_angle`](#SpaceCenter.ControlSurface.deploy_angle "SpaceCenter.ControlSurface.deploy_angle") of the control surface, in degrees.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    max\_deploy\_angle
    :   The maximum [`ControlSurface.deploy_angle`](#SpaceCenter.ControlSurface.deploy_angle "SpaceCenter.ControlSurface.deploy_angle") of the control surface, in degrees.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    deflection\_override
    :   Whether the control surface deflection is being set directly, bypassing the vessel’s
        normal flight control. When enabled, the surface holds the deflection set by
        [`ControlSurface.deflection`](#SpaceCenter.ControlSurface.deflection "SpaceCenter.ControlSurface.deflection") instead of responding to pitch, yaw and roll control inputs.
        The prior state is restored when the override is released, when the
        controlling client disconnects, or when the vessel changes.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    deflection
    :   The deflection command applied when [`ControlSurface.deflection_override`](#SpaceCenter.ControlSurface.deflection_override "SpaceCenter.ControlSurface.deflection_override") is enabled, as a
        value between -1 and 1, mapped onto the surface’s deploy angle range.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    surface\_area
    :   Surface area of the control surface in \(m^2\).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    available\_torque
    :   The available torque, in Newton meters, that can be produced by this control surface,
        in the positive and negative pitch, roll and yaw axes of the vessel. These axes
        correspond to the coordinate axes of the [`Vessel.reference_frame`](./vessel.md#SpaceCenter.Vessel.reference_frame "SpaceCenter.Vessel.reference_frame").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`AxisRange`](./vessel.md#SpaceCenter.AxisRange "SpaceCenter.AxisRange")

        Game Scenes:
        :   Flight

### [Decoupler](#id64)

class Decoupler
:   A decoupler. Obtained by calling [`Part.decoupler`](#SpaceCenter.Part.decoupler "SpaceCenter.Part.decoupler")

    part
    :   The part object for this decoupler.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    decouple()
    :   Fires the decoupler. Returns the new vessel created when the decoupler fires.
        Throws an exception if the decoupler has already fired.

        Return type:
        :   [`Vessel`](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")

        Game Scenes:
        :   Flight

        > **Note**
        >
        > When called, the active vessel may change. It is therefore possible that,
        > after calling this function, the object(s) returned by previous call(s) to
        > [`active_vessel`](./space-center.md#SpaceCenter.active_vessel "SpaceCenter.active_vessel") no longer refer to the active vessel.

    decoupled
    :   Whether the decoupler has fired.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    staged
    :   Whether the decoupler is enabled in the staging sequence.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    impulse
    :   The impulse that the decoupler imparts when it is fired, in Newton seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    is\_omni\_decoupler
    :   Whether the decoupler is an omni-decoupler (e.g. stack separator)

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    attached\_part
    :   The part attached to this decoupler’s explosive node.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part") or None

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

### [Deployable State](#id65)

The deployment state shared by antennas, cargo bays, landing legs, radiators,
resource harvesters, solar panels and wheels.

class DeployableState
:   The state of a deployable part.
    [`Antenna.state`](#SpaceCenter.Antenna.state "SpaceCenter.Antenna.state"), [`CargoBay.state`](#SpaceCenter.CargoBay.state "SpaceCenter.CargoBay.state"),
    [`Leg.state`](#SpaceCenter.Leg.state "SpaceCenter.Leg.state"), [`Radiator.state`](#SpaceCenter.Radiator.state "SpaceCenter.Radiator.state"),
    [`ResourceHarvester.state`](#SpaceCenter.ResourceHarvester.state "SpaceCenter.ResourceHarvester.state"), [`SolarPanel.state`](#SpaceCenter.SolarPanel.state "SpaceCenter.SolarPanel.state"),
    [`Wheel.state`](#SpaceCenter.Wheel.state "SpaceCenter.Wheel.state")

    deployed
    :   The part is fully deployed. A cargo bay in this state is fully open.
        Parts that cannot be retracted, such as fixed radiators, solar panels,
        antennas, landing legs and wheels, are always in this state.

    retracted
    :   The part is fully retracted. A cargo bay in this state is closed and locked.

    deploying
    :   The part is being deployed. A cargo bay in this state is opening.

    retracting
    :   The part is being retracted. A cargo bay in this state is closing.

    broken
    :   The part is broken. Cargo bays and resource harvesters never report
        this state, as the game does not track damage for them.

### [Docking Port](#id66)

class DockingPort
:   A docking port. Obtained by calling [`Part.docking_port`](#SpaceCenter.Part.docking_port "SpaceCenter.Part.docking_port")

    part
    :   The part object for this docking port.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    state
    :   The current state of the docking port.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`DockingPortState`](#SpaceCenter.DockingPortState "SpaceCenter.DockingPortState")

        Game Scenes:
        :   Flight

    docked\_part
    :   The part that this docking port is docked to. Returns `None` if this
        docking port is not docked to anything.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part") or None

        Game Scenes:
        :   Flight

    undock()
    :   Undocks the docking port and returns the new [`Vessel`](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel") that is created.
        This method can be called for either docking port in a docked pair.
        Throws an exception if the docking port is not docked to anything.

        Return type:
        :   [`Vessel`](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel")

        Game Scenes:
        :   Flight

        > **Note**
        >
        > When called, the active vessel may change. It is therefore possible that,
        > after calling this function, the object(s) returned by previous call(s) to
        > [`active_vessel`](./space-center.md#SpaceCenter.active_vessel "SpaceCenter.active_vessel") no longer refer to the active vessel.

    reengage\_distance
    :   The distance a docking port must move away when it undocks before it
        becomes ready to dock with another port, in meters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    has\_shield
    :   Whether the docking port has a shield.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    shielded
    :   The state of the docking ports shield, if it has one.

        Returns `True` if the docking port has a shield, and the shield is
        closed. Otherwise returns `False`. When set to `True`, the shield is
        closed, and when set to `False` the shield is opened. If the docking
        port does not have a shield, setting this attribute has no effect.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    can\_rotate
    :   Whether the docking port can be commanded to rotate while docked.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    maximum\_rotation
    :   Maximum rotation angle in degrees.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    minimum\_rotation
    :   Minimum rotation angle in degrees.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    rotation\_target
    :   Rotation target angle in degrees.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    rotation\_locked
    :   Lock rotation. When locked, allows auto-strut to work across the joint.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    position(*reference\_frame*)
    :   The position of the docking port, in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned position vector is in.

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    direction(*reference\_frame*)
    :   The direction that docking port points in, in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned direction is in.

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    rotation(*reference\_frame*)
    :   The rotation of the docking port, in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned rotation is in.

        Returns:
        :   The rotation as a quaternion of the form \((x, y, z, w)\).

        Return type:
        :   tuple(float, float, float, float)

        Game Scenes:
        :   Flight

    reference\_frame
    :   The reference frame that is fixed relative to this docking port, and
        oriented with the port.

        - The origin is at the position of the docking port.
        - The axes rotate with the docking port.
        - The x-axis points out to the right side of the docking port.
        - The y-axis points in the direction the docking port is facing.
        - The z-axis points out of the bottom off the docking port.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Game Scenes:
        :   Flight

        > **Note**
        >
        > This reference frame is not necessarily equivalent to the reference frame
        > for the part, returned by [`Part.reference_frame`](#SpaceCenter.Part.reference_frame "SpaceCenter.Part.reference_frame").

        ![../../../_images/docking-port.png](../../../_images/docking-port.png)

        Docking port reference frame origin and axes

        ![../../../_images/docking-port-inline.png](../../../_images/docking-port-inline.png)

        Inline docking port reference frame origin and axes

class DockingPortState
:   The state of a docking port. See [`DockingPort.state`](#SpaceCenter.DockingPort.state "SpaceCenter.DockingPort.state").

    ready
    :   The docking port is ready to dock to another docking port.

    docked
    :   The docking port is docked to another docking port, or docked to
        another part (from the VAB/SPH).

    docking
    :   The docking port is very close to another docking port,
        but has not docked. It is using magnetic force to acquire a solid dock.

    undocking
    :   The docking port has just been undocked from another docking port,
        and is disabled until it moves away by a sufficient distance
        ([`DockingPort.reengage_distance`](#SpaceCenter.DockingPort.reengage_distance "SpaceCenter.DockingPort.reengage_distance")).

    shielded
    :   The docking port has a shield, and the shield is closed.

    moving
    :   The docking ports shield is currently opening/closing.

### [Engine](#id67)

class Engine
:   An engine, including ones of various types.
    For example liquid fuelled gimballed engines, solid rocket boosters and jet engines.
    Obtained by calling [`Part.engine`](#SpaceCenter.Part.engine "SpaceCenter.Part.engine").

    > **Note**
    >
    > For RCS thrusters [`Part.rcs`](#SpaceCenter.Part.rcs "SpaceCenter.Part.rcs").

    part
    :   The part object for this engine.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    active
    :   Whether the engine is active. Setting this attribute may have no effect,
        depending on [`Engine.can_shutdown`](#SpaceCenter.Engine.can_shutdown "SpaceCenter.Engine.can_shutdown") and [`Engine.can_restart`](#SpaceCenter.Engine.can_restart "SpaceCenter.Engine.can_restart").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    thrust
    :   The current amount of thrust being produced by the engine, in Newtons.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    available\_thrust
    :   The amount of thrust, in Newtons, that would be produced by the engine
        when activated and with its throttle set to 100%.
        Returns zero if the engine does not have any fuel.
        Takes the engine’s current [`Engine.thrust_limit`](#SpaceCenter.Engine.thrust_limit "SpaceCenter.Engine.thrust_limit") and atmospheric conditions
        into account.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    available\_thrust\_at(*pressure*)
    :   The amount of thrust, in Newtons, that would be produced by the engine
        when activated and with its throttle set to 100%.
        Returns zero if the engine does not have any fuel.
        Takes the given pressure into account.

        Parameters:
        :   **pressure** (*float*) – Atmospheric pressure in atmospheres

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_thrust
    :   The amount of thrust, in Newtons, that would be produced by the engine
        when activated and fueled, with its throttle and throttle limiter set to 100%.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_thrust\_at(*pressure*)
    :   The amount of thrust, in Newtons, that would be produced by the engine
        when activated and fueled, with its throttle and throttle limiter set to 100%.
        Takes the given pressure into account.

        Parameters:
        :   **pressure** (*float*) – Atmospheric pressure in atmospheres

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    max\_vacuum\_thrust
    :   The maximum amount of thrust that can be produced by the engine in a
        vacuum, in Newtons. This is the amount of thrust produced by the engine
        when activated, [`Engine.thrust_limit`](#SpaceCenter.Engine.thrust_limit "SpaceCenter.Engine.thrust_limit") is set to 100%, the main
        vessel’s throttle is set to 100% and the engine is in a vacuum.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    thrust\_limit
    :   The thrust limiter of the engine. A value between 0 and 1. Setting this
        attribute may have no effect, for example the thrust limit for a solid
        rocket booster cannot be changed in flight.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    thrusters
    :   The components of the engine that generate thrust.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Thruster`](#SpaceCenter.Thruster "SpaceCenter.Thruster"))

        Game Scenes:
        :   Flight

        > **Note**
        >
        > For example, this corresponds to the rocket nozzel on a solid rocket booster,
        > or the individual nozzels on a RAPIER engine.
        > The overall thrust produced by the engine, as reported by [`Engine.available_thrust`](#SpaceCenter.Engine.available_thrust "SpaceCenter.Engine.available_thrust"),
        > [`Engine.max_thrust`](#SpaceCenter.Engine.max_thrust "SpaceCenter.Engine.max_thrust") and others, is the sum of the thrust generated by each thruster.

    specific\_impulse
    :   The current specific impulse of the engine, in seconds. Returns zero
        if the engine is not active.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    specific\_impulse\_at(*pressure*)
    :   The specific impulse of the engine under the given pressure, in seconds. Returns zero
        if the engine is not active.

        Parameters:
        :   **pressure** (*float*) – Atmospheric pressure in atmospheres

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    vacuum\_specific\_impulse
    :   The vacuum specific impulse of the engine, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    kerbin\_sea\_level\_specific\_impulse
    :   The specific impulse of the engine at sea level on Kerbin, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    propellant\_names
    :   The names of the propellants that the engine consumes.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list(str)

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    propellant\_ratios
    :   The ratio of resources that the engine consumes. A dictionary mapping resource names
        to the ratio at which they are consumed by the engine.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   dict(str, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > For example, if the ratios are 0.6 for LiquidFuel and 0.4 for Oxidizer, then for every
        > 0.6 units of LiquidFuel that the engine burns, it will burn 0.4 units of Oxidizer.

    propellants
    :   The propellants that the engine consumes.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Propellant`](#SpaceCenter.Propellant "SpaceCenter.Propellant"))

        Game Scenes:
        :   Flight

    has\_fuel
    :   Whether the engine has any fuel available.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    flameout
    :   Whether the engine has flamed out - it is active but has run out of
        propellant, so it is producing no thrust. This is the signal a client
        can use to decide when to activate the next stage. Note that on a
        vessel with drop tanks or asparagus staging some engines can flame out
        while others keep burning.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    throttle
    :   The current throttle setting for the engine. A value between 0 and 1.
        This is not necessarily the same as the vessel’s main throttle
        setting, as some engines take time to adjust their throttle
        (such as jet engines), or independent throttle may be enabled.

        When the engine’s independent throttle is enabled
        (see [`Engine.independent_throttle`](#SpaceCenter.Engine.independent_throttle "SpaceCenter.Engine.independent_throttle")), can be used to set the throttle percentage.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    throttle\_locked
    :   Whether the [`Control.throttle`](./control.md#SpaceCenter.Control.throttle "SpaceCenter.Control.throttle") affects the engine. For example,
        this is `True` for liquid fueled rockets, and `False` for solid rocket
        boosters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    independent\_throttle
    :   Whether the independent throttle is enabled for the engine.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    can\_restart
    :   Whether the engine can be restarted once shutdown. If the engine cannot be shutdown,
        returns `False`. For example, this is `True` for liquid fueled rockets
        and `False` for solid rocket boosters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    can\_shutdown
    :   Whether the engine can be shutdown once activated. For example, this is
        `True` for liquid fueled rockets and `False` for solid rocket boosters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    has\_modes
    :   Whether the engine has multiple modes of operation.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    mode
    :   The name of the current engine mode.

        Attribute:
        :   Can be read or written

        Return type:
        :   str

        Game Scenes:
        :   Flight

    modes
    :   The available modes for the engine.
        A dictionary mapping mode names to [`Engine`](#SpaceCenter.Engine "SpaceCenter.Engine") objects.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   dict(str, [`Engine`](#SpaceCenter.Engine "SpaceCenter.Engine"))

        Game Scenes:
        :   Flight

    toggle\_mode()
    :   Toggle the current engine mode.

        Game Scenes:
        :   Flight

    auto\_mode\_switch
    :   Whether the engine will automatically switch modes.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    gimballed
    :   Whether the engine is gimballed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    gimbal\_range
    :   The range over which the gimbal can move, in degrees.
        Returns 0 if the engine is not gimballed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    gimbal\_locked
    :   Whether the engines gimbal is locked in place. Setting this attribute has
        no effect if the engine is not gimballed.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    gimbal\_limit
    :   The gimbal limiter of the engine. A value between 0 and 1.
        Returns 0 if the gimbal is locked.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    gimbal\_override
    :   Whether the gimbal is being controlled directly, bypassing the vessel’s normal
        flight controls. When enabled, the gimbal deflection is set by
        [`Engine.gimbal_actuation`](#SpaceCenter.Engine.gimbal_actuation "SpaceCenter.Engine.gimbal_actuation") instead of the normal control inputs.
        The override is automatically released if the controlling client disconnects or the
        vessel changes. Has no effect if the engine is not gimballed.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    gimbal\_actuation
    :   The gimbal actuation command applied when [`Engine.gimbal_override`](#SpaceCenter.Engine.gimbal_override "SpaceCenter.Engine.gimbal_override") is enabled, in
        the pitch, roll and yaw axes. Each component is a normalized control input between -1
        and 1. The physical deflection is scaled by [`Engine.gimbal_range`](#SpaceCenter.Engine.gimbal_range "SpaceCenter.Engine.gimbal_range")
        and [`Engine.gimbal_limit`](#SpaceCenter.Engine.gimbal_limit "SpaceCenter.Engine.gimbal_limit"). When the gimbal is not being overridden,
        returns the current actuation.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    available\_torque
    :   The available torque, in Newton meters, that can be produced by this engine,
        in the positive and negative pitch, roll and yaw axes of the vessel. These axes
        correspond to the coordinate axes of the [`Vessel.reference_frame`](./vessel.md#SpaceCenter.Vessel.reference_frame "SpaceCenter.Vessel.reference_frame").
        Returns zero if the engine is inactive, or not gimballed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`AxisRange`](./vessel.md#SpaceCenter.AxisRange "SpaceCenter.AxisRange")

        Game Scenes:
        :   Flight

    can\_reverse\_thrust
    :   Whether the engine has a recognized thrust reverser. This is `True` for
        stock engines with a thrust reverser (such as the J-33 “Wheesley” and
        J-90 “Goliath” turbofan engines) and for selected mod engines. It is
        `False` for engines whose reverser is not recognized, as KSP provides no
        standard way to detect one.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    thrust\_reversed
    :   Whether the engine’s thrust reverser is engaged, reversing the
        direction of thrust. While the reverser is still moving, this reports
        the state it is moving to, so a read immediately after a set returns
        the value that was set and setting the same value again has no effect.
        Throws an exception if the engine does not have a thrust reverser
        (see [`Engine.can_reverse_thrust`](#SpaceCenter.Engine.can_reverse_thrust "SpaceCenter.Engine.can_reverse_thrust")).

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    toggle\_thrust\_reversal()
    :   Toggle the engine’s thrust reverser. Throws an exception if the engine
        does not have a thrust reverser (see [`Engine.can_reverse_thrust`](#SpaceCenter.Engine.can_reverse_thrust "SpaceCenter.Engine.can_reverse_thrust")).

        Game Scenes:
        :   Flight

    has\_real\_fuels
    :   Whether the engine is managed by
        [RealFuels](https://forum.kerbalspaceprogram.com/topic/58236-18-real-fuels/).
        The remaining properties in this section are only available when it is.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    ignitions
    :   The number of times the engine can still be ignited, or -1 if it can be ignited an
        unlimited number of times. Throws an exception if the engine is not managed by
        RealFuels (see [`Engine.has_real_fuels`](#SpaceCenter.Engine.has_real_fuels "SpaceCenter.Engine.has_real_fuels")).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   int

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    ignition\_resources
    :   The resources the engine consumes each time it ignites, keyed by resource name, with
        the amount of each that is required. The engine uses up an ignition and fails to
        light if they are not available. Throws an exception if the engine is not managed by
        RealFuels (see [`Engine.has_real_fuels`](#SpaceCenter.Engine.has_real_fuels "SpaceCenter.Engine.has_real_fuels")).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   dict(str, float)

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    has\_ullage
    :   Whether the engine’s propellant is subject to ullage, so that it must be settled
        before the engine will run reliably. Throws an exception if the engine is not
        managed by RealFuels (see [`Engine.has_real_fuels`](#SpaceCenter.Engine.has_real_fuels "SpaceCenter.Engine.has_real_fuels")).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    propellant\_stability
    :   How settled the propellant feeding the engine is, from 0 (fully unsettled) to 1
        (fully settled). Acceleration along the engine’s thrust axis settles it, and
        coasting and rotation unsettle it. Throws an exception if the engine is not managed
        by RealFuels (see [`Engine.has_real_fuels`](#SpaceCenter.Engine.has_real_fuels "SpaceCenter.Engine.has_real_fuels")).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    propellant\_reliability
    :   The probability that the propellant feeding the engine behaves normally, from 0 to 1.
        This is [`Engine.propellant_stability`](#SpaceCenter.Engine.propellant_stability "SpaceCenter.Engine.propellant_stability") scaled by the game’s ullage difficulty
        setting, and is the value RealFuels itself tests against when deciding whether the
        engine runs. Throws an exception if the engine is not managed by RealFuels
        (see [`Engine.has_real_fuels`](#SpaceCenter.Engine.has_real_fuels "SpaceCenter.Engine.has_real_fuels")).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    pressure\_fed
    :   Whether the engine is fed by tank pressure rather than by a pump, and so needs to
        draw from a highly pressurized tank. Throws an exception if the engine is not
        managed by RealFuels (see [`Engine.has_real_fuels`](#SpaceCenter.Engine.has_real_fuels "SpaceCenter.Engine.has_real_fuels")).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    has\_feed\_pressure
    :   Whether the tanks feeding the engine supply the pressure it needs. Always
        `True` for an engine that is not pressure fed. Throws an exception if the
        engine is not managed by RealFuels (see [`Engine.has_real_fuels`](#SpaceCenter.Engine.has_real_fuels "SpaceCenter.Engine.has_real_fuels")).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

class Propellant
:   A propellant for an engine. Obtains by calling [`Engine.propellants`](#SpaceCenter.Engine.propellants "SpaceCenter.Engine.propellants").

    name
    :   The name of the propellant.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

        Game Scenes:
        :   Flight

    current\_amount
    :   The current amount of propellant consumed in the current physics update.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    current\_requirement
    :   The required amount of propellant for the current physics update.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    total\_resource\_available
    :   The total amount of the underlying resource currently reachable given
        resource flow rules.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    total\_resource\_capacity
    :   The total vehicle capacity for the underlying propellant resource,
        restricted by resource flow rules.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    ignore\_for\_isp
    :   If this propellant should be ignored when calculating required mass flow
        given specific impulse.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    ignore\_for\_thrust\_curve
    :   If this propellant should be ignored for thrust curve calculations.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    draw\_stack\_gauge
    :   If this propellant has a stack gauge or not.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    is\_deprived
    :   If this propellant is deprived.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    ratio
    :   The propellant ratio.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

### [Experiment](#id68)

class Experiment
:   Obtained by calling [`Part.experiment`](#SpaceCenter.Part.experiment "SpaceCenter.Part.experiment").

    part
    :   The part object for this experiment.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    name
    :   Internal name of the experiment, as used in
        [part cfg files](https://wiki.kerbalspaceprogram.com/wiki/CFG_File_Documentation).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

        Game Scenes:
        :   Flight

    title
    :   Title of the experiment, as shown on the in-game UI.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

        Game Scenes:
        :   Flight

    run()
    :   Run the experiment.

        Game Scenes:
        :   Flight

    transmit()
    :   Transmit all experimental data contained by this part.

        Game Scenes:
        :   Flight

    dump()
    :   Dump the experimental data contained by the experiment.

        Game Scenes:
        :   Flight

    reset()
    :   Reset the experiment.

        Game Scenes:
        :   Flight

    deployed
    :   Whether the experiment has been deployed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    rerunnable
    :   Whether the experiment can be re-run.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    inoperable
    :   Whether the experiment is inoperable.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    has\_data
    :   Whether the experiment contains data.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    data
    :   The data contained in this experiment.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`ScienceData`](#SpaceCenter.ScienceData "SpaceCenter.ScienceData"))

        Game Scenes:
        :   Flight

    biome
    :   The name of the biome the experiment is currently in.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

        Game Scenes:
        :   Flight

    available
    :   Determines if the experiment is available given the current conditions.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    science\_subject
    :   Containing information on the corresponding specific science result for the current
        conditions. Returns `None` if the experiment is unavailable.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ScienceSubject`](#SpaceCenter.ScienceSubject "SpaceCenter.ScienceSubject") or None

        Game Scenes:
        :   Flight

class ScienceData(*data\_amount*, *science\_value*, *transmit\_value*)
:   Obtained by calling [`Experiment.data`](#SpaceCenter.Experiment.data "SpaceCenter.Experiment.data").

    data\_amount
    :   Data amount.

        Return type:
        :   float

    science\_value
    :   Science value.

        Return type:
        :   float

    transmit\_value
    :   Transmit value.

        Return type:
        :   float

class ScienceSubject(*id*, *title*, *is\_complete*, *science*, *science\_cap*, *data\_scale*, *subject\_value*, *scientific\_value*)
:   Obtained by calling [`Experiment.science_subject`](#SpaceCenter.Experiment.science_subject "SpaceCenter.Experiment.science_subject").

    id
    :   The identifier of the subject, e.g. `crewReport@KerbinSrfLandedLaunchPad`.

        Return type:
        :   str

    title
    :   The title of the subject, as shown in the science archives.

        Return type:
        :   str

    is\_complete
    :   Whether the subject has been fully researched. This is true once the science banked for
        the subject reaches the science cap. As the banked science (see [`ScienceSubject.science`](#SpaceCenter.ScienceSubject.science "SpaceCenter.ScienceSubject.science")) is
        only updated after transmission/recovery, this reflects fully mining the subject over
        repeated experiments, not whether a single run has produced data. To check whether a run
        has produced data, and how valuable it is, use [`Experiment.has_data`](#SpaceCenter.Experiment.has_data "SpaceCenter.Experiment.has_data") and
        [`ScienceData.transmit_value`](#SpaceCenter.ScienceData.transmit_value "SpaceCenter.ScienceData.transmit_value").

        Return type:
        :   bool

    science
    :   Amount of science already earned from this subject, not updated until after
        transmission/recovery.

        Return type:
        :   float

    science\_cap
    :   Total science allowable for this subject.

        Return type:
        :   float

    data\_scale
    :   Multiply science value by this to determine data amount in mits.

        Return type:
        :   float

    subject\_value
    :   Multiplier for specific Celestial Body/Experiment Situation combination.

        Return type:
        :   float

    scientific\_value
    :   Diminishing value multiplier for decreasing the science value returned from repeated
        experiments.

        Return type:
        :   float

### [Fairing](#id69)

class Fairing
:   A fairing. Obtained by calling [`Part.fairing`](#SpaceCenter.Part.fairing "SpaceCenter.Part.fairing").
    Supports both stock fairings, and those from the ProceduralFairings mod.

    part
    :   The part object for this fairing.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    jettison()
    :   Jettison the fairing. Has no effect if it has already been jettisoned.

        Game Scenes:
        :   Flight

    jettisoned
    :   Whether the fairing has been jettisoned.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

### [Intake](#id70)

class Intake
:   An air intake. Obtained by calling [`Part.intake`](#SpaceCenter.Part.intake "SpaceCenter.Part.intake").

    part
    :   The part object for this intake.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    open
    :   Whether the intake is open.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    speed
    :   Speed of the flow into the intake, in \(m/s\).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    flow
    :   The rate of flow into the intake, in units of resource per second.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    area
    :   The area of the intake’s opening, in square meters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

### [Leg](#id71)

class Leg
:   A landing leg. Obtained by calling [`Part.leg`](#SpaceCenter.Part.leg "SpaceCenter.Part.leg").

    part
    :   The part object for this landing leg.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    state
    :   The current state of the landing leg.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`DeployableState`](#SpaceCenter.DeployableState "SpaceCenter.DeployableState")

        Game Scenes:
        :   Flight

    deployable
    :   Whether the leg is deployable.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    deployed
    :   Whether the landing leg is deployed.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Fixed landing legs are always deployed.
        > Returns an error if you try to deploy fixed landing gear.

    is\_grounded
    :   Returns whether the leg is touching the ground.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

### [Launch Clamp](#id72)

class LaunchClamp
:   A launch clamp. Obtained by calling [`Part.launch_clamp`](#SpaceCenter.Part.launch_clamp "SpaceCenter.Part.launch_clamp").

    part
    :   The part object for this launch clamp.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    release()
    :   Releases the docking clamp. Has no effect if the clamp has already been released.

        Game Scenes:
        :   Flight

### [Light](#id73)

class Light
:   A light. Obtained by calling [`Part.light`](#SpaceCenter.Part.light "SpaceCenter.Part.light").

    part
    :   The part object for this light.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    active
    :   Whether the light is switched on.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    color
    :   The color of the light.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`StdLib.ColorRGB`](../stdlib.md#StdLib.ColorRGB "StdLib.ColorRGB")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    blink
    :   Whether blinking is enabled.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    blink\_rate
    :   The blink rate of the light.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    power\_usage
    :   The current power usage, in units of charge per second.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

### [Parachute](#id74)

class Parachute
:   A parachute. Obtained by calling [`Part.parachute`](#SpaceCenter.Part.parachute "SpaceCenter.Part.parachute").

    part
    :   The part object for this parachute.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    deploy()
    :   Deploys the parachute. This has no effect if the parachute has already
        been deployed.

        Game Scenes:
        :   Flight

    deployed
    :   Whether the parachute has been deployed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    arm()
    :   Deploys the parachute. This has no effect if the parachute has already
        been armed or deployed.

        Game Scenes:
        :   Flight

    armed
    :   Whether the parachute has been armed or deployed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    cut()
    :   Cuts the parachute.

        Game Scenes:
        :   Flight

    state
    :   The current state of the parachute.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ParachuteState`](#SpaceCenter.ParachuteState "SpaceCenter.ParachuteState")

        Game Scenes:
        :   Flight

    deploy\_altitude
    :   The altitude at which the parachute will full deploy, in meters.
        Only applicable to stock parachutes.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    deploy\_min\_pressure
    :   The minimum pressure at which the parachute will semi-deploy, in atmospheres.
        Only applicable to stock parachutes.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    safe\_state
    :   Whether it is currently safe to deploy the parachute, given the vessel’s
        flight conditions. Computed by KSP from the current airspeed, pressure and
        temperature. Only applicable to stock parachutes.
        Note that KSP reports [`ParachuteSafeState.unsafe`](#SpaceCenter.ParachuteSafeState.unsafe "SpaceCenter.ParachuteSafeState.unsafe") for the first
        second after the vessel unpacks, before its thermal data is valid.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ParachuteSafeState`](#SpaceCenter.ParachuteSafeState "SpaceCenter.ParachuteSafeState")

        Game Scenes:
        :   Flight

class ParachuteState
:   The state of a parachute. See [`Parachute.state`](#SpaceCenter.Parachute.state "SpaceCenter.Parachute.state").

    stowed
    :   The parachute is safely tucked away inside its housing.

    armed
    :   The parachute is armed for deployment.

    semi\_deployed
    :   The parachute has been deployed and is providing some drag,
        but is not fully deployed yet. (Stock parachutes only)

    deployed
    :   The parachute is fully deployed.

    cut
    :   The parachute has been cut.

class ParachuteSafeState
:   The safety state of deploying a parachute. See [`Parachute.safe_state`](#SpaceCenter.Parachute.safe_state "SpaceCenter.Parachute.safe_state").

    safe
    :   The parachute is safe to deploy.

    risky
    :   Deploying the parachute is risky, and it may break off.

    unsafe
    :   The parachute is unsafe to deploy, and will break off.

    none
    :   The safety state is not available, because the vessel is not in an
        atmosphere or the parachute has already been deployed or cut.

### [Radiator](#id75)

class Radiator
:   A radiator. Obtained by calling [`Part.radiator`](#SpaceCenter.Part.radiator "SpaceCenter.Part.radiator").

    part
    :   The part object for this radiator.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    deployable
    :   Whether the radiator is deployable.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    deployed
    :   For a deployable radiator, `True` if the radiator is extended.
        If the radiator is not deployable, this is always `True`.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    state
    :   The current state of the radiator.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`DeployableState`](#SpaceCenter.DeployableState "SpaceCenter.DeployableState")

        Game Scenes:
        :   Flight

        > **Note**
        >
        > A fixed radiator is always [`DeployableState.deployed`](#SpaceCenter.DeployableState.deployed "SpaceCenter.DeployableState.deployed").

### [Resource Converter](#id76)

class ResourceConverter
:   A resource converter. Obtained by calling [`Part.resource_converter`](#SpaceCenter.Part.resource_converter "SpaceCenter.Part.resource_converter").

    part
    :   The part object for this converter.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    count
    :   The number of converters in the part.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   int

        Game Scenes:
        :   Flight

    name(*index*)
    :   The name of the specified converter.

        Parameters:
        :   **index** (*int*) – Index of the converter.

        Return type:
        :   str

        Game Scenes:
        :   Flight

    active(*index*)
    :   True if the specified converter is active.

        Parameters:
        :   **index** (*int*) – Index of the converter.

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    start(*index*)
    :   Start the specified converter.

        Parameters:
        :   **index** (*int*) – Index of the converter.

        Game Scenes:
        :   Flight

    stop(*index*)
    :   Stop the specified converter.

        Parameters:
        :   **index** (*int*) – Index of the converter.

        Game Scenes:
        :   Flight

    state(*index*)
    :   The state of the specified converter.

        Parameters:
        :   **index** (*int*) – Index of the converter.

        Return type:
        :   [`ResourceConverterState`](#SpaceCenter.ResourceConverterState "SpaceCenter.ResourceConverterState")

        Game Scenes:
        :   Flight

    status\_info(*index*)
    :   Status information for the specified converter.
        This is the full status message shown in the in-game UI.

        Parameters:
        :   **index** (*int*) – Index of the converter.

        Return type:
        :   str

        Game Scenes:
        :   Flight

    inputs(*index*)
    :   List of the names of resources consumed by the specified converter.

        Parameters:
        :   **index** (*int*) – Index of the converter.

        Return type:
        :   list(str)

        Game Scenes:
        :   Flight

    outputs(*index*)
    :   List of the names of resources produced by the specified converter.

        Parameters:
        :   **index** (*int*) – Index of the converter.

        Return type:
        :   list(str)

        Game Scenes:
        :   Flight

    optimum\_core\_temperature
    :   The core temperature at which the converter will operate with peak efficiency, in Kelvin.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    core\_temperature
    :   The core temperature of the converter, in Kelvin.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    thermal\_efficiency
    :   The thermal efficiency of the converter, as a percentage of its maximum.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

class ResourceConverterState
:   The state of a resource converter. See [`ResourceConverter.state()`](#SpaceCenter.ResourceConverter.state "SpaceCenter.ResourceConverter.state").

    running
    :   Converter is running.

    idle
    :   Converter is idle.

    missing\_resource
    :   Converter is missing a required resource.

    storage\_full
    :   No available storage for output resource.

    capacity
    :   At preset resource capacity.

    unknown
    :   Unknown state. Possible with modified resource converters.
        In this case, check [`ResourceConverter.status_info()`](#SpaceCenter.ResourceConverter.status_info "SpaceCenter.ResourceConverter.status_info") for more information.

### [Resource Harvester](#id77)

class ResourceHarvester
:   A resource harvester (drill). Obtained by calling [`Part.resource_harvester`](#SpaceCenter.Part.resource_harvester "SpaceCenter.Part.resource_harvester").

    part
    :   The part object for this harvester.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    state
    :   The deployment state of the harvester. Whether it is drilling is
        reported separately by [`ResourceHarvester.active`](#SpaceCenter.ResourceHarvester.active "SpaceCenter.ResourceHarvester.active").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`DeployableState`](#SpaceCenter.DeployableState "SpaceCenter.DeployableState")

        Game Scenes:
        :   Flight

        > **Note**
        >
        > A harvester is never [`DeployableState.broken`](#SpaceCenter.DeployableState.broken "SpaceCenter.DeployableState.broken"), as the game
        > does not track damage for them.

    deployed
    :   Whether the harvester is deployed.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    active
    :   Whether the harvester is actively drilling.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

        > **Note**
        >
        > A value set while the harvester is deploying is applied when the
        > deploy completes, so it can be set immediately after setting
        > [`ResourceHarvester.deployed`](#SpaceCenter.ResourceHarvester.deployed "SpaceCenter.ResourceHarvester.deployed") without waiting for the deploy animation.
        > Setting it has no effect while the harvester is retracted or
        > retracting.

    extraction\_rate
    :   The rate at which the drill is extracting ore, in units per second.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    thermal\_efficiency
    :   The thermal efficiency of the drill, as a percentage of its maximum.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    core\_temperature
    :   The core temperature of the drill, in Kelvin.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    optimum\_core\_temperature
    :   The core temperature at which the drill will operate with peak efficiency, in Kelvin.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

### [Reaction Wheel](#id78)

class ReactionWheel
:   A reaction wheel. Obtained by calling [`Part.reaction_wheel`](#SpaceCenter.Part.reaction_wheel "SpaceCenter.Part.reaction_wheel").

    part
    :   The part object for this reaction wheel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    active
    :   Whether the reaction wheel is active.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    broken
    :   Whether the reaction wheel is broken.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    available\_torque
    :   The available torque, in Newton meters, that can be produced by this reaction wheel,
        in the positive and negative pitch, roll and yaw axes of the vessel. These axes
        correspond to the coordinate axes of the [`Vessel.reference_frame`](./vessel.md#SpaceCenter.Vessel.reference_frame "SpaceCenter.Vessel.reference_frame").
        Returns zero if the reaction wheel is inactive or broken.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`AxisRange`](./vessel.md#SpaceCenter.AxisRange "SpaceCenter.AxisRange")

        Game Scenes:
        :   Flight

    max\_torque
    :   The maximum torque, in Newton meters, that can be produced by this reaction wheel,
        when it is active, in the positive and negative pitch, roll and yaw axes of the vessel.
        These axes correspond to the coordinate axes of the [`Vessel.reference_frame`](./vessel.md#SpaceCenter.Vessel.reference_frame "SpaceCenter.Vessel.reference_frame").

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`AxisRange`](./vessel.md#SpaceCenter.AxisRange "SpaceCenter.AxisRange")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    authority\_limiter
    :   The authority limiter for the reaction wheel, as a percentage of maximum torque.
        A value between 0 and 1.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

### [Resource Drain](#id79)

class ResourceDrain
:   A resource drain. Obtained by calling [`Part.resource_drain`](#SpaceCenter.Part.resource_drain "SpaceCenter.Part.resource_drain").

    part
    :   The part object for this resource drain.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    available\_resources
    :   List of available resources.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Resource`](./resources.md#SpaceCenter.Resource "SpaceCenter.Resource"))

        Game Scenes:
        :   Flight

    set\_resource(*resource*, *enabled*)
    :   Whether the given resource should be drained.

        Parameters:
        :   - **resource** ([*Resource*](./resources.md#SpaceCenter.Resource "SpaceCenter.Resource"))
            - **enabled** (*bool*)

        Game Scenes:
        :   Flight

    check\_resource(*resource*)
    :   Whether the provided resource is enabled for draining.

        Parameters:
        :   **resource** ([*Resource*](./resources.md#SpaceCenter.Resource "SpaceCenter.Resource"))

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    drain\_mode
    :   The drain mode.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`DrainMode`](#SpaceCenter.DrainMode "SpaceCenter.DrainMode")

        Game Scenes:
        :   Flight

    min\_rate
    :   Minimum possible drain rate

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_rate
    :   Maximum possible drain rate.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    rate
    :   Current drain rate.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    start()
    :   Activates resource draining for all enabled parts.

        Game Scenes:
        :   Flight

    stop()
    :   Turns off resource draining.

        Game Scenes:
        :   Flight

class DrainMode
:   Resource drain mode.
    See [`ResourceDrain.drain_mode`](#SpaceCenter.ResourceDrain.drain_mode "SpaceCenter.ResourceDrain.drain_mode").

    part
    :   Drains from the parent part.

    vessel
    :   Drains from all available parts.

### [Robotic Controller](#id80)

class RoboticController
:   A robotic controller. Obtained by calling [`Part.robotic_controller`](#SpaceCenter.Part.robotic_controller "SpaceCenter.Part.robotic_controller").

    part
    :   The part object for this controller.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    enabled
    :   Whether the controller is enabled.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    playing
    :   Whether the controller’s sequence is currently playing.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    position
    :   The current position along the sequence, in seconds.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    length
    :   The length of the sequence, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    play\_speed
    :   The speed at which the sequence is played back, as a multiple of normal speed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    play()
    :   Start playing the controller’s sequence.

        Game Scenes:
        :   Flight

    stop()
    :   Stop playing the controller’s sequence.

        Game Scenes:
        :   Flight

    has\_part(*part*)
    :   Whether the controller has a part.

        Parameters:
        :   **part** ([*Part*](#SpaceCenter.Part "SpaceCenter.Part"))

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    axes()
    :   The axes for the controller.

        Return type:
        :   list([`RoboticAxis`](#SpaceCenter.RoboticAxis "SpaceCenter.RoboticAxis"))

        Game Scenes:
        :   Flight

    add\_axis(*module*, *field\_name*)
    :   Add an axis to the controller.

        Parameters:
        :   - **module** ([*Module*](#SpaceCenter.Module "SpaceCenter.Module")) – The part module that the axis belongs to.
            - **field\_name** (*str*) – The name of the axis field, as returned by [`RoboticController.axes()`](#SpaceCenter.RoboticController.axes "SpaceCenter.RoboticController.axes").

        Returns:
        :   Returns `True` if the axis is added successfully.

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    add\_key\_frame(*module*, *field\_name*, *time*, *value*)
    :   Add key frame value for controller axis.

        Parameters:
        :   - **module** ([*Module*](#SpaceCenter.Module "SpaceCenter.Module")) – The part module that the axis belongs to.
            - **field\_name** (*str*) – The name of the axis field, as returned by [`RoboticController.axes()`](#SpaceCenter.RoboticController.axes "SpaceCenter.RoboticController.axes").
            - **time** (*float*) – The time of the key frame.
            - **value** (*float*) – The value of the key frame.

        Returns:
        :   Returns `True` if the key frame is added successfully.

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    clear\_axis(*module*, *field\_name*)
    :   Clear axis.

        Parameters:
        :   - **module** ([*Module*](#SpaceCenter.Module "SpaceCenter.Module")) – The part module that the axis belongs to.
            - **field\_name** (*str*) – The name of the axis field, as returned by [`RoboticController.axes()`](#SpaceCenter.RoboticController.axes "SpaceCenter.RoboticController.axes").

        Returns:
        :   Returns `True` if the axis is cleared successfully.

        Return type:
        :   bool

        Game Scenes:
        :   Flight

class RoboticAxis(*module*, *field\_name*)
:   An axis of a robotic controller.

    > **Note**
    >
    > See [`RoboticController.axes()`](#SpaceCenter.RoboticController.axes "SpaceCenter.RoboticController.axes").

    module
    :   The part module the axis belongs to.

        Return type:
        :   [`Module`](#SpaceCenter.Module "SpaceCenter.Module")

    field\_name
    :   The name of the field the axis controls.

        Return type:
        :   str

### [Robotic Hinge](#id81)

class RoboticHinge
:   A robotic hinge. Obtained by calling [`Part.robotic_hinge`](#SpaceCenter.Part.robotic_hinge "SpaceCenter.Part.robotic_hinge").

    part
    :   The part object for this robotic hinge.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    target\_angle
    :   Target angle.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    current\_angle
    :   Current angle.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    min\_angle
    :   The minimum angle the hinge can move to, in degrees.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_angle
    :   The maximum angle the hinge can move to, in degrees.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    rate
    :   Target movement rate in degrees per second.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    damping
    :   Damping percentage.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    locked
    :   Lock movement.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    motor\_engaged
    :   Whether the motor is engaged.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    is\_moving
    :   Whether the servo is currently moving.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    move\_home()
    :   Move hinge to its built position.

        Game Scenes:
        :   Flight

### [Robotic Piston](#id82)

class RoboticPiston
:   A robotic piston part. Obtained by calling [`Part.robotic_piston`](#SpaceCenter.Part.robotic_piston "SpaceCenter.Part.robotic_piston").

    part
    :   The part object for this robotic piston.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    target\_extension
    :   Target extension of the piston.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    current\_extension
    :   Current extension of the piston.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    min\_extension
    :   The minimum extension of the piston, in meters.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_extension
    :   The maximum extension of the piston, in meters.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    rate
    :   Target movement rate in meters per second.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    damping
    :   Damping percentage.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    locked
    :   Lock movement.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    motor\_engaged
    :   Whether the motor is engaged.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    is\_moving
    :   Whether the servo is currently moving.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    move\_home()
    :   Move piston to its built position.

        Game Scenes:
        :   Flight

### [Robotic Rotation](#id83)

class RoboticRotation
:   A robotic rotation servo. Obtained by calling [`Part.robotic_rotation`](#SpaceCenter.Part.robotic_rotation "SpaceCenter.Part.robotic_rotation").

    part
    :   The part object for this robotic rotation servo.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    target\_angle
    :   Target angle.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    current\_angle
    :   Current angle.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    min\_angle
    :   The minimum angle the servo can rotate to, in degrees.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_angle
    :   The maximum angle the servo can rotate to, in degrees.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    allow\_full\_rotation
    :   Whether the servo is allowed to rotate freely through a full revolution,
        ignoring the angle limits.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    rate
    :   Target movement rate in degrees per second.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    damping
    :   Damping percentage.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    locked
    :   Lock Movement

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    motor\_engaged
    :   Whether the motor is engaged.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    is\_moving
    :   Whether the servo is currently moving.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    move\_home()
    :   Move rotation servo to its built position.

        Game Scenes:
        :   Flight

### [Robotic Rotor](#id84)

class RoboticRotor
:   A robotic rotor. Obtained by calling [`Part.robotic_rotor`](#SpaceCenter.Part.robotic_rotor "SpaceCenter.Part.robotic_rotor").

    part
    :   The part object for this robotic rotor.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    target\_rpm
    :   Target RPM.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    current\_rpm
    :   Current RPM.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    inverted
    :   Whether the rotor direction is inverted.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    torque\_limit
    :   Torque limit percentage.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_torque
    :   The maximum torque the rotor can generate, in kilonewtons.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    brake\_percentage
    :   The percentage of braking force applied to the rotor.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    locked
    :   Lock movement.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    motor\_engaged
    :   Whether the motor is engaged.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    is\_moving
    :   Whether the rotor is currently moving.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

### [RCS](#id85)

class RCS
:   An RCS block or thruster. Obtained by calling [`Part.rcs`](#SpaceCenter.Part.rcs "SpaceCenter.Part.rcs").

    part
    :   The part object for this RCS.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    active
    :   Whether the RCS thrusters are active.
        An RCS thruster is inactive if the RCS action group is disabled
        ([`Control.rcs`](./control.md#SpaceCenter.Control.rcs "SpaceCenter.Control.rcs")), the RCS thruster itself is not enabled
        ([`RCS.enabled`](#SpaceCenter.RCS.enabled "SpaceCenter.RCS.enabled")), or it is covered by a fairing
        ([`Part.shielded`](#SpaceCenter.Part.shielded "SpaceCenter.Part.shielded")) and cannot thrust while shielded.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    enabled
    :   Whether the RCS thrusters are enabled.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    pitch\_enabled
    :   Whether the RCS thruster will fire when pitch control input is given.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    yaw\_enabled
    :   Whether the RCS thruster will fire when yaw control input is given.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    roll\_enabled
    :   Whether the RCS thruster will fire when roll control input is given.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    forward\_enabled
    :   Whether the RCS thruster will fire when pitch control input is given.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    up\_enabled
    :   Whether the RCS thruster will fire when yaw control input is given.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    right\_enabled
    :   Whether the RCS thruster will fire when roll control input is given.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    input\_override
    :   Whether the RCS control is being set directly, bypassing the vessel’s
        normal flight controls. When enabled, the rotation and translation demand is set by
        [`RCS.rotation_override`](#SpaceCenter.RCS.rotation_override "SpaceCenter.RCS.rotation_override") and [`RCS.translation_override`](#SpaceCenter.RCS.translation_override "SpaceCenter.RCS.translation_override") instead of the
        normal control inputs. The override is automatically released if the
        controlling client disconnects or the vessel changes.
        Precision mode leaves an overridden block at full thrust.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    rotation\_override
    :   The rotation demand applied when [`RCS.input_override`](#SpaceCenter.RCS.input_override "SpaceCenter.RCS.input_override") is enabled, in the pitch,
        roll and yaw axes. Each component is a normalized control input between -1 and 1.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    translation\_override
    :   The translation demand applied when [`RCS.input_override`](#SpaceCenter.RCS.input_override "SpaceCenter.RCS.input_override") is enabled, in the
        right, up and forward axes. Each component is a normalized control input between -1
        and 1.

        Attribute:
        :   Can be read or written

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    available\_torque
    :   The available torque, in Newton meters, that can be produced by this RCS,
        in the positive and negative pitch, roll and yaw axes of the vessel. These axes
        correspond to the coordinate axes of the [`Vessel.reference_frame`](./vessel.md#SpaceCenter.Vessel.reference_frame "SpaceCenter.Vessel.reference_frame").
        Returns zero if RCS is disable.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`AxisRange`](./vessel.md#SpaceCenter.AxisRange "SpaceCenter.AxisRange")

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Precision mode weakens the block, and the available torque follows it. A block
        > driven by [`RCS.input_override`](#SpaceCenter.RCS.input_override "SpaceCenter.RCS.input_override") is exempt, so [`RCS.override_wrench()`](#SpaceCenter.RCS.override_wrench "SpaceCenter.RCS.override_wrench")
        > can report the larger torque an override reaches.

    available\_force
    :   The available force, in Newtons, that can be produced by this RCS,
        in the positive and negative x, y and z axes of the vessel. These axes
        correspond to the coordinate axes of the [`Vessel.reference_frame`](./vessel.md#SpaceCenter.Vessel.reference_frame "SpaceCenter.Vessel.reference_frame").
        Returns zero if RCS is disabled.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`AxisRange`](./vessel.md#SpaceCenter.AxisRange "SpaceCenter.AxisRange")

        Game Scenes:
        :   Flight

        > **Note**
        >
        > Precision mode weakens the block, and the available force follows it. A block
        > driven by [`RCS.input_override`](#SpaceCenter.RCS.input_override "SpaceCenter.RCS.input_override") is exempt, so [`RCS.override_wrench()`](#SpaceCenter.RCS.override_wrench "SpaceCenter.RCS.override_wrench")
        > can report the larger force an override reaches.

    force
    :   The force being applied by the RCS thrusters, in Newtons. The vector is in the
        vessel’s reference frame ([`Vessel.reference_frame`](./vessel.md#SpaceCenter.Vessel.reference_frame "SpaceCenter.Vessel.reference_frame")).
        Returns zero if the RCS is inactive.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    torque
    :   The torque being applied by the RCS thrusters about the vessel’s center of mass,
        in Newton meters. The vector is in the vessel’s reference frame
        ([`Vessel.reference_frame`](./vessel.md#SpaceCenter.Vessel.reference_frame "SpaceCenter.Vessel.reference_frame")). Returns zero if the RCS is inactive.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    override\_wrench(*rotation*, *translation*)
    :   The force and torque the given demands would produce, in Newtons and Newton meters.

        Parameters:
        :   - **rotation** (*tuple*) – A rotation demand, as set by [`RCS.rotation_override`](#SpaceCenter.RCS.rotation_override "SpaceCenter.RCS.rotation_override").
            - **translation** (*tuple*) – A translation demand, as set by [`RCS.translation_override`](#SpaceCenter.RCS.translation_override "SpaceCenter.RCS.translation_override").

        Returns:
        :   The force, and the torque about the vessel’s center of mass, in the vessel’s reference frame ([`Vessel.reference_frame`](./vessel.md#SpaceCenter.Vessel.reference_frame "SpaceCenter.Vessel.reference_frame")), or zero if the RCS is inactive.

        Return type:
        :   [`Wrench`](./flight.md#SpaceCenter.Wrench "SpaceCenter.Wrench")

        Game Scenes:
        :   Flight

        > **Note**
        >
        > The demands do not have to be applied. The prediction leaves out full thrust, which
        > [`RCS.force`](#SpaceCenter.RCS.force "SpaceCenter.RCS.force") and [`RCS.torque`](#SpaceCenter.RCS.torque "SpaceCenter.RCS.torque") include.

    available\_thrust
    :   The amount of thrust, in Newtons, that would be produced by the thruster when activated.
        Returns zero if the thruster does not have any fuel.
        Takes the thrusters current [`RCS.thrust_limit`](#SpaceCenter.RCS.thrust_limit "SpaceCenter.RCS.thrust_limit") and atmospheric conditions
        into account.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_thrust
    :   The maximum amount of thrust that can be produced by the RCS thrusters when active,
        in Newtons, with the [`RCS.thrust_limit`](#SpaceCenter.RCS.thrust_limit "SpaceCenter.RCS.thrust_limit") set to 100%.
        Takes atmospheric conditions into account.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_vacuum\_thrust
    :   The maximum amount of thrust that can be produced by the RCS thrusters when active
        in a vacuum, in Newtons.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    thrust\_limit
    :   The thrust limiter of the thruster. A value between 0 and 1.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    thrusters
    :   A list of thrusters, one of each nozzel in the RCS part.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Thruster`](#SpaceCenter.Thruster "SpaceCenter.Thruster"))

        Game Scenes:
        :   Flight

    specific\_impulse
    :   The current specific impulse of the RCS, in seconds. Returns zero
        if the RCS is not active.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    vacuum\_specific\_impulse
    :   The vacuum specific impulse of the RCS, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    kerbin\_sea\_level\_specific\_impulse
    :   The specific impulse of the RCS at sea level on Kerbin, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    propellants
    :   The names of resources that the RCS consumes.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list(str)

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    propellant\_ratios
    :   The ratios of resources that the RCS consumes. A dictionary mapping resource names
        to the ratios at which they are consumed by the RCS.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   dict(str, float)

        Game Scenes:
        :   Flight

    has\_fuel
    :   Whether the RCS has fuel available.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

### [Sensor](#id86)

class Sensor
:   A sensor, such as a thermometer. Obtained by calling [`Part.sensor`](#SpaceCenter.Part.sensor "SpaceCenter.Part.sensor").

    part
    :   The part object for this sensor.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    active
    :   Whether the sensor is active.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    value
    :   The current value of the sensor.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

        Game Scenes:
        :   Flight

        > **Note**
        >
        > This is the same readout string shown in the part’s right-click menu
        > in-game, and is intended for display rather than for parsing. Both its
        > units and the way it formats numbers follow the language the game is
        > running in, so it is not stable across locales. For programmatic access
        > to the underlying quantities, use
        > [`Part.temperature`](#SpaceCenter.Part.temperature "SpaceCenter.Part.temperature") (temperature sensors),
        > [`Flight.g_force`](./flight.md#SpaceCenter.Flight.g_force "SpaceCenter.Flight.g_force") (accelerometers) or
        > [`Flight.static_pressure`](./flight.md#SpaceCenter.Flight.static_pressure "SpaceCenter.Flight.static_pressure") (barometers).

### [Solar Panel](#id87)

class SolarPanel
:   A solar panel. Obtained by calling [`Part.solar_panel`](#SpaceCenter.Part.solar_panel "SpaceCenter.Part.solar_panel").

    part
    :   The part object for this solar panel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    deployable
    :   Whether the solar panel is deployable.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight, Editor Vab, Editor Sph

    deployed
    :   Whether the solar panel is extended.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    state
    :   The current state of the solar panel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`DeployableState`](#SpaceCenter.DeployableState "SpaceCenter.DeployableState")

        Game Scenes:
        :   Flight

    energy\_flow
    :   The current amount of energy being generated by the solar panel, in
        units of charge per second.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    sun\_exposure
    :   The current amount of sunlight that is incident on the solar panel,
        as a percentage. A value between 0 and 1.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

### [Thruster](#id88)

class Thruster
:   The component of an [`Engine`](#SpaceCenter.Engine "SpaceCenter.Engine") or [`RCS`](#SpaceCenter.RCS "SpaceCenter.RCS") part that generates thrust.
    Can obtained by calling [`Engine.thrusters`](#SpaceCenter.Engine.thrusters "SpaceCenter.Engine.thrusters") or [`RCS.thrusters`](#SpaceCenter.RCS.thrusters "SpaceCenter.RCS.thrusters").

    > **Note**
    >
    > Engines can consist of multiple thrusters.
    > For example, the S3 KS-25x4 “Mammoth” has four rocket nozzels, and so consists of
    > four thrusters.

    part
    :   The [`Thruster.part`](#SpaceCenter.Thruster.part "SpaceCenter.Thruster.part") that contains this thruster.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

        Game Scenes:
        :   Flight

    thrust
    :   The thrust being produced by the thruster, in Newtons.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    thrust\_position(*reference\_frame*)
    :   The position at which the thruster generates thrust, in the given reference frame.
        For gimballed engines, this takes into account the current rotation of the gimbal.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned position vector is in.

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    thrust\_direction(*reference\_frame*)
    :   The direction of the force generated by the thruster, in the given reference frame.
        This is opposite to the direction in which the thruster expels propellant.
        For gimballed engines, this takes into account the current rotation of the gimbal.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned direction is in.

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    thrust\_reference\_frame
    :   A reference frame that is fixed relative to the thruster and orientated with
        its thrust direction ([`Thruster.thrust_direction()`](#SpaceCenter.Thruster.thrust_direction "SpaceCenter.Thruster.thrust_direction")).
        For gimballed engines, this takes into account the current rotation of the gimbal.

        - The origin is at the position of thrust for this thruster
            ([`Thruster.thrust_position()`](#SpaceCenter.Thruster.thrust_position "SpaceCenter.Thruster.thrust_position")).
        - The axes rotate with the thrust direction.
            This is the direction in which the thruster expels propellant, including any gimballing.
        - The y-axis points along the thrust direction.
        - The x-axis and z-axis are perpendicular to the thrust direction.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ReferenceFrame`](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")

        Game Scenes:
        :   Flight

    gimballed
    :   Whether the thruster is gimballed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    gimbal\_position(*reference\_frame*)
    :   Position around which the gimbal pivots.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned position vector is in.

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    gimbal\_angle
    :   The current gimbal angle in the pitch, roll and yaw axes, in degrees.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

    initial\_thrust\_position(*reference\_frame*)
    :   The position at which the thruster generates thrust, when the engine is in its
        initial position (no gimballing), in the given reference frame.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned position vector is in.

        Returns:
        :   The position as a vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

        > **Note**
        >
        > This position can move when the gimbal rotates. This is because the thrust position and
        > gimbal position are not necessarily the same.

    initial\_thrust\_direction(*reference\_frame*)
    :   The direction of the force generated by the thruster, when the engine is in its
        initial position (no gimballing), in the given reference frame.
        This is opposite to the direction in which the thruster expels propellant.

        Parameters:
        :   **reference\_frame** ([*ReferenceFrame*](./reference-frame.md#SpaceCenter.ReferenceFrame "SpaceCenter.ReferenceFrame")) – The reference frame that the returned direction is in.

        Returns:
        :   The direction as a unit vector.

        Return type:
        :   tuple(float, float, float)

        Game Scenes:
        :   Flight

### [Wheel](#id89)

class Wheel
:   A wheel. Includes landing gear and rover wheels.
    Obtained by calling [`Part.wheel`](#SpaceCenter.Part.wheel "SpaceCenter.Part.wheel").
    Can be used to control the motors, steering and deployment of wheels, among other things.

    part
    :   The part object for this wheel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](#SpaceCenter.Part "SpaceCenter.Part")

    state
    :   The current state of the wheel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`DeployableState`](#SpaceCenter.DeployableState "SpaceCenter.DeployableState")

    radius
    :   Radius of the wheel, in meters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    grounded
    :   Whether the wheel is touching the ground.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    has\_brakes
    :   Whether the wheel has brakes.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    brakes
    :   The braking force, as a percentage of maximum, when the brakes are applied.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

    auto\_friction\_control
    :   Whether automatic friction control is enabled.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    manual\_friction\_control
    :   Manual friction control value. Only has an effect if automatic friction control is disabled.
        A value between 0 and 5 inclusive.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

    deployable
    :   Whether the wheel is deployable.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    deployed
    :   Whether the wheel is deployed.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    powered
    :   Whether the wheel is powered by a motor.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    motor\_enabled
    :   Whether the motor is enabled.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    motor\_inverted
    :   Whether the direction of the motor is inverted.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    motor\_state
    :   Whether the direction of the motor is inverted.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`MotorState`](#SpaceCenter.MotorState "SpaceCenter.MotorState")

    motor\_output
    :   The output of the motor. This is the torque currently being generated, in Newton meters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    traction\_control\_enabled
    :   Whether automatic traction control is enabled.
        A wheel only has traction control if it is powered.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    traction\_control
    :   Setting for the traction control.
        Only takes effect if the wheel has automatic traction control enabled.
        A value between 0 and 5 inclusive.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

    drive\_limiter
    :   Manual setting for the motor limiter.
        Only takes effect if the wheel has automatic traction control disabled.
        A value between 0 and 100 inclusive.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

    steerable
    :   Whether the wheel has steering.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    steering\_enabled
    :   Whether the wheel steering is enabled.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    steering\_inverted
    :   Whether the wheel steering is inverted.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    steering\_angle\_auto
    :   Whether the steering angle is automatically limited based on the vessel’s speed,
        reducing the maximum angle as the vessel moves faster. See also
        [`Wheel.steering_angle_limit`](#SpaceCenter.Wheel.steering_angle_limit "SpaceCenter.Wheel.steering_angle_limit").

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

    steering\_angle\_limit
    :   The steering angle limit.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

    steering\_response\_time
    :   Steering response time.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

    has\_suspension
    :   Whether the wheel has suspension.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    suspension\_spring\_strength
    :   Suspension spring strength, as set in the editor.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    suspension\_damper\_strength
    :   Suspension damper strength, as set in the editor.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    broken
    :   Whether the wheel is broken.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    repairable
    :   Whether the wheel is repairable.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    stress
    :   Current stress on the wheel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    stress\_tolerance
    :   Stress tolerance of the wheel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    stress\_percentage
    :   Current stress on the wheel as a percentage of its stress tolerance.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    deflection
    :   Current deflection of the wheel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    slip
    :   Current slip of the wheel.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

class MotorState
:   The state of the motor on a powered wheel. See [`Wheel.motor_state`](#SpaceCenter.Wheel.motor_state "SpaceCenter.Wheel.motor_state").

    idle
    :   The motor is idle.

    running
    :   The motor is running.

    disabled
    :   The motor is disabled.

    inoperable
    :   The motor is inoperable.

    not\_enough\_resources
    :   The motor does not have enough resources to run.

## [Trees of Parts](#id56)

Vessels in KSP are comprised of a number of parts, connected to one another in a
*tree* structure. An example vessel is shown in Figure 1, and the corresponding
tree of parts in Figure 2. The craft file for this example can also be
[`downloaded here`](../../../_downloads/6ccebbadf7bf4826270606c3200d307e/PartsTree.craft).

![../../../_images/parts.png](../../../_images/parts.png)

**Figure 1** – Example parts making up a vessel.

![../../../_images/parts-tree.png](../../../_images/parts-tree.png)

**Figure 2** – Tree of parts for the vessel in Figure 1. Arrows point from the parent part to the child part.

### [Traversing the Tree](#id57)

The tree of parts can be traversed using the attributes [`Parts.root`](#SpaceCenter.Parts.root "SpaceCenter.Parts.root"),
[`Part.parent`](#SpaceCenter.Part.parent "SpaceCenter.Part.parent") and [`Part.children`](#SpaceCenter.Part.children "SpaceCenter.Part.children").

The root of the tree is the same as the vessels *root part* (part number 1 in
the example above) and can be obtained by calling [`Parts.root`](#SpaceCenter.Parts.root "SpaceCenter.Parts.root").
A parts children can be obtained by calling [`Part.children`](#SpaceCenter.Part.children "SpaceCenter.Part.children").
If the part does not have any children, [`Part.children`](#SpaceCenter.Part.children "SpaceCenter.Part.children")
returns an empty list. A parts parent can be obtained by calling
[`Part.parent`](#SpaceCenter.Part.parent "SpaceCenter.Part.parent"). If the part does not have a parent
(as is the case for the root part), [`Part.parent`](#SpaceCenter.Part.parent "SpaceCenter.Part.parent")
returns `None`.

The following Python example uses these attributes to perform a
depth-first traversal over all of the parts in a vessel:

```py
import krpc

conn = krpc.connect()
vessel = conn.space_center.active_vessel

root = vessel.parts.root
stack = [(root, 0)]
while stack:
    part, depth = stack.pop()
    print(" " * depth, part.title)
    for child in part.children:
        stack.append((child, depth + 1))
```

When this code is execute using the craft file for the example vessel pictured
above, the following is printed out:

```py
Command Pod Mk1
 TR-18A Stack Decoupler
  FL-T400 Fuel Tank
   LV-909 Liquid Fuel Engine
    TR-18A Stack Decoupler
     FL-T800 Fuel Tank
      LV-909 Liquid Fuel Engine
      TT-70 Radial Decoupler
       FL-T400 Fuel Tank
        TT18-A Launch Stability Enhancer
        FTX-2 External Fuel Duct
        LV-909 Liquid Fuel Engine
        Aerodynamic Nose Cone
      TT-70 Radial Decoupler
       FL-T400 Fuel Tank
        TT18-A Launch Stability Enhancer
        FTX-2 External Fuel Duct
        LV-909 Liquid Fuel Engine
        Aerodynamic Nose Cone
   LT-1 Landing Struts
   LT-1 Landing Struts
 Mk16 Parachute
```

### [Attachment Modes](#id58)

Parts can be attached to other parts either *radially* (on the side of the
parent part) or *axially* (on the end of the parent part, to form a stack).

For example, in the vessel pictured above, the parachute (part 2) is *axially*
connected to its parent (the command pod – part 1), and the landing leg
(part 5) is *radially* connected to its parent (the fuel tank – part 4).

The root part of a vessel (for example the command pod – part 1) does not have
a parent part, so does not have an attachment mode. However, the part is
consider to be *axially* attached to nothing.

The following Python example does a depth-first traversal as before,
but also prints out the attachment mode used by the part:

```py
import krpc

conn = krpc.connect()
vessel = conn.space_center.active_vessel

root = vessel.parts.root
stack = [(root, 0)]
while stack:
    part, depth = stack.pop()
    if part.axially_attached:
        attach_mode = "axial"
    else:  # radially_attached
        attach_mode = "radial"
    print(" " * depth, part.title, "-", attach_mode)
    for child in part.children:
        stack.append((child, depth + 1))
```

When this code is execute using the craft file for the example vessel pictured
above, the following is printed out:

```py
Command Pod Mk1 - axial
 TR-18A Stack Decoupler - axial
  FL-T400 Fuel Tank - axial
   LV-909 Liquid Fuel Engine - axial
    TR-18A Stack Decoupler - axial
     FL-T800 Fuel Tank - axial
      LV-909 Liquid Fuel Engine - axial
      TT-70 Radial Decoupler - radial
       FL-T400 Fuel Tank - radial
        TT18-A Launch Stability Enhancer - radial
        FTX-2 External Fuel Duct - radial
        LV-909 Liquid Fuel Engine - axial
        Aerodynamic Nose Cone - axial
      TT-70 Radial Decoupler - radial
       FL-T400 Fuel Tank - radial
        TT18-A Launch Stability Enhancer - radial
        FTX-2 External Fuel Duct - radial
        LV-909 Liquid Fuel Engine - axial
        Aerodynamic Nose Cone - axial
   LT-1 Landing Struts - radial
   LT-1 Landing Struts - radial
 Mk16 Parachute - axial
```

## [Fuel Lines](#id59)

![../../../_images/parts-fuel-lines.png](../../../_images/parts-fuel-lines.png)

**Figure 5** – Fuel lines from the example in Figure 1. Fuel flows from the parts highlighted in green, into the part highlighted in blue.

![../../../_images/parts-fuel-lines-tree.png](../../../_images/parts-fuel-lines-tree.png)

**Figure 4** – A subset of the parts tree from Figure 2 above.

Fuel lines are considered parts, and are included in the parts tree (for
example, as pictured in Figure 4). However, the parts tree does not contain
information about which parts fuel lines connect to. The parent part of a fuel
line is the part from which it will take fuel (as shown in Figure 4) however the
part that it will send fuel to is not represented in the parts tree.

Figure 5 shows the fuel lines from the example vessel pictured earlier. Fuel
line part 15 (in red) takes fuel from a fuel tank (part 11 – in green) and
feeds it into another fuel tank (part 9 – in blue). The fuel line is therefore
a child of part 11, but its connection to part 9 is not represented in the tree.

The attributes [`Part.fuel_lines_from`](#SpaceCenter.Part.fuel_lines_from "SpaceCenter.Part.fuel_lines_from") and
[`Part.fuel_lines_to`](#SpaceCenter.Part.fuel_lines_to "SpaceCenter.Part.fuel_lines_to") can be used to discover these
connections. In the example in Figure 5, when
[`Part.fuel_lines_to`](#SpaceCenter.Part.fuel_lines_to "SpaceCenter.Part.fuel_lines_to") is called on fuel tank part
11, it will return a list of parts containing just fuel tank part 9 (the blue
part). When [`Part.fuel_lines_from`](#SpaceCenter.Part.fuel_lines_from "SpaceCenter.Part.fuel_lines_from") is called on
fuel tank part 9, it will return a list containing fuel tank parts 11 and 17
(the parts colored green).

## [Staging](#id60)

For vessel-level summaries (delta-v, thrust, fuel, and resources grouped per
stage), use the [Stage](./stage.md#python-api-stage) class via
[`Vessel.stages`](./vessel.md#SpaceCenter.Vessel.stages "SpaceCenter.Vessel.stages") and
[`Vessel.decouple_stages`](./vessel.md#SpaceCenter.Vessel.decouple_stages "SpaceCenter.Vessel.decouple_stages").

![../../../_images/parts-staging.png](../../../_images/parts-staging.png)

**Figure 6** – Example vessel from Figure 1 with a staging sequence.

Each part has two staging numbers associated with it: the stage in which the
part is *activated* and the stage in which the part is *decoupled*. These values
can be obtained using [`Part.stage`](#SpaceCenter.Part.stage "SpaceCenter.Part.stage") and
[`Part.decouple_stage`](#SpaceCenter.Part.decouple_stage "SpaceCenter.Part.decouple_stage") respectively. For parts that
are not activated by staging, [`Part.stage`](#SpaceCenter.Part.stage "SpaceCenter.Part.stage") returns
-1. For parts that are never decoupled,
[`Part.decouple_stage`](#SpaceCenter.Part.decouple_stage "SpaceCenter.Part.decouple_stage") returns a value of -1.

Figure 6 shows an example staging sequence for a vessel. Figure 7 shows the
stages in which each part of the vessel will be *activated*. Figure 8 shows the
stages in which each part of the vessel will be *decoupled*.

![../../../_images/parts-staging-activate.png](../../../_images/parts-staging-activate.png)

**Figure 7** – The stage in which each part is *activated*.

![../../../_images/parts-staging-decouple.png](../../../_images/parts-staging-decouple.png)

**Figure 8** – The stage in which each part is *decoupled*.
