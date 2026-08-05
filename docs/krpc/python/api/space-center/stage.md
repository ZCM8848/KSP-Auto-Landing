# Stage

class Stage
:   A single stage of a vessel. Obtain activation (burn) stages from
    `Vessel.Stages` / `Vessel.StageAt`, and decouple stages from
    `Vessel.DecoupleStages` / `Vessel.DecoupleStageAt`.

    > **Note**
    >
    > Delta-v, thrust, TWR, specific impulse, burn time and mass members are only
    > available on activation stages. On decouple stages those members throw
    > InvalidOperationException because stock delta-v data does not
    > apply. Thrust is reported in newtons and masses in kilograms (stock values
    > are converted from kilonewtons and tonnes).

    number
    :   The stage number (activation stage for burn stages, decouple stage otherwise).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   int

    parts
    :   The parts that belong to this stage.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Part`](./parts.md#SpaceCenter.Part "SpaceCenter.Part"))

    resources([*cumulative=True*])
    :   Returns a [`Stage.resources()`](#SpaceCenter.Stage.resources "SpaceCenter.Stage.resources") object for this stage.

        Parameters:
        :   **cumulative** (*bool*) – When `False`, only resources assigned to this stage. When `True`, resources for this stage and all later activation or decouple stage numbers are included. On activation stages, unstaged resource containers (for example fuel tanks) are grouped with the first higher activation stage before they are detached. Defaults to `True` so decouple-stage calls match the legacy `Vessel.ResourcesInDecoupleStage` RPC.

        Return type:
        :   [`Resources`](./resources.md#SpaceCenter.Resources "SpaceCenter.Resources")

        > **Note**
        >
        > This is an RPC method: call `resources()` (optional `cumulative` argument).
        > The default `cumulative=true` matches the legacy
        > [`Vessel.resources_in_decouple_stage()`](./vessel.md#SpaceCenter.Vessel.resources_in_decouple_stage "SpaceCenter.Vessel.resources_in_decouple_stage") RPC.
        > For decouple stages, grouping is the same as that deprecated method.
        > For activation stages, unstaged resource containers (for example fuel tanks)
        > are grouped with the first higher activation stage before they detach, matching
        > the C# `Resources()` documentation.

    delta\_v
    :   Delta-v for this stage in the current situation, in m/s.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        > **Note**
        >
        > Delta-v is only defined for **activation** stages (`decouple` is `false`).
        > Calling this on a decouple stage raises an error.

    vacuum\_delta\_v
    :   Vacuum delta-v for this stage, in m/s.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    sea\_level\_delta\_v
    :   Sea-level delta-v for this stage, in m/s.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    thrust
    :   Thrust in the current situation, in newtons.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    vacuum\_thrust
    :   Vacuum thrust, in newtons.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    sea\_level\_thrust
    :   Sea-level thrust, in newtons.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    twr
    :   Thrust-to-weight ratio in the current situation.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    vacuum\_twr
    :   Vacuum thrust-to-weight ratio.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    sea\_level\_twr
    :   Sea-level thrust-to-weight ratio.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    specific\_impulse
    :   Specific impulse in the current situation, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    vacuum\_specific\_impulse
    :   Vacuum specific impulse, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    sea\_level\_specific\_impulse
    :   Sea-level specific impulse, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    burn\_time
    :   Burn time for this stage, in seconds.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    start\_mass
    :   Start mass for this stage, in kg.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    end\_mass
    :   End mass for this stage, in kg.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    dry\_mass
    :   Dry mass for this stage, in kg.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    fuel\_mass
    :   Fuel mass for this stage, in kg.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float
