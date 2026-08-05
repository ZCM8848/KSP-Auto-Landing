# Resources

class Resources
:   Represents the collection of resources stored in a vessel, stage or part.
    Created by calling [`Vessel.resources`](./vessel.md#SpaceCenter.Vessel.resources "SpaceCenter.Vessel.resources"),
    [`Vessel.resources_in_decouple_stage()`](./vessel.md#SpaceCenter.Vessel.resources_in_decouple_stage "SpaceCenter.Vessel.resources_in_decouple_stage") or
    [`Part.resources`](./parts.md#SpaceCenter.Part.resources "SpaceCenter.Part.resources").

    all
    :   All the individual resources that can be stored.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Resource`](#SpaceCenter.Resource "SpaceCenter.Resource"))

        Game Scenes:
        :   Flight

    with\_resource(*name*)
    :   All the individual resources with the given name that can be stored.

        Parameters:
        :   **name** (*str*)

        Return type:
        :   list([`Resource`](#SpaceCenter.Resource "SpaceCenter.Resource"))

        Game Scenes:
        :   Flight

    names
    :   A list of resource names that can be stored.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list(str)

        Game Scenes:
        :   Flight

    has\_resource(*name*)
    :   Check whether the named resource can be stored.

        Parameters:
        :   **name** (*str*) – The name of the resource.

        Return type:
        :   bool

        Game Scenes:
        :   Flight

    amount(*name*)
    :   Returns the amount of a resource that is currently stored.

        Parameters:
        :   **name** (*str*) – The name of the resource.

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max(*name*)
    :   Returns the amount of a resource that can be stored.

        Parameters:
        :   **name** (*str*) – The name of the resource.

        Return type:
        :   float

        Game Scenes:
        :   Flight

    static density(*name*)
    :   Returns the density of a resource, in \(kg/l\).

        Parameters:
        :   **name** (*str*) – The name of the resource.

        Return type:
        :   float

        Game Scenes:
        :   Flight

    static flow\_mode(*name*)
    :   Returns the flow mode of a resource.

        Parameters:
        :   **name** (*str*) – The name of the resource.

        Return type:
        :   [`ResourceFlowMode`](#SpaceCenter.ResourceFlowMode "SpaceCenter.ResourceFlowMode")

        Game Scenes:
        :   Flight

    enabled
    :   Whether use of all the resources are enabled.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

        Game Scenes:
        :   Flight

        > **Note**
        >
        > This is `True` if all of the resources are enabled.
        > If any of the resources are not enabled, this is `False`.

class Resource
:   An individual resource stored within a part.
    Created using methods in the [`Resources`](#SpaceCenter.Resources "SpaceCenter.Resources") class.

    name
    :   The name of the resource.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    part
    :   The part containing the resource.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Part`](./parts.md#SpaceCenter.Part "SpaceCenter.Part")

    amount
    :   The amount of the resource that is currently stored in the part.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    max
    :   The total amount of the resource that can be stored in the part.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    density
    :   The density of the resource, in \(kg/l\).

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    flow\_mode
    :   The flow mode of the resource.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ResourceFlowMode`](#SpaceCenter.ResourceFlowMode "SpaceCenter.ResourceFlowMode")

    enabled
    :   Whether use of this resource is enabled.

        Attribute:
        :   Can be read or written

        Return type:
        :   bool

class ResourceTransfer
:   Transfer resources between parts.

    static start(*from\_part*, *to\_part*, *resource*, *max\_amount*)
    :   Start transferring a resource transfer between a pair of parts. The transfer will move
        at most *max\_amount* units of the resource, depending on how much of
        the resource is available in the source part and how much storage is available in the
        destination part.
        Use [`ResourceTransfer.complete`](#SpaceCenter.ResourceTransfer.complete "SpaceCenter.ResourceTransfer.complete") to check if the transfer is complete.
        Use [`ResourceTransfer.amount`](#SpaceCenter.ResourceTransfer.amount "SpaceCenter.ResourceTransfer.amount") to see how much of the resource has been transferred.

        Parameters:
        :   - **from\_part** ([*Part*](./parts.md#SpaceCenter.Part "SpaceCenter.Part")) – The part to transfer to.
            - **to\_part** ([*Part*](./parts.md#SpaceCenter.Part "SpaceCenter.Part")) – The part to transfer from.
            - **resource** (*str*) – The name of the resource to transfer.
            - **max\_amount** (*float*) – The maximum amount of resource to transfer.

        Return type:
        :   [`ResourceTransfer`](#SpaceCenter.ResourceTransfer "SpaceCenter.ResourceTransfer")

        > **Note**
        >
        > The transfer is canceled if the client that started it disconnects;
        > a canceled transfer is marked as complete.

    amount
    :   The amount of the resource that has been transferred.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    complete
    :   Whether the transfer has completed. Also becomes true if the transfer is
        canceled because the client that started it disconnected.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

class ResourceFlowMode
:   The way in which a resource flows between parts. See [`Resources.flow_mode()`](#SpaceCenter.Resources.flow_mode "SpaceCenter.Resources.flow_mode").

    vessel
    :   The resource flows to any part in the vessel. For example, electric charge.

    stage
    :   The resource flows from parts in the first stage, followed by the second,
        and so on. For example, mono-propellant.

    adjacent
    :   The resource flows between adjacent parts within the vessel. For example,
        liquid fuel or oxidizer.

    none
    :   The resource does not flow. For example, solid fuel.
