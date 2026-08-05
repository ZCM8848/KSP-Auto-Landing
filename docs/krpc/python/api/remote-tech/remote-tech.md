# RemoteTech

This service provides functionality to interact with
[RemoteTech](https://forum.kerbalspaceprogram.com/index.php?/topic/139167-13-remotetech-v188-2017-09-03/).

available
:   Whether RemoteTech is installed.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   bool

ground\_stations
:   The names of the ground stations.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   list(str)

static antenna(*part*)
:   Get the antenna object for a particular part.

    Parameters:
    :   **part** ([*SpaceCenter.Part*](../space-center/parts.md#SpaceCenter.Part "SpaceCenter.Part"))

    Return type:
    :   [`Antenna`](./antenna.md#RemoteTech.Antenna "RemoteTech.Antenna")

static comms(*vessel*)
:   Get a communications object, representing the communication capability of a particular vessel.

    Parameters:
    :   **vessel** ([*SpaceCenter.Vessel*](../space-center/vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel"))

    Return type:
    :   [`Comms`](./comms.md#RemoteTech.Comms "RemoteTech.Comms")
