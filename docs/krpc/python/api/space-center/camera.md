# Camera

class Camera
:   Controls the game’s camera.
    Obtained by calling [`camera`](./space-center.md#SpaceCenter.camera "SpaceCenter.camera").

    mode
    :   The current mode of the camera.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`CameraMode`](#SpaceCenter.CameraMode "SpaceCenter.CameraMode")

        Game Scenes:
        :   Flight

    next\_camera()
    :   Switch to the next available camera.
        In flight, this moves to the next flight camera mode. In IVA, it moves the
        view to the next crew member in the active vessel.

        Game Scenes:
        :   Flight

    previous\_camera()
    :   Switch to the previous available camera.
        In flight, this moves to the previous flight camera mode. In IVA, it moves
        the view to the previous crew member in the active vessel.

        Game Scenes:
        :   Flight

    pitch
    :   The pitch of the camera, in degrees.
        A value between [`Camera.min_pitch`](#SpaceCenter.Camera.min_pitch "SpaceCenter.Camera.min_pitch") and [`Camera.max_pitch`](#SpaceCenter.Camera.max_pitch "SpaceCenter.Camera.max_pitch")

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    heading
    :   The heading of the camera, in degrees.

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    distance
    :   The distance from the camera to the subject, in meters.
        A value between [`Camera.min_distance`](#SpaceCenter.Camera.min_distance "SpaceCenter.Camera.min_distance") and [`Camera.max_distance`](#SpaceCenter.Camera.max_distance "SpaceCenter.Camera.max_distance").

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    fo\_v
    :   The Field of View of the camera, in degrees.
        A value between [`Camera.min_fo_v`](#SpaceCenter.Camera.min_fo_v "SpaceCenter.Camera.min_fo_v") and [`Camera.max_fo_v`](#SpaceCenter.Camera.max_fo_v "SpaceCenter.Camera.max_fo_v").

        Attribute:
        :   Can be read or written

        Return type:
        :   float

        Game Scenes:
        :   Flight

    min\_pitch
    :   The minimum pitch of the camera.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_pitch
    :   The maximum pitch of the camera.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    min\_distance
    :   Minimum distance from the camera to the subject, in meters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_distance
    :   Maximum distance from the camera to the subject, in meters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    min\_fo\_v
    :   The minimum field of view the camera in degrees.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    max\_fo\_v
    :   The maximum field of view the camera in degrees.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    default\_distance
    :   Default distance from the camera to the subject, in meters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    default\_fo\_v
    :   The default field of view the camera in degrees.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

        Game Scenes:
        :   Flight

    focussed\_body
    :   In map mode, the celestial body that the camera is focussed on.
        Returns `None` if the camera is not focussed on a celestial body.
        Returns an error is the camera is not in map mode.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`CelestialBody`](./celestial-body.md#SpaceCenter.CelestialBody "SpaceCenter.CelestialBody") or None

        Game Scenes:
        :   Flight

    focussed\_vessel
    :   In map mode, the vessel that the camera is focussed on.
        Returns `None` if the camera is not focussed on a vessel.
        Returns an error is the camera is not in map mode.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`Vessel`](./vessel.md#SpaceCenter.Vessel "SpaceCenter.Vessel") or None

        Game Scenes:
        :   Flight

    focussed\_node
    :   In map mode, the maneuver node that the camera is focussed on.
        Returns `None` if the camera is not focussed on a maneuver node.
        Returns an error is the camera is not in map mode.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`Node`](./node.md#SpaceCenter.Node "SpaceCenter.Node") or None

        Game Scenes:
        :   Flight

    focussed\_crew\_member
    :   In IVA mode, the crew member whose view the camera is showing.
        Returns `None` if no crew member is in view.
        Returns an error if the camera is not in IVA mode.
        Setting this moves the view to the given crew member, switching the camera to
        IVA mode if it is not already. The crew member must be in the active vessel,
        and in a part whose interior the game has placed them in.

        Attribute:
        :   Can be read or written

        Return type:
        :   [`CrewMember`](./vessel.md#SpaceCenter.CrewMember "SpaceCenter.CrewMember") or None

        Game Scenes:
        :   Flight

class CameraMode
:   See [`Camera.mode`](#SpaceCenter.Camera.mode "SpaceCenter.Camera.mode").

    automatic
    :   The camera is showing the active vessel, in “auto” mode.

    free
    :   The camera is showing the active vessel, in “free” mode.

    chase
    :   The camera is showing the active vessel, in “chase” mode.

    locked
    :   The camera is showing the active vessel, in “locked” mode.

    orbital
    :   The camera is showing the active vessel, in “orbital” mode.

    iva
    :   The Intra-Vehicular Activity view is being shown.

    map
    :   The map view is being shown.
