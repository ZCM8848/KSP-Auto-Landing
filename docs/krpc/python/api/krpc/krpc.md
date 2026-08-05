# KRPC

Main kRPC service, used by clients to interact with basic server functionality.

static get\_client\_id()
:   Returns the identifier for the current client.

    Return type:
    :   bytes

static get\_client\_name()
:   Returns the name of the current client.
    This is an empty string if the client has no name.

    Return type:
    :   str

clients
:   A list of RPC clients that are currently connected to the server.
    Each entry in the list is a clients identifier, name and address.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   list(tuple(bytes, str, str))

static get\_status()
:   Returns some information about the server, such as the version.

    Return type:
    :   `krpc.schema.KRPC.Status`

static get\_services()
:   Returns information on all services, procedures, classes, properties etc. provided by the server.
    Can be used by client libraries to automatically create functionality such as stubs.

    Return type:
    :   `krpc.schema.KRPC.Services`

game\_scene
:   The current game scene. Setting this switches the game to the given
    scene, or opens/closes the corresponding facility for the pseudo-scenes.
    Scene changes happen asynchronously: setting this property returns
    immediately, and clients should poll it until it reports the requested
    scene. Setting it to [`GameScene.flight`](#KRPC.GameScene.flight "KRPC.GameScene.flight") resumes the save’s
    active vessel, and fails if there is none.

    Attribute:
    :   Can be read or written

    Return type:
    :   [`GameScene`](#KRPC.GameScene "KRPC.GameScene")

current\_game\_scene
:   > **Warning**
    >
    > Deprecated. Use [`game_scene`](#KRPC.game_scene "KRPC.game_scene") instead.

    Get the current game scene.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   [`GameScene`](#KRPC.GameScene "KRPC.GameScene")

paused
:   Whether the game is paused.

    Attribute:
    :   Can be read or written

    Return type:
    :   bool

class GameScene
:   The game scene. See [`game_scene`](#KRPC.game_scene "KRPC.game_scene").

    space\_center
    :   The game scene showing the Kerbal Space Center buildings.

    flight
    :   The game scene showing a vessel in flight (or on the launchpad/runway).

    tracking\_station
    :   The tracking station.

    editor\_vab
    :   The Vehicle Assembly Building.

    editor\_sph
    :   The Space Plane Hangar.

    mission\_builder
    :   The mission builder.

    astronaut\_complex
    :   The astronaut complex. This is a pseudo-scene, shown when the
        astronaut complex facility is open within the space center scene.

    mission\_control
    :   Mission control. This is a pseudo-scene, shown when the
        mission control facility is open within the space center scene.

    research\_and\_development
    :   Research and development. This is a pseudo-scene, shown when the
        research and development facility is open within the space center scene.

    administration
    :   The administration facility. This is a pseudo-scene, shown when the
        administration facility is open within the space center scene.

class InvalidOperationException
:   A method call was made to a method that is invalid
    given the current state of the object.

class ArgumentException
:   A method was invoked where at least one of the passed arguments does not
    meet the parameter specification of the method.

class ArgumentNullException
:   A null reference was passed to a method that does not accept it as a valid argument.

class ArgumentOutOfRangeException
:   The value of an argument is outside the allowable range of values as defined by the invoked method.
