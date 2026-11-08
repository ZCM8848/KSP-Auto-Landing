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
:   The RPC clients that are currently connected to the server.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   list([`ClientInfo`](#KRPC.ClientInfo "KRPC.ClientInfo"))

static get\_status()
:   Returns some information about the server, such as the version.

    Return type:
    :   `krpc.schema.KRPC.Status`

static get\_services()
:   Returns information on all services, procedures, classes, properties etc. provided by the server.
    Can be used by client libraries to automatically create functionality such as stubs.

    Return type:
    :   `krpc.schema.KRPC.Services`

static add\_function\_stream(*function*)
:   Add a stream of the value a server side function computes on each update.

    Parameters:
    :   **function** ([*Expression*](./expressions.md#KRPC.Expression "KRPC.Expression"))

    Return type:
    :   `krpc.schema.KRPC.Stream`

    > **Note**
    >
    > Each call adds a new stream, with values of the type given by
    > [`Expression.return_type`](./expressions.md#KRPC.Expression.return_type "KRPC.Expression.return_type"). Any exception from the function ends the
    > stream, including one for an object that is not loaded. A procedure that pauses
    > execution to resume on a later tick produces an error on the stream.

static hold\_tick()
:   Hold the game on its current physics tick until [`release_tick()`](#KRPC.release_tick "KRPC.release_tick") is called
    or the hold times out. Every call made in between is executed before the game
    advances, so a read, compute and write all happen in the state of one tick.

    > **Note**
    >
    > The game renders no frame, takes no input and runs no physics while the tick is
    > held. Keep a hold as short as possible, and release it even when the code between
    > the two calls fails.
    >
    > A tick is held once, so this waits for the next tick when the current one has
    > already been released. One client holds the tick at a time, and only a client
    > making a call can hold it: a stream or an event cannot.
    >
    > A hold ends by itself when it times out, when the client disconnects, and when the
    > client calls a procedure that takes more than one tick to finish, such as staging
    > or warping. Stream updates are sent once the tick is released, so waiting for a
    > stream or an event inside a hold waits out the timeout.

static release\_tick()
:   Let the game move on from the tick that [`hold_tick()`](#KRPC.hold_tick "KRPC.hold_tick") held it on. Does
    nothing when no tick is held, which is the case once a hold has timed out.

static next\_tick()
:   Let the game move on from the tick being held and hold the one after it, so that a
    program acts on consecutive physics ticks. Holds the soonest tick it can if no tick
    is currently being held.

    > **Note**
    >
    > A release and a hold made as two separate calls let the game advance between them.
    > This takes the next tick without that gap. Call it at the top of a loop that has to
    > act on every tick, work, and come back to it.
    >
    > The game advances one tick per call, so it runs at the rate of the loop for as long
    > as the loop runs. End the loop with [`release_tick()`](#KRPC.release_tick "KRPC.release_tick").
    >
    > Every limit of [`hold_tick()`](#KRPC.hold_tick "KRPC.hold_tick") applies, the timeout included. A hold lost to
    > the timeout is not reported, and the next call takes whatever tick it lands on.

static compile\_function(*function*)
:   Compile a server side function, so that running, streaming or adding it as an
    event does not compile it.

    Parameters:
    :   **function** ([*Expression*](./expressions.md#KRPC.Expression "KRPC.Expression"))

    > **Note**
    >
    > Compiling a large function takes time on the game’s main thread. Compiling it
    > in advance keeps that time out of its first run.

static run\_function(*function*[, *arguments=[]*])
:   Run a server side function once, within a single physics tick, and return
    the value it produces.

    Parameters:
    :   - **function** ([*Expression*](./expressions.md#KRPC.Expression "KRPC.Expression")) – The function to run.
        - **arguments** (*list*) – The encoded arguments, one per parameter.

    Returns:
    :   The encoded value of the function, or no bytes for a function with no value.

    Return type:
    :   bytes or None

    > **Note**
    >
    > The arguments are encoded at the types given by [`Expression.parameter_types`](./expressions.md#KRPC.Expression.parameter_types "KRPC.Expression.parameter_types").
    > Throws an exception if a procedure the function calls pauses execution, and the
    > effects up to the pause remain.

game\_scene
:   The current game scene. Setting this switches the game to the given
    scene, or opens/closes the corresponding facility for the pseudo-scenes.
    Scene changes happen asynchronously: setting this property returns
    immediately, and clients should poll it until it reports the requested
    scene. Setting it to [`GameScene.flight`](#KRPC.GameScene.flight "KRPC.GameScene.flight") resumes the save’s
    active vessel, and fails if there is none. Moving between
    [`GameScene.editor_vab`](#KRPC.GameScene.editor_vab "KRPC.GameScene.editor_vab") and [`GameScene.editor_sph`](#KRPC.GameScene.editor_sph "KRPC.GameScene.editor_sph")
    keeps the vessel that is being constructed, as the game’s own switch
    editor button does. Leaving flight saves the game first, unless the game
    cannot be saved, e.g. while under acceleration.

    Attribute:
    :   Can be read or written

    Return type:
    :   [`GameScene`](#KRPC.GameScene "KRPC.GameScene")

paused
:   Whether the game is paused.

    Attribute:
    :   Can be read or written

    Return type:
    :   bool

game\_state\_loads
:   The number of game states that have finished loading.

    Attribute:
    :   Read-only, cannot be set

    Return type:
    :   int

    > **Note**
    >
    > Increases once a game load, quickload, revert or scene change has finished.

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

class ClientInfo(*id*, *name*, *address*)
:   A client connected to the server.

    > **Note**
    >
    > See [`clients`](#KRPC.clients "KRPC.clients").

    id
    :   The identifier of the client.

        Return type:
        :   bytes

    name
    :   The name of the client, or an empty string if it has no name.

        Return type:
        :   str

    address
    :   The address of the client.

        Return type:
        :   str

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

class NullReferenceException
:   A value was null where its type does not allow one.

class KeyNotFoundException
:   A key was not found in a dictionary.

class IndexOutOfRangeException
:   An index was outside the bounds of a collection, string or array.

class DivideByZeroException
:   An integer was divided by zero.

class NotSupportedException
:   A method was invoked that is not supported.

class TimeoutException
:   An operation did not complete in the time allotted to it.

class OverflowException
:   An arithmetic or conversion operation overflowed.

class ObjectDestroyedException
:   The game object that a service object refers to no longer exists,
    so the object cannot be used. For example, the part it represents
    was destroyed, or the vessel it belongs to was recovered.

class ObjectNotLoadedException
:   The game object that a service object refers to exists, but is not loaded.
    For example, the game is loading, or the vessel carrying the part is unloaded.

class UnsupportedSceneException
:   The procedure is not available in the current game scene.

class StreamRemovedException
:   A stream was removed by the server, e.g. an event that has finished.

class Error
:   A general error, described by its message.
