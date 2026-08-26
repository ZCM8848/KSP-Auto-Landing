# KSP Gateway API

`src/recovery/ksp/` is the **only** part of the codebase that imports `krpc`.
Upper layers (guidance / control / data and the future orchestration layer)
read and write KSP solely through this layer's public API — direct kRPC access
is forbidden.

> The API reference below is generated from the source docstrings with
> [mkdocstrings](https://mkdocstrings.github.io/); the docstrings in `src/`
> are the single source of truth. The prose describes the design, lifecycle and
> conventions that glue the classes together.

## Design conventions

- **One independent kRPC connection per vessel**, each with its own telemetry
  thread. Connections are independent, so RPC latency parallelizes naturally.
- **Reads**: kRPC streams are pushed continuously by the server; the telemetry
  thread samples them at `telemetry_hz` and atomically copies them into a
  frozen snapshot (`FlightState`). The control loop reads snapshots — a pure
  in-memory read, µs-level, **zero network round-trips**.
- **Writes**: the `VesselControls` gateway. Attitude has two paths: (1) the
  server-side `AutoPilot` (pass `target_direction`, closed-loop, no per-tick
  stick commands); (2) **local attitude control** (`LocalAttitudeController.step`
  computes sticks each tick → `apply(roll/yaw/pitch)`). `throttle` etc. is one
  RPC per setter.
- **No loss of capability**: `VesselHandle.raw` / `VesselControls.raw` escape
  hatches reach the underlying kRPC objects.
- Connection lifecycle is managed centrally by `ConnectionManager`, with an
  `atexit` fallback close.

## Quick start

```python
from recovery import ConnectionManager

with ConnectionManager(address="127.0.0.1") as km:
    booster = km.add_booster("booster-01", "Booster 1", control_hz=50.0, telemetry_hz=20.0)
    km.register_target("booster-01", lon=-74.4730, lat=-0.1853)   # must precede start
    km.start()

    state = km.snapshot("booster-01")        # FlightState | None (None before the first frame)
    booster.controls.apply(
        target_direction=(0.0, -1.0, 0.0),
        reference_frame=km.frame("booster-01", "target"),
        throttle=0.85,
    )

    km.abort_all()                           # emergency stop: zero throttle + disengage AutoPilot
```

> `abort_all()` / `close()` must run **inside** the `with` block — after the
> block exits the connection is closed, and commanding a closed connection
> raises `OSError` (WinError 10038). `abort_all()` returns the ids of boosters
> it could not abort (their connection died mid-flight); already-closed
> boosters are skipped silently.

## Lifecycle (three forms)

```python
# ① context manager (recommended)
with ConnectionManager() as km:
    ...

# ② explicit lifecycle
km = ConnectionManager()
km.add_booster(...)
km.start()
try:
    ...
finally:
    km.close()

# ③ atexit fallback: even if close() is forgotten, process exit disconnects
```

## Class reference

### `recovery.ConnectionManager`

Aggregates all booster connections; manages the lifecycle and global operations.

```python
ConnectionManager(
    *,
    address: str = "127.0.0.1",
    rpc_port: int = 50000,
    stream_port: int = 50001,
    telemetry_hz: float = 20.0,     # global default telemetry rate
)
```

`__enter__` / `__exit__` support `with`. `close()` also calls `disable_debug()`.

::: recovery.ksp.manager.ConnectionManager

### `recovery.VesselHandle`

Per-vessel handle (returned by `km.add_booster` / `km.vessel`).

::: recovery.ksp.vessel.VesselHandle

### `recovery.VesselControls`

Per-vessel control gateway; `controls.apply(...)` is the recommended entry
point. It writes raw stick inputs and system toggles, and engages the kRPC
server-side `AutoPilot` when `target_direction` is supplied.

::: recovery.ksp.control.VesselControls

### `recovery.FlightState`

Frozen dataclass — one atomic telemetry snapshot. `position` / `velocity` /
`rotation` are expressed in the frame referenced by `frame`.

::: recovery.types.FlightState

### `recovery.Situation`

::: recovery.types.Situation

### `recovery.Vector3` / `recovery.Quaternion` / `recovery.TorquePair`

`NamedTuple` value types used throughout the framework.

::: recovery.types.Vector3
::: recovery.types.Quaternion
::: recovery.types.TorquePair

### `recovery.FramePacer`

Realtime loop pacer (used by the orchestration scheduler).

::: recovery.clock.FramePacer

## Coordinate frames

- **`"surface"`**: the built-in kRPC surface frame (for altitude / surface speed).
- **`"target"`**: created and cached by `register_target(lon, lat)` via
  `create_target_reference_frame`: `ReferenceFrame.create_relative` applies a
  longitude/latitude rotation of the body frame + a surface-height translation
  + two 90° spins. **The axis convention is inherited verbatim from the legacy
  implementation** and is not reinterpreted — the guidance solver owns its
  meaning.
- `register_target` must run before `start()`; afterwards the snapshot's
  `position/velocity/rotation` are expressed in the target frame (falls back to
  the surface frame if no target is registered).
- Constraint: the AutoPilot's `reference_frame` must not rotate with the
  controlled vessel; the target/surface frames are body-fixed, so they satisfy
  this naturally.

## Thread model

```
ConnectionManager
 ├─ booster A: krpc.Client[A] ── telemetry thread[A] ── atomic snapshot[A]
 ├─ booster B: krpc.Client[B] ── telemetry thread[B] ── atomic snapshot[B]
 └─ ... (one connection + one thread per vessel)
```

- The control loop (orchestration layer) only reads snapshots and writes
  commands via `VesselControls` — it **holds no connection locks**.
- Snapshot reads are thread-safe (guarded by a `threading.Lock`); `snapshot()`
  returns a frozen object, so there is no concurrent-modification risk.
- `snapshot()` returns `None` until the first frame; the orchestration layer
  should poll for readiness.

## Debug drawing

All drawing runs over a **dedicated debug connection** (`km.enable_debug()`,
shared by all vessels), fully isolated from the control connection's RPC. The
frames are rebuilt equivalently on the debug connection.

Available frame names: `"target"` (requires `register_target`), `"body"`
(body-fixed), `"vessel"` (body axes, origin at centre of mass), `"surface"`
(surface translation), `"orbital"` (orbital).

```python
km.enable_debug()            # open the shared debug connection

# frame axes (x=red, y=green, z=blue)
marker = b.debug.reference_frame(frame_name="target", length=10.0)
marker.clear()                               # remove the axes

# direction vector (from the frame origin)
b.debug.direction((0, -1, 0), length=30, color=(1, 0.5, 0), thickness=0.5)

# arbitrary line segment
line = b.debug.line((0, 0, 0), (10, 0, 0), color=(0, 0, 1))
line.visible = False                         # hide
line.color = (1, 1, 1)
line.remove()

# dense trajectory: create N-1 add_line up front, update only start/end later
b.debug.trajectory(gfld_points, name="predicted", frame_name="target", color=(0, 1, 0))
b.debug.trajectory("predicted")              # fetch by name
b.debug.trajectory("predicted").update(new_points)   # rebuilt automatically if count changes
b.debug.trajectory("predicted").clear()      # remove that trajectory
b.debug.trajectories                          # -> dict[str, DebugTrajectory]
b.debug.clear("predicted")                    # remove one group
b.debug.clear_all()                           # remove all drawings of this vessel

km.disable_debug()           # close the debug connection; KSP clears all drawings
```

### `VesselHandle.debug` → `DebugProxy`

::: recovery.ksp.debug.DebugProxy

`DebugLine`: `color` / `visible` / `thickness` read/write; `set_points`;
`remove()` / `clear()`.
`DebugTrajectory`: `update(positions)` (only rewrites `start`/`end` when the
count is unchanged, zero `add_line`); `color` / `visible` / `thickness`
read/write; `clear()`.
`DebugMarker`: `visible` read/write; `clear()`.

> Requirements: `frame_name="target"` requires a prior `km.register_target(...)`
> (`TargetNotRegistered`); `b.debug` requires `km.enable_debug()`
> (`DebugNotEnabled`). `positions` accepts `list[tuple]` / `list[Vector3]` /
> `numpy.ndarray`.

## Roll and wind-facing control

`up` and `roll_angle` jointly control the rotation of the hull about the thrust
axis — for grid-fin recoveries like Super Heavy, **keeping the wind-facing side
into the airflow** is critical.

### Core parameters

| Parameter | Meaning | Default behaviour |
|---|---|---|
| `up` | Reference direction the roof should point to | the frame's built-in up |
| `roll_angle` | Extra roll about the nose (°) | roll unconstrained |
| `target_smoothing_time` | Smoothing time for direction changes (s) | 0 (instant) |

### Glide phase: wind-facing tracking

With the nose tilted, `up` can be derived from the airflow direction each frame
so the roof always faces the wind:

```python
def wind_up(nose, velocity=(0, 0, -1)):
    """Project wind-facing direction onto nose-perpendicular plane."""
    belly = (-velocity[0], -velocity[1], -velocity[2])   # wind-facing side
    d = nose
    dot = d[0]*belly[0] + d[1]*belly[1] + d[2]*belly[2]
    px, py, pz = belly[0]-dot*d[0], belly[1]-dot*d[1], belly[2]-dot*d[2]
    n2 = px*px + py*py + pz*pz
    if n2 < 1e-12:
        return (1.0, 0.0, 0.0)                           # nose parallel to airflow: fallback
    inv = n2 ** -0.5
    return (-px*inv, -py*inv, -pz*inv)                    # roof = opposite the wind-facing side

# cone sweep — each frame a new direction + up, the wind-facing side rotates
for phi in sweep:
    direction = cone(phi)
    controls.apply(
        target_direction=direction,
        reference_frame=frame,
        up=wind_up(direction),
        roll_angle=0.0,
    )
```

As the nose sweeps the cone, `wind_up` yields the corresponding wind-facing `up`
each frame → the AutoPilot keeps the roof into the wind → the grid fins always
present maximum area.

### Landing phase: mode switch

As the nose approaches vertical (angle to airflow < ~20°) the `wind_up`
projection degenerates (nose parallel to velocity → `n2 → 0`), which makes the
AutoPilot snap 180° in roll. **Solution**: freeze `up` to a horizontal
direction near the singularity; the nose can then pass through vertical without
flipping.

```
glide phase                    →  nose near vertical(<20°)  →  landing phase
up = wind_up(nose)                 up = freeze                  up = (1, 0, 0)
roll = 0                           roll = freeze                roll = capture_angle
```

The switch is triggered by `n2 < threshold` (`threshold ≈ sin²(20°) ≈ 0.12`) and
needs a single `apply`:

```python
controls.apply(
    target_direction=(0, 0, 1),
    reference_frame=frame,
    up=(1.0, 0.0, 0.0),          # switch to a horizontal reference
    roll_angle=capture_angle,     # chopstick-slot capture angle
)
```

After the switch `up` is always perpendicular to the nose (horizontal vs
up), no singularity — roll accuracy is guaranteed through the terminal phase.

## Local attitude control (`LocalAttitudeController`)

`src/recovery/control/local_attitude.py` — a **pure client-side** attitude
controller, an alternative to the server-side `AutoPilot`. It touches no kRPC:
it consumes a `FlightState` snapshot + a target direction and produces raw
sticks, which the caller writes via `VesselControls.apply(roll/yaw/pitch)`.
The control law is **rate-agnostic** (fixed `settling_time`), so the loop
frequency is owned entirely by the caller (the orchestration scheduler).

### Data flow

```
guidance:  snapshot ──► target_dir (horizontal direction toward the target)
control:   snapshot + target_dir ──► StickCommand(roll, yaw, pitch)   # pure, zero IO
IO:        StickCommand ──► controls.apply(roll/yaw/pitch)            # write kRPC raw sticks
```

### API

```python
from recovery.control import LocalAttitudeController

ctrl = LocalAttitudeController(settling_time=0.5, config_interval=0.5)

s = b.snapshot()
target_dir = (-mx / miss, -my / miss, 0.0)      # produced by guidance
sticks = ctrl.step(s, target_dir)               # StickCommand(roll, yaw, pitch)
b.controls.apply(roll=sticks.roll, yaw=sticks.yaw, pitch=sticks.pitch)

# roll policy: roll_target=None (default) only damps the roll rate; a value locks it
sticks = ctrl.step(s, target_dir, roll_target=0.0)
```

::: recovery.control.local_attitude.LocalAttitudeController
::: recovery.control.local_attitude.StickCommand

- Depends on snapshot fields `direction` / `bottom_axis` / `angular_velocity` /
  `available_*_torque` / `moment_of_inertia` (all in the snapshot, zero-RPC reads).
- Unlike the server-side `AutoPilot`, this controller outputs sticks each tick
  (velocity profile + damping, `rot_flag=-1`); the `AutoPilot` closes a PID
  loop on the server instead.

The underlying legacy control law is fully typed and behaviour-locked:

::: recovery.control.auto_pilot.AutoPilot
::: recovery.control.dynamics.ApproachingModel
::: recovery.control.pid.PID

## Exceptions

All framework-level exceptions inherit from `recovery.RecoveryError` and also
chain standard exceptions (`RuntimeError` / `ValueError`) for backward
compatibility.

| Scenario | Exception |
|---|---|
| Vessel name not found | `VesselNotFound` (`ValueError`) |
| Duplicate names (≥ 2) | `AmbiguousVesselName` (`ValueError`) |
| Duplicate `booster_id` | `DuplicateBooster` (`ValueError`) |
| `vessel`/`controls` accessed before `resolve_vessel` | `VesselNotResolved` (`RuntimeError`) |
| `add_booster` / `register_target` after `start()` | `InvalidState` (`RuntimeError`) |
| Debug not enabled | `DebugNotEnabled` (`RuntimeError`) |
| `frame("target")` / `body_spec` before `register_target` | `TargetNotRegistered` (`RuntimeError`) |
| Unknown `booster_id` | `KeyError` |
| `target_direction` missing `reference_frame` / zero vector | `ValueError` |
| Unknown frame name | `KeyError` |

::: recovery.ksp.exceptions

## Performance notes

- Read: a snapshot is an in-memory copy, µs-level. Freshness is bounded by the
  server stream rate (`telemetry_hz` is only a sampling ceiling).
- Write: one RPC per setter (sub-ms to ~1 ms locally). Prefer `target_direction`
  for attitude (server-side closed loop); in a hot loop write only `throttle`.
- 3–4 vessels × 50 Hz × a few attributes ≈ hundreds of RPC/s — no pressure on
  localhost kRPC.
- All telemetry fields are read straight from kRPC streams (including
  `specific_impulse`).
- Local attitude control: 3 stick RPCs per tick (kRPC has no batched stick
  API); reads are all snapshot-based (zero RPC). `max_acc` re-estimation is
  throttled to 0.5 s of game time — negligible.
- Avoid synchronous RPC reads via `raw` inside the control loop (the
  "repeated `vessel.position()` in a loop" anti-pattern from `client.md`).

## Guidance module

`src/recovery/guidance/` — pure trajectory prediction and aerodynamic
modelling, **never touches kRPC**. All KSP I/O is done by
`recovery.ksp.sampling`, which produces pure-data specs consumed by this layer.

### Layering

```
kRPC objects ──→ recovery.ksp.sampling (RPC sampling) ──→ recovery.specs (pure data)
                                                              │
recovery.guidance (pure construction) ←───────────────────────┘
```

### `recovery.specs` — pure-data specs

::: recovery.specs.BodySpec
::: recovery.specs.DragSpec

### `recovery.ksp.sampling` — RPC sampling (the only prediction-RPC module)

::: recovery.ksp.sampling.sample_body_spec
::: recovery.ksp.sampling.sample_drag_spec

```python
from recovery.ksp.sampling import sample_body_spec, sample_drag_spec

body_spec = sample_body_spec(body, target_frame, lat, lon)
drag_spec = sample_drag_spec(
    body, flight, target_frame,
    space_center=conn.space_center,  # optional; enables FAR auto-detection
    mass=vessel.mass,            # for β back-calculation when not using FAR
    manual_beta=None,            # force β, None = auto
    altitude_samples=64,         # adaptive floor = max(32, depth/500)
)
```

### `recovery.guidance.LandingPredictor`

RK45 numerical impact-point predictor; integrates the equations of motion to
the surface in the body-fixed (target) frame.

```python
# pure construction (no RPC)
predictor = LandingPredictor.from_body_spec(body_spec, aero=drag_model)

# or pass scalars directly
predictor = LandingPredictor(
    mu=..., omega=..., body_center=..., body_radius=..., aero=None,
)
```

::: recovery.guidance.predictor.LandingPredictor

Dynamics: `a = g_μ(r) − 2ω×v − ω×(ω×r) + a_aero(r,v)`, where `a_aero` is an
optional aerodynamic contribution.

### `recovery.guidance.ImpactResult`

::: recovery.guidance.predictor.ImpactResult

### Aerodynamic models (AeroModel protocol)

Every aero model implements `acceleration(position, velocity) -> Vec3`:

| Model | Source | RPC/step? | Use case |
|---|---|---|---|
| none (`aero=None`) | — | — | fast pure-ballistic prediction |
| `KrpcAeroModel` | `Flight.simulate_aerodynamic_force_at` (AoA 180° / nose aft) | **yes** (one RPC per step) | offline validation, high-precision analysis |
| `DragModel` | one-shot sampled density table + ballistic coefficient β | **no** | **hot control loop (zero RPC/step)** |

::: recovery.guidance.aerodynamics.DragModel
::: recovery.guidance.aerodynamics.KrpcAeroModel

### Typical usage: ZEM boosterback

```python
with ConnectionManager() as km:
    b = km.add_booster("booster", "SuperHeavy")
    km.register_target("booster", lon=LAUNCHPAD.lon, lat=LAUNCHPAD.lat)
    km.start()

    frame = km.frame("booster", "target")
    body_spec, drag_spec = b.sample_predictor_specs()   # one-shot RPC; body from the registered target
    predictor = LandingPredictor.from_body_spec(
        body_spec, aero=DragModel.from_spec(drag_spec)
    )

    while True:
        s = b.snapshot()
        r = predictor.predict_from(s, rtol=1e-6, atol=1e-6)
        if r is None:
            continue
        mx, my = r.position[0], r.position[1]
        miss = (mx**2 + my**2) ** 0.5

        # nose horizontal toward the target — thrust pushes purely horizontally
        if miss < 1.0:
            target_dir = (0.0, 0.0, -1.0)
        else:
            target_dir = (-mx / miss, -my / miss, 0.0)

        b.controls.apply(
            target_direction=target_dir,
            reference_frame=frame,
            throttle=1.0,
        )

        if miss > min_history:
            break  # local minimum — boosterback complete
```

Full example: `scripts/zem_boosterback.py`.

### `recovery.guidance.gfold` — G-FOLD powered-descent planner

A thin wrapper over the `gfold` SOCP solver, **+z = zenith** (consistent with
the target frame and the gfold library; gravity `[0, 0, -g]`, thrust along +z).
`GfoldParams` (frozen dataclass) centralizes the tunables; the fixed quantities
(mass / fuel / thrust / Isp / gravity / initial state) are derived from the
`FlightState` snapshot + surface gravity `g0`.

::: recovery.guidance.gfold

### `recovery.guidance.tofnet` — TOF-Net TOF predictor

Loads a pretrained ONNX model and uses its `(p_feasible, tf)` prediction to
**replace the internal TOF search of `solve_optimal`**. Pure black box: 14 raw
features in, `(p_feasible, tf)` out; normalization is baked into the ONNX
forward pass. `onnxruntime` is an optional dependency (lazy import, only needed
when instantiating `TofPredictor`).

```python
from recovery.guidance.gfold import GfoldParams, features_of, replan, solve_optimal
from recovery.guidance.tofnet import TofPredictor

tofnet = TofPredictor()                          # loads the bundled assets/tofnet.onnx by default
p, tf = tofnet.predict(features_of(s, params))   # microsecond inference
if p >= tofnet.threshold:
    traj = replan(s, g0, params, tof=tf)         # fixed TOF, skip the search
else:
    traj = solve_optimal(s, g0, params)          # fallback: full TOF search
```

::: recovery.guidance.tofnet.TofPredictor

The **14-feature order is the training contract** (`FEATURES` in GFOLD-solver
`common/config.py`): `x,y,z, vx,vy,vz, dry_mass, fuel, real_max_thrust,
min_thrust_pct, max_thrust_pct, fuel_consumption, glide_slope_angle_deg,
max_angle_deg`. `features_of` is the sole producer; `fuel_consumption` uses
standard gravity `G0=9.80665` (Isp conversion), **not** the surface gravity `g0`.

> The model is **Kerbin-specific** (trained with gravity `[0,0,-9.81]` hardcoded;
> gravity is not part of the feature vector). Retrain — or add gravity to the
> features and retrain — for other bodies. `onnxruntime` is an optional extra
> (`pip install -e .[tofnet]`).

## Testing

```bash
python -m pytest               # unit tests (fake client, no KSP needed)
python -m pytest -m live       # live integration — needs KSP + kRPC; vessel from $env:KSP_VESSEL (default "Booster 1")
```
