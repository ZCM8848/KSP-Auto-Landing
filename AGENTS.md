# AGENTS.md

KSP booster-recovery framework (`ksp-auto-landing`). Python **>= 3.12**, `src/` layout, package name `recovery`.

## Commands

```powershell
python -m pytest              # unit tests — no KSP needed (fake kRPC client)
python -m pytest -m live      # live integration — needs KSP + kRPC server running
ruff check .                  # lint
mypy                         # typecheck (strict)
```

- Tests import the package from `src/` (`pythonpath = ["src"]` in pyproject). No install needed, but scripts outside pytest do `sys.path.insert(0, "src")` manually — keep that pattern when adding scripts.
- `python -m pytest` **skips** `live`-marked tests automatically (see `tests/conftest.py`); only `-m live` runs them.
- Live tests target vessel `$env:KSP_VESSEL` (default `"Booster 1"`).
- Docs reference a conda env `KRPC` (e.g. `D:\miniconda3\envs\KRPC\python.exe`); `.vscode/settings.json` also assumes conda.

## Architecture

- `src/recovery/ksp/` is the **only** place that imports `krpc`. All KSP I/O goes through this layer (`ConnectionManager`, `VesselHandle`, `VesselControls`) plus `ksp/sampling.py` — the sole producer of the pure-data specs that guidance consumes. Guidance/control/data must stay KSP-free.
- `src/recovery/types.py` — shared value types (`FlightState`, `Vector3`, `Quaternion`, `Situation`). `src/recovery/specs.py` — pure-data sampling specs (`BodySpec`, `DragSpec`).
- `src/recovery/guidance/` — impact predictor (`LandingPredictor`, scipy RK45 + numba RK4 fast path), aero models (`DragModel` offline, `KrpcAeroModel` per-step RPC). Pure — imports only `types`/`specs`, never `ksp`.
- `src/recovery/control/` — local PID, `AutoPilot` (client-side stick control), `ApproachingModel`, `LocalAttitudeController`/`StickCommand` (snapshot-driven attitude wrapper), and `control_utils` vector math. Pure — imports only `types`, never `ksp`.
- `src/recovery/data/targets.py` — frozen `LandingSite` coordinates (STOCK and JNSQ).
- `main.py` and `scripts/*.py` are live demo/diagnostic entrypoints, not library code.

## Framework gotchas

- Lifecycle order matters: `add_booster()` → `register_target()` → `start()`. `register_target` after `start()` raises; frame `"target"` requires it.
- `snapshot()` returns `None` until the telemetry thread produces its first frame — poll for readiness before driving a control loop.
- Snapshots are frozen `FlightState` objects (thread-safe reads). Control loop reads snapshots; only throttle/attitude setters do RPCs.
- Building a predictor: `b.sample_predictor_specs()` → `(BodySpec, DragSpec)` (the body part is the cached `b.body_spec`, one-time ~25 ms RPC) → `LandingPredictor.from_body_spec()` + `DragModel.from_spec()` (pure). `predict()` after that is pure local and safe at control-loop rates.
- The `"target"` reference-frame axis convention is inherited verbatim from the legacy implementation (`reference_frames.py` says "do not reinterpret these axes"). Don't "fix" it.
- `abort_all()`/`close()` must run **inside** the `with` block; commanding a closed connection raises `OSError` (WinError 10038).

## Lint / type config quirks

- The control layer (`auto_pilot`, `control_utils`, `dynamics`, `local_attitude`) is fully typed and linted; its control law is guarded bit-for-bit by `tests/test_local_attitude.py`.
- Third-party libs without type stubs (`numba`, `gfold`, `onnxruntime`, `scipy`) use `ignore_missing_imports` in mypy; all first-party `src/` code (including the `_numba` kernels) typechecks under strict mypy.

## Key docs

- `docs/ksp-gateway-api.md` — authoritative (Chinese) API reference for the KSP layer: lifecycle, coordinate frames, thread model, debug drawing, guidance usage. Read it before changing `src/recovery/ksp/`.
- `docs/krpc/` and `gfold_Python_API文档.md` — vendored kRPC / gfold reference material.

## Conventions

- Tests use `tests/fakes.py` (a `FakeClient`) injected via `monkeypatch.setattr("recovery.ksp.connection.krpc.connect", ...)`. Follow this for new connection-level tests.
- Commit messages are short and terse (see `git log`).
