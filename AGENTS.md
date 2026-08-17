# AGENTS.md

KSP booster-recovery framework (`ksp-auto-landing`). Python **>= 3.12**, `src/` layout, package name `recovery`.

## Commands

```powershell
pip install -e .              # editable install (one-time setup; makes `recovery` importable everywhere)
python -m pytest              # unit tests — no KSP needed (fake kRPC client)
python -m pytest -m live      # live integration — needs KSP + kRPC server running
ruff check .                  # lint
mypy                         # typecheck (strict)
mkdocs build                 # build the docs site (site/)
mkdocs serve                 # live-reload docs server (http://127.0.0.1:8000)
```

- Install the package editable (`pip install -e .`) so `recovery` is importable everywhere; `scripts/` run against the installed package. pytest additionally uses `pythonpath = ["src"]` from pyproject, so tests also work without installing.
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
- `abort_all()`/`close()` must run **inside** the `with` block; `abort_all()` returns the ids of boosters it could not abort (connection died mid-flight) and silently skips already-closed ones.

## Lint / type config quirks

- The control layer (`auto_pilot`, `control_utils`, `dynamics`, `local_attitude`) is fully typed and linted; its control law is guarded bit-for-bit by `tests/test_local_attitude.py`.
- Third-party libs without type stubs (`numba`, `gfold`, `onnxruntime`, `scipy`) use `ignore_missing_imports` in mypy; all first-party `src/` code (including the `_numba` kernels) typechecks under strict mypy.

## Key docs

- `docs/ksp-gateway-api.md` — authoritative API reference for the KSP layer (English narrative + auto-generated reference from source docstrings via mkdocstrings): lifecycle, coordinate frames, thread model, debug drawing, guidance usage. Read it before changing `src/recovery/ksp/`. The docstrings in `src/` are the single source of truth for API details.
- `docs/krpc/` and `gfold_Python_API文档.md` — vendored kRPC / gfold reference material.

## Conventions

- Tests use `tests/fakes.py` (a `FakeClient`) injected via `monkeypatch.setattr("recovery.ksp.connection.krpc.connect", ...)`. Follow this for new connection-level tests.
- Commit messages are short and terse (see `git log`).
