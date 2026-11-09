"""C-scheme hopper: the SCvx OCP chooses its own engine ignition time.

This script is deliberately separate from the stable ``grasshopper.py`` so it
can be observed without changing the known-good fixed-ignition mission.

Sequence
--------
1. Ascent to 10 km, cutoff.
2. Coast to ``-200 m/s``.  That state seeds the free-ignition 6-DOF OCP.
3. The OCP has a free ``ignition_time`` state: it integrates the unpowered
   coast itself and decides when to relight.  The engine is off until the
   plan's smooth ignition gate opens.
4. Track the published plan to 500 m, then polynomial terminal guidance.
5. Cut at contact; no stick commands on the ground.

The console and ``grasshopper_c_flight.csv`` expose:
  * the plan's chosen ignition time and altitude;
  * the actual ignition time;
  * plan vs actual lateral error during the coast;
  * solver convergence and plan source.

Run with the KRPC environment::

    D:\\miniforge3\\envs\\KRPC\\python.exe scripts/grasshopper_c.py

A trajectory of the planned path is drawn in the KSP flight scene when debug
drawing is available.  The script does **not** silently fall back to the fixed
ignition scheme: if the free-ignition solve does not converge, it aborts with a
clear error so the failure is observable.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np

# The reusable helpers live in the stable script; importing it has no side
# effects because its ``main()`` is guard-protected.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from grasshopper import (  # noqa: E402
    ASCENT_ALT,
    IGNITION_GUESS,
    MIN_THROTTLE,
    TERMINAL_ALT,
    _ascent_command,
    _engine_model_from_part,
    _plan_thrust,
    _polynomial_terminal_command,
    _synthetic_descent_state,
    _terminal_command,
)

from recovery import ConnectionManager, FramePacer
from recovery.control import LocalAttitudeController
from recovery.data.targets import LAUNCHPAD_JNSQ
from recovery.guidance import (
    AttitudeModel,
    EngineModel,
    PolynomialConfig,
    PolynomialTerminalGuidance,
    ScvxConfig,
    ScvxSolveService,
    build_six_dof_problem,
)
from recovery.types import FlightState, Situation

VESSEL = "VTVL"
BOOSTER_ID = "vtvl_c"
TARGET = LAUNCHPAD_JNSQ

COAST_VZ = -200.0          # state that seeds the free-ignition OCP (m/s)
COAST_MIN_S = 4.0          # minimum engine-off time before the seed
SCVX_ITERS = 120           # root solve needs ~40; cap high enough to see it
SCVX_TF_HINT = 60.0        # free final time / ignition seed (s)
LOOP_HZ = 50.0
LOG_PATH = "grasshopper_c_flight.csv"


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    with ConnectionManager(address="127.0.0.1") as manager:
        manager.enable_debug()
        booster = manager.add_booster(BOOSTER_ID, VESSEL)
        manager.register_target(BOOSTER_ID, lon=TARGET.lon, lat=TARGET.lat)
        manager.start()

        state: FlightState | None = None
        deadline = time.monotonic() + 10.0
        while state is None and time.monotonic() < deadline:
            state = booster.snapshot()
            time.sleep(0.02)
        if state is None:
            raise RuntimeError("no telemetry snapshot from KSP")
        if state.situation != Situation.PRE_LAUNCH:
            raise RuntimeError(
                "C-scheme requires the vessel on the launch pad in PRE_LAUNCH; "
                f"current situation is {state.situation.name}. "
                "Quickload the pre-launch save first."
            )

        target_height = float(state.position.z)
        body_spec, drag_spec = booster.sample_predictor_specs(
            mass=float(booster.raw.mass)
        )
        up_vector = -np.asarray(body_spec.body_center, dtype=float) / max(
            float(np.linalg.norm(np.asarray(body_spec.body_center, dtype=float))),
            1e-9,
        )
        print(
            f"C-scheme: mass={state.mass:.0f} kg dry={state.dry_mass:.0f} kg "
            f"target_height={target_height:.3f} m"
        )
        print(
            f"CdA={drag_spec.cd_area:.3f} m² beta={drag_spec.ballistic_coefficient:.1f}"
        )

        booster.physics_range = 200_000.0
        booster.controls.sas = False
        booster.controls.rcs = True
        booster.controls.legs = True
        booster.controls.throttle = 0.0
        time.sleep(0.3)
        state = booster.snapshot()
        if state is None:
            raise RuntimeError("no telemetry snapshot after RCS enable")

        engine_model = _engine_model_from_part(booster.raw, state)
        attitude_model = AttitudeModel.from_state(state)
        print(
            f"Engine Tmax={engine_model.available_thrust:.0f} N "
            f"Isp={engine_model.isp:.1f} s; torque={attitude_model.max_torque}"
        )

        # Build at the seed state (-200 m/s) so the OCP has the right bounds.
        synthetic = _synthetic_descent_state(
            state,
            mass=float(state.mass),
            altitude=9_700.0,
            vertical_speed=COAST_VZ,
            ut=float(state.ut) + 60.0,
        )
        config = ScvxConfig(
            n=18,
            min_throttle=MIN_THROTTLE,
            max_iters_per_refresh=SCVX_ITERS,
            horizon_factor=2.0,
            horizon_min=1.0,
            multishot_plan=True,
            free_ignition=True,
            ignition_guess=IGNITION_GUESS,
        )
        problem = build_six_dof_problem(
            body_spec,
            drag_spec,
            state=synthetic,
            target_height=target_height,
            engine=engine_model,
            attitude=attitude_model,
            config=config,
        )
        service = ScvxSolveService(workers=1)
        service.register(BOOSTER_ID, problem, config=config)
        print("Compiling free-ignition 6-DOF SCvx problem (~10 s)...")
        service.initialize()
        service.start()

        booster.client.space_center.quicksave()
        print("Saved quicksave.  Igniting ascent...")
        booster.controls.activate_next_stage()
        time.sleep(0.2)
        booster.controls.clear_cache()
        booster.controls.apply(rcs=True, sas=False, legs=True)
        booster.controls.raw.rcs = True
        booster.controls.raw.sas = False

        attitude = LocalAttitudeController()
        terminal_guidance = PolynomialTerminalGuidance(
            PolynomialConfig(min_time_to_go=4.0, time_to_go_gain=2.0)
        )
        pacer = FramePacer(hz=LOOP_HZ)
        log = open(LOG_PATH, "w", encoding="utf-8", newline="")
        log.write(
            "wall,ut,phase,z,vz,x,y,throttle,plan_elapsed,plan_duration,"
            "plan_ignition,actual_ignition,err_x,err_y,terminal_source,"
            "solver_converged,plan_source,solver_error\n"
        )
        log.flush()

        phase = "ascent"
        coast_ut = 0.0
        descent_started_ut = 0.0
        latest_plan: object | None = None
        last_plan_id: int | None = None
        actual_ignition: float | None = None
        terminal_source = "plan"
        last_log = time.monotonic()
        started = time.monotonic()
        last_snapshot_ut: float | None = None
        last_snapshot_change = time.monotonic()

        def log_row(
            s: FlightState,
            throttle: float,
            elapsed: float,
            desired_p: np.ndarray | None,
        ) -> None:
            plan = latest_plan
            plan_duration = float(plan.duration) if plan is not None else float("nan")  # type: ignore[attr-defined]
            plan_ignition = getattr(plan, "ignition_time", None) if plan is not None else None
            plan_source = ""
            solver_converged = ""
            solver_error = ""
            if plan is not None:
                stats = service.stats(BOOSTER_ID)
                plan_source = stats.plan_source
                solver_converged = str(int(stats.solver_converged))
                solver_error = (stats.last_error or "").replace(",", ";")
            if desired_p is None:
                err = np.asarray(s.position, dtype=float) - np.array(
                    [0.0, 0.0, target_height], dtype=float
                )
            else:
                err = np.asarray(s.position, dtype=float) - desired_p
            log.write(
                f"{time.monotonic() - started:.3f},{s.ut:.3f},{phase},"
                f"{s.position.z:.3f},{s.velocity.z:.3f},"
                f"{s.position.x:.3f},{s.position.y:.3f},{throttle:.4f},"
                f"{elapsed:.3f},{plan_duration:.3f},"
                f"{'' if plan_ignition is None else f'{float(plan_ignition):.3f}'},"
                f"{'' if actual_ignition is None else f'{actual_ignition:.3f}'},"
                f"{err[0]:.3f},{err[1]:.3f},{terminal_source},"
                f"{solver_converged},{plan_source},{solver_error}\n"
            )
            log.flush()

        try:
            while True:
                pacer.tick()
                s = booster.snapshot()
                if s is None:
                    continue
                now = time.monotonic()

                telemetry_error = booster.telemetry_error
                if telemetry_error is not None:
                    raise RuntimeError(f"telemetry failed: {telemetry_error!r}")
                if last_snapshot_ut is None or float(s.ut) != last_snapshot_ut:
                    last_snapshot_ut = float(s.ut)
                    last_snapshot_change = now
                elif now - last_snapshot_change > 1.5:
                    raise RuntimeError(
                        f"telemetry stalled at ut={s.ut:.3f} for "
                        f"{now - last_snapshot_change:.1f}s"
                    )

                if phase == "ascent":
                    throttle, nose = _ascent_command(s, ASCENT_ALT, up_vector)
                    sticks = attitude.step(s, nose)
                    booster.controls.apply(
                        throttle=throttle,
                        pitch=sticks.pitch,
                        yaw=sticks.yaw,
                        roll=sticks.roll,
                    )
                    log_row(s, throttle, 0.0, None)
                    if now - last_log >= 1.0:
                        print(
                            f"[ascent] t={now - started:5.1f}s z={s.position.z:8.1f} "
                            f"vz={s.velocity.z:6.1f} thr={throttle:.3f}"
                        )
                        last_log = now
                    if s.position.z >= ASCENT_ALT:
                        booster.controls.throttle = 0.0
                        phase = "coast"
                        coast_ut = float(s.ut)
                        print(
                            f"Cutoff z={s.position.z:.1f} m vz={s.velocity.z:.1f} m/s; "
                            f"coasting to {COAST_VZ:.0f} m/s"
                        )
                    continue

                if phase == "coast":
                    sticks = attitude.step(s, (0.0, 0.0, 1.0))
                    booster.controls.apply(
                        throttle=0.0,
                        pitch=sticks.pitch,
                        yaw=sticks.yaw,
                        roll=sticks.roll,
                    )
                    log_row(s, 0.0, 0.0, None)
                    if now - last_log >= 1.0:
                        print(
                            f"[coast ] t={now - started:5.1f}s z={s.position.z:8.1f} "
                            f"vz={s.velocity.z:6.1f} xy=({s.position.x:6.1f},"
                            f"{s.position.y:6.1f})"
                        )
                        last_log = now
                    if (
                        float(s.ut) - coast_ut >= COAST_MIN_S
                        and s.velocity.z <= COAST_VZ
                    ):
                        service.refresh(
                            BOOSTER_ID,
                            s,
                            engine=EngineModel.from_state(s),
                            tf_hint=SCVX_TF_HINT,
                            attitude=AttitudeModel.from_state(s),
                        )
                        phase = "descent"
                        descent_started_ut = float(s.ut)
                        print(
                            f"Free-ignition OCP seeded at z={s.position.z:.1f} m "
                            f"vz={s.velocity.z:.1f} m/s, mass={s.mass:.0f} kg"
                        )
                    continue

                # ---- free-ignition powered descent -----------------------
                terminal_source = "plan"
                if s.position.z <= target_height + TERMINAL_ALT:
                    height = s.position.z - target_height
                    if height > 5.0:
                        try:
                            throttle, nose, desired_p, terminal_source = (
                                _polynomial_terminal_command(
                                    s,
                                    body_spec,
                                    drag_spec,
                                    target_height,
                                    terminal_guidance,
                                )
                            )
                        except Exception as exc:  # noqa: BLE001
                            print(f"[terminal] polynomial error: {exc!r}")
                            throttle, nose, desired_p = _terminal_command(
                                s, body_spec, drag_spec, target_height
                            )
                            terminal_source = "hoverslam-fallback"
                        if terminal_source == "poly-coast":
                            throttle, nose, desired_p = _terminal_command(
                                s, body_spec, drag_spec, target_height
                            )
                            terminal_source = "hoverslam-fallback"
                    else:
                        throttle, nose, desired_p = _terminal_command(
                            s, body_spec, drag_spec, target_height
                        )
                        terminal_source = "hoverslam-low"
                    sticks = attitude.step(s, nose)
                    booster.controls.apply(
                        throttle=throttle,
                        pitch=sticks.pitch,
                        yaw=sticks.yaw,
                        roll=sticks.roll,
                    )
                    log_row(s, throttle, 0.0, desired_p)
                    if now - last_log >= 1.0:
                        print(
                            f"[terminal] t={now - started:5.1f}s z={s.position.z:8.1f} "
                            f"vz={s.velocity.z:7.2f} thr={throttle:.3f} "
                            f"src={terminal_source}"
                        )
                        last_log = now
                else:
                    latest_plan = service.latest_plan(BOOSTER_ID)
                    stats = service.stats(BOOSTER_ID)
                    if latest_plan is None or not stats.solver_converged:
                        if float(s.ut) - descent_started_ut > 20.0:
                            raise RuntimeError(
                                "free-ignition plan did not converge within 20 s "
                                f"(iters={stats.iterations}, "
                                f"converged={stats.converged}, "
                                f"solver={stats.solver_converged}, "
                                f"error={service.last_error(BOOSTER_ID)!r})"
                            )
                        sticks = attitude.step(s, (0.0, 0.0, 1.0))
                        booster.controls.apply(
                            throttle=0.0,
                            pitch=sticks.pitch,
                            yaw=sticks.yaw,
                            roll=sticks.roll,
                        )
                        log_row(s, 0.0, 0.0, None)
                        if now - last_log >= 1.0:
                            print(
                                f"[solve ] t={now - started:5.1f}s z={s.position.z:8.1f} "
                                f"iters={stats.iterations} "
                                f"conv={stats.converged} "
                                f"solver={stats.solver_converged}"
                            )
                            last_log = now
                        continue

                    if id(latest_plan) != last_plan_id:
                        last_plan_id = id(latest_plan)
                        ignition = getattr(latest_plan, "ignition_time", None)
                        print(
                            f"[plan  ] solver={stats.solver_converged} "
                            f"src={stats.plan_source} duration="
                            f"{float(latest_plan.duration):.2f}s "
                            f"ignition={ignition}"
                        )
                        # Draw the OCP's own coast+burn path in the flight scene.
                        try:
                            positions = np.asarray(latest_plan.positions, dtype=float)
                            step = max(1, len(positions) // 80)
                            booster.debug.trajectory(
                                positions[::step],
                                name="c_plan",
                                frame_name="target",
                                color=(0.1, 1.0, 0.2),
                                thickness=0.3,
                            )
                        except Exception as exc:  # noqa: BLE001
                            print(f"[debug] could not draw plan: {exc!r}")

                    current_ut = booster.current_ut()
                    elapsed = max(0.0, float(current_ut) - float(latest_plan.t0_ut))
                    elapsed = min(elapsed, float(latest_plan.duration))
                    throttle, nose, desired_p = _plan_thrust(latest_plan, s, elapsed)
                    if throttle > 0.01 and actual_ignition is None:
                        actual_ignition = elapsed
                        print(
                            f"[ignite] actual burn start at z={s.position.z:.1f} m, "
                            f"vz={s.velocity.z:.1f} m/s, plan_t={elapsed:.2f}s"
                        )
                    sticks = attitude.step(s, nose)
                    booster.controls.apply(
                        throttle=throttle,
                        pitch=sticks.pitch,
                        yaw=sticks.yaw,
                        roll=sticks.roll,
                    )
                    log_row(s, throttle, elapsed, desired_p)
                    if now - last_log >= 1.0:
                        print(
                            f"[descent] t={now - started:5.1f}s z={s.position.z:8.1f} "
                            f"vz={s.velocity.z:7.2f} thr={throttle:.3f} "
                            f"plan={elapsed:5.1f}/{float(latest_plan.duration):5.1f}s "
                            f"xy=({s.position.x:6.1f},{s.position.y:6.1f}) "
                            f"ign={actual_ignition if actual_ignition is not None else '-'}"
                        )
                        last_log = now

                # Touchdown: zero throttle; no stick commands on the ground.
                if s.landed or (
                    s.position.z <= target_height + 0.12
                    and s.velocity.z > -1.2
                ):
                    booster.controls.throttle = 0.0
                    booster.controls.apply(pitch=0.0, yaw=0.0, roll=0.0)
                    print(
                        f"Touchdown z={s.position.z:.3f} m "
                        f"(target {target_height:.3f}) vz={s.velocity.z:.3f} m/s "
                        f"xy=({s.position.x:.3f}, {s.position.y:.3f}) m"
                    )
                    settle_start = time.monotonic()
                    settle_ut = float(s.ut)
                    while time.monotonic() - settle_start < 2.5:
                        pacer.tick()
                        settled = booster.snapshot()
                        if settled is None:
                            break
                        s = settled
                        if not s.loaded or s.mass <= 0.0:
                            break
                        if float(s.ut) - settle_ut > 2.0:
                            break
                        booster.controls.apply(
                            throttle=0.0, pitch=0.0, yaw=0.0, roll=0.0
                        )
                    booster.controls.cut_thrust()
                    tilt = math.degrees(
                        math.acos(min(max(float(s.direction.z), -1.0), 1.0))
                    )
                    print(
                        f"Settled z={s.position.z:.3f} m "
                        f"vz={s.velocity.z:.3f} m/s tilt={tilt:.2f} deg"
                    )
                    break
        except Exception as exc:  # noqa: BLE001 - explicit abort
            print(f"FLIGHT ABORTED: {type(exc).__name__}: {exc}")
            raise
        finally:
            booster.controls.cut_thrust()
            service.stop()
            log.close()

        final = booster.snapshot()
        if final is not None:
            xy_error = math.hypot(float(final.position.x), float(final.position.y))
            print(
                f"Final error: xy={xy_error:.3f} m, "
                f"z={float(final.position.z) - target_height:.3f} m, "
                f"mass={final.mass:.1f} kg, situation={final.situation.name}"
            )
        print(f"Flight log: {LOG_PATH}")


if __name__ == "__main__":
    main()
