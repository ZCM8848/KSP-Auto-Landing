"""C-scheme hopper: the SCvx OCP chooses its own engine ignition time.

This script is deliberately separate from the stable ``grasshopper.py`` so it
can be observed without changing the known-good fixed-ignition mission.

Sequence
--------
1. Ascent to 5 km, cutoff, coast to ``-100 m/s``.
2. Controlled powered descent to a 2 km hover checkpoint using the
   predictor-free terminal law (vertical braking plus lateral damping).
3. Hold the 2 km hover for ``HOVER_HOLD_S`` seconds while pre-solving the
   landing OCP from the hover state.
4. Track the landing OCP with a free ``ignition_time`` state, re-seeding it
   from the current measurement every ``MPC_PERIOD`` seconds, and hand off to
   polynomial terminal guidance at 500 m.
5. Cut at contact; no stick commands on the ground.

The console and ``grasshopper_c_flight.csv`` expose:
  * the plan's chosen ignition time and altitude;
  * the actual ignition time;
  * plan vs actual lateral error and the commanded tilt;
  * whether a command was a planned coast or hit the tilt guard;
  * solver convergence, plan source, and the number of accepted replans.

Run with the KRPC environment::

    D:\\miniforge3\\envs\\KRPC\\python.exe scripts/grasshopper_c.py

The green debug line is the remaining plan; the orange line is the actual
flown path.  The script does **not** silently fall back to the fixed ignition
scheme: if no usable plan appears, it aborts with a clear error.
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

# The reusable helpers live in the stable script; importing it has no side
# effects because its ``main()`` is guard-protected.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from grasshopper import (  # noqa: E402
    ASCENT_ALT,
    HOVERSLAM_ALT,
    IGNITION_GUESS,
    MIN_THROTTLE,
    TERMINAL_ALT,
    _ascent_command,
    _engine_model_from_part,
    _polynomial_terminal_command,
    _synthetic_descent_state,
    _terminal_command,
)

from recovery import ConnectionManager, FramePacer
from recovery.control import LocalAttitudeController
from recovery.data.targets import LAUNCHPAD_JNSQ
from recovery.guidance import (
    AttitudeModel,
    PlanThrust,
    PlanTracker,
    PolynomialConfig,
    PolynomialTerminalGuidance,
    ScvxConfig,
    ScvxSolveService,
    TofFeatureParams,
    TofPredictor,
    TrackingConfig,
    build_six_dof_problem,
    features_of,
)
from recovery.specs import BodySpec
from recovery.types import FlightState, Situation

VESSEL = "VTVL"
BOOSTER_ID = "vtvl_c"
TARGET = LAUNCHPAD_JNSQ

COAST_VZ = -100.0          # state that ends the initial coast (m/s)
COAST_MIN_S = 4.0          # minimum engine-off time before the seed
HOVER_ALT = 2_000.0        # intermediate hover checkpoint above the pad (m)
HOVER_HOLD_S = 3.0         # hover duration before starting the landing OCP (s)
HOVER_BAND = 5.0           # altitude band for switching to hover (m)
HOVER_VZ_TOL = 1.0         # vertical-speed band for switching to hover (m/s)
HOVER_KP = 0.40            # hover altitude gain (1/s^2)
HOVER_KD = 1.50            # hover vertical-speed gain (1/s)
HOVER_MAX_HACC = 2.0       # hover lateral acceleration limit (m/s^2)
SCVX_ITERS = 120           # root solve needs ~40; cap high enough to see it
SCVX_TF_HINT = 60.0        # free final time / ignition seed (s)
MAX_IGNITION_S = 2.0       # never coast below this delay from the solve seed (s)
MPC_PERIOD = 2.0           # re-seed the horizon from the current state (s)
DRAW_PERIOD = 1.0          # update the planned/actual debug lines (s)
TOF_PERIOD = 1.0           # refresh the TOF-Net prediction (s)
LOOP_HZ = 50.0
LOG_PATH = "grasshopper_c_flight.csv"


def _hover_command(
    state: FlightState,
    body_spec: BodySpec,
    *,
    hover_alt: float,
    up: np.ndarray,
) -> tuple[float, tuple[float, float, float], np.ndarray]:
    """Hold altitude and x/y near the launch-pad axis."""
    position = np.asarray(state.position, dtype=float)
    velocity = np.asarray(state.velocity, dtype=float)
    center = np.asarray(body_spec.body_center, dtype=float)
    radius = float(np.linalg.norm(position - center))
    g_local = float(body_spec.mu) / max(radius * radius, 1.0)
    height = float(position[2]) - float(hover_alt)
    vertical_acc = max(
        g_local + HOVER_KP * height - HOVER_KD * velocity[2],
        0.0,
    )

    horizontal_acc = -0.5 * position[:2] - 1.0 * velocity[:2]
    horizontal_norm = float(np.linalg.norm(horizontal_acc))
    if horizontal_norm > HOVER_MAX_HACC:
        horizontal_acc *= HOVER_MAX_HACC / horizontal_norm
    horizontal_3d = np.array(
        [horizontal_acc[0], horizontal_acc[1], 0.0], dtype=float
    )
    acceleration = horizontal_3d + vertical_acc * np.asarray(up, dtype=float)
    magnitude = float(np.linalg.norm(acceleration))
    thrust = max(float(state.available_thrust), float(state.max_thrust))
    desired = np.array([0.0, 0.0, float(hover_alt)], dtype=float)
    if magnitude <= 1e-9 or thrust <= 0.0:
        return 0.0, (0.0, 0.0, 1.0), desired
    throttle = min(max(float(state.mass) * magnitude / thrust, MIN_THROTTLE), 1.0)
    direction = acceleration / magnitude
    return (
        throttle,
        (float(direction[0]), float(direction[1]), float(direction[2])),
        desired,
    )


def _tof_prediction(
    predictor: TofPredictor | None,
    params: TofFeatureParams,
    state: FlightState,
    *,
    fallback_isp: float,
) -> tuple[float, float] | None:
    """TOF-Net ``(p_feasible, tf)`` or ``None`` if unavailable."""
    if predictor is None:
        return None
    try:
        if float(state.specific_impulse) <= 0.0:
            state = replace(state, specific_impulse=float(fallback_isp))
        p_feasible, tf = predictor.predict(features_of(state, params))
    except Exception:  # noqa: BLE001 - optional diagnostic aid
        return None
    if not (math.isfinite(p_feasible) and math.isfinite(tf)) or tf <= 0.0:
        return None
    return p_feasible, tf


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
            altitude=ASCENT_ALT - 300.0,
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
            max_ignition_time=MAX_IGNITION_S,
            monotonic_altitude=True,
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

        tof_params = TofFeatureParams(
            min_throttle=MIN_THROTTLE,
            max_throttle=1.0,
            glide_slope_angle_deg=float(config.glide_slope_deg),
            max_angle_deg=float(config.max_tilt_deg),
        )
        try:
            tof = TofPredictor()
            print(
                f"TOF-Net loaded (threshold={tof.threshold:.3f}, "
                f"features={len(tof.features)})"
            )
        except Exception as exc:  # noqa: BLE001 - optional diagnostic aid
            tof = None
            print(f"TOF-Net unavailable: {exc!r}")

        service = ScvxSolveService(workers=1)
        service.register(BOOSTER_ID, problem, config=config)
        print("Compiling free-ignition 6-DOF SCvx problem (~10 s)...")
        service.initialize()
        service.start()

        tracker = PlanTracker(
            booster_id=BOOSTER_ID,
            service=service,
            tracking=TrackingConfig(
                min_throttle=MIN_THROTTLE,
                kp_position=0.20,
                kd_velocity=1.00,
                max_feedback=2.0,
                max_tilt_deg=float(config.max_tilt_deg),
                light_fraction=MIN_THROTTLE,
                cutoff_fraction=0.05,
            ),
            target_height=target_height,
            mpc_period=MPC_PERIOD,
            terminal_alt=TERMINAL_ALT,
            tf_hint=SCVX_TF_HINT,
        )

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
            "wall,ut,phase,z,vz,x,y,throttle,tilt_deg,nose_z,cmd_nose_z,"
            "plan_frac,coast,tilt_limited,plan_elapsed,plan_duration,plan_iters,"
            "plan_ignition,actual_ignition,err_x,err_y,terminal_source,"
            "solver_iterations,solver_converged,plan_source,plan_age,replans,"
            "tof_p,tof_tf,solver_error\n"
        )
        log.flush()

        phase = "ascent"
        coast_ut = 0.0
        actual_ignition: float | None = None
        hover_started_ut = 0.0
        hover_landing_cmd: PlanThrust | None = None
        tof_p = float("nan")
        tof_tf = float("nan")
        last_tof_wall = 0.0
        terminal_source = "plan"
        last_log = time.monotonic()
        last_replans = 0
        last_draw_wall = 0.0
        actual_path: list[tuple[float, float, float]] = []
        started = time.monotonic()
        last_snapshot_ut: float | None = None
        last_snapshot_change = time.monotonic()

        def log_row(
            s: FlightState,
            throttle: float,
            elapsed: float,
            desired_p: np.ndarray | None,
            nose: tuple[float, float, float] = (0.0, 0.0, 1.0),
            plan_fraction: float = float("nan"),
            coast: bool = False,
            tilt_limited: bool = False,
        ) -> None:
            now = time.monotonic()
            plan = tracker.plan
            plan_duration = float(plan.duration) if plan is not None else float("nan")
            plan_iters = int(plan.iterations) if plan is not None else 0
            plan_ignition = plan.ignition_time if plan is not None else None
            plan_age = (
                max(0.0, now - tracker.last_accept_wall)
                if plan is not None
                else float("nan")
            )
            plan_source = ""
            solver_converged = ""
            solver_iterations = 0
            solver_error = ""
            if plan is not None:
                stats = service.stats(BOOSTER_ID)
                plan_source = stats.plan_source
                solver_converged = str(int(stats.solver_converged))
                solver_iterations = int(stats.iterations)
                solver_error = (stats.last_error or "").replace(",", ";")
            if desired_p is None:
                err = np.asarray(s.position, dtype=float) - np.array(
                    [0.0, 0.0, target_height], dtype=float
                )
            else:
                err = np.asarray(s.position, dtype=float) - desired_p
            tilt_deg = math.degrees(
                math.acos(min(max(float(s.direction.z), -1.0), 1.0))
            )
            log.write(
                f"{now - started:.3f},{s.ut:.3f},{phase},"
                f"{s.position.z:.3f},{s.velocity.z:.3f},"
                f"{s.position.x:.3f},{s.position.y:.3f},{throttle:.4f},"
                f"{tilt_deg:.3f},{float(s.direction.z):.4f},"
                f"{float(nose[2]):.4f},{plan_fraction:.6f},"
                f"{int(coast)},{int(tilt_limited)},"
                f"{elapsed:.3f},{plan_duration:.3f},{plan_iters},"
                f"{'' if plan_ignition is None else f'{float(plan_ignition):.3f}'},"
                f"{'' if actual_ignition is None else f'{actual_ignition:.3f}'},"
                f"{err[0]:.3f},{err[1]:.3f},{terminal_source},"
                f"{solver_iterations},{solver_converged},{plan_source},"
                f"{plan_age:.3f},{tracker.replans},{tof_p:.5f},{tof_tf:.3f},"
                f"{solver_error}\n"
            )
            log.flush()

        def update_tof(s: FlightState, now_wall: float) -> None:
            nonlocal tof_p, tof_tf, last_tof_wall
            if now_wall - last_tof_wall < TOF_PERIOD:
                return
            last_tof_wall = now_wall
            prediction = _tof_prediction(
                tof,
                tof_params,
                s,
                fallback_isp=engine_model.isp,
            )
            if prediction is None:
                return
            tof_p, tof_tf = prediction
            tracker.set_tf_hint(tof_tf)

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
                        phase = "approach"
                        print(
                            f"Powered approach to {HOVER_ALT:.0f} m at "
                            f"z={s.position.z:.1f} m vz={s.velocity.z:.1f} m/s, "
                            f"mass={s.mass:.0f} kg"
                        )
                    continue

                # ---- controlled descent to the 2 km hover checkpoint ------
                if phase == "approach":
                    terminal_source = "approach"
                    throttle, nose, desired_p = _terminal_command(
                        s, body_spec, drag_spec, HOVER_ALT
                    )
                    sticks = attitude.step(s, nose)
                    booster.controls.apply(
                        throttle=throttle,
                        pitch=sticks.pitch,
                        yaw=sticks.yaw,
                        roll=sticks.roll,
                    )
                    log_row(s, throttle, 0.0, desired_p, nose=nose)
                    if now - last_log >= 1.0:
                        print(
                            f"[approach] t={now - started:5.1f}s "
                            f"z={s.position.z:8.1f} vz={s.velocity.z:7.2f} "
                            f"thr={throttle:.3f}"
                        )
                        last_log = now
                    if (
                        s.position.z <= HOVER_ALT + HOVER_BAND
                        and abs(float(s.velocity.z)) <= HOVER_VZ_TOL
                    ):
                        phase = "hover"
                        hover_started_ut = float(s.ut)
                        hover_landing_cmd = None
                        print(
                            f"2 km hover reached at z={s.position.z:.1f} m, "
                            f"vz={s.velocity.z:.2f} m/s; holding "
                            f"{HOVER_HOLD_S:.1f}s"
                        )
                    continue

                # ---- hover checkpoint; pre-solve the landing OCP ---------
                if phase == "hover":
                    terminal_source = "hover"
                    update_tof(s, now)
                    cmd = tracker.step(s)
                    if cmd is None:
                        if float(s.ut) - tracker.started_ut > 20.0:
                            stats = service.stats(BOOSTER_ID)
                            raise RuntimeError(
                                "landing SCvx plan not usable after 20 s of hover "
                                f"(iters={stats.iterations}, "
                                f"converged={stats.converged}, "
                                f"solver={stats.solver_converged}, "
                                f"error={service.last_error(BOOSTER_ID)!r})"
                            )
                    else:
                        hover_landing_cmd = cmd
                    throttle, nose, desired_p = _hover_command(
                        s,
                        body_spec,
                        hover_alt=HOVER_ALT,
                        up=up_vector,
                    )
                    sticks = attitude.step(s, nose)
                    booster.controls.apply(
                        throttle=throttle,
                        pitch=sticks.pitch,
                        yaw=sticks.yaw,
                        roll=sticks.roll,
                    )
                    log_row(s, throttle, 0.0, desired_p, nose=nose)
                    if now - last_log >= 1.0:
                        print(
                            f"[hover ] t={now - started:5.1f}s "
                            f"z={s.position.z:8.1f} vz={s.velocity.z:7.2f} "
                            f"thr={throttle:.3f} plan_ready="
                            f"{hover_landing_cmd is not None}"
                        )
                        last_log = now
                    if (
                        float(s.ut) - hover_started_ut >= HOVER_HOLD_S
                        and hover_landing_cmd is not None
                    ):
                        phase = "descent"
                        print(
                            f"Landing SCvx seeded from {s.position.z:.1f} m hover"
                        )
                    continue

                # ---- free-ignition powered descent -----------------------
                terminal_source = "plan"
                if s.position.z <= target_height + TERMINAL_ALT:
                    update_tof(s, now)
                    height = s.position.z - target_height
                    if height > HOVERSLAM_ALT:
                        try:
                            throttle, nose, desired_p, terminal_source = (
                                _polynomial_terminal_command(
                                    s,
                                    body_spec,
                                    drag_spec,
                                    target_height,
                                    terminal_guidance,
                                    time_to_go=(
                                        tof_tf if math.isfinite(tof_tf) else None
                                    ),
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
                    log_row(s, throttle, 0.0, desired_p, nose=nose)
                    if now - last_log >= 1.0:
                        print(
                            f"[terminal] t={now - started:5.1f}s z={s.position.z:8.1f} "
                            f"vz={s.velocity.z:7.2f} thr={throttle:.3f} "
                            f"src={terminal_source}"
                        )
                        last_log = now
                else:
                    update_tof(s, now)
                    cmd = tracker.step(s)
                    if cmd is None:
                        stats = service.stats(BOOSTER_ID)
                        if float(s.ut) - tracker.started_ut > 20.0:
                            raise RuntimeError(
                                "free-ignition SCvx plan not usable after 20 s "
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

                    if tracker.replans != last_replans:
                        last_replans = tracker.replans
                        plan = tracker.plan
                        stats = service.stats(BOOSTER_ID)
                        if plan is not None:
                            print(
                                f"[mpc  ] plan #{tracker.replans} "
                                f"t0_ut={plan.t0_ut:.1f} "
                                f"duration={plan.duration:.2f}s "
                                f"ignition={plan.ignition_time} "
                                f"solver={stats.solver_converged} "
                                f"src={stats.plan_source or '-'}"
                            )

                    elapsed = tracker.elapsed
                    throttle = cmd.throttle
                    nose = cmd.nose
                    desired_p = cmd.desired_position
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
                    log_row(
                        s,
                        throttle,
                        elapsed,
                        desired_p,
                        nose=nose,
                        plan_fraction=cmd.planned_fraction,
                        coast=cmd.coast,
                        tilt_limited=cmd.tilt_limited,
                    )

                    if now - last_draw_wall >= DRAW_PERIOD:
                        last_draw_wall = now
                        try:
                            plan = tracker.plan
                            if plan is not None:
                                shifted = plan.shifted(elapsed)
                                if shifted is not None:
                                    positions = np.asarray(shifted.positions, dtype=float)
                                    step = max(1, len(positions) // 80)
                                    booster.debug.trajectory(
                                        positions[::step],
                                        name="c_plan",
                                        frame_name="target",
                                        color=(0.1, 1.0, 0.2),
                                        thickness=0.3,
                                    )
                            actual_path.append(
                                (
                                    float(s.position.x),
                                    float(s.position.y),
                                    float(s.position.z),
                                )
                            )
                            if len(actual_path) >= 2:
                                booster.debug.trajectory(
                                    actual_path,
                                    name="c_actual",
                                    frame_name="target",
                                    color=(1.0, 0.35, 0.1),
                                    thickness=0.25,
                                )
                        except Exception as exc:  # noqa: BLE001
                            print(f"[debug] could not draw trajectory: {exc!r}")

                    if now - last_log >= 1.0:
                        stats = service.stats(BOOSTER_ID)
                        plan = tracker.plan
                        plan_text = (
                            f"{float(plan.duration):5.1f}s"
                            if plan is not None
                            else "  nan s"
                        )
                        tilt_deg = math.degrees(
                            math.acos(min(max(float(s.direction.z), -1.0), 1.0))
                        )
                        print(
                            f"[descent] t={now - started:5.1f}s z={s.position.z:8.1f} "
                            f"vz={s.velocity.z:7.2f} thr={throttle:.3f} "
                            f"tilt={tilt_deg:5.1f} "
                            f"cmd_z={float(nose[2]):5.3f} frac={cmd.planned_fraction:7.5f} "
                            f"plan={elapsed:5.1f}/{plan_text} "
                            f"xy=({s.position.x:6.1f},{s.position.y:6.1f}) "
                            f"replans={tracker.replans} "
                            f"ign={actual_ignition if actual_ignition is not None else '-'} "
                            f"err={service.last_error(BOOSTER_ID)}"
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
