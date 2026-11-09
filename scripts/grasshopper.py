"""Grasshopper hop: vertical ascent to 5 km, engine cutoff, 6-DOF SCvx landing.

Mission profile
---------------
1. Ignite on the pad and follow a vertical velocity profile to ``ASCENT_ALT``
   (default 5 km).  The throttle is held inside the engine's allowed
   ``[MIN_THROTTLE, 1]`` band.
2. Cut the engine at the top, coast at least ``COAST_S`` seconds down to
   ``DESCENT_VZ`` (attitude held vertical, RCS active).
3. Solve a 6-DOF OpenSCvx powered-descent problem from the post-coast state to
   the launch-pad centre, then re-seed it from the current measurement every
   ``MPC_PERIOD`` seconds and track the latest usable plan at 50 Hz.  The
   control loop never blocks on the asynchronous solver.
4. Cut the engine at touchdown and report the landing error.

The target frame is the landing-site frame built by ``recovery``: origin at
the pad surface, ``+z`` local up.  ``target_height`` is the vessel CoM height
when it is sitting on its legs (measured before liftoff), so the plan's final
position is the actual touchdown CoM position.

Run with the KRPC environment::

    D:\\miniforge3\\envs\\KRPC\\python.exe scripts/grasshopper.py
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import replace

import numpy as np

from recovery import ConnectionManager, FramePacer
from recovery.control import LocalAttitudeController
from recovery.data.targets import LAUNCHPAD_JNSQ
from recovery.guidance import (
    AttitudeModel,
    EngineModel,
    PlanTracker,
    PolynomialConfig,
    PolynomialTerminalGuidance,
    ScvxConfig,
    ScvxSolveService,
    TerminalTarget,
    TrackingConfig,
    build_six_dof_problem,
)
from recovery.specs import BodySpec, DragSpec
from recovery.types import FlightState, Situation, Vector3

# ---------------------------------------------------------------------------
# Mission tuning
# ---------------------------------------------------------------------------

VESSEL = "VTVL"
BOOSTER_ID = "vtvl"
TARGET = LAUNCHPAD_JNSQ

ASCENT_ALT = 5_000.0      # cutoff altitude above the pad surface (m)
VZ_MAX = 150.0            # maximum ascent vertical speed (m/s)
A_BRAKE = 4.0             # ascent profile deceleration near cutoff (m/s虏)
ASCENT_KP = 0.25          # vertical-speed loop gain (1/s)

COAST_S = 4.0             # minimum engine-off coast before the SCvx solve (s)
DESCENT_VZ = -100.0       # state that seeds the SCvx descent (m/s)
MIN_THROTTLE = 0.40       # engine lower throttle limit
LOOP_HZ = 50.0            # control-loop rate
SCVX_N = 18               # transcription nodes
SCVX_ITERS = 40           # SCP iterations per refresh
SCVX_TF_HINT = 45.0       # time-of-flight seed (s)
IGNITION_GUESS = 15.0     # initial guess for a free ignition time (s)
TERMINAL_ALT = 500.0      # hand off from SCvx to terminal guidance at this height (m)
HOVERSLAM_ALT = 50.0      # below this height, prioritise upright attitude over lateral correction
MPC_PERIOD = 2.0          # re-seed the SCvx horizon from the current state (s)
COAST_FRACTION = 1e-3     # plan thrust below this fraction of max is a coast
TERM_KP = 0.8             # terminal horizontal position gain (1/s虏)
TERM_KD = 1.8             # terminal horizontal velocity gain (1/s)
TERM_MAX_HACC = 4.0       # terminal horizontal acceleration limit (m/s虏)

# Plan-tracking feedback (thrust-acceleration correction, m/s虏).
KP_POS = 0.20
KD_VEL = 1.00
MAX_FEEDBACK = 2.0

G0 = 9.80665
LOG_PATH = "grasshopper_flight.csv"


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def _vec3(vector: np.ndarray | tuple[float, float, float]) -> Vector3:
    return Vector3(float(vector[0]), float(vector[1]), float(vector[2]))


def _engine_model_from_part(vessel: object, state: FlightState) -> EngineModel:
    """Engine capability before ignition (when ``FlightState`` reports zeros)."""
    engines = list(vessel.parts.engines)  # type: ignore[attr-defined]
    active = [engine for engine in engines if bool(engine.active)]
    engine = active[0] if active else engines[0]
    return EngineModel(
        available_thrust=float(engine.max_thrust),
        isp=float(engine.specific_impulse),
        dry_mass=float(state.dry_mass),
    )


def _ascent_command(
    state: FlightState,
    target_z: float,
    up: np.ndarray,
) -> tuple[float, tuple[float, float, float]]:
    """Vertical profile plus a gentle lateral station-keeping term.

    Holding x/y near the pad during the 5 km climb removes most of the wind
    drift before the terminal phase; without it the booster can be hundreds of
    metres downwind by cutoff.
    """
    h = target_z - float(state.position.z)
    vz = float(state.velocity.z)
    if h <= 0.0:
        return 0.0, (float(up[0]), float(up[1]), float(up[2]))
    brake_height = VZ_MAX * VZ_MAX / (2.0 * A_BRAKE)
    vz_des = min(VZ_MAX, math.sqrt(2.0 * A_BRAKE * h))
    feedforward = -A_BRAKE if h < brake_height else 0.0
    a_cmd = feedforward + ASCENT_KP * (vz_des - vz)
    vertical_acc = G0 + a_cmd

    position = np.asarray(state.position, dtype=float)
    velocity = np.asarray(state.velocity, dtype=float)
    horizontal_acc = -0.15 * position[:2] - 0.8 * velocity[:2]
    horizontal_norm = float(np.linalg.norm(horizontal_acc))
    if horizontal_norm > 1.5:
        horizontal_acc *= 1.5 / horizontal_norm

    acceleration = (
        np.array([horizontal_acc[0], horizontal_acc[1], 0.0], dtype=float)
        + vertical_acc * up
    )
    magnitude = float(np.linalg.norm(acceleration))
    thrust = max(float(state.available_thrust), float(state.max_thrust))
    if thrust <= 0.0 or magnitude <= 1e-9:
        return 0.0, (float(up[0]), float(up[1]), float(up[2]))
    direction = acceleration / magnitude
    throttle = float(state.mass) * magnitude / thrust
    return _clamp(throttle, MIN_THROTTLE, 1.0), (
        float(direction[0]),
        float(direction[1]),
        float(direction[2]),
    )


def _synthetic_descent_state(
    state: FlightState,
    *,
    mass: float,
    altitude: float,
    vertical_speed: float,
    ut: float,
) -> FlightState:
    """A post-cutoff state used only to size/compile the OCP before launch."""
    return replace(
        state,
        ut=ut,
        met=float(state.met),
        position=Vector3(0.0, 0.0, altitude),
        velocity=Vector3(0.0, 0.0, vertical_speed),
        altitude=altitude,
        surface_altitude=altitude,
        mass=mass,
        thrust=0.0,
        throttle=0.0,
        available_thrust=float(state.available_thrust),
        max_thrust=float(state.max_thrust),
        max_vacuum_thrust=float(state.max_vacuum_thrust),
        specific_impulse=float(state.specific_impulse),
        max_acceleration=float(state.available_thrust) / max(mass, 1e-3),
        situation=Situation.FLYING,
        landed=False,
    )


def _terminal_command(
    state: FlightState,
    body_spec: BodySpec,
    drag_spec: DragSpec,
    target_height: float,
) -> tuple[float, tuple[float, float, float], np.ndarray]:
    """Terminal hoverslam: zero vertical speed at the pad, correct lateral drift.

    This is used only for the last ``TERMINAL_ALT`` metres.  It is a
    predictor-free feedback law: the vertical term asks for the constant
    deceleration that nulls ``vz`` at the target, and the horizontal term
    damps the measured lateral position/velocity.  The SCvx plan owns the
    high-altitude descent; this law removes the accumulated wind/model error
    at touchdown.
    """
    position = np.asarray(state.position, dtype=float)
    velocity = np.asarray(state.velocity, dtype=float)
    center = np.asarray(body_spec.body_center, dtype=float)
    up = -center / max(float(np.linalg.norm(center)), 1e-9)
    radius = float(np.linalg.norm(position - center))
    g_local = float(body_spec.mu) / max(radius * radius, 1.0)
    height = float(position[2]) - float(target_height)
    vz = float(velocity[2])

    # Atmospheric drag along the velocity (positive upward for a descent).
    sea_radius = float(drag_spec.sea_level_radius)
    altitude = radius - sea_radius
    alts = np.asarray(drag_spec.density_alts, dtype=float)
    vals = np.asarray(drag_spec.density_vals, dtype=float)
    if altitude <= 0.0 or altitude >= alts[-1]:
        density = float(vals[0]) if altitude <= 0.0 else 0.0
    else:
        density = float(np.interp(altitude, alts, vals))
    drag_force = 0.5 * density * float(drag_spec.cd_area or 0.0) * vz * abs(vz)
    drag_acc = drag_force / max(float(state.mass), 1e-6)

    if height <= 0.05:
        vertical_acc = g_local
    else:
        required_decel = (vz * vz) / (2.0 * height)
        vertical_acc = g_local + required_decel - drag_acc

    # Lateral policy near the ground:
    #   > 200 m : full position/velocity PD
    #   200..20 : velocity damping only (small tilt, no position chasing)
    #   < 20 m  : vertical attitude; the legs must take any residual drift
    if height > 200.0:
        horizontal_acc = -TERM_KP * position[:2] - TERM_KD * velocity[:2]
    else:
        # Below 200 m do not chase position, but always kill lateral velocity.
        # Leaving lateral speed at touchdown tips the booster even when the
        # vertical speed and tilt look perfect.
        horizontal_acc = -2.0 * velocity[:2]
    horizontal_norm = float(np.linalg.norm(horizontal_acc))
    if horizontal_norm > TERM_MAX_HACC:
        horizontal_acc *= TERM_MAX_HACC / horizontal_norm

    thrust_capability = max(float(state.available_thrust), float(state.max_thrust))
    if thrust_capability <= 0.0:
        return 0.0, (0.0, 0.0, 1.0), np.array([0.0, 0.0, target_height])
    accel_limit = thrust_capability / max(float(state.mass), 1e-6)
    vertical_acc = min(vertical_acc, accel_limit)
    remaining = max(accel_limit * accel_limit - vertical_acc * vertical_acc, 0.0) ** 0.5
    if horizontal_norm > remaining and horizontal_norm > 1e-9:
        horizontal_acc *= remaining / horizontal_norm
    horizontal_3d = np.array(
        [horizontal_acc[0], horizontal_acc[1], 0.0], dtype=float
    )
    acceleration = horizontal_3d + vertical_acc * up
    magnitude = float(np.linalg.norm(acceleration))
    if magnitude < 1e-9:
        return 0.0, (0.0, 0.0, 1.0), np.array([0.0, 0.0, target_height])
    throttle = float(state.mass) * magnitude / thrust_capability
    throttle = _clamp(throttle, MIN_THROTTLE, 1.0)
    direction = acceleration / magnitude
    return throttle, (float(direction[0]), float(direction[1]), float(direction[2])), np.array(
        [0.0, 0.0, target_height]
    )


def _polynomial_terminal_command(
    state: FlightState,
    body_spec: BodySpec,
    drag_spec: DragSpec,
    target_height: float,
    guidance: PolynomialTerminalGuidance,
) -> tuple[float, tuple[float, float, float], np.ndarray, str]:
    """Polynomial terminal guidance with drag feed-forward.

    The cubic profile supplies the desired total acceleration; the engine
    command is ``a_total - gravity - drag``.  If the polynomial returns a coast
    command (or a degenerate acceleration), the caller falls back to the
    hoverslam law.
    """
    position = np.asarray(state.position, dtype=float)
    velocity = np.asarray(state.velocity, dtype=float)
    center = np.asarray(body_spec.body_center, dtype=float)
    radius = float(np.linalg.norm(position - center))
    g_local = float(body_spec.mu) / max(radius * radius, 1.0)
    command = guidance.command(
        state,
        target=TerminalTarget((0.0, 0.0, float(target_height)), (0.0, 0.0, 0.0)),
        gravity=(0.0, 0.0, -g_local),
        available_thrust=max(float(state.available_thrust), float(state.max_thrust)),
    )
    if command.status != "terminal":
        return 0.0, (0.0, 0.0, 1.0), np.array([0.0, 0.0, target_height]), "poly-coast"

    # Drag feed-forward: subtract the aerodynamic acceleration from the engine
    # command so the total acceleration follows the polynomial profile.
    altitude = radius - float(drag_spec.sea_level_radius)
    alts = np.asarray(drag_spec.density_alts, dtype=float)
    vals = np.asarray(drag_spec.density_vals, dtype=float)
    if altitude <= 0.0:
        density = float(vals[0])
    elif altitude >= alts[-1]:
        density = 0.0
    else:
        density = float(np.interp(altitude, alts, vals))
    speed = float(np.linalg.norm(velocity))
    drag_acc = np.zeros(3, dtype=float)
    if speed > 1e-9:
        drag_force = (
            0.5
            * density
            * float(drag_spec.cd_area or 0.0)
            * speed
            * velocity
        )
        drag_acc = -drag_force / max(float(state.mass), 1e-6)
    engine_acc = np.asarray(command.thrust, dtype=float) / max(float(state.mass), 1e-6)
    engine_acc = engine_acc - drag_acc
    magnitude = float(np.linalg.norm(engine_acc))
    if not np.isfinite(magnitude) or magnitude < 1e-9:
        return 0.0, (0.0, 0.0, 1.0), np.array([0.0, 0.0, target_height]), "poly-coast"
    thrust_capability = max(float(state.available_thrust), float(state.max_thrust))
    throttle = _clamp(
        float(state.mass) * magnitude / max(thrust_capability, 1e-6),
        MIN_THROTTLE,
        1.0,
    )
    direction = engine_acc / magnitude
    return throttle, (float(direction[0]), float(direction[1]), float(direction[2])), np.array(
        [0.0, 0.0, target_height]
    ), "poly"


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    with ConnectionManager(address="127.0.0.1") as manager:
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

        target_height = float(state.position.z)
        body_spec, drag_spec = booster.sample_predictor_specs(mass=float(booster.raw.mass))
        up_vector = -np.asarray(body_spec.body_center, dtype=float) / max(
            float(np.linalg.norm(np.asarray(body_spec.body_center, dtype=float))), 1e-9
        )
        print(
            f"Vessel {VESSEL!r}: mass={state.mass:.0f} kg dry={state.dry_mass:.0f} kg "
            f"target_height={target_height:.3f} m"
        )
        print(
            f"Body: r={body_spec.body_radius:.0f} m mu={body_spec.mu:.3e} "
            f"CdA={drag_spec.cd_area:.3f} m虏 beta={drag_spec.ballistic_coefficient:.1f}"
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

        # The SCvx attitude model must include RCS torque, so build it after
        # RCS has been enabled.  The engine's gimbal authority is not reported
        # while the engine is off; RCS + reaction wheels are the conservative
        # envelope used to compile the problem.
        engine_model = _engine_model_from_part(booster.raw, state)
        attitude_model = AttitudeModel.from_state(state)
        print(
            f"Engine: Tmax={engine_model.available_thrust:.0f} N "
            f"Isp={engine_model.isp:.1f} s; "
            f"torque={attitude_model.max_torque} N路m"
        )

        # Size and compile the 6-DOF OCP while still on the pad.  The actual
        # initial state is injected at the post-coast refresh; the synthetic
        # state only needs to make the bounds large enough for the real hop.
        synthetic = _synthetic_descent_state(
            state,
            mass=float(state.mass),
            altitude=ASCENT_ALT - 300.0,
            vertical_speed=-200.0,
            ut=float(state.ut) + 60.0,
        )
        config = ScvxConfig(
            n=SCVX_N,
            min_throttle=0.0,
            max_iters_per_refresh=SCVX_ITERS,
            horizon_factor=2.0,
            horizon_min=1.0,
            multishot_plan=True,
            free_ignition=False,
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
        print("Compiling 6-DOF SCvx problem (about 10 s)...")
        service.initialize()
        service.start()

        tracker = PlanTracker(
            booster_id=BOOSTER_ID,
            service=service,
            tracking=TrackingConfig(
                min_throttle=MIN_THROTTLE,
                kp_position=KP_POS,
                kd_velocity=KD_VEL,
                max_feedback=MAX_FEEDBACK,
                max_tilt_deg=float(config.max_tilt_deg),
                coast_fraction=COAST_FRACTION,
            ),
            target_height=target_height,
            mpc_period=MPC_PERIOD,
            terminal_alt=TERMINAL_ALT,
            tf_hint=SCVX_TF_HINT,
        )

        # Safety net: the user can always quickload this save.
        booster.client.space_center.quicksave()
        print("Saved quicksave.  Igniting...")

        # Ignite at zero throttle, then let the ascent controller take over.
        booster.controls.activate_next_stage()
        time.sleep(0.2)
        # Staging can reset the RCS/SAS state; the write-through cache would
        # otherwise keep suppressing a re-write because it believes RCS is
        # already on.  Clear it, then re-assert through the gateway.
        booster.controls.clear_cache()
        booster.controls.apply(rcs=True, sas=False, legs=True)
        booster.controls.raw.rcs = True
        booster.controls.raw.sas = False
        check = booster.snapshot()
        if check is not None:
            print(f"Post-stage RCS torque: {check.available_rcs_torque}")

        attitude = LocalAttitudeController()
        terminal_guidance = PolynomialTerminalGuidance(
            PolynomialConfig(min_time_to_go=4.0, time_to_go_gain=2.0)
        )
        pacer = FramePacer(hz=LOOP_HZ)
        log = open(LOG_PATH, "w", encoding="utf-8", newline="")
        log.write(
            "wall,ut,phase,z,vz,speed,mass,throttle,tilt_deg,nose_z,cmd_nose_z,"
            "plan_frac,coast,tilt_limited,plan_elapsed,plan_duration,"
            "plan_iters,plan_converged,plan_age,replans,err_x,err_y,err_z,"
            "desired_x,desired_y,desired_z,solver_error,plan_source,terminal_source\n"
        )
        log.flush()

        phase = "ascent"
        coast_ut = 0.0
        last_plan_source = ""
        last_replans = 0
        terminal_source = "plan"
        last_log = time.monotonic()
        started = time.monotonic()
        touchdown_z = target_height
        last_snapshot_ut: float | None = None
        last_snapshot_change = time.monotonic()

        def log_row(
            *,
            s: FlightState,
            throttle: float,
            plan_elapsed: float,
            desired_p: np.ndarray | None,
            solver_error: str | None,
            nose: tuple[float, float, float] = (0.0, 0.0, 1.0),
            plan_fraction: float = float("nan"),
            coast: bool = False,
            tilt_limited: bool = False,
        ) -> None:
            now = time.monotonic()
            plan = tracker.plan
            plan_duration = float(plan.duration) if plan is not None else float("nan")
            plan_iters = int(plan.iterations) if plan is not None else 0
            plan_converged = bool(plan.converged) if plan is not None else False
            plan_age = (
                max(0.0, now - tracker.last_accept_wall)
                if plan is not None
                else float("nan")
            )
            tilt_deg = math.degrees(
                math.acos(min(max(float(s.direction.z), -1.0), 1.0))
            )
            if desired_p is None:
                err = np.asarray(s.position, dtype=float) - np.array(
                    [0.0, 0.0, touchdown_z], dtype=float
                )
                des = np.array([float("nan")] * 3)
            else:
                des = desired_p
                err = np.asarray(s.position, dtype=float) - des
            log.write(
                f"{now - started:.3f},{s.ut:.3f},{phase},"
                f"{s.position.z:.3f},{s.velocity.z:.3f},"
                f"{math.hypot(s.velocity.x, s.velocity.y, s.velocity.z):.3f},"
                f"{s.mass:.2f},{throttle:.4f},{tilt_deg:.3f},"
                f"{float(s.direction.z):.4f},{float(nose[2]):.4f},"
                f"{plan_fraction:.6f},{int(coast)},{int(tilt_limited)},"
                f"{plan_elapsed:.3f},{plan_duration:.3f},"
                f"{plan_iters},{int(plan_converged)},{plan_age:.3f},"
                f"{tracker.replans},"
                f"{err[0]:.3f},{err[1]:.3f},{err[2]:.3f},"
                f"{des[0]:.3f},{des[1]:.3f},{des[2]:.3f},"
                f"{'' if solver_error is None else solver_error.replace(',', ';')},"
                f"{last_plan_source},{terminal_source}\n"
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
                    print(f"ABORT: telemetry thread failed: {telemetry_error!r}")
                    raise RuntimeError(f"telemetry thread failed: {telemetry_error!r}")
                if last_snapshot_ut is None or float(s.ut) != last_snapshot_ut:
                    last_snapshot_ut = float(s.ut)
                    last_snapshot_change = now
                elif now - last_snapshot_change > 1.5:
                    raise RuntimeError(
                        f"telemetry stalled at ut={s.ut:.3f} for "
                        f"{now - last_snapshot_change:.1f}s"
                    )

                if phase == "ascent":
                    throttle, nose = _ascent_command(
                        s, ASCENT_ALT, up_vector
                    )
                    sticks = attitude.step(s, nose)
                    booster.controls.apply(
                        throttle=throttle,
                        pitch=sticks.pitch,
                        yaw=sticks.yaw,
                        roll=sticks.roll,
                    )
                    log_row(s=s, throttle=throttle, plan_elapsed=0.0,
                            desired_p=None, solver_error=None)
                    if now - last_log >= 1.0:
                        print(
                            f"[ascent] t={now - started:5.1f}s z={s.position.z:8.1f} "
                            f"vz={s.velocity.z:6.1f} thr={throttle:.3f} "
                            f"mass={s.mass:7.0f}"
                        )
                        last_log = now
                    if s.position.z >= ASCENT_ALT:
                        booster.controls.throttle = 0.0
                        phase = "coast"
                        coast_ut = float(s.ut)
                        print(
                            f"Engine cutoff at z={s.position.z:.1f} m "
                            f"vz={s.velocity.z:.1f} m/s; coasting to "
                            f"{DESCENT_VZ:.0f} m/s for the SCvx solve"
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
                    log_row(s=s, throttle=0.0, plan_elapsed=0.0,
                            desired_p=None, solver_error=None)
                    if now - last_log >= 1.0:
                        print(
                            f"[coast ] t={now - started:5.1f}s z={s.position.z:8.1f} "
                            f"vz={s.velocity.z:6.1f} mass={s.mass:7.0f}"
                        )
                        last_log = now
                    if (
                        float(s.ut) - coast_ut >= COAST_S
                        and s.velocity.z <= DESCENT_VZ
                    ):
                        phase = "descent"
                        print(
                            f"SCvx descent phase at z={s.position.z:.1f} m "
                            f"vz={s.velocity.z:.1f} m/s mass={s.mass:.0f} kg; "
                            f"MPC period {MPC_PERIOD:.1f}s"
                        )
                    continue

                # ---- powered descent -------------------------------------
                terminal_source = "plan"
                if s.position.z <= target_height + TERMINAL_ALT:
                    if s.position.z <= target_height + 50.0:
                        # Bypass the write-through cache: staging may have
                        # changed the real control state behind our back.
                        booster.controls.raw.legs = True
                        booster.controls.raw.rcs = True
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
                                )
                            )
                        except Exception as exc:  # noqa: BLE001 - fall back
                            print(f"[terminal] polynomial error: {exc!r}; using hoverslam")
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
                    elapsed = 0.0
                    terminal_mode = True
                    plan_fraction = float("nan")
                    coast = False
                    tilt_limited = False
                else:
                    terminal_mode = False
                    cmd = tracker.step(s)
                    if cmd is None:
                        plan_stats = service.stats(BOOSTER_ID)
                        if float(s.ut) - tracker.started_ut > 20.0:
                            raise RuntimeError(
                                "SCvx plan not usable after 20 s "
                                f"(iterations={plan_stats.iterations}, "
                                f"converged={plan_stats.converged}, "
                                f"solver={plan_stats.solver_converged}, "
                                f"error={service.last_error(BOOSTER_ID)!r})"
                            )
                        sticks = attitude.step(s, (0.0, 0.0, 1.0))
                        booster.controls.apply(
                            throttle=0.0,
                            pitch=sticks.pitch,
                            yaw=sticks.yaw,
                            roll=sticks.roll,
                        )
                        log_row(s=s, throttle=0.0, plan_elapsed=0.0,
                                desired_p=None, solver_error=service.last_error(BOOSTER_ID))
                        continue

                    if tracker.replans != last_replans:
                        last_replans = tracker.replans
                        plan_stats = service.stats(BOOSTER_ID)
                        last_plan_source = plan_stats.plan_source
                        plan = tracker.plan
                        if plan is not None:
                            print(
                                f"[mpc] plan #{tracker.replans} "
                                f"t0_ut={plan.t0_ut:.1f} "
                                f"duration={plan.duration:.1f}s "
                                f"ignition={plan.ignition_time} "
                                f"source={last_plan_source or '-'}"
                            )

                    elapsed = tracker.elapsed
                    throttle, nose, desired_p = (
                        cmd.throttle,
                        cmd.nose,
                        cmd.desired_position,
                    )
                    plan_fraction = cmd.planned_fraction
                    coast = cmd.coast
                    tilt_limited = cmd.tilt_limited

                sticks = attitude.step(s, nose)
                booster.controls.apply(
                    throttle=throttle,
                    pitch=sticks.pitch,
                    yaw=sticks.yaw,
                    roll=sticks.roll,
                )
                log_row(
                    s=s,
                    throttle=throttle,
                    plan_elapsed=elapsed,
                    desired_p=desired_p,
                    solver_error=service.last_error(BOOSTER_ID),
                    nose=nose,
                    plan_fraction=plan_fraction,
                    coast=coast,
                    tilt_limited=tilt_limited,
                )
                if now - last_log >= 1.0:
                    stats = service.stats(BOOSTER_ID)
                    plan = tracker.plan
                    plan_text = (
                        f"terminal:{terminal_source}"
                        if terminal_mode
                        else (
                            f"{elapsed:5.1f}/"
                            f"{plan.duration if plan is not None else float('nan'):5.1f}s"
                        )
                    )
                    tilt_deg = math.degrees(
                        math.acos(min(max(float(s.direction.z), -1.0), 1.0))
                    )
                    print(
                        f"[descent] t={now - started:5.1f}s z={s.position.z:8.1f} "
                        f"vz={s.velocity.z:7.2f} thr={throttle:.3f} "
                        f"tilt={tilt_deg:5.1f} "
                        f"cmd_z={float(nose[2]):5.3f} frac={plan_fraction:7.5f} "
                        f"{plan_text} iters={stats.iterations} "
                        f"conv={stats.converged} solver={stats.solver_converged} "
                        f"replans={tracker.replans} "
                        f"src={stats.plan_source or '-'} "
                        f"err={service.last_error(BOOSTER_ID)}"
                    )
                    last_log = now

                # Touchdown: cut the engine when the CoM reaches its
                # on-the-legs height, or when KSP reports a landed situation.
                # Keep the RCS/attitude loop alive for a short settle window:
                # cutting the sticks at the instant of contact lets a small
                # residual tilt/rate topple the booster.
                if s.landed or (
                    s.position.z <= target_height + 0.12
                    and s.velocity.z > -1.2
                ):
                    booster.controls.throttle = 0.0
                    booster.controls.apply(pitch=0.0, yaw=0.0, roll=0.0)
                    print(
                        f"Touchdown: z={s.position.z:.3f} m "
                        f"(target {target_height:.3f}), "
                        f"vz={s.velocity.z:.3f} m/s, "
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
                        # Do not drive attitude on the ground: a stick command
                        # with the legs planted produces a toppling torque.
                        booster.controls.apply(
                            throttle=0.0, pitch=0.0, yaw=0.0, roll=0.0
                        )
                    booster.controls.cut_thrust()
                    tilt = math.degrees(
                        math.acos(min(max(float(s.direction.z), -1.0), 1.0))
                    )
                    print(
                        f"Settled: z={s.position.z:.3f} m, "
                        f"vz={s.velocity.z:.3f} m/s, tilt={tilt:.2f} deg"
                    )
                    break
        except Exception as exc:  # noqa: BLE001 - explicit flight abort
            print(f"FLIGHT ABORTED: {type(exc).__name__}: {exc}")
            raise
        finally:
            booster.controls.cut_thrust()
            service.stop()
            log.close()

        final = booster.snapshot()
        if final is not None:
            xy_error = math.hypot(float(final.position.x), float(final.position.y))
            z_error = float(final.position.z) - target_height
            print(
                f"Landing error: xy={xy_error:.3f} m, z={z_error:.3f} m, "
                f"mass={final.mass:.1f} kg, situation={final.situation.name}"
            )
        print(f"Flight log: {LOG_PATH}")


if __name__ == "__main__":
    main()
