"""G-FOLD powered-descent landing guidance (multi-threaded replanner).

The vessel coasts (throttle zero, nose retrograde) while TOF-Net predicts
feasibility and the optimal time-of-flight; the landing burn ignites the moment
the prediction deems a landing feasible.  After ignition a dedicated replanner
thread predicts TOF and re-solves G-FOLD back-to-back at maximum rate.

The control thread follows the freshest trajectory *by time* (interpolating
along ``time_points`` with the elapsed mission time), which removes the
one-solver-latency lag that made the rocket shake.  Throttle and nose direction
are additionally low-passed.  Below ``DEAD_ZONE_ALT`` the replanner is halted
and the rocket simply follows the last solved trajectory to touchdown.
"""

import csv
import sys
import threading
import time

import numpy as np

sys.path.insert(0, "src")
from recovery import ConnectionManager, FramePacer
from recovery.control import LocalAttitudeController
from recovery.data.targets import LAUNCHPAD_JNSQ
from recovery.guidance.gfold import (
    GfoldParams,
    command_at_time,
    features_of,
    replan,
)
from recovery.guidance.tofnet import TofPredictor

VESSEL = "Booster 1"
TARGET = LAUNCHPAD_JNSQ

CONTROL_HZ = 30.0          # control-apply rate (Hz)
REPLAN_HZ = 4.0            # replan rate (Hz) — re-solve periodically, follow trajectory in between
TOF_MIN = 3.0              # require this much predicted time before ignition
DEAD_ZONE_ALT = 200.0      # below this altitude, stop replanning and follow the last trajectory
THROTTLE_SMOOTH = 0.25     # per-tick low-pass alpha on throttle (at CONTROL_HZ)
THROTTLE_MAX_STEP = 0.04   # max throttle change per tick (rate limit, at CONTROL_HZ)
DIRECTION_SMOOTH = 0.30    # per-tick lerp alpha on the nose direction

DRAW_HZ = 30.0             # debug-drawing rate (Hz)
ACC_ARROW_LEN = 40.0       # waypoint acceleration-arrow length (m)
TARGET_ARROW_LEN = 500.0   # rocket-centred target-direction arrow length (m)


def _fmt3(v) -> str:
    """Format a 3-vector as ``(x,y,z)`` with sign, for log lines."""
    return f"({v[0]:+.2f},{v[1]:+.2f},{v[2]:+.2f})"


def _ang(v0, v1) -> float:
    """Angle (deg) between two vectors."""
    a = np.asarray(v0, dtype=float)
    b = np.asarray(v1, dtype=float)
    na = float(np.linalg.norm(a))
    nb = float(np.linalg.norm(b))
    if na < 1e-12 or nb < 1e-12:
        return 0.0
    c = float(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0))
    return float(np.degrees(np.arccos(c)))


def _make_arrow_lines(booster, n: int, color, thickness: float):
    """Create *n* zero-length line segments (repositioned in place later)."""
    return [
        booster.debug.line(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            frame_name="target",
            color=color,
            thickness=thickness,
        )
        for _ in range(n)
    ]


class _CommandSlot:
    """Thread-safe holder for the latest solved trajectory.

    Carries the trajectory and the mission-elapsed time (``s.met``) of the
    snapshot it was solved from, so the control loop can interpolate along the
    trajectory by the elapsed simulation time instead of blindly re-applying
    node 0 (which lags by one solver latency).  Also carries the TOF-Net
    feasibility/TOF used for that solve, for telemetry.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._traj = None
        self._t0_met = 0.0
        self._p = 0.0
        self._tf = 0.0

    def set(self, traj, t0_met: float, p: float, tf: float) -> None:
        with self._lock:
            self._traj = traj
            self._t0_met = t0_met
            self._p = p
            self._tf = tf

    def get(self):
        with self._lock:
            return self._traj, self._t0_met, self._p, self._tf


class _DrawState:
    """Thread-safe holder for the latest command, consumed by the drawer thread.

    The control loop publishes the rocket position and smoothed command here
    (a cheap lock + assignment, no RPC); a dedicated drawer thread reads it and
    does all the debug-drawing RPCs so the 30 Hz control loop stays clean.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._position: tuple[float, float, float] | None = None
        self._sm_dir: tuple[float, float, float] | None = None

    def set(
        self,
        position: tuple[float, float, float],
        sm_dir: tuple[float, float, float] | None,
    ) -> None:
        with self._lock:
            self._position = position
            self._sm_dir = sm_dir

    def get(self):
        with self._lock:
            return self._position, self._sm_dir


def _drawer(
    booster,
    slot: _CommandSlot,
    draw_state: _DrawState,
    stop: threading.Event,
    log,
) -> None:
    """Draw trajectory, waypoint acceleration arrows, and the rocket-centred
    target direction on the dedicated debug connection at ``DRAW_HZ``.
    """
    acc_lines = None
    target_line = None
    while not stop.is_set():
        time.sleep(1.0 / DRAW_HZ)
        traj, _t0, _p, _tf = slot.get()
        rp, sm_dir = draw_state.get()
        try:
            if traj is not None:
                booster.debug.trajectory(
                    traj.positions,
                    name="gfold",
                    frame_name="target",
                    color=(1.0, 0.5, 0.0),
                    thickness=0.3,
                )
                pos = np.asarray(traj.positions, dtype=float)
                u = np.asarray(traj.u_values, dtype=float)
                npos = len(pos)
                if acc_lines is None or len(acc_lines) != npos:
                    if acc_lines is not None:
                        for ln in acc_lines:
                            ln.remove()
                    acc_lines = _make_arrow_lines(
                        booster, npos, color=(0.2, 0.8, 1.0), thickness=0.4
                    )
                for i, ln in enumerate(acc_lines):
                    mag = float(np.linalg.norm(u[i]))
                    if mag < 1e-9:
                        ln.set_points(tuple(pos[i]), tuple(pos[i]))
                    else:
                        d = u[i] / mag
                        start = tuple(pos[i])
                        ln.set_points(
                            start,
                            tuple(np.asarray(start) + d * ACC_ARROW_LEN),
                        )
            if sm_dir is not None and rp is not None:
                if target_line is None:
                    target_line = booster.debug.line(
                        rp,
                        rp,
                        frame_name="target",
                        color=(1.0, 0.3, 0.3),
                        thickness=1.0,
                    )
                target_line.set_points(
                    rp,
                    tuple(np.asarray(rp) + np.asarray(sm_dir) * TARGET_ARROW_LEN),
                )
        except Exception as exc:  # noqa: BLE001 — draw failure must not kill guidance
            log(f"draw error: {exc}")


def _planner(
    booster,
    g0: float,
    params: GfoldParams,
    tofnet: TofPredictor,
    slot: _CommandSlot,
    stop: threading.Event,
    log,
) -> None:
    """Replan G-FOLD at ``REPLAN_HZ``, TOF predicted by TOF-Net each cycle.

    Between re-solves the control loop follows the trajectory *by time* (the
    solver's node-0 throttle oscillates if re-solved back-to-back, which was
    the ~9 Hz engine pulse).  When TOF-Net's feasibility ``p`` drops below its
    threshold the replanner stops updating the trajectory entirely.
    """
    pacer = FramePacer(hz=REPLAN_HZ)
    last_report = 0.0
    while not stop.is_set():
        pacer.tick()
        s = booster.snapshot()
        if s is None:
            continue
        p, tf = tofnet.predict(features_of(s, params))
        if p < tofnet.threshold:
            continue
        traj = replan(s, g0, params, tof=tf)
        if traj is None:
            continue
        slot.set(traj, s.met, p, tf)

        now = time.monotonic()
        if now - last_report >= 0.2:
            last_report = now
            p0 = traj.positions[0]
            pn = traj.positions[-1]
            log(
                f"p={p:.3f} tf={tf:6.2f} "
                f"status={traj.status} "
                f"pos0=({p0[0]:7.1f},{p0[1]:7.1f},{p0[2]:7.1f}) "
                f"posN=({pn[0]:7.1f},{pn[1]:7.1f},{pn[2]:7.1f}) "
                f"mN={traj.final_mass:.0f}"
            )


def _wait_telemetry(km, booster_id: str, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while km.snapshot(booster_id) is None:
        if time.monotonic() > deadline:
            raise RuntimeError("telemetry not ready")
        time.sleep(0.02)


def main() -> None:
    with ConnectionManager(address="127.0.0.1") as km:
        b = km.add_booster("gfold", VESSEL)
        km.register_target("gfold", lon=TARGET.lon, lat=TARGET.lat)
        km.start()
        _wait_telemetry(km, "gfold")

        # -- logging (console + file) ----------------------------------------
        log_file = open("gfold_landing_debug.log", "w", encoding="utf-8")
        log_lock = threading.Lock()

        def log(msg: str) -> None:
            with log_lock:
                print(msg)
                print(msg, file=log_file)
                log_file.flush()

        # -- in-game debug drawing -------------------------------------------
        km.enable_debug()
        b.debug.reference_frame(frame_name="target", length=50.0)

        frame = km.frame("gfold", "target")
        g0 = b.body_spec.surface_gravity
        params = GfoldParams()
        tofnet = TofPredictor()

        b.physics_range = 200000.0
        b.controls.target_smoothing_time = 0.3
        b.controls.rcs = True
        b.controls.apply(throttle=0.0)

        log(f"target={TARGET}  g0={g0:.3f}  threshold={tofnet.threshold:.4f}")
        log("phase=coast")

        # -- coast / trigger: ignite only when the braking phase (thrust up) is
        # reached — during the initial dive the thrust points down and we coast.
        traj = None
        last_report = 0.0
        while True:
            s = b.snapshot()
            if s is None:
                continue
            if s.landed:
                return

            v = s.velocity
            spd = (v.x * v.x + v.y * v.y + v.z * v.z) ** 0.5
            if spd > 1e-6:
                b.controls.apply(
                    target_direction=(-v.x / spd, -v.y / spd, -v.z / spd),
                    reference_frame=frame,
                )

            p, tf = tofnet.predict(features_of(s, params))
            if p < tofnet.threshold or tf < TOF_MIN:
                continue
            traj = replan(s, g0, params, tof=tf)
            if traj is None:
                continue
            u0 = traj.u_values[0]
            now = time.monotonic()
            if now - last_report >= 1.0:
                last_report = now
                log(
                    f"coast alt={s.surface_altitude:7.1f}  "
                    f"p={p:.3f} tf={tf:6.2f}  uz={u0[2]:+.2f}"
                )
            if u0[2] > 0.0:  # thrust points up — braking phase reached
                break

        log(
            f"IGNITE  alt={s.surface_altitude:.1f}  "
            f"uz={traj.u_values[0][2]:+.2f}  |u|={np.linalg.norm(traj.u_values[0]):.2f}"
        )

        # -- ignition: the coast loop already solved a braking trajectory -----
        # Switch from the kRPC server-side AutoPilot to the local client-side
        # attitude controller (snapshot-driven stick-level control).
        b.controls.disengage_auto_pilot()
        b.controls.apply(sas=False)
        local_ap = LocalAttitudeController(settling_time=0.5)

        slot = _CommandSlot()
        slot.set(traj, s.met, p, tf)

        stop = threading.Event()
        planner = threading.Thread(
            target=_planner,
            args=(b, g0, params, tofnet, slot, stop, log),
            daemon=True,
        )
        planner.start()
        draw_state = _DrawState()
        draw_stop = threading.Event()
        drawer = threading.Thread(
            target=_drawer,
            args=(b, slot, draw_state, draw_stop, log),
            daemon=True,
        )
        drawer.start()
        log("phase=powered")

        # -- telemetry CSV (per-tick) for offline shake analysis --------------
        telemetry = open("gfold_telemetry.csv", "w", newline="", encoding="utf-8")
        csvw = csv.writer(telemetry)
        csvw.writerow([
            "met", "alt", "vx", "vy", "vz",
            "thr_raw", "thr_sm",
            "drx", "dry", "drz", "dsx", "dsy", "dsz",
            "nx", "ny", "nz", "avx", "avy", "avz",
            "ang_err_deg", "elapsed", "p", "tf",
            "u0mag", "mass", "thrust",
        ])

        # -- control loop: apply freshest command until landed ---------------
        pacer = FramePacer(hz=CONTROL_HZ)
        last_report = 0.0
        in_dead_zone = False
        sm_thr = 0.0
        sm_dir: tuple[float, float, float] | None = None
        while True:
            pacer.tick()
            s = b.snapshot()
            if s is None:
                continue

            if not in_dead_zone and s.surface_altitude <= DEAD_ZONE_ALT:
                in_dead_zone = True
                stop.set()  # halt replanning: follow the last trajectory to touchdown
                log(f"dead zone entered at alt={s.surface_altitude:.1f}")

            traj, t0_met, p, tf = slot.get()
            thr_raw = 0.0
            dir_raw: tuple[float, float, float] | None = None
            elapsed = 0.0
            nose = (0.0, 0.0, 1.0)
            if traj is not None:
                elapsed = s.met - t0_met
                throttle, direction = command_at_time(
                    traj,
                    elapsed,
                    mass=s.mass,
                    available_thrust=s.available_thrust,
                    min_throttle=params.min_throttle,
                    max_throttle=params.max_throttle,
                    max_angle_deg=params.max_angle_deg,
                )
                thr_raw = throttle
                dir_raw = direction
                # low-pass the throttle and slew-limit the direction to remove
                # solver-to-solver chattering
                target = sm_thr + THROTTLE_SMOOTH * (throttle - sm_thr)
                # rate-limit the per-tick change so the engine never pulses (~6 Hz)
                sm_thr = min(max(target, sm_thr - THROTTLE_MAX_STEP), sm_thr + THROTTLE_MAX_STEP)
                if sm_dir is None:
                    sm_dir = direction
                else:
                    dx = direction[0] - sm_dir[0]
                    dy = direction[1] - sm_dir[1]
                    dz = direction[2] - sm_dir[2]
                    sm_dir = (
                        sm_dir[0] + DIRECTION_SMOOTH * dx,
                        sm_dir[1] + DIRECTION_SMOOTH * dy,
                        sm_dir[2] + DIRECTION_SMOOTH * dz,
                    )
                    n = (sm_dir[0] ** 2 + sm_dir[1] ** 2 + sm_dir[2] ** 2) ** 0.5
                    if n > 1e-12:
                        sm_dir = (sm_dir[0] / n, sm_dir[1] / n, sm_dir[2] / n)

                nose = (s.direction.x, s.direction.y, s.direction.z)
                sticks = local_ap.step(s, sm_dir)
                b.controls.apply(
                    throttle=sm_thr,
                    pitch=sticks.pitch,
                    yaw=sticks.yaw,
                    roll=sticks.roll,
                )

            # publish for the drawer thread (cheap — no RPC here)
            draw_state.set(
                (s.position.x, s.position.y, s.position.z),
                sm_dir,
            )

            # -- per-tick diagnostics ----------------------------------------
            av = (s.angular_velocity.x, s.angular_velocity.y, s.angular_velocity.z)
            ang_err = _ang(sm_dir, nose) if sm_dir is not None else 0.0
            ds = sm_dir if sm_dir is not None else (0.0, 0.0, 0.0)
            dr = dir_raw if dir_raw is not None else (0.0, 0.0, 0.0)
            u0mag = float(np.linalg.norm(traj.u_values[0])) if traj is not None else 0.0
            csvw.writerow([
                f"{s.met:.3f}", f"{s.surface_altitude:.2f}",
                f"{s.velocity.x:.3f}", f"{s.velocity.y:.3f}", f"{s.velocity.z:.3f}",
                f"{thr_raw:.4f}", f"{sm_thr:.4f}",
                f"{dr[0]:.4f}", f"{dr[1]:.4f}", f"{dr[2]:.4f}",
                f"{ds[0]:.4f}", f"{ds[1]:.4f}", f"{ds[2]:.4f}",
                f"{nose[0]:.4f}", f"{nose[1]:.4f}", f"{nose[2]:.4f}",
                f"{av[0]:.4f}", f"{av[1]:.4f}", f"{av[2]:.4f}",
                f"{ang_err:.3f}", f"{elapsed:.3f}", f"{p:.4f}", f"{tf:.4f}",
                f"{u0mag:.3f}", f"{s.mass:.1f}", f"{s.available_thrust:.1f}",
            ])

            now = time.monotonic()
            if now - last_report >= 0.5:
                last_report = now
                avm = (av[0] ** 2 + av[1] ** 2 + av[2] ** 2) ** 0.5
                log(
                    f"C alt={s.surface_altitude:7.1f} dz={int(in_dead_zone)} "
                    f"v=({s.velocity.x:+6.1f},{s.velocity.y:+6.1f},{s.velocity.z:+6.1f}) "
                    f"thr={sm_thr:.2f} dir={_fmt3(ds)} "
                    f"av={avm:5.3f}rad/s angerr={ang_err:5.1f}deg"
                )
            if s.landed:
                break

        telemetry.close()
        b.controls.cut_thrust()
        b.controls.apply(pitch=0.0, yaw=0.0, roll=0.0)
        stop.set()
        draw_stop.set()
        planner.join(timeout=2.0)
        drawer.join(timeout=2.0)
        log("Touchdown.")
        log_file.close()


if __name__ == "__main__":
    main()
