"""Streamed telemetry with coherent, rate-limited snapshot publishing."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Any

from ..types import FlightState, Quaternion, Situation, TorquePair, Vector3


def _torque_pair(value: Any) -> TorquePair:
    """Convert a kRPC ``available_*_torque`` 2-tuple of 3-vectors to a
    :class:`TorquePair`."""
    return TorquePair(
        Vector3(value[0][0], value[0][1], value[0][2]),
        Vector3(value[1][0], value[1][1], value[1][2]),
    )


def _vec3(values: dict[str, Any], name: str) -> Vector3:
    """Extract a 3-vector stream value as a :class:`Vector3`."""
    v = values[name]
    return Vector3(float(v[0]), float(v[1]), float(v[2]))


def _quat(values: dict[str, Any], name: str) -> Quaternion:
    """Extract a quaternion stream value as a :class:`Quaternion`."""
    q = values[name]
    return Quaternion(float(q[0]), float(q[1]), float(q[2]), float(q[3]))


class Telemetry:
    """Stream-based telemetry aggregator for a single vessel.

    Registers kRPC streams for position, velocity, attitude, mass, thrust,
    throttle, situation, and atmosphere density.  A background thread polls
    the stream cache at *telemetry_hz* and builds frozen
    :class:`FlightState` snapshots.

    Keyword Args:
        client: kRPC ``Client`` for this vessel.
        vessel: Resolved kRPC ``Vessel``.
        frame: Reference frame to express position/velocity/rotation in.
        telemetry_hz: Snapshot publication frequency.
    """

    def __init__(
        self,
        *,
        client: Any,
        vessel: Any,
        frame: Any,
        telemetry_hz: float = 20.0,
    ) -> None:
        self._client = client
        self._vessel = vessel
        self._frame = frame
        self._telemetry_interval = 1.0 / telemetry_hz
        self._streams: list[tuple[str, Any]] = []
        self._snapshot: FlightState | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        """Register kRPC streams and launch the background updater thread."""
        self._register_streams()
        self._thread = threading.Thread(target=self._run, name="telemetry", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Signal the background thread to exit, join it, and remove all
        streams from the server.

        If the thread is mid-``sleep`` when the stop flag is set, ``join``
        waits up to one extra telemetry interval for it to wake.  If the
        thread does not exit within the join timeout (e.g. stuck on a blocked
        stream read), the streams are left in place rather than removed
        concurrently with a live read.
        """
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
            self._thread = None
            if thread.is_alive():
                return
        for _, stream in self._streams:
            stream.remove()
        self._streams.clear()

    def get(self) -> FlightState | None:
        """Return the latest snapshot, or ``None`` before the first frame."""
        with self._lock:
            return self._snapshot

    def _register(self, name: str, func: Callable[..., Any], *args: Any) -> None:
        stream = self._client.add_stream(func, *args)
        stream.start()
        self._streams.append((name, stream))

    def _register_streams(self) -> None:
        vessel = self._vessel
        space_center = self._client.space_center
        surface = vessel.surface_reference_frame
        flight = vessel.flight(surface)
        add = self._register

        add("ut", getattr, space_center, "ut")
        add("met", getattr, vessel, "met")
        add("position", vessel.position, self._frame)
        add("velocity", vessel.velocity, self._frame)
        add("velocity_surface", vessel.velocity, surface)
        add("rotation", vessel.rotation, self._frame)
        add("angular_velocity", vessel.angular_velocity, self._frame)
        add("direction", vessel.direction, self._frame)
        add(
            "bottom_axis",
            space_center.transform_direction,
            (0.0, 0.0, 1.0),
            vessel.reference_frame,
            self._frame,
        )
        add("altitude", getattr, flight, "mean_altitude")
        add("surface_altitude", getattr, flight, "surface_altitude")
        add("atmosphere_density", getattr, flight, "atmosphere_density")
        add("atmosphere_depth", getattr, vessel.orbit.body, "atmosphere_depth")
        add("mass", getattr, vessel, "mass")
        add("dry_mass", getattr, vessel, "dry_mass")
        add("thrust", getattr, vessel, "thrust")
        add("available_thrust", getattr, vessel, "available_thrust")
        add("max_thrust", getattr, vessel, "max_thrust")
        add("max_vacuum_thrust", getattr, vessel, "max_vacuum_thrust")
        add("specific_impulse", getattr, vessel, "specific_impulse")
        add("available_reaction_wheel_torque", getattr, vessel, "available_reaction_wheel_torque")
        add("available_rcs_torque", getattr, vessel, "available_rcs_torque")
        add("available_engine_torque", getattr, vessel, "available_engine_torque")
        add("available_control_surface_torque", getattr, vessel, "available_control_surface_torque")
        add("moment_of_inertia", getattr, vessel, "moment_of_inertia")
        add("throttle", getattr, vessel.control, "throttle")
        add("situation", getattr, vessel, "situation")
        add("loaded", getattr, vessel, "loaded")
        add("packed", getattr, vessel, "packed")

    def _run(self) -> None:
        while not self._stop.is_set():
            snapshot = self._build_snapshot()
            with self._lock:
                self._snapshot = snapshot
            # Block until the next frame boundary.  ``time.sleep`` paces far
            # more accurately on Windows than ``Event.wait`` (which rounds up
            # to ~15.6 ms timer-resolution ticks).  The cost: ``stop()`` may
            # wait up to one full telemetry interval for the sleep to finish.
            time.sleep(self._telemetry_interval)

    def _build_snapshot(self) -> FlightState:
        values = {name: stream() for name, stream in self._streams}
        situation = Situation.from_krpc(values["situation"])
        mass = float(values["mass"])
        max_thrust = float(values["max_thrust"])
        return FlightState(
            ut=float(values["ut"]),
            met=float(values["met"]),
            position=_vec3(values, "position"),
            velocity=_vec3(values, "velocity"),
            velocity_surface=_vec3(values, "velocity_surface"),
            rotation=_quat(values, "rotation"),
            angular_velocity=_vec3(values, "angular_velocity"),
            direction=_vec3(values, "direction"),
            bottom_axis=_vec3(values, "bottom_axis"),
            available_reaction_wheel_torque=_torque_pair(values["available_reaction_wheel_torque"]),
            available_rcs_torque=_torque_pair(values["available_rcs_torque"]),
            available_engine_torque=_torque_pair(values["available_engine_torque"]),
            available_control_surface_torque=_torque_pair(values["available_control_surface_torque"]),
            moment_of_inertia=_vec3(values, "moment_of_inertia"),
            altitude=float(values["altitude"]),
            surface_altitude=float(values["surface_altitude"]),
            mass=mass,
            dry_mass=float(values["dry_mass"]),
            thrust=float(values["thrust"]),
            available_thrust=float(values["available_thrust"]),
            max_thrust=max_thrust,
            max_vacuum_thrust=float(values["max_vacuum_thrust"]),
            specific_impulse=float(values["specific_impulse"]),
            max_acceleration=max_thrust / mass if mass > 0.0 else 0.0,
            throttle=float(values["throttle"]),
            situation=situation,
            loaded=bool(values["loaded"]),
            packed=bool(values["packed"]),
            landed=situation in (Situation.LANDED, Situation.PRE_LAUNCH, Situation.SPLASHED),
            atmosphere_density=float(values["atmosphere_density"]),
            atmosphere_depth=float(values["atmosphere_depth"]),
            frame=self._frame,
        )
