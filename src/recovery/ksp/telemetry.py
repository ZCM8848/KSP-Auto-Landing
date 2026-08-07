"""Streamed telemetry with coherent, rate-limited snapshot publishing."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Any

from .types import FlightState, Quaternion, Situation, Vector3


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
        """
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
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
        add("altitude", getattr, flight, "mean_altitude")
        add("surface_altitude", getattr, flight, "surface_altitude")
        add("atmosphere_density", getattr, flight, "atmosphere_density")
        add("mass", getattr, vessel, "mass")
        add("dry_mass", getattr, vessel, "dry_mass")
        add("thrust", getattr, vessel, "thrust")
        add("available_thrust", getattr, vessel, "available_thrust")
        add("max_thrust", getattr, vessel, "max_thrust")
        add("max_vacuum_thrust", getattr, vessel, "max_vacuum_thrust")
        add("specific_impulse", getattr, vessel, "specific_impulse")
        add("throttle", getattr, vessel.control, "throttle")
        add("situation", getattr, vessel, "situation")
        add("loaded", getattr, vessel, "loaded")
        add("packed", getattr, vessel, "packed")

    def _run(self) -> None:
        while not self._stop.is_set():
            time.sleep(self._telemetry_interval)
            if self._stop.is_set():
                break
            snapshot = self._build_snapshot()
            with self._lock:
                self._snapshot = snapshot

    def _build_snapshot(self) -> FlightState:
        values = {name: stream() for name, stream in self._streams}
        situation = Situation.from_krpc(values["situation"])
        mass = float(values["mass"])
        max_thrust = float(values["max_thrust"])
        return FlightState(
            ut=float(values["ut"]),
            met=float(values["met"]),
            position=Vector3(values["position"][0], values["position"][1], values["position"][2]),
            velocity=Vector3(values["velocity"][0], values["velocity"][1], values["velocity"][2]),
            velocity_surface=Vector3(
                values["velocity_surface"][0],
                values["velocity_surface"][1],
                values["velocity_surface"][2],
            ),
            rotation=Quaternion(
                values["rotation"][0],
                values["rotation"][1],
                values["rotation"][2],
                values["rotation"][3],
            ),
            angular_velocity=Vector3(
                values["angular_velocity"][0],
                values["angular_velocity"][1],
                values["angular_velocity"][2],
            ),
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
            frame=self._frame,
        )
