"""Aggregation and lifecycle for all booster connections."""

from __future__ import annotations

import atexit
import threading
from typing import Any

from ..types import FlightState
from .connection import KspConnection
from .debug import DebugConnection
from .exceptions import DuplicateBooster, InvalidState
from .vessel import VesselHandle


class ConnectionManager:
    """Central registry and lifecycle manager for all booster connections.

    Supports three usage patterns::

        # Context manager (recommended — guarantees cleanup)
        with ConnectionManager() as km:
            b = km.add_booster(...)
            km.start()
            ...

        # Explicit lifecycle
        km = ConnectionManager()
        km.add_booster(...)
        km.start()
        ...
        km.close()

        # Callback-driven (not yet implemented — planned for the Scheduler)

    An ``atexit`` handler is registered on construction as an additional
    safety net, zeroing throttle and closing every connection on process exit.

    Keyword Args:
        address: kRPC server hostname / IP.
        rpc_port: kRPC RPC port.
        stream_port: kRPC stream port.
        telemetry_hz: Default snapshot rate for boosters that don't
            specify their own.
    """

    def __init__(
        self,
        *,
        address: str = "127.0.0.1",
        rpc_port: int = 50000,
        stream_port: int = 50001,
        telemetry_hz: float = 20.0,
    ) -> None:
        self._address = address
        self._rpc_port = rpc_port
        self._stream_port = stream_port
        self._telemetry_hz = telemetry_hz
        self._boosters: dict[str, VesselHandle] = {}
        self._debug: DebugConnection | None = None
        self._started = False
        # Guards ``_boosters`` / ``_debug`` / ``_started`` so lifecycle calls
        # (``add_booster`` / ``start`` / ``close`` / debug enable/disable) are
        # safe even when the atexit handler races the main thread.  Handles are
        # snapshotted under the lock and the actual (potentially slow) work
        # runs outside it, so no lock is ever held across a nested
        # ``KspConnection`` call.
        self._lock = threading.Lock()
        atexit.register(self.close)

    def add_booster(
        self,
        booster_id: str,
        vessel_name: str,
        *,
        control_hz: float = 50.0,
        telemetry_hz: float | None = None,
    ) -> VesselHandle:
        """Open a kRPC connection for *vessel_name* and register it under
        *booster_id*.

        *booster_id* is an arbitrary string used as a key for methods like
        :meth:`snapshot`, :meth:`vessel` and :meth:`register_target`.
        *vessel_name* must match an in-game vessel name exactly.

        Return the :class:`VesselHandle` that can be used directly or
        retrieved later via :meth:`vessel`.

        Raises:
            DuplicateBooster: if *booster_id* is a duplicate.
            InvalidState: if called after :meth:`start`.
            VesselNotFound: if *vessel_name* is not found (the error
                message lists available names).
        """
        with self._lock:
            if booster_id in self._boosters:
                raise DuplicateBooster(f"duplicate booster id {booster_id!r}")
            if self._started:
                raise InvalidState("cannot add a booster after start()")
            connection = KspConnection(
                name=f"recovery-{booster_id}",
                address=self._address,
                rpc_port=self._rpc_port,
                stream_port=self._stream_port,
                telemetry_hz=telemetry_hz if telemetry_hz is not None else self._telemetry_hz,
            )
            try:
                connection.resolve_vessel(vessel_name)
                handle = VesselHandle(
                    connection=connection,
                    vessel=connection.vessel,
                    controls=connection.controls,
                    control_hz=control_hz,
                    debug_provider=lambda: self._debug,
                )
            except Exception:
                connection.close()
                raise
            self._boosters[booster_id] = handle
        return handle

    def start(self) -> None:
        """Launch telemetry threads for all registered boosters.
        Safe to call multiple times (subsequent calls are no-ops for
        already-started connections).

        Call this once all boosters and targets have been registered.
        """
        with self._lock:
            if self._started:
                return
            handles = list(self._boosters.values())
            self._started = True
        for handle in handles:
            handle.start()

    def close(self) -> None:
        """Stop telemetry, zero throttle, and close every connection.

        Also closes the debug connection (if enabled).  Safe to call
        multiple times.  Registered as an ``atexit`` handler so it runs
        even if the user forgets to call it explicitly.
        """
        with self._lock:
            handles = list(self._boosters.values())
            debug = self._debug
            self._debug = None
            self._started = False
        for handle in handles:
            handle.close()
        if debug is not None:
            debug.close()

    def enable_debug(self) -> None:
        """Open a shared debug kRPC connection used by all vessels'
        :class:`DebugProxy` instances.  Calling this after :meth:`start`
        is fine — the debug connection is independent of the control
        connections.
        """
        with self._lock:
            if self._debug is not None:
                return
            self._debug = DebugConnection(
                name="recovery-debug",
                address=self._address,
                rpc_port=self._rpc_port,
                stream_port=self._stream_port,
            )

    def disable_debug(self) -> None:
        """Close the debug connection, removing all in-game drawn objects
        (kRPC automatically clears drawings when a client disconnects).
        """
        with self._lock:
            debug = self._debug
            self._debug = None
        if debug is not None:
            debug.close()

    def __enter__(self) -> ConnectionManager:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    def vessel(self, booster_id: str) -> VesselHandle:
        """Return the :class:`VesselHandle` registered under *booster_id*.

        Raises:
            KeyError: if *booster_id* is unknown.
        """
        return self._require(booster_id)

    def snapshot(self, booster_id: str) -> FlightState | None:
        """Return the latest :class:`FlightState` for *booster_id*, or
        ``None`` before the telemetry thread has produced its first frame.
        """
        return self._require(booster_id).snapshot()

    def snapshot_all(self) -> dict[str, FlightState | None]:
        """Return a ``{booster_id: FlightState | None}`` mapping for all
        registered boosters.
        """
        with self._lock:
            boosters = list(self._boosters.items())
        return {booster_id: handle.snapshot() for booster_id, handle in boosters}

    def frame(self, booster_id: str, name: str = "target") -> Any:
        """Return a kRPC reference frame handle for *booster_id*.

        Args:
            name: ``"target"`` or ``"surface"``.
        """
        return self._require(booster_id).frame(name)

    def register_target(self, booster_id: str, *, lon: float, lat: float) -> None:
        """Register a landing-site target for *booster_id*.

        Must be called **before** :meth:`start`.
        """
        self._require(booster_id).register_target(lon=lon, lat=lat)

    def abort_all(self) -> list[str]:
        """Emergency stop for every open connection: zero throttle and
        disengage autopilot.  Returns the ids of boosters that could not be
        aborted (their connection died mid-flight); already-closed boosters
        are skipped silently.
        """
        with self._lock:
            handles = list(self._boosters.items())
        failed: list[str] = []
        for booster_id, handle in handles:
            if not handle.is_open:
                continue
            try:
                handle.controls.cut_thrust()
            except OSError:
                failed.append(booster_id)
        return failed

    def _require(self, booster_id: str) -> VesselHandle:
        try:
            return self._boosters[booster_id]
        except KeyError:
            registered = ", ".join(self._boosters)
            raise KeyError(
                f"unknown booster id {booster_id!r} (registered: {registered})"
            ) from None
