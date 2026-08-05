"""Public handle for one registered booster vessel."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, Any

from .control import VesselControls
from .debug import DebugProxy
from .types import FlightState

if TYPE_CHECKING:
    from .connection import KspConnection
    from .debug import DebugConnection


class VesselHandle:
    def __init__(
        self,
        *,
        connection: KspConnection,
        vessel: Any,
        controls: VesselControls,
        control_hz: float = 50.0,
        debug_provider: Callable[[], DebugConnection | None] | None = None,
    ) -> None:
        self._connection = connection
        self._vessel = vessel
        self._controls = controls
        self.control_hz = control_hz
        self._debug_provider = debug_provider if debug_provider is not None else (lambda: None)
        self._debug_proxy: DebugProxy | None = None
        self._target_lon: float | None = None
        self._target_lat: float | None = None

    @property
    def name(self) -> str:
        return str(self._vessel.name)

    @property
    def controls(self) -> VesselControls:
        return self._controls

    @property
    def is_open(self) -> bool:
        return not self._connection.closed

    @property
    def debug(self) -> DebugProxy:
        if self._debug_proxy is None:
            connection = self._debug_provider()
            if connection is None:
                raise RuntimeError("debug not enabled; call ConnectionManager.enable_debug()")
            self._debug_proxy = DebugProxy(
                client=connection.client,
                body_name=str(self._vessel.orbit.body.name),
                vessel_name=str(self._vessel.name),
                target_lon=self._target_lon,
                target_lat=self._target_lat,
            )
        return self._debug_proxy

    @property
    def raw(self) -> Any:
        """The underlying kRPC Vessel object.

        Escape hatch for anything not yet wrapped; bypasses snapshot isolation
        and all safety checks.
        """
        return self._vessel

    def snapshot(self) -> FlightState | None:
        return self._connection.snapshot()

    def frame(self, name: str = "target") -> Any:
        return self._connection.frame(name)

    def register_target(self, *, lon: float, lat: float) -> None:
        self._target_lon = lon
        self._target_lat = lat
        self._connection.register_target(lon=lon, lat=lat)
        if self._debug_proxy is not None:
            self._debug_proxy.set_target(lon=lon, lat=lat)

    def start(self) -> None:
        self._connection.start()

    def close(self) -> None:
        self._connection.close()

    @property
    def physics_range(self) -> float:
        return float(self._vessel.physics_range)

    @physics_range.setter
    def physics_range(self, value: float) -> None:
        self._vessel.physics_range = value

    def is_controllable(self) -> bool:
        state = self.snapshot()
        if state is None:
            return False
        return state.loaded and not state.packed
