"""Public handle for one registered booster vessel."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from .control import VesselControls
from .types import FlightState

if TYPE_CHECKING:
    from .connection import KspConnection


class VesselHandle:
    def __init__(
        self,
        *,
        connection: KspConnection,
        vessel: Any,
        controls: VesselControls,
        control_hz: float = 50.0,
    ) -> None:
        self._connection = connection
        self._vessel = vessel
        self._controls = controls
        self.control_hz = control_hz

    @property
    def name(self) -> str:
        return str(self._vessel.name)

    @property
    def controls(self) -> VesselControls:
        return self._controls

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
        self._connection.register_target(lon=lon, lat=lat)

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
