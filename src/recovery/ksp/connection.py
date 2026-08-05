"""One kRPC connection unit: a single vessel's transport, telemetry and controls."""

from __future__ import annotations

from typing import Any

import krpc

from .control import VesselControls
from .reference_frames import create_target_reference_frame
from .telemetry import Telemetry
from .types import FlightState


class KspConnection:
    def __init__(
        self,
        *,
        name: str,
        address: str = "127.0.0.1",
        rpc_port: int = 50000,
        stream_port: int = 50001,
        telemetry_hz: float = 20.0,
        isp_refresh_hz: float = 2.0,
    ) -> None:
        self._client = krpc.connect(
            name=name, address=address, rpc_port=rpc_port, stream_port=stream_port
        )
        self._name = name
        self._telemetry_hz = telemetry_hz
        self._isp_refresh_hz = isp_refresh_hz
        self._vessel: Any = None
        self._controls: VesselControls | None = None
        self._telemetry: Telemetry | None = None
        self._target_frame: Any = None
        self._snapshot_frame: Any = None
        self._started = False
        self._closed = False

    @property
    def name(self) -> str:
        return self._name

    @property
    def client(self) -> Any:
        return self._client

    @property
    def vessel(self) -> Any:
        if self._vessel is None:
            raise RuntimeError("no vessel resolved")
        return self._vessel

    @property
    def controls(self) -> VesselControls:
        if self._controls is None:
            raise RuntimeError("no vessel resolved")
        return self._controls

    @property
    def closed(self) -> bool:
        return self._closed

    def resolve_vessel(self, vessel_name: str) -> None:
        if self._vessel is not None:
            raise RuntimeError("a vessel is already resolved")
        for vessel in self._client.space_center.vessels:
            if vessel.name == vessel_name:
                self._vessel = vessel
                self._controls = VesselControls(vessel)
                return
        available = ", ".join(vessel.name for vessel in self._client.space_center.vessels)
        raise ValueError(f"no vessel named {vessel_name!r} found (available: {available})")

    def register_target(self, *, lon: float, lat: float) -> None:
        if self._started:
            raise RuntimeError("register_target must be called before start()")
        body = self.vessel.orbit.body
        self._target_frame = create_target_reference_frame(
            self._client.space_center, body, lon, lat
        )

    def start(self) -> None:
        if self._started:
            return
        if self._vessel is None:
            raise RuntimeError("no vessel resolved; call resolve_vessel first")
        self._snapshot_frame = self._target_frame
        if self._snapshot_frame is None:
            self._snapshot_frame = self.vessel.surface_reference_frame
        self._telemetry = Telemetry(
            client=self._client,
            vessel=self._vessel,
            frame=self._snapshot_frame,
            telemetry_hz=self._telemetry_hz,
            isp_refresh_hz=self._isp_refresh_hz,
        )
        self._telemetry.start()
        self._started = True

    def snapshot(self) -> FlightState | None:
        if self._telemetry is None:
            return None
        return self._telemetry.get()

    def frame(self, name: str = "target") -> Any:
        if name == "target":
            if self._target_frame is None:
                raise KeyError("no target frame registered")
            return self._target_frame
        if name == "surface":
            return self.vessel.surface_reference_frame
        raise KeyError(f"unknown frame {name!r} (available: target, surface)")

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._controls is not None:
            try:
                self._controls.cut_thrust()
            except Exception:
                pass
        if self._telemetry is not None:
            self._telemetry.stop()
            self._telemetry = None
        self._client.close()
        self._started = False
