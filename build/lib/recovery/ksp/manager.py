"""Aggregation and lifecycle for all booster connections."""

from __future__ import annotations

import atexit
from typing import Any

from .connection import KspConnection
from .types import FlightState
from .vessel import VesselHandle


class ConnectionManager:
    def __init__(
        self,
        *,
        address: str = "127.0.0.1",
        rpc_port: int = 50000,
        stream_port: int = 50001,
        telemetry_hz: float = 20.0,
        isp_refresh_hz: float = 2.0,
    ) -> None:
        self._address = address
        self._rpc_port = rpc_port
        self._stream_port = stream_port
        self._telemetry_hz = telemetry_hz
        self._isp_refresh_hz = isp_refresh_hz
        self._boosters: dict[str, VesselHandle] = {}
        self._started = False
        atexit.register(self.close)

    def add_booster(
        self,
        booster_id: str,
        vessel_name: str,
        *,
        control_hz: float = 50.0,
        telemetry_hz: float | None = None,
        isp_refresh_hz: float | None = None,
    ) -> VesselHandle:
        if booster_id in self._boosters:
            raise ValueError(f"duplicate booster id {booster_id!r}")
        if self._started:
            raise RuntimeError("cannot add a booster after start()")
        connection = KspConnection(
            name=f"recovery-{booster_id}",
            address=self._address,
            rpc_port=self._rpc_port,
            stream_port=self._stream_port,
            telemetry_hz=telemetry_hz if telemetry_hz is not None else self._telemetry_hz,
            isp_refresh_hz=isp_refresh_hz if isp_refresh_hz is not None else self._isp_refresh_hz,
        )
        try:
            connection.resolve_vessel(vessel_name)
            handle = VesselHandle(
                connection=connection,
                vessel=connection.vessel,
                controls=connection.controls,
                control_hz=control_hz,
            )
        except Exception:
            connection.close()
            raise
        self._boosters[booster_id] = handle
        return handle

    def start(self) -> None:
        for handle in self._boosters.values():
            handle.start()
        self._started = True

    def close(self) -> None:
        for handle in self._boosters.values():
            handle.close()
        self._started = False

    def __enter__(self) -> ConnectionManager:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    def vessel(self, booster_id: str) -> VesselHandle:
        return self._require(booster_id)

    def snapshot(self, booster_id: str) -> FlightState | None:
        return self._require(booster_id).snapshot()

    def snapshot_all(self) -> dict[str, FlightState | None]:
        return {booster_id: handle.snapshot() for booster_id, handle in self._boosters.items()}

    def frame(self, booster_id: str, name: str = "target") -> Any:
        return self._require(booster_id).frame(name)

    def register_target(self, booster_id: str, *, lon: float, lat: float) -> None:
        self._require(booster_id).register_target(lon=lon, lat=lat)

    def abort_all(self) -> None:
        for handle in self._boosters.values():
            handle.controls.cut_thrust()

    def _require(self, booster_id: str) -> VesselHandle:
        try:
            return self._boosters[booster_id]
        except KeyError:
            registered = ", ".join(self._boosters)
            raise KeyError(
                f"unknown booster id {booster_id!r} (registered: {registered})"
            ) from None
