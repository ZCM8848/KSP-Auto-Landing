"""One kRPC connection unit: a single vessel's transport, telemetry and controls."""

from __future__ import annotations

from typing import Any

import krpc

from ..types import FlightState
from .control import VesselControls
from .exceptions import (
    AmbiguousVesselName,
    InvalidState,
    TargetNotRegistered,
    VesselNotFound,
    VesselNotResolved,
)
from .reference_frames import create_target_reference_frame
from .telemetry import Telemetry


class KspConnection:
    """One kRPC connection dedicated to a single vessel.

    Encapsulates the kRPC ``Client``, vessel resolution, reference frame
    creation, telemetry streaming, and the :class:`VesselControls` gateway.
    Normally constructed and owned by :class:`ConnectionManager`; users
    interact through :class:`VesselHandle`.

    Keyword Args:
        name: Display name shown in the in-game kRPC window.
        address: Server hostname or IP.
        rpc_port: kRPC RPC port (default 50000).
        stream_port: kRPC stream port (default 50001).
        telemetry_hz: Snapshot publishing frequency.
    """

    def __init__(
        self,
        *,
        name: str,
        address: str = "127.0.0.1",
        rpc_port: int = 50000,
        stream_port: int = 50001,
        telemetry_hz: float = 20.0,
    ) -> None:
        self._client = krpc.connect(
            name=name, address=address, rpc_port=rpc_port, stream_port=stream_port
        )
        self._name = name
        self._telemetry_hz = telemetry_hz
        self._vessel: Any = None
        self._controls: VesselControls | None = None
        self._telemetry: Telemetry | None = None
        self._target_frame: Any = None
        self._snapshot_frame: Any = None
        self._started = False
        self._closed = False

    @property
    def name(self) -> str:
        """Connection display name (shown in the kRPC server window)."""
        return self._name

    @property
    def client(self) -> Any:
        """The underlying kRPC ``Client`` object."""
        return self._client

    @property
    def vessel(self) -> Any:
        """Resolved kRPC ``Vessel`` remote object.

        Raises ``RuntimeError`` if ``resolve_vessel`` has not been called.
        """
        if self._vessel is None:
            raise VesselNotResolved("no vessel resolved")
        return self._vessel

    @property
    def controls(self) -> VesselControls:
        """The :class:`VesselControls` gateway for this vessel.

        Raises ``VesselNotResolved`` if ``resolve_vessel`` has not been called.
        """
        if self._controls is None:
            raise VesselNotResolved("no vessel resolved")
        return self._controls

    @property
    def closed(self) -> bool:
        """Whether the connection has been closed."""
        return self._closed

    def resolve_vessel(self, vessel_name: str) -> None:
        """Look up a vessel by name in the current game and attach it to
        this connection.

        Raises:
            InvalidState: if a vessel is already resolved.
            VesselNotFound: if no vessel with *vessel_name* exists.
            AmbiguousVesselName: if multiple vessels share *vessel_name*.
        """
        if self._vessel is not None:
            raise InvalidState("a vessel is already resolved")
        matches = [v for v in self._client.space_center.vessels if v.name == vessel_name]
        if not matches:
            available = ", ".join(
                v.name for v in self._client.space_center.vessels
            )
            raise VesselNotFound(
                f"no vessel named {vessel_name!r} found (available: {available})"
            )
        if len(matches) > 1:
            raise AmbiguousVesselName(
                vessel_name, [v.situation.name for v in matches]
            )
        self._vessel = matches[0]
        self._controls = VesselControls(matches[0])

    def register_target(self, *, lon: float, lat: float) -> None:
        """Build a landing-site reference frame at (*lon*, *lat*) on the
        vessel's current celestial body.

        Must be called **before** :meth:`start`.

        Raises:
            RuntimeError: if called after :meth:`start`.
        """
        if self._started:
            raise InvalidState("register_target must be called before start()")
        body = self.vessel.orbit.body
        self._target_frame = create_target_reference_frame(
            self._client.space_center, body, lon, lat
        )

    def start(self) -> None:
        """Register kRPC streams and launch the telemetry background thread.
        Idempotent.

        Raises:
            RuntimeError: if no vessel has been resolved.
        """
        if self._started:
            return
        if self._vessel is None:
            raise VesselNotResolved("no vessel resolved; call resolve_vessel first")
        self._snapshot_frame = self._target_frame
        if self._snapshot_frame is None:
            self._snapshot_frame = self.vessel.surface_reference_frame
        self._telemetry = Telemetry(
            client=self._client,
            vessel=self._vessel,
            frame=self._snapshot_frame,
            telemetry_hz=self._telemetry_hz,
        )
        self._telemetry.start()
        self._started = True

    def snapshot(self) -> FlightState | None:
        """Return the latest frozen telemetry snapshot, or ``None`` when
        the telemetry thread has not yet produced its first frame.
        """
        if self._telemetry is None:
            return None
        return self._telemetry.get()

    def frame(self, name: str = "target") -> Any:
        """Return a kRPC reference frame handle.

        Args:
            name: ``"target"`` (landing-site, requires ``register_target``
                first) or ``"surface"`` (surface-relative frame).

        Raises:
            TargetNotRegistered: if ``"target"`` is requested but
                ``register_target`` was never called.
            KeyError: if *name* is unrecognised.
        """
        if name == "target":
            if self._target_frame is None:
                raise TargetNotRegistered("no target frame registered")
            return self._target_frame
        if name == "surface":
            return self.vessel.surface_reference_frame
        raise KeyError(f"unknown frame {name!r} (available: target, surface)")

    def close(self) -> None:
        """Stop telemetry, zero the throttle, disengage the autopilot,
        and close the kRPC connection. Idempotent.

        The throttle is explicitly zeroed here because kRPC **keeps** the
        throttle value when a client disconnects (unlike other control
        inputs, which are automatically zeroed).
        """
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
