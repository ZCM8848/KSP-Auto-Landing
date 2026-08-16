"""Public handle for one registered booster vessel."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, Any

from ..specs import BodySpec, DragSpec
from ..types import FlightState
from .control import VesselControls
from .debug import DebugProxy
from .exceptions import DebugNotEnabled, TargetNotRegistered
from .sampling import sample_body_spec, sample_drag_spec

if TYPE_CHECKING:
    from .connection import KspConnection
    from .debug import DebugConnection


class VesselHandle:
    """Public handle returned by ``ConnectionManager.add_booster`` and
    ``ConnectionManager.vessel``.

    The handle is the primary entry point for interaction with a single
    booster: reading telemetry snapshots, issuing control commands,
    managing the physics bubble, and accessing debug drawing tools.

    Attributes:
        control_hz: Suggested control-loop frequency (Hz) for this booster.
            Set during registration; consumed by the orchestration scheduler.
    """

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
        self._body_spec: BodySpec | None = None

    @property
    def name(self) -> str:
        """The vessel name as it appears in-game."""
        return str(self._vessel.name)

    @property
    def controls(self) -> VesselControls:
        """The :class:`VesselControls` gateway for this vessel."""
        return self._controls

    @property
    def is_open(self) -> bool:
        """``True`` while the underlying kRPC connection is still alive."""
        return not self._connection.closed

    @property
    def debug(self) -> DebugProxy:
        """Lazily-built :class:`DebugProxy` for in-game drawing.

        Requires ``ConnectionManager.enable_debug()`` to have been called
        first; otherwise raises ``RuntimeError``.
        """
        if self._debug_proxy is None:
            connection = self._debug_provider()
            if connection is None:
                raise DebugNotEnabled("debug not enabled; call ConnectionManager.enable_debug()")
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
        """Escape hatch: the underlying kRPC ``Vessel`` remote object.

        Use for any kRPC functionality not yet wrapped.  Bypasses
        snapshot isolation and all safety checks — the caller is
        responsible for avoiding expensive synchronous RPCs inside the
        control loop.
        """
        return self._vessel

    @property
    def client(self) -> Any:
        """Escape hatch: the underlying kRPC ``Client`` for this connection."""
        return self._connection.client

    def snapshot(self) -> FlightState | None:
        """Return the latest frozen telemetry snapshot, or ``None`` when
        the telemetry thread has not yet produced its first frame.
        """
        return self._connection.snapshot()

    def frame(self, name: str = "target") -> Any:
        """Return a kRPC reference frame handle by name.

        *name* must be ``"target"`` (requires a prior ``register_target``)
        or ``"surface"``.

        Raises:
            TargetNotRegistered: if ``"target"`` is requested without a prior
                ``register_target``.
            KeyError: if *name* is unrecognised.
        """
        return self._connection.frame(name)

    @property
    def body_spec(self) -> BodySpec:
        """The sampled planetary constants for this vessel's current body.

        Expressed in this vessel's ``"target"`` reference frame, with
        ``body_radius`` evaluated at the registered landing target.  Sampled
        once (one-time RPC) and cached — subsequent reads are pure field
        access, safe at control-loop rates.

        Raises:
            TargetNotRegistered: if ``register_target`` was not called first.
        """
        if self._body_spec is None:
            lat = self._target_lat
            lon = self._target_lon
            if lat is None or lon is None:
                raise TargetNotRegistered("no target registered; call register_target() first")
            body = self._vessel.orbit.body
            frame = self._connection.frame("target")
            self._body_spec = sample_body_spec(body, frame, lat, lon)
        return self._body_spec

    def sample_predictor_specs(
        self,
        *,
        mass: float | None = None,
        manual_beta: float | None = None,
        altitude_samples: int = 64,
    ) -> tuple[BodySpec, DragSpec]:
        """One-shot sample of the specs needed to build a
        :class:`~recovery.guidance.LandingPredictor`.

        The body spec is the cached :attr:`body_spec`; only the drag spec is
        sampled here.  *mass* defaults to the vessel's current mass.  Requires
        a prior :meth:`register_target`.  One-time RPC cost only — do not call
        inside the control loop.
        """
        frame = self._connection.frame("target")
        body = self._vessel.orbit.body
        flight = self._vessel.flight(frame)
        if mass is None:
            mass = float(self._vessel.mass)
        lat = self._target_lat
        lon = self._target_lon
        if lat is None or lon is None:
            raise TargetNotRegistered("no target registered; call register_target() first")
        drag_spec = sample_drag_spec(
            body,
            flight,
            frame,
            lat=lat,
            lon=lon,
            mass=mass,
            manual_beta=manual_beta,
            altitude_samples=altitude_samples,
        )
        return self.body_spec, drag_spec

    def register_target(self, *, lon: float, lat: float) -> None:
        """Register a landing-site target for this vessel.

        Must be called **before** ``start()``.  Builds the target
        reference frame on the control connection and records the
        coordinates so the debug proxy can recreate it later.

        Raises:
            RuntimeError: if called after ``start()``.
        """
        self._target_lon = lon
        self._target_lat = lat
        self._connection.register_target(lon=lon, lat=lat)
        if self._debug_proxy is not None:
            self._debug_proxy.set_target(lon=lon, lat=lat)

    def start(self) -> None:
        """Start telemetry streaming for this vessel. Idempotent."""
        self._connection.start()

    def close(self) -> None:
        """Stop telemetry, zero the throttle, and close the kRPC
        connection. Idempotent.
        """
        self._connection.close()

    @property
    def physics_range(self) -> float:
        """Physics bubble radius in metres.  Set to a large value to keep
        this vessel off-rails while the active vessel is far away (enables
        simultaneous control of multiple boosters).  Set to 0 to restore
        the game's default.
        """
        return float(self._vessel.physics_range)

    @physics_range.setter
    def physics_range(self, value: float) -> None:
        self._vessel.physics_range = value

    def is_controllable(self) -> bool:
        """Return ``True`` when the vessel is loaded and has its physics
        simulation running (i.e. it can accept control inputs).  Returns
        ``False`` while the telemetry snapshot is not yet available.
        """
        state = self.snapshot()
        if state is None:
            return False
        return state.loaded and not state.packed
