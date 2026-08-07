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
        """
        return self._connection.frame(name)

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

    def init_predictor(
        self,
        *,
        altitude_samples: int = 64,
        manual_beta: float | None = None,
    ) -> Any:
        """Build a :class:`~recovery.guidance.LandingPredictor` with an
        offline :class:`~recovery.guidance.DragModel` attached.

        Samples the celestial body's atmosphere density profile and the
        vessel's current ballistic coefficient from kRPC **once** (~25 ms);
        subsequent :meth:`predict` calls run entirely offline and are safe
        to invoke from the 20 Hz control loop.

        Requires :meth:`register_target` to have been called first.

        Keyword Args:
            altitude_samples: Minimum number of density sample points
                (actual count is auto-scaled for deep atmospheres).
            manual_beta: Explicit ballistic coefficient override (kg/m²).
                When ``None`` the coefficient is read from FAR (if
                installed) or estimated from the current drag force.
        """
        if self._target_lat is None or self._target_lon is None:
            raise RuntimeError(
                "register_target() must be called before init_predictor()"
            )
        from ..guidance.predictor import DragModel, LandingPredictor

        frame = self.frame("target")
        body = self._vessel.orbit.body
        flight = self._vessel.flight(frame)
        mass = float(self._vessel.mass)

        drag = DragModel.from_krpc(
            body=body,
            flight=flight,
            target_frame=frame,
            mass=mass,
            manual_beta=manual_beta,
            altitude_samples=altitude_samples,
        )
        return LandingPredictor.from_body(
            body=body,
            target_frame=frame,
            lat=self._target_lat,
            lon=self._target_lon,
            aero=drag,
        )
