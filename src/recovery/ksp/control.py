"""Control gateway for a single vessel.

Attitude is commanded server-side through the kRPC AutoPilot when a
``target_direction`` is provided; raw stick inputs remain available for local
high-frequency control loops.
"""

from __future__ import annotations

from collections.abc import Callable
from math import sqrt
from typing import Any

from ..perf import PROFILER

Vec3 = tuple[float, float, float]

_MISSING = object()


class VesselControls:
    """High-level control gateway for a single vessel.

    Attitude is commanded server-side through the kRPC ``AutoPilot`` when
    ``target_direction`` is provided via :meth:`apply`; raw stick inputs
    (pitch / yaw / roll) remain available for local high-frequency control
    loops that wish to drive the vessel with their own PID.

    System toggles (SAS, RCS, landing legs, gear, lights, brakes, abort
    action group) are exposed as read/write properties.

    Redundant writes are suppressed: a value already written through this
    gateway is not re-sent until it changes.  This removes the per-tick cost
    of re-issuing constants such as ``throttle`` / ``up_reference`` in a hot
    control loop.  Staging and action-group changes clear the cache, since they
    may alter control state externally.

    The underlying kRPC ``Control`` and ``AutoPilot`` objects are accessible
    via the ``raw`` and ``auto_pilot`` escape-hatch properties.
    """

    def __init__(self, vessel: Any) -> None:
        """*vessel* is a kRPC ``Vessel`` remote object from which the
        gateway obtains ``.control`` and ``.auto_pilot``.

        Normally constructed by ``KspConnection.resolve_vessel`` — users
        obtain it via ``VesselHandle.controls``.
        """
        self._vessel = vessel
        self._control = vessel.control
        self._auto_pilot = vessel.auto_pilot
        self._last_ref_frame: Any = None
        # Last raw values written through this gateway.  Re-issuing an
        # unchanged kRPC control write costs a full server round-trip (the
        # throttle alone measured ~8 ms per frame in the aero phase), so
        # redundant writes are suppressed.  The cache is cleared after staging
        # / action-group changes, which can alter control state externally.
        self._written: dict[str, Any] = {}

    def _cached(self, key: str, value: Any, write: Callable[[], None]) -> None:
        """Run *write* unless *key* already holds *value*, then remember it."""
        if self._written.get(key, _MISSING) == value:
            return
        write()
        self._written[key] = value

    def clear_cache(self) -> None:
        """Forget every remembered control write.

        The cache only tracks values written *through this gateway*.  KSP can
        change control state behind its back (staging, action groups, another
        client, a scene reload).  Call this after any such event, or before
        re-asserting safety-critical switches such as RCS/SAS/legs.
        """
        self._written.clear()

    @property
    def raw(self) -> Any:
        """Direct access to the underlying kRPC ``Control`` object.
        Bypasses throttle clamping and all safety checks.
        """
        return self._control

    @property
    def auto_pilot(self) -> Any:
        """Direct access to the underlying kRPC ``AutoPilot`` object.
        Use for tuning gains, smoothing, and other advanced parameters
        not exposed by :meth:`apply`.
        """
        return self._auto_pilot

    # -- throttle -----------------------------------------------------------

    @property
    def throttle(self) -> float:
        """Vessel throttle, clamped to [0.0, 1.0] on write.  Reads the
        value currently applied to the vessel (combined kRPC + keyboard +
        SAS + trim input), which refreshes one physics tick after a set.
        """
        return float(self._control.throttle)

    @throttle.setter
    def throttle(self, value: float) -> None:
        v = min(1.0, max(0.0, float(value)))
        self._cached("throttle", v, lambda: setattr(self._control, "throttle", v))

    # -- raw stick inputs ---------------------------------------------------

    @property
    def pitch(self) -> float:
        """Pitch control input (-1.0 … 1.0). Equivalent to W/S keys."""
        return float(self._control.pitch)

    @pitch.setter
    def pitch(self, value: float) -> None:
        v = float(value)
        self._cached("pitch", v, lambda: setattr(self._control, "pitch", v))

    @property
    def yaw(self) -> float:
        """Yaw control input (-1.0 … 1.0). Equivalent to A/D keys."""
        return float(self._control.yaw)

    @yaw.setter
    def yaw(self, value: float) -> None:
        v = float(value)
        self._cached("yaw", v, lambda: setattr(self._control, "yaw", v))

    @property
    def roll(self) -> float:
        """Roll control input (-1.0 … 1.0). Equivalent to Q/E keys."""
        return float(self._control.roll)

    @roll.setter
    def roll(self, value: float) -> None:
        v = float(value)
        self._cached("roll", v, lambda: setattr(self._control, "roll", v))

    # -- systems ------------------------------------------------------------

    @property
    def sas(self) -> bool:
        """SAS (Stability Assist System) state.  Note that engaging the
        AutoPilot forces SAS off; setting this to ``True`` while the
        autopilot is engaged throws an exception.
        """
        return bool(self._control.sas)

    @sas.setter
    def sas(self, value: bool) -> None:
        v = bool(value)
        self._cached("sas", v, lambda: setattr(self._control, "sas", v))

    @property
    def rcs(self) -> bool:
        """RCS (Reaction Control System) state."""
        return bool(self._control.rcs)

    @rcs.setter
    def rcs(self, value: bool) -> None:
        v = bool(value)
        self._cached("rcs", v, lambda: setattr(self._control, "rcs", v))

    @property
    def legs(self) -> bool:
        """Deployment state of all landing legs."""
        return bool(self._control.legs)

    @legs.setter
    def legs(self, value: bool) -> None:
        v = bool(value)
        self._cached("legs", v, lambda: setattr(self._control, "legs", v))

    @property
    def gear(self) -> bool:
        """Deployment state of landing gear / wheels."""
        return bool(self._control.gear)

    @gear.setter
    def gear(self, value: bool) -> None:
        v = bool(value)
        self._cached("gear", v, lambda: setattr(self._control, "gear", v))

    @property
    def lights(self) -> bool:
        """Lights toggle."""
        return bool(self._control.lights)

    @lights.setter
    def lights(self, value: bool) -> None:
        v = bool(value)
        self._cached("lights", v, lambda: setattr(self._control, "lights", v))

    @property
    def brakes(self) -> bool:
        """Wheel brakes state."""
        return bool(self._control.brakes)

    @brakes.setter
    def brakes(self, value: bool) -> None:
        v = bool(value)
        self._cached("brakes", v, lambda: setattr(self._control, "brakes", v))

    @property
    def abort(self) -> bool:
        """Abort action-group state."""
        return bool(self._control.abort)

    @abort.setter
    def abort(self, value: bool) -> None:
        v = bool(value)
        self._cached("abort", v, lambda: setattr(self._control, "abort", v))

    # -- auto-pilot ---------------------------------------------------------

    @property
    def auto_pilot_engaged(self) -> bool:
        """Whether the kRPC AutoPilot is currently engaged (read-only)."""
        return bool(self._auto_pilot.engaged)

    def engage_auto_pilot(self) -> None:
        """Engage the kRPC AutoPilot. Sets ``auto_pilot.engaged = True``."""
        self._auto_pilot.engaged = True

    def disengage_auto_pilot(self) -> None:
        """Disengage the kRPC AutoPilot. Sets ``auto_pilot.engaged = False``."""
        self._auto_pilot.engaged = False

    # -- combined command ---------------------------------------------------

    def apply(
        self,
        *,
        throttle: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        roll: float | None = None,
        target_direction: Vec3 | None = None,
        up: Vec3 | None = None,
        roll_angle: float | None = None,
        reference_frame: Any = None,
        sas: bool | None = None,
        rcs: bool | None = None,
        legs: bool | None = None,
        gear: bool | None = None,
    ) -> None:
        """Apply a set of control commands in a single call.

        Every parameter is keyword-only and optional — only those that are
        not ``None`` are written to kRPC. This lets the caller update
        throttle and attitude independently::

            controls.apply(throttle=0.7)
            controls.apply(target_direction=(0, -1, 0), reference_frame=frame)

        Keyword Args:
            throttle: Main throttle (0.0 … 1.0, clamped automatically).
            pitch: Raw pitch stick input (-1.0 … 1.0).
            yaw: Raw yaw stick input (-1.0 … 1.0).
            roll: Raw roll stick input (-1.0 … 1.0).
            target_direction: Nose direction vector (normalised
                automatically).  When provided the kRPC AutoPilot is
                engaged and SAS is turned off by the server.
            up: Roll reference vector passed to
                ``AutoPilot.up_reference``. Optional; defaults to the
                frame's built-in up axis.
            roll_angle: Additional roll about the nose axis in degrees.
                Optional; when omitted roll is not constrained.
            reference_frame: kRPC reference frame that *target_direction*
                and *up* are expressed in.  **Required** if
                *target_direction* is given, otherwise ignored.
            sas: SAS state toggle.
            rcs: RCS state toggle.
            legs: Landing-legs deployment toggle.
            gear: Landing-gear/wheels deployment toggle.

        Raises:
            ValueError: if *target_direction* is given but
                *reference_frame* is ``None``, or if *target_direction*
                is the zero vector.
        """
        with PROFILER.section("ctl.apply"):
            if throttle is not None:
                with PROFILER.section("ctl.throttle"):
                    self.throttle = throttle
            if pitch is not None:
                self.pitch = pitch
            if yaw is not None:
                self.yaw = yaw
            if roll is not None:
                self.roll = roll
            if sas is not None:
                self.sas = sas
            if rcs is not None:
                self.rcs = rcs
            if legs is not None:
                self.legs = legs
            if gear is not None:
                self.gear = gear
            if target_direction is not None:
                self._point_at(
                    target_direction,
                    up=up,
                    roll_angle=roll_angle,
                    reference_frame=reference_frame,
                )

    def _point_at(
        self,
        direction: Vec3,
        *,
        up: Vec3 | None,
        roll_angle: float | None,
        reference_frame: Any,
    ) -> None:
        if reference_frame is None:
            raise ValueError("reference_frame is required when commanding target_direction")
        # kRPC remote objects compare by server object id via ``__eq__`` (each
        # attribute access returns a fresh proxy), so ``!=`` — not ``is`` — is
        # required to detect an actual reference-frame change.
        with PROFILER.section("ctl.refframe"):
            if reference_frame != self._last_ref_frame:
                self._auto_pilot.reference_frame = reference_frame  # RPC — only on frame change
                self._last_ref_frame = reference_frame
        norm = sqrt(sum(component * component for component in direction))
        if norm == 0:
            raise ValueError("target_direction must be non-zero")
        unit_dir = tuple(component / norm for component in direction)
        with PROFILER.section("ctl.dir"):
            self._cached(
                "target_direction",
                unit_dir,
                lambda: setattr(self._auto_pilot, "target_direction", unit_dir),
            )
        if up is not None:
            up_vec = tuple(up)
            with PROFILER.section("ctl.up"):
                self._cached(
                    "up_reference",
                    up_vec,
                    lambda: setattr(self._auto_pilot, "up_reference", up_vec),
                )
        if roll_angle is not None:
            with PROFILER.section("ctl.roll"):
                self._cached(
                    "target_roll",
                    float(roll_angle),
                    lambda: setattr(self._auto_pilot, "target_roll", roll_angle),
                )
        with PROFILER.section("ctl.engaged"):
            engaged = self.auto_pilot_engaged
        if not engaged:
            with PROFILER.section("ctl.engage"):
                self.engage_auto_pilot()

    @property
    def target_smoothing_time(self) -> float:
        """The duration (seconds) over which a change to ``target_direction``
        is slewed smoothly by the kRPC AutoPilot.  Set to 0.0 for instant
        response; 0.2--0.5 is typical for gfld-guided trajectories.
        """
        return float(self._auto_pilot.target_smoothing_time)

    @target_smoothing_time.setter
    def target_smoothing_time(self, value: float) -> None:
        self._auto_pilot.target_smoothing_time = value

    # -- staging / action groups -------------------------------------------

    def activate_next_stage(self) -> list[Any]:
        """Activate the next stage (equivalent to Space). Returns any
        vessels that were jettisoned; they could be re-registered with
        ``ConnectionManager.add_booster`` for simultaneous recovery.
        """
        result = list(self._control.activate_next_stage())
        self._written.clear()
        return result

    def set_action_group(self, group: int, state: bool) -> None:
        """Set the state of action group *group* (0-9, or 0-250 with the
        Extended Action Groups mod).
        """
        self._control.set_action_group(group, state)
        self._written.clear()

    def get_action_group(self, group: int) -> bool:
        """Return whether action group *group* is currently enabled."""
        return bool(self._control.get_action_group(group))

    def toggle_action_group(self, group: int) -> None:
        """Toggle the state of action group *group*."""
        self._control.toggle_action_group(group)
        self._written.clear()

    # -- emergency ----------------------------------------------------------

    def cut_thrust(self) -> None:
        """Emergency stop: zero throttle, raw sticks, and disengage the AutoPilot.

        This is the primitive behind ``ConnectionManager.abort_all``.
        """
        self._control.throttle = 0.0
        self._control.pitch = 0.0
        self._control.yaw = 0.0
        self._control.roll = 0.0
        self.disengage_auto_pilot()
        self._written.clear()
