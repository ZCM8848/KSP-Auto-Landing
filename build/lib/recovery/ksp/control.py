"""Control gateway for a single vessel.

Attitude is commanded server-side through the kRPC AutoPilot when a
``target_direction`` is provided; raw stick inputs remain available for local
high-frequency control loops.
"""

from __future__ import annotations

from math import sqrt
from typing import Any

Vec3 = tuple[float, float, float]


class VesselControls:
    def __init__(self, vessel: Any) -> None:
        self._vessel = vessel
        self._control = vessel.control
        self._auto_pilot = vessel.auto_pilot

    @property
    def raw(self) -> Any:
        """The underlying kRPC Control object (escape hatch)."""
        return self._control

    @property
    def auto_pilot(self) -> Any:
        """The underlying kRPC AutoPilot object (escape hatch)."""
        return self._auto_pilot

    # -- throttle -----------------------------------------------------------

    @property
    def throttle(self) -> float:
        return float(self._control.throttle)

    @throttle.setter
    def throttle(self, value: float) -> None:
        self._control.throttle = min(1.0, max(0.0, float(value)))

    # -- raw stick inputs ---------------------------------------------------

    @property
    def pitch(self) -> float:
        return float(self._control.pitch)

    @pitch.setter
    def pitch(self, value: float) -> None:
        self._control.pitch = value

    @property
    def yaw(self) -> float:
        return float(self._control.yaw)

    @yaw.setter
    def yaw(self, value: float) -> None:
        self._control.yaw = value

    @property
    def roll(self) -> float:
        return float(self._control.roll)

    @roll.setter
    def roll(self, value: float) -> None:
        self._control.roll = value

    # -- systems ------------------------------------------------------------

    @property
    def sas(self) -> bool:
        return bool(self._control.sas)

    @sas.setter
    def sas(self, value: bool) -> None:
        self._control.sas = value

    @property
    def rcs(self) -> bool:
        return bool(self._control.rcs)

    @rcs.setter
    def rcs(self, value: bool) -> None:
        self._control.rcs = value

    @property
    def legs(self) -> bool:
        return bool(self._control.legs)

    @legs.setter
    def legs(self, value: bool) -> None:
        self._control.legs = value

    @property
    def gear(self) -> bool:
        return bool(self._control.gear)

    @gear.setter
    def gear(self, value: bool) -> None:
        self._control.gear = value

    @property
    def lights(self) -> bool:
        return bool(self._control.lights)

    @lights.setter
    def lights(self, value: bool) -> None:
        self._control.lights = value

    @property
    def brakes(self) -> bool:
        return bool(self._control.brakes)

    @brakes.setter
    def brakes(self, value: bool) -> None:
        self._control.brakes = value

    @property
    def abort(self) -> bool:
        return bool(self._control.abort)

    @abort.setter
    def abort(self, value: bool) -> None:
        self._control.abort = value

    # -- auto-pilot ---------------------------------------------------------

    @property
    def auto_pilot_engaged(self) -> bool:
        return bool(self._auto_pilot.engaged)

    def engage_auto_pilot(self) -> None:
        self._auto_pilot.engaged = True

    def disengage_auto_pilot(self) -> None:
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
        if throttle is not None:
            self.throttle = throttle
        if pitch is not None:
            self._control.pitch = pitch
        if yaw is not None:
            self._control.yaw = yaw
        if roll is not None:
            self._control.roll = roll
        if sas is not None:
            self._control.sas = sas
        if rcs is not None:
            self._control.rcs = rcs
        if legs is not None:
            self._control.legs = legs
        if gear is not None:
            self._control.gear = gear
        if target_direction is not None:
            self._point_at(
                target_direction, up=up, roll_angle=roll_angle, reference_frame=reference_frame
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
        self._auto_pilot.reference_frame = reference_frame
        norm = sqrt(sum(component * component for component in direction))
        if norm == 0:
            raise ValueError("target_direction must be non-zero")
        self._auto_pilot.target_direction = tuple(component / norm for component in direction)
        if up is not None:
            self._auto_pilot.up_reference = tuple(up)
        if roll_angle is not None:
            self._auto_pilot.target_roll = roll_angle
        self.engage_auto_pilot()

    # -- staging / action groups -------------------------------------------

    def activate_next_stage(self) -> list[Any]:
        return list(self._control.activate_next_stage())

    def set_action_group(self, group: int, state: bool) -> None:
        self._control.set_action_group(group, state)

    def get_action_group(self, group: int) -> bool:
        return bool(self._control.get_action_group(group))

    def toggle_action_group(self, group: int) -> None:
        self._control.toggle_action_group(group)

    # -- emergency ----------------------------------------------------------

    def cut_thrust(self) -> None:
        self._control.throttle = 0.0
        self.disengage_auto_pilot()
