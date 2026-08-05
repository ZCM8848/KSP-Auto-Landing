"""Public value types crossing the KSP isolation layer."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Any, NamedTuple


class Vector3(NamedTuple):
    x: float
    y: float
    z: float


class Quaternion(NamedTuple):
    x: float
    y: float
    z: float
    w: float


class Situation(StrEnum):
    PRE_LAUNCH = "pre_launch"
    ORBITING = "orbiting"
    SUB_ORBITAL = "sub_orbital"
    ESCAPING = "escaping"
    FLYING = "flying"
    LANDED = "landed"
    SPLASHED = "splashed"
    DOCKED = "docked"
    UNKNOWN = "unknown"

    @classmethod
    def from_krpc(cls, value: Any) -> Situation:
        name = getattr(value, "name", None)
        if name is None:
            return cls.UNKNOWN
        try:
            return cls(name)
        except ValueError:
            return cls.UNKNOWN


@dataclass(frozen=True)
class FlightState:
    """Coherent, immutable telemetry snapshot of one vessel.

    All vector fields are expressed in the frame referenced by ``frame`` (an
    opaque kRPC reference frame handle, typically the registered target frame
    or the vessel surface frame).
    """

    ut: float
    met: float
    position: Vector3
    velocity: Vector3
    velocity_surface: Vector3
    rotation: Quaternion
    altitude: float
    surface_altitude: float
    mass: float
    dry_mass: float
    thrust: float
    available_thrust: float
    max_thrust: float
    max_vacuum_thrust: float
    specific_impulse: float
    max_acceleration: float
    throttle: float
    situation: Situation
    loaded: bool
    packed: bool
    landed: bool
    atmosphere_density: float
    frame: Any
