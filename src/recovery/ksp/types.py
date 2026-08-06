"""Public value types crossing the KSP isolation layer."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Any, NamedTuple


class Vector3(NamedTuple):
    """A 3-component vector (x, y, z) used by ``FlightState`` for position
    and velocity fields. The components are in metres or metres per second
    depending on context.
    """

    x: float
    y: float
    z: float


class Quaternion(NamedTuple):
    """A unit-orientation quaternion (x, y, z, w) as returned by kRPC for
    ``Vessel.rotation``.  Component order matches the kRPC convention.
    """

    x: float
    y: float
    z: float
    w: float


class Situation(StrEnum):
    """Mirrors kRPC ``VesselSituation`` — the flight state a vessel is in."""

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
        """Convert a kRPC situation value into this enumeration.

        *value* is expected to have a ``.name`` attribute (e.g.
        ``vessel.situation.name``). Unrecognised names map to ``UNKNOWN``.
        """
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

    Fields:
        ut: Universal time in seconds (kRPC ``SpaceCenter.ut``).
        met: Mission elapsed time in seconds (``Vessel.met``).
        position: Position in the snapshot frame (m).
        velocity: Velocity in the snapshot frame (m/s).
        velocity_surface: Surface-relative velocity (m/s).
        rotation: Attitude quaternion in the snapshot frame.
        altitude: Mean altitude above sea level (m).
        surface_altitude: Altitude above terrain (m).
        mass: Total vessel mass including resources (kg).
        dry_mass: Vessel mass excluding resources (kg).
        thrust: Current thrust produced by active engines (N).
        available_thrust: Maximum thrust available from active engines (N).
        max_thrust: Theoretical maximum thrust from active engines (N).
        max_vacuum_thrust: Maximum thrust in vacuum from active engines (N).
        specific_impulse: Thrust-weighted combined Isp of active engines (s).
            Refreshed at ``isp_refresh_hz`` (default 2 Hz).
        max_acceleration: ``max_thrust / mass`` (m/s²).
        throttle: Vessel throttle (0.0--1.0).
        situation: Flight situation (pre-launch, flying, landed, …).
        loaded: Whether the vessel is within the physics bubble.
        packed: Whether the vessel is on-rails (no physics simulation).
        landed: ``True`` when *situation* is LANDED, PRE_LAUNCH or SPLASHED.
        atmosphere_density: Ambient atmospheric density (kg/m³).
        frame: Opaque kRPC reference frame this snapshot is expressed in.
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
