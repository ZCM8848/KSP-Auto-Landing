"""KSP isolation layer: the only place that imports kRPC."""

from ..types import FlightState, Quaternion, Situation, Vector3
from .control import VesselControls
from .debug import DebugLine, DebugMarker, DebugProxy, DebugTrajectory
from .exceptions import (
    AmbiguousVesselName,
    DebugNotEnabled,
    DuplicateBooster,
    InvalidState,
    RecoveryError,
    TargetNotRegistered,
    VesselNotFound,
    VesselNotResolved,
)
from .manager import ConnectionManager
from .vessel import VesselHandle

__all__ = [
    "ConnectionManager",
    "VesselHandle",
    "VesselControls",
    "DebugProxy",
    "DebugLine",
    "DebugMarker",
    "DebugTrajectory",
    "FlightState",
    "Situation",
    "Vector3",
    "Quaternion",
    "RecoveryError",
    "InvalidState",
    "VesselNotResolved",
    "VesselNotFound",
    "AmbiguousVesselName",
    "DuplicateBooster",
    "TargetNotRegistered",
    "DebugNotEnabled",
]
