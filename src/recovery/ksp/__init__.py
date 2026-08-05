"""KSP isolation layer: the only place that imports kRPC."""

from .control import VesselControls
from .debug import DebugLine, DebugMarker, DebugProxy, DebugTrajectory
from .manager import ConnectionManager
from .types import FlightState, Quaternion, Situation, Vector3
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
]
