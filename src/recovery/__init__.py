"""Extensible multi-booster kRPC rocket recovery framework."""

from .clock import FramePacer
from .control import (
    PID,
    ApproachingModel,
    AutoPilot,
    LocalAttitudeController,
    StickCommand,
)
from .ksp import (
    ConnectionManager,
    DebugLine,
    DebugMarker,
    DebugProxy,
    DebugTrajectory,
    FlightState,
    Quaternion,
    Situation,
    Vector3,
    VesselControls,
    VesselHandle,
)

__all__ = [
    "FramePacer",
    "AutoPilot",
    "ApproachingModel",
    "LocalAttitudeController",
    "StickCommand",
    "PID",
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
