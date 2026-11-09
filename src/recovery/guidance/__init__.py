"""Guidance algorithms: trajectory prediction, aero models, TOF-Net."""

from .aerodynamics import LiftDragModel, LiftTableModel
from .control import (
    BrakeToThrottle,
    ConstantThrottle,
    ControlSegment,
    FixedNose,
    NoseRule,
    RetrogradeNose,
    ThrottleRule,
    TowardTargetNose,
    Trigger,
    TriggerKind,
    UpNose,
    VirtualControl,
)
from .controlled import ControlledPredictor, ImpactResult, Trajectory
from .correction import Correction, TerminalCorrector
from .plan import GuidanceCommand, Plan, PlanExecutor, TerminalTarget, plan_to_virtual_control
from .scvx import (
    AttitudeModel,
    DescentProblem,
    EngineModel,
    ScvxConfig,
    ScvxPlanner,
    ScvxSolveService,
    SolveStats,
    build_descent_problem,
    build_six_dof_problem,
)
from .terminal import PolynomialConfig, PolynomialTerminalGuidance
from .tofnet import TofFeatureParams, TofPredictor, features_of
from .tracking import (
    PlanThrust,
    PlanTracker,
    TrackingConfig,
    plan_is_usable,
    plan_thrust,
)

__all__ = [
    "AttitudeModel",
    "BrakeToThrottle",
    "ConstantThrottle",
    "ControlSegment",
    "ControlledPredictor",
    "Correction",
    "DescentProblem",
    "EngineModel",
    "FixedNose",
    "GuidanceCommand",
    "ImpactResult",
    "LiftDragModel",
    "LiftTableModel",
    "NoseRule",
    "Plan",
    "PlanExecutor",
    "PlanThrust",
    "PlanTracker",
    "PolynomialConfig",
    "PolynomialTerminalGuidance",
    "RetrogradeNose",
    "ScvxConfig",
    "ScvxPlanner",
    "ScvxSolveService",
    "SolveStats",
    "TerminalCorrector",
    "TerminalTarget",
    "TofFeatureParams",
    "TofPredictor",
    "TrackingConfig",
    "TowardTargetNose",
    "Trajectory",
    "ThrottleRule",
    "Trigger",
    "TriggerKind",
    "UpNose",
    "VirtualControl",
    "build_descent_problem",
    "build_six_dof_problem",
    "features_of",
    "plan_is_usable",
    "plan_thrust",
    "plan_to_virtual_control",
]
