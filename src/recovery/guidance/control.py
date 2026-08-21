"""Virtual control model for the controlled trajectory predictor.

A *virtual control* is a piecewise-constant thrust schedule plus a steering
rule, describing the commanded behaviour of a vessel over a predicted
trajectory.  It is pure data (no kRPC, no behaviour) so it can be built,
compared and stored independently of any integration.

The model is deliberately general so that arbitrary guidance algorithms can
encode their commanded profile as a :class:`VirtualControl` and ask the
predictor to propagate it:

* :class:`Trigger` — *when* a segment becomes active: at an absolute mission
  time (``time``) or when the vessel descends through an altitude
  (``altitude``).  Supporting both lets a control law switch on either clock
  or geometry.
* :class:`NoseRule` — *where* the thrust axis points, as a rule evaluated
  against the instantaneous state (retrograde / up / fixed / toward the
  target), rather than a frozen vector.
* :class:`ControlSegment` — one interval of constant throttle, engine-set
  thrust and steering rule, active from its trigger until a later segment
  takes over.
* :class:`VirtualControl` — the ordered schedule of segments plus the vessel
  dry mass and standard gravity, everything the predictor needs.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from ..types import Vector3


class TriggerKind(StrEnum):
    """How a :class:`Trigger` is interpreted."""

    TIME = "time"
    """Fire at an absolute mission elapsed time (seconds)."""

    ALTITUDE = "altitude"
    """Fire when the vessel descends through the given altitude (metres)."""


class NoseRuleKind(StrEnum):
    """Steering rules for the thrust axis of a segment."""

    RETROGRADE = "retrograde"
    """Point opposite the velocity vector (tail-first / braking)."""

    UP = "up"
    """Point along local up (radial, in the target frame +z)."""

    FIXED = "fixed"
    """Point along a caller-supplied constant direction."""

    TOWARD_TARGET = "toward_target"
    """Point from the vessel toward the target (the frame origin)."""


class ThrottleRuleKind(StrEnum):
    """Throttle rules: a fixed setting, or a state-dependent feedback law."""

    CONSTANT = "constant"
    """Fixed throttle (``value`` in 0..1)."""

    ENERGY = "energy"
    """Brake to ``value`` (m/s) at ``reference_altitude`` (m), full throttle
    below it — the classic energy landing law (gravity-compensated)."""

    CONSTANT_DECEL = "constant_decel"
    """Constant-deceleration (suicide-burn) law: brake to ``value`` (m/s) at
    the surface."""


@dataclass(frozen=True)
class ThrottleRule:
    """A throttle law evaluated against the instantaneous state.

    ``value`` and ``reference_altitude`` are interpreted per :class:`ThrottleRuleKind`:

    * CONSTANT — ``value`` is the fixed throttle (0..1); ``reference_altitude`` unused.
    * ENERGY — ``value`` is the target speed ``v_gfold`` (m/s);
      ``reference_altitude`` is ``h_gfold`` (m).
    * CONSTANT_DECEL — ``value`` is the touchdown speed ``vt`` (m/s);
      ``reference_altitude`` unused.
    """

    kind: ThrottleRuleKind
    value: float = 0.0
    reference_altitude: float = 3000.0

    @classmethod
    def constant(cls, throttle: float) -> "ThrottleRule":
        return cls(ThrottleRuleKind.CONSTANT, float(throttle))

    @classmethod
    def energy(cls, v_gfold: float = 140.0, h_gfold: float = 3000.0) -> "ThrottleRule":
        return cls(ThrottleRuleKind.ENERGY, float(v_gfold), float(h_gfold))

    @classmethod
    def constant_decel(cls, vt: float = 0.1) -> "ThrottleRule":
        return cls(ThrottleRuleKind.CONSTANT_DECEL, float(vt))


@dataclass(frozen=True)
class Trigger:
    """When a control segment becomes active."""

    kind: TriggerKind
    """Time- or altitude-based."""

    value: float
    """Seconds (for ``time``) or metres (for ``altitude``)."""

    @classmethod
    def at_time(cls, t: float) -> "Trigger":
        """A time-based trigger at mission elapsed time *t* (s)."""
        return cls(TriggerKind.TIME, float(t))

    @classmethod
    def at_altitude(cls, alt: float) -> "Trigger":
        """An altitude-based trigger that fires on descent through *alt* (m)."""
        return cls(TriggerKind.ALTITUDE, float(alt))


@dataclass(frozen=True)
class NoseRule:
    """Steering rule producing the thrust-axis (nose) direction from state."""

    kind: NoseRuleKind
    """Which rule to apply."""

    direction: Vector3 = Vector3(0.0, 0.0, 0.0)
    """Constant direction for ``FIXED`` (unit-length; normalised if not)."""

    @classmethod
    def retrograde(cls) -> "NoseRule":
        return cls(NoseRuleKind.RETROGRADE)

    @classmethod
    def up(cls) -> "NoseRule":
        return cls(NoseRuleKind.UP)

    @classmethod
    def fixed(cls, direction: Vector3) -> "NoseRule":
        return cls(NoseRuleKind.FIXED, direction)

    @classmethod
    def toward_target(cls) -> "NoseRule":
        return cls(NoseRuleKind.TOWARD_TARGET)


@dataclass(frozen=True)
class ControlSegment:
    """One interval of constant control, active from *trigger*.

    The first segment of a :class:`VirtualControl` is active from the start of
    the propagation (its ``trigger`` is ignored); subsequent segments fire in
    list order when their trigger is met.
    """

    throttle: ThrottleRule
    """Throttle law for this segment (fixed or a feedback rule)."""

    max_thrust: float
    """Maximum thrust of the active engine set (N); 0 means engines off."""

    isp: float
    """Specific impulse of the active engine set (s)."""

    nose: NoseRule
    """Steering rule for this segment."""

    trigger: Trigger | None = None
    """When this segment becomes active (``None`` for the initial segment)."""

    @classmethod
    def coast(cls, nose: NoseRule | None = None) -> "ControlSegment":
        """An engine-off segment (throttle 0)."""
        return cls(throttle=ThrottleRule.constant(0.0), max_thrust=0.0, isp=0.0,
                   nose=nose or NoseRule.retrograde())


@dataclass(frozen=True)
class VirtualControl:
    """The complete commanded profile for a controlled prediction."""

    segments: tuple[ControlSegment, ...]
    """Ordered control schedule; ``segments[0]`` is active from t=0."""

    dry_mass: float
    """Vessel dry mass (kg) — the floor below which mass cannot drop."""

    g0: float = 9.80665
    """Standard gravity (m/s²), used for the mass-flow rate."""

    def __post_init__(self) -> None:
        if not self.segments:
            raise ValueError("VirtualControl requires at least one segment")
        for seg in self.segments:
            thr = seg.throttle
            if thr.kind == ThrottleRuleKind.CONSTANT and not (0.0 <= thr.value <= 1.0):
                raise ValueError(f"constant throttle out of range: {thr.value}")
            if seg.max_thrust < 0.0:
                raise ValueError(f"max_thrust must be >= 0: {seg.max_thrust}")
            if seg.isp < 0.0:
                raise ValueError(f"isp must be >= 0: {seg.isp}")
