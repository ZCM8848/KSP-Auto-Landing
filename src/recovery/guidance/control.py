"""Virtual control model for the controlled trajectory predictor.

A *virtual control* is a piecewise-constant thrust schedule plus a steering
rule, describing the commanded behaviour of a vessel over a predicted
trajectory.  It is pure data (no kRPC, no behaviour) so it can be built,
compared and stored independently of any integration.

Steering (``NoseRule``) and throttle (``ThrottleRule``) are *tagged unions* of
frozen dataclasses — one concrete class per law — so the type system enforces
which parameters belong to which rule, instead of overloading a single
``value``/``reference_altitude`` pair across an enum.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

from ..types import Vector3


class TriggerKind(IntEnum):
    """How a :class:`Trigger` is interpreted; values are the kernel codes."""

    TIME = 0
    """Fire at an absolute mission elapsed time (seconds)."""

    ALTITUDE = 1
    """Fire when the vessel descends through the given altitude (metres)."""


@dataclass(frozen=True)
class Trigger:
    """When a control segment becomes active."""

    kind: TriggerKind
    """Time- or altitude-based."""

    value: float
    """Seconds (for ``time``) or metres (for ``altitude``)."""

    @classmethod
    def at_time(cls, t: float) -> Trigger:
        """A time-based trigger at mission elapsed time *t* (s)."""
        return cls(TriggerKind.TIME, float(t))

    @classmethod
    def at_altitude(cls, alt: float) -> Trigger:
        """An altitude-based trigger that fires on descent through *alt* (m)."""
        return cls(TriggerKind.ALTITUDE, float(alt))


# ---------------------------------------------------------------------------
# Steering rules (tagged union)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RetrogradeNose:
    """Point opposite the velocity vector (tail-first / braking)."""


@dataclass(frozen=True)
class UpNose:
    """Point along local up (radial, in the target frame +z)."""


@dataclass(frozen=True)
class FixedNose:
    """Point along a caller-supplied constant direction (normalised if not)."""

    direction: Vector3


@dataclass(frozen=True)
class TowardTargetNose:
    """Point from the vessel toward the target (the frame origin)."""


NoseRule = RetrogradeNose | UpNose | FixedNose | TowardTargetNose
"""A steering rule: where the thrust axis points, evaluated against state."""


# ---------------------------------------------------------------------------
# Throttle rules (tagged union)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ConstantThrottle:
    """Fixed throttle (``throttle`` in 0..1)."""

    throttle: float


@dataclass(frozen=True)
class BrakeToThrottle:
    """Brake to ``v_terminal`` at ``h_terminal``, full throttle below it.

    ``h_terminal=0`` is the constant-deceleration (suicide-burn) case: brake to
    ``v_terminal`` at the surface.  With ``h_terminal>0`` the law brakes to
    ``v_terminal`` at that altitude and goes full throttle below it, so a
    follow-up segment takes over the handoff there.
    """

    v_terminal: float
    """Target speed (m/s) reached at ``h_terminal``; a touchdown speed when
    ``h_terminal==0``."""

    h_terminal: float = 0.0
    """Handoff altitude (m); ``0`` means the surface."""


ThrottleRule = ConstantThrottle | BrakeToThrottle
"""A throttle law: a fixed setting or a state-dependent feedback rule."""


# ---------------------------------------------------------------------------
# Segments / schedule
# ---------------------------------------------------------------------------


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
    def coast(cls, nose: NoseRule | None = None) -> ControlSegment:
        """An engine-off segment (throttle 0)."""
        return cls(throttle=ConstantThrottle(0.0), max_thrust=0.0, isp=0.0,
                   nose=nose if nose is not None else RetrogradeNose())


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
            if isinstance(thr, ConstantThrottle) and not (0.0 <= thr.throttle <= 1.0):
                raise ValueError(f"constant throttle out of range: {thr.throttle}")
            if seg.max_thrust < 0.0:
                raise ValueError(f"max_thrust must be >= 0: {seg.max_thrust}")
            if seg.isp < 0.0:
                raise ValueError(f"isp must be >= 0: {seg.isp}")
