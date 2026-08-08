"""Framework-specific exceptions for the KSP isolation layer."""

from __future__ import annotations


class RecoveryError(Exception):
    """Base for all recovery-framework exceptions."""


class InvalidState(RecoveryError, RuntimeError):
    """Operation attempted in an invalid lifecycle state."""


class VesselNotResolved(RecoveryError, RuntimeError):
    """A kRPC Vessel handle has not yet been resolved."""


class VesselNotFound(RecoveryError, ValueError):
    """No vessel matched the requested name."""


class AmbiguousVesselName(RecoveryError, ValueError):
    """Multiple vessels share the same name."""

    def __init__(self, name: str, candidates: list[str]) -> None:
        self.name = name
        self.candidates = candidates
        joined = ", ".join(candidates)
        super().__init__(f"ambiguous name {name!r}: {len(candidates)} matches ({joined})")


class DuplicateBooster(RecoveryError, ValueError):
    """A booster with this id has already been registered."""


class TargetNotRegistered(RecoveryError, RuntimeError):
    """A target reference frame has not been registered for this vessel."""


class DebugNotEnabled(RecoveryError, RuntimeError):
    """Debug connection has not been enabled (call enable_debug first)."""
