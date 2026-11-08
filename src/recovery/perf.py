"""Opt-in microsecond profiling hooks for the recovery framework.

Instrumentation is cheap when disabled: :meth:`Profiler.section` returns a
shared no-op context manager after a single boolean check, so the hooks can
stay in production control-loop paths.

Usage::

    from recovery.perf import PROFILER

    PROFILER.enable()
    ...                         # framework records named section durations
    per_frame = PROFILER.take()  # {name: nanoseconds}, resets the totals

Section names currently emitted by the framework:

* ``pace.tick`` / ``pace.sleep`` — :meth:`recovery.clock.FramePacer.tick`
* ``io.snapshot`` — :meth:`recovery.ksp.connection.KspConnection.snapshot`
* ``ctl.apply`` — :meth:`recovery.ksp.control.VesselControls.apply`
* ``ctl.throttle`` / ``ctl.refframe`` / ``ctl.dir`` / ``ctl.up`` /
  ``ctl.roll`` / ``ctl.engaged`` / ``ctl.engage`` — individual RPCs inside
  ``VesselControls.apply``.
"""

from __future__ import annotations

import time
from typing import Any


class _Section:
    """Times one ``with`` block and folds it into a :class:`Profiler`."""

    __slots__ = ("_prof", "_name", "_t0")

    def __init__(self, prof: Profiler, name: str) -> None:
        self._prof = prof
        self._name = name
        self._t0 = 0

    def __enter__(self) -> None:
        self._t0 = time.perf_counter_ns()

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        dt = time.perf_counter_ns() - self._t0
        totals = self._prof._totals
        totals[self._name] = totals.get(self._name, 0) + dt
        counts = self._prof._counts
        counts[self._name] = counts.get(self._name, 0) + 1


class _Noop:
    """Zero-overhead stand-in used while the profiler is disabled."""

    __slots__ = ()

    def __enter__(self) -> None:
        return None

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        return None


_NOOP = _Noop()


class Profiler:
    """Accumulates named section durations in nanoseconds.

    Not thread-safe by design: the instrumented framework paths all run on the
    control thread, and diagnostic accuracy is preferred over locking overhead.
    """

    def __init__(self) -> None:
        self._enabled = False
        self._totals: dict[str, int] = {}
        self._counts: dict[str, int] = {}

    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        self._enabled = True

    def disable(self) -> None:
        self._enabled = False

    def section(self, name: str) -> Any:
        """Return a context manager timing *name*, or a no-op when disabled."""
        if not self._enabled:
            return _NOOP
        return _Section(self, name)

    def take(self) -> dict[str, int]:
        """Return accumulated nanoseconds per section since the last take.

        Resets the accumulators, so the next :meth:`take` covers only the
        sections recorded afterwards.
        """
        out = self._totals
        self._totals = {}
        self._counts = {}
        return out

    def report(self) -> dict[str, tuple[int, int]]:
        """Return ``{name: (calls, total_ns)}`` without resetting."""
        return {name: (self._counts.get(name, 0), total) for name, total in self._totals.items()}


PROFILER = Profiler()
