"""Realtime loop pacing on top of a monotonic clock."""

from __future__ import annotations

import time


class FramePacer:
    """Paces a loop to a target frequency with drift correction.

    Frame boundaries are anchored to integer multiples of the period from a
    fixed origin, so the long-run average rate matches ``hz`` exactly even when
    individual ticks jitter. If a tick runs past several boundaries the missed
    frames are skipped instead of accumulating lag.

    Usage::

        pacer = FramePacer(hz=50.0)
        while running:
            pacer.tick()
            # ... per-frame work using pacer.hz or pacer.period ...
    """

    def __init__(self, hz: float, *, max_dt: float | None = None) -> None:
        """Create a pacer locked to *hz* frames per second.

        *max_dt* optionally caps the *dt* returned by :meth:`tick` — a frame
        that stalls for longer (GC pause, debugger breakpoint) reports
        ``max_dt`` instead of the true elapsed time, protecting downstream
        integrators from a single huge step.

        Raises:
            ValueError: if *hz* is not positive.
            ValueError: if *max_dt* is not positive.
        """
        self.hz = hz
        self.max_dt = max_dt
        self._origin = time.monotonic()
        self._prev: float | None = None

    @property
    def hz(self) -> float:
        """Target frequency in frames per second. Setting it re-computes
        ``period`` immediately; the next ``tick()`` call honours the new rate.
        """
        return self._hz

    @hz.setter
    def hz(self, value: float) -> None:
        if value <= 0:
            raise ValueError(f"hz must be positive, got {value}")
        self._hz = float(value)

    @property
    def period(self) -> float:
        """Target frame duration in seconds (``1.0 / hz``)."""
        return 1.0 / self._hz

    @property
    def max_dt(self) -> float | None:
        """Upper bound on the *dt* returned by :meth:`tick` (``None`` = no cap)."""
        return self._max_dt

    @max_dt.setter
    def max_dt(self, value: float | None) -> None:
        if value is not None and value <= 0:
            raise ValueError(f"max_dt must be positive, got {value}")
        self._max_dt = value

    def tick(self) -> float:
        """Block until the next frame boundary and return the elapsed *dt* in
        seconds.

        The returned *dt* equals ``period`` on the first call and the real
        wall-clock delta on subsequent calls (which may be larger than
        ``period`` if the preceding tick was slow — the pacer catches up by
        skipping missed boundaries rather than queuing them).
        """
        now = time.monotonic()
        elapsed = now - self._origin
        frame = int(elapsed * self._hz + 1e-12) + 1
        target = self._origin + frame * self.period
        delay = target - now
        if delay > 0:
            time.sleep(delay)
            now = target
        dt = self.period if self._prev is None else now - self._prev
        if dt <= 0:
            dt = self.period
        if self._max_dt is not None:
            dt = min(dt, self._max_dt)
        self._prev = now
        return dt
