"""Realtime loop pacing on top of a monotonic clock."""

from __future__ import annotations

import math
import time


class FramePacer:
    """Paces a loop to a target frequency with drift correction.

    Frame boundaries are anchored to integer multiples of the period from a
    fixed origin, so the long-run average rate matches ``hz`` exactly even when
    individual ticks jitter. If a tick runs past several boundaries the missed
    frames are skipped instead of accumulating lag.
    """

    def __init__(self, hz: float) -> None:
        self.hz = hz
        self._origin = time.monotonic()
        self._prev: float | None = None

    @property
    def hz(self) -> float:
        return self._hz

    @hz.setter
    def hz(self, value: float) -> None:
        if value <= 0:
            raise ValueError(f"hz must be positive, got {value}")
        self._hz = float(value)

    @property
    def period(self) -> float:
        return 1.0 / self._hz

    def tick(self) -> float:
        """Block until the next frame boundary and return the elapsed dt."""
        now = time.monotonic()
        elapsed = now - self._origin
        frame = max(1, math.ceil(elapsed / self.period))
        target = self._origin + frame * self.period
        delay = target - now
        if delay > 0:
            time.sleep(delay)
            now = target
        dt = self.period if self._prev is None else now - self._prev
        self._prev = now
        return dt
