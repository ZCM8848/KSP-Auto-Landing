"""Velocity-profile approaching model for the local attitude controller.

Ported from the legacy control layer; behaviour is preserved bit-for-bit.
"""

from __future__ import annotations

from typing import Any

import numpy as np


class ApproachingModel:
    """A 1-D speed-vs-distance profile used to approach a setpoint.

    Given an error *err* and current rate *v*, :meth:`next_acc` returns the
    acceleration needed to track ``spd_at_dist``.  Both *err* and *v* may be a
    scalar (roll channel) or a 3-vector (direction channel); the return value
    matches the input shape.
    """

    def __init__(
        self,
        k: float,
        b: float,
        max_acc: float,
        max_spd: float,
        accuracy: float = 0.5,
        lock_accuracy_ratio: float = 0.9,
    ) -> None:
        self.k = k
        self.b = b
        self.max_acc = max_acc
        self.max_spd = max_spd
        self.accuracy = accuracy
        self.lock_accuracy_ratio = lock_accuracy_ratio

    def spd_at_dist(self, s: float) -> float:
        """Return the target speed at distance *s* from the setpoint."""
        return float(np.sqrt(2 * self.max_acc * s) / (1 + np.exp(self.k * (s + self.b))))

    def next_acc(self, err: Any, v: Any, dt: float) -> Any:
        """Return the acceleration to track the velocity profile for *err*."""
        dist = np.linalg.norm(err)
        next_spd = self.spd_at_dist(dist)
        next_spd = min(next_spd, self.max_spd)
        direction = 0.0 if dist == 0 else err / dist
        next_v = direction * next_spd
        acc = (next_v - v) / dt
        if dist < self.accuracy:
            acc2 = -v / dt
            ratio = 1 - self.lock_accuracy_ratio
            acc = ratio * acc + (1 - ratio) * acc2
        return acc

    def find_dist_by_spd(self, spd: float, accuracy: float = 0.1) -> float:
        """Return the distance at which the profile reaches speed *spd*."""
        lo, hi = 0.0, 1e8
        while hi > lo + accuracy:
            m = (hi + lo) / 2
            v = self.spd_at_dist(m)
            if v > spd:
                hi = m
            else:
                lo = m
        return (lo + hi) / 2
