from __future__ import annotations


def lerp(vec1: float, vec2: float, t: float) -> float:
    return t * vec2 + (1 - t) * vec1


def clamp(num: float, limit1: float, limit2: float) -> float:
    return max(min(num, max(limit1, limit2)), min(limit1, limit2))


class PID:
    def __init__(self) -> None:
        self.d_prev: float | None = None
        self.i_prev: float | None = None
        self.p_prev: float | None = None
        self.result: float | None = None
        self.ep = True
        self.ei = True
        self.ed = True
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 1.0
        self.sd = 0.0
        self.diff = 0.0
        self.integral = 0.0
        self.integral_limit = 1.0
        self.error_prev = 0.0
        self.first = True
        self.second = True
        self.dumpf: float | None = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            dt = 1e-6
        if self.first:
            self.first = False
            self.error_prev = error
        elif self.second:
            self.second = False
            self.diff = (error - self.error_prev) / dt

        self.integral += error * dt * self.ki
        self.integral = clamp(self.integral, self.integral_limit, -self.integral_limit)
        self.diff = lerp(self.diff, (error - self.error_prev) / dt, 1 - self.sd)
        p = -error * self.kp
        i = -self.integral
        d = -self.diff * self.kd
        self.result = (
            p * (1 if self.ep else 0)
            + i * (1 if self.ei else 0)
            + d * (1 if self.ed else 0)
        )

        self.p_prev = p
        self.i_prev = i
        self.d_prev = d
        self.error_prev = error
        return self.result
