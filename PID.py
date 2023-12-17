def clamp(num, limit1, limit2):
    return max(min(num, max(limit1, limit2)), min(limit1, limit2))

# PID
class PID:
    def __init__(self, kp, ki, kd, integral_output_limit = 1):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral_output_limit = integral_output_limit
        self.integral = 0
        self.error_prev = 0

    def update(self, error, dt):
        #P
        p = error * self.kp
        #I
        self.integral += error * dt * self.ki
        self.integral = clamp(self.integral, self.integral_output_limit, -self.integral_output_limit)
        i = self.integral
        #D
        d = (error - self.error_prev) / dt * self.kd
        self.error_prev = error
        return p + i + d