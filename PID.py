from control_utils import lerp, clamp

class PID:
    def __init__(self):
        self.ep = True
        self.ei = True
        self.ed = True
        self.kp = 1
        self.ki = 0
        self.kd = 1
        self.sd = 0
        self.diff = 0
        self.integral = 0
        self.integral_limit = 1
        self.error_prev = 0
        self.first = True
        self.second = True
        self.dumpf = None
    
    def update(self, error, dt):
        if self.first:
            self.first = False
            self.error_prev = error
        elif self.second:
            self.second = False
            self.diff = (error - self.error_prev) / dt
        
        self.integral += error * dt * self.ki
        self.integral = clamp(self.integral, self.integral_limit, -self.integral_limit)
        self.diff = lerp(self.diff, (error - self.error_prev) / dt, 1-self.sd)
        p = -error * self.kp
        i = -self.integral
        d = -self.diff * self.kd
        self.result = p * (1 if self.ep else 0) + i * (1 if self.ei else 0) + d * (1 if self.ed else 0)
        
        self.p_prev = p
        self.i_prev = i
        self.d_prev = d
        self.error_prev = error
        return self.result