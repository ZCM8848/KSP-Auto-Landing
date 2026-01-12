import numpy as np
from scipy.integrate import solve_ivp

mass= 98887.5078125
dry_mass= 26361.478515625
available_thrust= 1902927.25
specific_impulse= 334.99658203125
initial_position= [55420.7212688, 13440.62966535, 24222.02999502]
initial_velocity= [1179.84897419, 388.63473972, 692.80155726]

v0 = 40
x0 = 1520
X0 = [x0, v0]

def dynamics(t, X):
    a = -9.80665
    # 对于一维问题，直接解包两个元素
    x, v = X  # ✅ x=X[0], v=X[1]
    return [v, a]  # ✅ 返回 [dx/dt, dv/dt]

def velocity_event(t, X):
    """
    事件检测函数：当速度达到-3 m/s时触发
    X[1] + 3 = 0  →  v = -3
    """
    return X[1] + 3

velocity_event.terminal = True
velocity_event.direction = -1

t_span = (0, 100)
solution = solve_ivp(dynamics, t_span, X0, events=velocity_event, max_step=0.01, dense_output=True)
print(solution.y)
print(solution.t)