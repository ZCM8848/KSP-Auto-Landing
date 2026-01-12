import numpy as np
from scipy.integrate import solve_ivp
from control import normalize

mass= 98887.5078125
dry_mass= 26361.478515625
available_thrust= 1902927.25
specific_impulse= 334.99658203125
initial_position= [13440.62966535, 24222.02999502, 55420.7212688]
initial_velocity= [388.63473972, 692.80155726, 1179.84897419]

X0 = np.concatenate([initial_position, initial_velocity])

def dynamics(t, X):
    r = X[0:3]
    v = X[3:6]
    a = np.array([0, 0, -9.80665])
    return np.concatenate([v, a])

def velocity_event(t, X):
    return X[2]

velocity_event.terminal = True
velocity_event.direction = -1

t_span = (0, 300)
solution = solve_ivp(dynamics, t_span, X0, events=velocity_event, max_step=0.01, dense_output=True)
print(solution.y)
print(solution.t)