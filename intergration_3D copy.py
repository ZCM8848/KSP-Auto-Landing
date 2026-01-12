import numpy as np
import casadi as ca
import math

def normalize(a: ca.MX) -> ca.MX:
    epsilon = 1e-8
    return a / (ca.norm_2(a) + epsilon)


mass= 98887.5078125
dry_mass= 26361.478515625
available_thrust= 1902927.25
specific_impulse= 334.99658203125
initial_position= np.array([150, -332, 2400])
initial_velocity= np.array([-12, 6.9, -179])
target_position = np.array([0, 0, 500]).reshape((1, 3))
target_velocity = np.array([0, 0, -30]).reshape((1, 3))
max_tilt = 10
glide_slope = 45
g = 9.80665
k = 0.35
throttle_range = [0.4, 1.1]
up = np.array([0, 0, 1]).reshape((1, 3))

problem = ca.Opti()
time_of_flight = 30
n = 100
dt = time_of_flight / n

x = problem.variable(n, 6)
t = problem.variable(n, 3)
m = problem.variable(n, 1)
problem.subject_to(x[0, :] == np.concatenate([initial_position, initial_velocity]).reshape((1, 6)))
problem.subject_to(m[0] == mass)
problem.subject_to(t[0, :] == np.zeros(shape=(1, 3)))

for i in range(n):
    problem.subject_to(ca.Opti_bounded(available_thrust*throttle_range[0], ca.norm_2(t[i]), available_thrust*throttle_range[1]))
    problem.subject_to(ca.dot(up, normalize(t[i, :])) >= ca.norm_2(up)*ca.norm_2(t[i, :])*ca.cos(math.radians(max_tilt)))
    problem.subject_to(x[i, 2] >= 0)
    problem.subject_to(ca.Opti_bounded(dry_mass, m[i], mass))
    if i < n-1:
        acc = np.array([0, 0, -g]).reshape((1, 3)) + t[i, :]/m[i]
        m[i+1] = m[i] - (ca.norm_2(t)/(specific_impulse*g))*i*dt
        x[i+1, 0:3] = x[i, 0:3] + 0.5*acc*dt**2
        x[i+1, 3:6] = x[i, 3:6] + acc*dt

problem.subject_to(x[-1, :3] == target_position)
problem.subject_to(x[-1, 3:] == target_velocity)
objective = mass - m[-1]
problem.minimize(objective)
opts = {
    'ipopt.max_iter': 10000,
    'ipopt.tol': 1e-6,
    'print_time': True,
    'ipopt.print_level': 3  # 减少打印冗余，方便查看关键信息
}
problem.solver('ipopt', opts)

# 求解（增加异常捕获，方便调试）
try:
    sol = problem.solve()
    # 提取结果
    x_sol = sol.value(x)
    t_sol = sol.value(t)
    m_sol = sol.value(m)
    print("="*50)
    print("求解成功！")
    print(f"终端质量：{m_sol[-1][0]:.2f} kg")
    print(f"燃料消耗：{mass - m_sol[-1][0]:.2f} kg")
    print(f"终端位置：{x_sol[-1, :3]}")
    print(f"终端速度：{x_sol[-1, 3:]}")
except Exception as e:
    print("="*50)
    print(f"求解失败，错误信息：{e}")
    # 提取调试中间结果
    x_sol = problem.debug.value(x)
    t_sol = problem.debug.value(t)
    m_sol = problem.debug.value(m)
    print("已提取调试用中间结果")