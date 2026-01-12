import cvxpy as cp
import numpy as np

def rocket_landing_convex(v0, r0, m0, T_max, Isp, g=9.81, N=50, tf_guess=30):
    """
    v0: [vx0, vy0] 初始速度
    r0: [x0, y0]   初始位置
    m0: 初始质量
    """
    # 参数
    g0 = 9.81
    dt = tf_guess / N
    
    # 变量
    vx = cp.Variable(N+1)
    vy = cp.Variable(N+1)
    z = cp.Variable(N+1)  # ln(m)
    a = cp.Variable(N)    # ux
    b = cp.Variable(N)    # uy
    
    # 比推力上界（保守估计）
    u_max = T_max / (m0 * 0.7)  # 考虑质量不会低于70%
    
    constraints = []
    
    # 初始条件
    constraints += [vx[0] == v0[0], vy[0] == v0[1], z[0] == np.log(m0)]
    
    # 动力学约束
    for i in range(N):
        constraints += [vx[i+1] == vx[i] + a[i]*dt]
        constraints += [vy[i+1] == vy[i] + b[i]*dt - g*dt]
        constraints += [z[i+1] == z[i] - cp.sqrt(a[i]**2 + b[i]**2)/(Isp*g0)*dt]
    
    # 推力约束（二阶锥）
    for i in range(N):
        constraints += [cp.SOC(u_max, cp.vstack([a[i], b[i]]))]
    
    # 终端约束：位置归零（通过速度积分实现）
    # 简化为：x(tf) = x0 + sum(vx*dt) = 0
    constraints += [r0[0] + cp.sum(vx[:-1])*dt == 0]
    constraints += [r0[1] + cp.sum(vy[:-1])*dt == 0]
    
    # 目标：最大化终端质量（最小化燃料）
    objective = cp.Maximize(z[N])
    
    problem = cp.Problem(objective, constraints)
    problem.solve(solver=cp.ECOS, verbose=True)
    
    return vx.value, vy.value, a.value, b.value

# 使用示例
vx_opt, vy_opt, a_opt, b_opt = rocket_landing_convex(
    v0=[500, 300],    # 分离速度
    r0=[5000, 10000], # 分离位置
    m0=20000,         # 初始质量
    T_max=500e3,      # 500kN推力
    Isp=300           # 比冲
)