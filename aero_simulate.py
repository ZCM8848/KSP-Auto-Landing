import numpy as np
from scipy.optimize import minimize
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# ===================== 1. 基础参数定义 =====================
# 火箭基本参数
m = 20000  # 一子级质量，kg
S = 12.57  # 箭体参考面积（直径3.7m），m²
g0 = 9.81  # 重力加速度，m/s²

# 大气模型（简化的1976标准大气，仅高度相关）
def atmosphere(h):
    """输入高度h(m)，返回大气密度ρ(kg/m³)、声速a(m/s)"""
    if h < 11000:  # 对流层
        T = 288.15 - 0.0065 * h
        p = 101325 * (T/288.15)**(-g0*0.0289644/(8.31432*0.0065))
    else:  # 平流层底部（简化）
        T = 216.65
        p = 22632.06 * np.exp(-g0*0.0289644*(h-11000)/(8.31432*T))
    ρ = p * 0.0289644 / (8.31432 * T)
    a = np.sqrt(1.4 * 8.31432 * T / 0.0289644)
    return ρ, a

# 气动系数（攻角α单位：rad，简化的Cd/Cl模型）
def aerodynamic_coeffs(α):
    """输入攻角α(rad)，返回Cd(阻力系数)、Cl(升力系数)"""
    α_deg = np.degrees(α)
    # 简化模型：攻角0°时Cd最小，升力系数和攻角线性相关
    Cd = 0.2 + 0.01 * α_deg**2  # 阻力系数随攻角增大而增大
    Cl = 0.1 * α_deg  # 升力系数和攻角线性相关
    return Cd, Cl

# ===================== 2. 动力学方程 =====================
def rocket_dynamics(t, state, alpha_seq, t_nodes):
    """
    火箭气动段动力学方程（无发动机）
    t: 当前时间，s
    state: [r, h, V, γ] 径向距离(m)、高度(m)、速度(m/s)、弹道倾角(rad)
    alpha_seq: 离散的攻角序列（优化变量）
    t_nodes: 攻角序列对应的时间节点
    """
    r, h, V, γ = state
    
    # 插值得到当前时刻的攻角（保证攻角连续）
    α = np.interp(t, t_nodes, alpha_seq)
    α = np.clip(α, -np.radians(10), np.radians(10))  # 攻角硬约束
    
    # 大气参数
    ρ, _ = atmosphere(h)
    # 气动系数
    Cd, Cl = aerodynamic_coeffs(α)
    
    # 气动阻力和升力
    D = 0.5 * ρ * V**2 * S * Cd
    L = 0.5 * ρ * V**2 * S * Cl
    
    # 重力（简化为g0，高度变化不大时）
    g = g0
    
    # 动力学导数
    drdt = V * np.cos(γ)
    dhdt = V * np.sin(γ)
    dVdt = -D/m - g * np.sin(γ)
    dγdt = (1/V) * ( (L*np.cos(α) - D*np.sin(α))/m - g*np.cos(γ) )
    
    return [drdt, dhdt, dVdt, dγdt]

# ===================== 3. 代价函数与约束 =====================
def objective(alpha_seq, t_nodes, initial_state, target_state, t_total):
    """代价函数：最小化终端误差 + 攻角变化率（平滑性）"""
    # 积分求解轨迹
    sol = solve_ivp(
        fun=rocket_dynamics,
        t_span=(0, t_total),
        y0=initial_state,
        args=(alpha_seq, t_nodes),
        t_eval=np.linspace(0, t_total, 100),
        method='RK45'
    )
    if not sol.success:
        return 1e9  # 积分失败则惩罚
    
    # 终端状态
    r_final = sol.y[0][-1]
    h_final = sol.y[1][-1]
    V_final = sol.y[2][-1]
    γ_final = sol.y[3][-1]
    
    # 终端误差（核心代价）
    error = (r_final - target_state[0])**2 + \
            (h_final - target_state[1])**2 + \
            (V_final - target_state[2])**2 + \
            (γ_final - target_state[3])**2
    
    # 攻角变化率惩罚（平滑性）
    alpha_deriv = np.sum(np.diff(alpha_seq)**2)
    
    # 总代价
    return error * 1e-3 + alpha_deriv * 1e2

def constraint_fun(alpha_seq, t_nodes, initial_state, target_state, t_total):
    """约束函数：动压不超过极限 + 终端状态约束"""
    sol = solve_ivp(
        fun=rocket_dynamics,
        t_span=(0, t_total),
        y0=initial_state,
        args=(alpha_seq, t_nodes),
        t_eval=np.linspace(0, t_total, 100),
        method='RK45'
    )
    if not sol.success:
        return [1e9] * (len(sol.t) + 4)
    
    # 1. 动压约束：q = 0.5*ρ*V² ≤ q_max（比如q_max=1e5 Pa）
    q_max = 1e5
    q_list = []
    for i in range(len(sol.t)):
        h = sol.y[1][i]
        V = sol.y[2][i]
        ρ, _ = atmosphere(h)
        q = 0.5 * ρ * V**2
        q_list.append(q - q_max)  # 约束：q - q_max ≤ 0
    
    # 2. 终端状态约束（等式约束，转化为不等式：|x - x_target| ≤ 1e-2）
    r_final = sol.y[0][-1]
    h_final = sol.y[1][-1]
    V_final = sol.y[2][-1]
    γ_final = sol.y[3][-1]
    
    terminal_constraints = [
        abs(r_final - target_state[0]) - 10,  # 径向误差≤10m
        abs(h_final - target_state[1]) - 10,  # 高度误差≤10m
        abs(V_final - target_state[2]) - 5,   # 速度误差≤5m/s
        abs(γ_final - target_state[3]) - 0.05 # 弹道倾角误差≤0.05rad
    ]
    
    return q_list + terminal_constraints

# ===================== 4. 优化求解 =====================
if __name__ == "__main__":
    # 初始状态（气动引导段起始点）
    initial_state = [0, 8000, 1800, np.radians(-15)]  # r=0, h=8000m, V=1800m/s, γ=-15°
    # 目标状态（引导段终点，比如着陆前的悬停准备点）
    target_state = [5000, 3000, 500, np.radians(-5)]   # r=5000m, h=3000m, V=500m/s, γ=-5°
    
    # 时间设置：总引导时间t_total=60s，离散为10个时间节点
    t_total = 60
    n_nodes = 10
    t_nodes = np.linspace(0, t_total, n_nodes)
    
    # 初始攻角猜测（全0攻角）
    alpha_init = np.zeros(n_nodes)  # 优化变量：每个时间节点的攻角（rad）
    
    # 约束设置
    constraints = {
        'type': 'ineq',  # 不等式约束：constraint_fun ≤ 0
        'fun': constraint_fun,
        'args': (t_nodes, initial_state, target_state, t_total)
    }
    
    # 攻角边界（-10°~10°）
    alpha_bounds = [(-np.radians(10), np.radians(10)) for _ in range(n_nodes)]
    
    # 优化求解（用SLSQP算法，适合带约束的非线性优化）
    result = minimize(
        fun=objective,
        x0=alpha_init,
        args=(t_nodes, initial_state, target_state, t_total),
        method='SLSQP',
        bounds=alpha_bounds,
        constraints=constraints,
        options={'maxiter': 100, 'disp': True}
    )
    
    # ===================== 5. 结果可视化 =====================
    if result.success:
        # 最优攻角序列
        alpha_opt = result.x
        
        # 积分得到最优轨迹
        sol_opt = solve_ivp(
            fun=rocket_dynamics,
            t_span=(0, t_total),
            y0=initial_state,
            args=(alpha_opt, t_nodes),
            t_eval=np.linspace(0, t_total, 200),
            method='RK45'
        )
        
        # 计算动压
        q_opt = []
        for i in range(len(sol_opt.t)):
            h = sol_opt.y[1][i]
            V = sol_opt.y[2][i]
            ρ, _ = atmosphere(h)
            q = 0.5 * ρ * V**2
            q_opt.append(q)
        
        # 绘图
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
        
        # 高度-时间
        ax1.plot(sol_opt.t, sol_opt.y[1], 'b-', label='高度')
        ax1.axhline(target_state[1], color='r--', label='目标高度')
        ax1.set_xlabel('时间 (s)')
        ax1.set_ylabel('高度 (m)')
        ax1.legend()
        ax1.grid(True)
        
        # 速度-时间
        ax2.plot(sol_opt.t, sol_opt.y[2], 'b-', label='速度')
        ax2.axhline(target_state[2], color='r--', label='目标速度')
        ax2.set_xlabel('时间 (s)')
        ax2.set_ylabel('速度 (m/s)')
        ax2.legend()
        ax2.grid(True)
        
        # 攻角-时间
        ax3.plot(t_nodes, np.degrees(alpha_opt), 'ro-', label='最优攻角')
        ax3.set_xlabel('时间 (s)')
        ax3.set_ylabel('攻角 (°)')
        ax3.legend()
        ax3.grid(True)
        
        # 动压-时间
        ax4.plot(sol_opt.t, q_opt, 'g-', label='动压')
        ax4.axhline(1e5, color='r--', label='动压极限')
        ax4.set_xlabel('时间 (s)')
        ax4.set_ylabel('动压 (Pa)')
        ax4.legend()
        ax4.grid(True)
        
        plt.tight_layout()
        plt.show()
        
        # 输出终端状态
        print("终端状态：")
        print(f"径向距离：{sol_opt.y[0][-1]:.2f} m (目标：{target_state[0]} m)")
        print(f"高度：{sol_opt.y[1][-1]:.2f} m (目标：{target_state[1]} m)")
        print(f"速度：{sol_opt.y[2][-1]:.2f} m/s (目标：{target_state[2]} m/s)")
        print(f"弹道倾角：{np.degrees(sol_opt.y[3][-1]):.2f} ° (目标：{np.degrees(target_state[3])} °)")
    else:
        print("优化失败：", result.message)