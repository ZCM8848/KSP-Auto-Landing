import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import minimize_scalar
from math import sin, cos, radians, pi, exp
from typing import Tuple

class RobustLandingSolver:
    def __init__(self, 
                 target_half_length: float,
                 body_radius: float,
                 target_height: float,
                 gravitational_parameter: float,
                 rotational_speed: float,
                 target_latitude: float,
                 rocket_diameter: float,
                 drag_coefficient: float,
                 max_thrust: float,
                 specific_impulse: float,
                 g0: float,
                 current_mass: float,
                 dry_mass: float):
        
        self.target_half_length = target_half_length
        self.R_origin = body_radius + target_height
        self.R_eq = body_radius
        self.mu = gravitational_parameter
        
        # 角速度矢量（目标系）
        lat_rad = radians(target_latitude)
        self.omega = np.array([
            -rotational_speed * sin(lat_rad),
            rotational_speed * cos(lat_rad),
            0.0
        ])
        
        # 火箭参数
        self.A = pi * (rocket_diameter / 2) ** 2
        self.Cd = drag_coefficient
        self.T_max = max_thrust
        self.Isp = specific_impulse
        self.mdot = max_thrust / (specific_impulse * g0)
        self.m0 = current_mass
        self.m_dry = dry_mass
        
        # 终端约束
        self.r_target = np.array([target_half_length, 0.0, 0.0])
        self.v_target = np.array([0.0, 0.0, 0.0])
        
        # 大气模型系数（截断处理）
        self._rho_coeffs = np.array([-1.379984e-13, 5.813944e-09, -1.261496e-04, 1.106141e+00])
        self._rho_max_h = 20000  # 模型有效高度上限（20km）
        
    def atmospheric_density(self, h: float) -> float:
        """安全的大气密度计算，高度超出范围时指数衰减"""
        if h > self._rho_max_h:
            # 在20km以上使用指数外推（避免多项式负值）
            rho_20k = self._atm_poly(self._rho_max_h)
            scale_height = 5000  # JNSQ特征高度约5km
            return rho_20k * exp(-(h - self._rho_max_h) / scale_height)
        return self._atm_poly(max(0.0, h))
    
    def _atm_poly(self, h: float) -> float:
        """原始多项式"""
        return ((self._rho_coeffs[0] * h + self._rho_coeffs[1]) * h + self._rho_coeffs[2]) * h + self._rho_coeffs[3]
    
    def geocentric_position(self, r_target: np.ndarray) -> np.ndarray:
        return np.array([self.R_origin + r_target[0], r_target[1], r_target[2]])
    
    def dynamics(self, t: float, X: np.ndarray, thrust: bool = True) -> np.ndarray:
        """动力学：thrust=True为制动段，False为滑行段"""
        r = X[0:3]
        v = X[3:6]
        m = X[6]
        
        # 高度计算
        r_geo = self.geocentric_position(r)
        r_norm = np.linalg.norm(r_geo)
        h = r_norm - self.R_eq
        
        # 重力
        g_vec = -(self.mu / r_norm**3) * r_geo
        
        # 非惯性力
        coriolis = -2.0 * np.cross(self.omega, v)
        centrifugal = -np.cross(self.omega, np.cross(self.omega, r_geo))
        
        v_norm = np.linalg.norm(v)
        if v_norm < 1e-3:
            return np.concatenate([v, g_vec + coriolis + centrifugal, [0.0]])
        
        v_dir = v / v_norm
        
        # 阻力
        rho = self.atmospheric_density(h)
        drag_mag = 0.5 * rho * v_norm**2 * self.Cd * self.A
        drag_acc = -(drag_mag / m) * v_dir
        
        # 推力（与速度反向）
        if thrust and m > self.m_dry:
            thrust_acc = -(self.T_max / m) * v_dir
            dm = -self.mdot
        else:
            thrust_acc = np.zeros(3)
            dm = 0.0
        
        a = g_vec + coriolis + centrifugal + thrust_acc + drag_acc
        return np.concatenate([v, a, [dm]])
    
    def simulate(self, r0: np.ndarray, v0: np.ndarray, m0: float, 
                 t_coast: float, t_burn_max: float = 30.0) -> dict:
        """
        完整模拟：滑行 + 制动
        返回详细的终端状态和误差
        """
        X0 = np.concatenate([r0, v0, [m0]])
        
        # ---- 阶段1：滑行 ----
        sol_coast = solve_ivp(
            fun=lambda t, X: self.dynamics(t, X, thrust=False),
            t_span=[0, t_coast],
            y0=X0,
            method='RK45',
            rtol=1e-7,
            atol=1e-10,
            events=self._ground_impact_event
        )
        
        # 检查是否提前撞地
        if len(sol_coast.t_events[0]) > 0:
            return {'error': 1e8, 'reason': 'crashed_during_coast', 
                    't_coast_actual': sol_coast.t_events[0][0]}
        
        X_ign = sol_coast.y[:, -1]
        r_ign = X_ign[0:3]
        v_ign = X_ign[3:6]
        m_ign = X_ign[6]
        
        # 检查制动初始高度
        if r_ign[0] <= self.target_half_length + 10:  # 至少留10米高度裕度
            return {'error': 1e8, 'reason': 'too_low_to_ignite', 'r_ign': r_ign}
        
        # ---- 阶段2：制动 ----
        # 事件：速度接近0（着陆完成）或撞地
        sol_burn = solve_ivp(
            fun=lambda t, X: self.dynamics(t, X, thrust=True),
            t_span=[0, t_burn_max],
            y0=X_ign,
            method='RK45',
            rtol=1e-7,
            atol=1e-10,
            events=[self._velocity_zero_event, self._ground_impact_event, 
                   self._fuel_depletion_event]
        )
        
        X_f = sol_burn.y[:, -1]
        r_f = X_f[0:3]
        v_f = X_f[3:6]
        m_f = X_f[6]
        
        # 分析终止原因
        terminated_early = len(sol_burn.t_events[0]) > 0 or len(sol_burn.t_events[1]) > 0
        
        # 计算误差
        pos_err = np.linalg.norm(r_f - self.r_target)
        vel_err = np.linalg.norm(v_f)
        
        # 综合误差（位置优先，速度权重高）
        # 如果速度未降到1m/s以下，给予大惩罚
        if vel_err > 1.0:
            error = 1e6 + vel_err * 100 + pos_err
        else:
            error = pos_err + vel_err * 10
        
        return {
            'error': error,
            'pos_err': pos_err,
            'vel_err': vel_err,
            't_coast': t_coast,
            't_burn': sol_burn.t[-1],
            'r_ign': r_ign,
            'v_ign': v_ign,
            'm_ign': m_ign,
            'r_f': r_f,
            'v_f': v_f,
            'm_f': m_f,
            'fuel_used': m_ign - m_f,
            'terminated_early': terminated_early,
            'reason': 'success' if vel_err <= 1.0 and pos_err < 100 else 'incomplete'
        }
    
    def _ground_impact_event(self, t, X):
        """撞地检测：x坐标小于火箭半长"""
        return X[0] - self.target_half_length - 0.1  # 留0.1m数值裕度
    _ground_impact_event.terminal = True
    _ground_impact_event.direction = -1
    
    def _velocity_zero_event(self, t, X):
        """速度接近0（着陆完成）"""
        return np.linalg.norm(X[3:6]) - 0.1  # 0.1m/s视为着陆完成
    _velocity_zero_event.terminal = True
    _velocity_zero_event.direction = -1
    
    def _fuel_depletion_event(self, t, X):
        return X[6] - self.m_dry
    _fuel_depletion_event.terminal = True
    _fuel_depletion_event.direction = -1
    
    def solve(self, r_current: np.ndarray, v_current: np.ndarray) -> dict:
        """
        主求解函数
        """
        print(f"开始求解: 高度 {r_current[0]:.1f}m, 速度 {np.linalg.norm(v_current):.1f}m/s")
        print(f"目标位置: [{self.target_half_length}, 0, 0]")
        
        # 估算物理边界
        h_current = r_current[0] - self.target_half_length
        v_down = abs(v_current[0])
        
        # 最大滑行时间：假设只受重力加速（保守估计）
        g_approx = self.mu / (self.R_origin + r_current[0])**2
        # 解方程: h = v*t + 0.5*g*t^2 => t_max
        discriminant = v_down**2 + 2 * g_approx * h_current
        if discriminant > 0:
            t_max_fall = (-v_down + np.sqrt(discriminant)) / g_approx
        else:
            t_max_fall = h_current / v_down if v_down > 1 else 60.0
        
        t_max = min(t_max_fall * 0.95, 40.0)  # 留5%余量，且不超过40秒
        t_min = 0.0
        
        print(f"搜索范围: t_coast ∈ [{t_min:.1f}, {t_max:.1f}] s")
        
        # 粗扫描 + 精细优化
        # 先粗扫描找大致范围
        n_scan = 20
        t_scan = np.linspace(t_min, t_max, n_scan)
        errors = []
        results = []
        
        for t in t_scan:
            res = self.simulate(r_current, v_current, self.m0, t)
            errors.append(res['error'])
            results.append(res)
            if res['error'] < 1e6:
                print(f"  t={t:.1f}s: 误差={res['error']:.1f}, "
                      f"点火高度={res['r_ign'][0]:.1f}, 终端速度={res['vel_err']:.2f}")
        
        best_idx = np.argmin(errors)
        if errors[best_idx] > 1e6:
            print("警告：未找到可行解，使用最小误差解")
        
        t_best = t_scan[best_idx]
        
        # 精细优化（在粗解附近）
        t_left = max(t_min, t_best - (t_max - t_min)/n_scan)
        t_right = min(t_max, t_best + (t_max - t_min)/n_scan)
        
        result = minimize_scalar(
            lambda t: self.simulate(r_current, v_current, self.m0, t)['error'],
            bounds=(t_left, t_right),
            method='bounded',
            options={'xatol': 0.01, 'maxiter': 20}
        )
        
        optimal_t = result.x
        solution = self.simulate(r_current, v_current, self.m0, optimal_t)
        
        print(f"\n优化完成:")
        print(f"最优滑行时间: {optimal_t:.2f}s")
        print(f"期望制动初始位置: [{solution['r_ign'][0]:.2f}, {solution['r_ign'][1]:.2f}, {solution['r_ign'][2]:.2f}]")
        print(f"期望制动初始速度: [{solution['v_ign'][0]:.2f}, {solution['v_ign'][1]:.2f}, {solution['v_ign'][2]:.2f}]")
        print(f"预计燃烧时间: {solution['t_burn']:.2f}s, 燃料消耗: {solution['fuel_used']:.1f}kg")
        print(f"终端误差: 位置 {solution['pos_err']:.2f}m, 速度 {solution['vel_err']:.2f}m/s")
        
        return solution


solver = RobustLandingSolver(
    target_half_length=9.84452301927887,
    body_radius=1600000.0,
    target_height=322.5390499008354,
    gravitational_parameter=25105024000000.0,
    rotational_speed=0.000145842581331033,
    target_latitude=5.1753303155099e-06,
    rocket_diameter=4.201615865637029,
    drag_coefficient=0.21182465553283691,
    max_thrust=2068187.25,
    specific_impulse=341.701416015625,
    g0=9.81,
    current_mass=36768.9921875,
    dry_mass=18588.76953125
)

r_current = np.array([18655.01534148, 1107.99472701, 4777.87684432])
v_current = np.array([-864.6877553, -67.77353271, -288.58563701])

solution = solver.solve(r_current, v_current)
