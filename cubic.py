import numpy as np

class CubicSplineTrajectory:
    """
    三维空间三次样条轨迹规划器
    生成满足边界条件的位置、速度、加速度序列
    """
    def __init__(self, N=100):
        """
        参数:
            N: 离散采样点数
        """
        self.N = N
    
    def solve(self, p0, v0, pf, vf, tf):
        """
        求解三次样条轨迹
        
        参数:
            p0: 初始位置 [3,]
            v0: 初始速度 [3,]
            pf: 目标位置 [3,]
            vf: 目标速度 [3,]
            tf: 总飞行时间
        
        返回:
            solution: 字典，包含状态x和控制u
        """
        # 时间离散化
        t = np.linspace(0, tf, self.N)
        
        # 求解多项式系数 [3,4]，每行对应[a,b,c,d]
        coeffs = self._solve_coefficients(p0, v0, pf, vf, tf)
        
        # 生成轨迹序列
        pos, vel, accel = self._generate_trajectory(coeffs, t)
        
        # 组合成标准格式
        solution = {
            'x': np.hstack([pos, vel]),  # [N, 6]
            'u': accel                   # [N, 3]
        }
        
        return solution
    
    def _solve_coefficients(self, p0, v0, pf, vf, tf):
        """
        解析求解每个维度的三次多项式系数
        p(t) = a*t³ + b*t² + c*t + d
        """
        coeffs = np.zeros((3, 4))
        
        # 防止除以零
        if tf <= 0:
            raise ValueError("飞行时间tf必须为正数")
        
        for i in range(3):
            # 已知边界值
            d = p0[i]
            c = v0[i]
            
            # 构建线性方程组求解a和b
            # 方程1: a*tf³ + b*tf² = pf - c*tf - d
            # 方程2: 3a*tf² + 2b*tf = vf - c
            
            A = np.array([[tf**3, tf**2],
                          [3*tf**2, 2*tf]])
            b_vec = np.array([pf[i] - c*tf - d, vf[i] - c])
            
            # 求解
            a, b = np.linalg.solve(A, b_vec)
            coeffs[i] = [a, b, c, d]
        
        return coeffs
    
    def _generate_trajectory(self, coeffs, t):
        """生成离散的轨迹数据"""
        N = len(t)
        pos = np.zeros((N, 3))
        vel = np.zeros((N, 3))
        accel = np.zeros((N, 3))
        
        for i in range(3):
            a, b, c, d = coeffs[i]
            
            # 位置: p(t) = at³ + bt² + ct + d
            pos[:, i] = a * t**3 + b * t**2 + c * t + d
            
            # 速度: v(t) = 3at² + 2bt + c
            vel[:, i] = 3*a * t**2 + 2*b * t + c
            
            # 加速度: a(t) = 6at + 2b
            accel[:, i] = 6*a * t + 2*b
        
        return pos, vel, accel


def generate_spline_trajectory(initial_state, target_pos, tf, N=100, target_vel=None):
    """
    主接口函数：生成三次样条轨迹
    
    参数:
        initial_state: [6,] 数组 [px,py,pz,vx,vy,vz]
        target_pos: [3,] 数组 [px,py,pz]
        tf: 总飞行时间（由你的估算函数提供）
        N: 采样点数（默认100）
        target_vel: [3,] 目标速度，默认为零向量
    
    返回:
        solution: 字典，可直接索引 solution['x'][:, :3] 等
    """
    if target_vel is None:
        target_vel = np.zeros(3)
    
    # 提取初始状态
    p0 = initial_state[:3]
    v0 = initial_state[3:]
    pf = target_pos
    vf = target_vel
    
    # 生成轨迹
    planner = CubicSplineTrajectory(N=N)
    solution = planner.solve(p0, v0, pf, vf, tf)
    
    # 验证边界条件（可选）
    _validate_boundary(solution, p0, v0, pf, vf)
    
    return solution


def _validate_boundary(solution, p0, v0, pf, vf):
    """验证轨迹是否精确满足边界条件"""
    pos_start = solution['x'][0, :3]
    vel_start = solution['x'][0, 3:6]
    pos_end = solution['x'][-1, :3]
    vel_end = solution['x'][-1, 3:6]
    
    print("边界条件验证:")
    print(f"  初始位置误差: {np.linalg.norm(pos_start - p0):.8e} m")
    print(f"  初始速度误差: {np.linalg.norm(vel_start - v0):.8e} m/s")
    print(f"  终点位置误差: {np.linalg.norm(pos_end - pf):.8e} m")
    print(f"  终点速度误差: {np.linalg.norm(vel_end - vf):.8e} m/s\n")


# ==================== 使用示例 ====================

if __name__ == "__main__":
    # 1. 定义初始状态和目标
    initial_state = np.array([500.0, 300.0, 200.0, -20.0, -15.0, -5.0])
    target_pos = np.array([0.0, 0.0, 0.0])
    
    # 2. 使用你的时间估算函数
    def estimate_landing_time(p0, v0, max_accel=10.0):
        """示例：基于最大加速度估算最小飞行时间"""
        dist = np.linalg.norm(p0 - target_pos)
        v_norm = np.linalg.norm(v0)
        
        # 考虑减速和滑行阶段
        t_decel = v_norm / max_accel
        t_coast = max(0, (dist - 0.5 * max_accel * t_decel**2) / max(v_norm, 1e-6))
        
        return max(t_decel + t_coast, 10.0)  # 最小10秒
    
    tf = estimate_landing_time(initial_state[:3], initial_state[3:])
    
    # 3. 生成轨迹
    solution = generate_spline_trajectory(
        initial_state=initial_state,
        target_pos=target_pos,
        tf=tf,
        N=100
    )
    
    # 4. 验证输出格式
    print("输出格式验证:")
    print(f"  solution['x'].shape = {solution['x'].shape}")  # (100, 6)
    print(f"  solution['u'].shape = {solution['u'].shape}")  # (100, 3)
    
    # 5. 索引访问示例（完全匹配你的需求）
    positions = solution['x'][:, :3]   # [N, 3]
    velocities = solution['x'][:, 3:6] # [N, 3]
    accelerations = solution['u']      # [N, 3]
    
    print("\n首点状态:")
    print(f"  位置: {positions[0]}")
    print(f"  速度: {velocities[0]}")
    print(f"  加速度: {accelerations[0]}")
    
    print("\n末点状态:")
    print(f"  位置: {positions[-1]}")
    print(f"  速度: {velocities[-1]}")
    print(f"  加速度: {accelerations[-1]}")
    
    # 6. 检查加速度约束
    max_accel = np.max(np.linalg.norm(accelerations, axis=1))
    print(f"\n最大加速度: {max_accel:.2f} m/s²")