#!/usr/bin/env python3
"""
火箭ignition时机优化程序
包含3D动力学、mass消耗和空气阻力模型
target：在指定altitudeh_t达到指定velocityv_t（向下）
"""

import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ==================== 火箭参数配置 ====================
class RocketParams:
    def __init__(self):
        # 初始状态
        self.mass = 98887.5078125          # 初始mass (kg)
        self.dry_mass = 26361.478515625    # 干mass (kg)
        self.available_thrust = 1902927.25  # 最大thrust (N)
        self.max_throttle = 1.1
        self.min_throttle = 0.4
        self.Isp = 334.99658203125         # 比冲 (s)
        self.g0 = 9.80665                  # 重力加velocity (m/s²)
        
        # 初始条件
        self.r0 = np.array([150, -332, 2400], dtype=float)  # 初始位置 [x, y, z] (m)
        self.v0 = np.array([-12, 6.9, -179], dtype=float)   # 初始velocity (m/s)
        
        # 空气阻力参数 (f_drag = k * v²)
        # 假设火箭截面积 A = 10 m², Cd = 0.5, ρ = 1.225 kg/m³ (海平面)
        self.drag_coeff = 0.30625  # k = 0.5 * ρ * Cd * A
        
        # 推进剂流量计算
        self.mdot_max = self.available_thrust / (self.Isp * self.g0)  # 最大推进剂流量

# ==================== 最优控制问题求解器 ====================
class RocketTimingOptimizer:
    def __init__(self, params:RocketParams, h_t, v_t, dt=0.5, N=200):
        """
        Args:
            params: RocketParams对象
            h_t: targetaltitude (m)
            v_t: targetvelocity（向下，正值）(m/s)
            dt: time步长 (s)
            N: 离散time步数
        """
        self.params = params
        self.h_t = h_t
        self.v_t = v_t
        self.dt = dt
        self.N = N
        
        # 创建Opti问题
        self.opti = ca.Opti()
        
        # 状态变量: [x, y, z, vx, vy, vz, m]
        self.X = self.opti.variable(7, N+1)
        
        # 控制变量: thrust大小 (可以变thrust)
        self.T = self.opti.variable(1, N)
        
        # time变量
        self.t = self.opti.variable()
        
    def setup_problem(self):
        """设置完整的最优控制问题"""
        params = self.params
        
        # 提取状态变量
        x, y, z = self.X[0, :], self.X[1, :], self.X[2, :]
        vx, vy, vz = self.X[3, :], self.X[4, :], self.X[5, :]
        mass = self.X[6, :]
        
        # ============== target函数 ==============
        # 最小化fuel consumption（等价于最大化最终mass）
        self.opti.minimize(-mass[-1])
        
        # ============== 动力学约束 ==============
        for k in range(self.N):
            # 当前状态
            pos = ca.vertcat(x[k], y[k], z[k])
            vel = ca.vertcat(vx[k], vy[k], vz[k])
            speed_sq = ca.dot(vel, vel)
            speed = ca.sqrt(speed_sq)
            
            # thrust方向：始终与velocity方向相反
            thrust_dir = -vel / (speed + 1e-6)  # 避免除以零
            
            # thrust向量
            thrust_vec = self.T[k] * thrust_dir
            
            # 空气阻力（与velocity方向相反）
            drag_force = params.drag_coeff * speed_sq
            drag_dir = -vel / (speed + 1e-6)
            drag_vec = drag_force * drag_dir
            
            # 重力
            gravity_vec = ca.vertcat(0, 0, -params.g0 * mass[k])
            
            # 合力
            total_force = thrust_vec + drag_vec + gravity_vec
            
            # 加velocity
            accel = total_force / mass[k]
            
            # mass流量
            mdot = self.T[k] / (params.Isp * params.g0)
            
            # 动力学方程（RK4积分）
            state_now = self.X[:, k]
            state_next = self.X[:, k+1]
            
            # RK4步进
            k1 = self._dynamics(state_now, accel, mdot)
            k2 = self._dynamics(state_now + self.dt/2 * k1, accel, mdot)
            k3 = self._dynamics(state_now + self.dt/2 * k2, accel, mdot)
            k4 = self._dynamics(state_now + self.dt * k3, accel, mdot)
            
            state_pred = state_now + self.dt/6 * (k1 + 2*k2 + 2*k3 + k4)
            
            # 添加动力学约束
            self.opti.subject_to(state_next == state_pred)
        
        # ============== 初始条件约束 ==============
        self.opti.subject_to(x[0] == params.r0[0])
        self.opti.subject_to(y[0] == params.r0[1])
        self.opti.subject_to(z[0] == params.r0[2])
        self.opti.subject_to(vx[0] == params.v0[0])
        self.opti.subject_to(vy[0] == params.v0[1])
        self.opti.subject_to(vz[0] == params.v0[2])
        self.opti.subject_to(mass[0] == params.mass)
        
        # ============== 终端约束 ==============
        # 在最终时刻达到指定altitude和velocity（向下）
        self.opti.subject_to(z[-1] == self.h_t)
        self.opti.subject_to(vz[-1] == -self.v_t)  # vz为负表示向下
        
        # ============== 路径约束 ==============
        # thrust约束：0 <= T <= available_thrust
        self.opti.subject_to(self.opti.bounded(self.params.available_thrust*self.params.min_throttle, 
                                               self.T, 
                                               self.params.available_thrust*self.params.max_throttle))
        
        # mass约束：dry_mass <= m <= initial_mass
        self.opti.subject_to(self.opti.bounded(params.dry_mass, mass, params.mass))
        
        # altitude约束：z >= 0（不能低于地面）
        self.opti.subject_to(z >= 0)
        
        # ============== 求解器设置 ==============
        opts = {
            'ipopt.max_iter': 3000,
            'ipopt.tol': 1e-6,
            'ipopt.acceptable_tol': 1e-5,
            'ipopt.print_level': 5,
            'print_time': True
        }
        self.opti.solver('ipopt', opts)
    
    def _dynamics(self, state, accel, mdot):
        """动力学函数"""
        # state = [x, y, z, vx, vy, vz, m]
        # accel = [ax, ay, az]
        return ca.vertcat(
            state[3], state[4], state[5],  # xdot, ydot, zdot
            accel[0], accel[1], accel[2],  # vxdot, vydot, vzdot
            -mdot                          # mdot
        )
    
    def solve(self, initial_guess=None):
        """求解最优控制问题"""
        if initial_guess is None:
            # 提供初始猜测
            self.opti.set_initial(self.X, np.tile(
                np.concatenate([self.params.r0, self.params.v0, [self.params.mass]]),
                (self.N+1, 1)
            ).T)
            self.opti.set_initial(self.T, self.params.available_thrust * 0.5)
        
        try:
            sol = self.opti.solve()
            return sol
        except Exception as e:
            print(f"求解失败: {e}")
            return None
    
    def extract_solution(self, sol):
        """提取并格式化求解结果"""
        X_opt = sol.value(self.X)
        T_opt = sol.value(self.T)
        
        solution = {
            'time': np.linspace(0, self.N * self.dt, self.N+1),
            'position': X_opt[:3, :].T,
            'velocity': X_opt[3:6, :].T,
            'mass': X_opt[6, :],
            'thrust': np.concatenate([T_opt, [T_opt[-1]]]),  # 补全长度
            'dt': self.dt,
            'N': self.N
        }
        return solution

# ==================== 可视化工具 ====================
class TrajectoryVisualizer:
    @staticmethod
    def plot_trajectory(solution, target_height, target_vel):
        """绘制3D轨迹和状态time历程"""
        fig = plt.figure(figsize=(16, 10))
        
        # 3D轨迹
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        pos = solution['position']
        ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'b-', linewidth=2, label='Trajectory')
        ax1.scatter(pos[0, 0], pos[0, 1], pos[0, 2], 'go', s=100, label='start')
        ax1.scatter(pos[-1, 0], pos[-1, 1], pos[-1, 2], 'ro', s=100, label='target')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('altitude (m)')
        ax1.set_title('3D Trajectory')
        ax1.legend()
        ax1.grid(True)
        
        # altitude-time曲线
        ax2 = fig.add_subplot(2, 3, 2)
        ax2.plot(solution['time'], solution['position'][:, 2], 'b-', linewidth=2)
        ax2.axhline(y=target_height, color='r', linestyle='--', label=f'target altitude: {target_height}m')
        ax2.set_xlabel('time (s)')
        ax2.set_ylabel('altitude (m)')
        ax2.set_title('altitude vs time')
        ax2.legend()
        ax2.grid(True)
        
        # velocity分量
        ax3 = fig.add_subplot(2, 3, 3)
        vel = solution['velocity']
        ax3.plot(solution['time'], vel[:, 0], 'r-', label='Vx')
        ax3.plot(solution['time'], vel[:, 1], 'g-', label='Vy')
        ax3.plot(solution['time'], vel[:, 2], 'b-', label='Vz')
        ax3.axhline(y=-target_vel, color='k', linestyle='--', label=f'targetVz: {-target_vel}m/s')
        ax3.set_xlabel('time (s)')
        ax3.set_ylabel('velocity (m/s)')
        ax3.set_title('velocity vs time')
        ax3.legend()
        ax3.grid(True)
        
        # thrust曲线
        ax4 = fig.add_subplot(2, 3, 4)
        ax4.plot(solution['time'][:-1], solution['thrust'][:-1], 'm-', linewidth=2)
        ax4.set_xlabel('time (s)')
        ax4.set_ylabel('thrust (N)')
        ax4.set_title('thrust vs time')
        ax4.grid(True)
        
        # mass变化
        ax5 = fig.add_subplot(2, 3, 5)
        ax5.plot(solution['time'], solution['mass'], 'c-', linewidth=2)
        ax5.set_xlabel('time (s)')
        ax5.set_ylabel('mass (kg)')
        ax5.set_title('mass vs time')
        ax5.grid(True)
        
        # velocity-altitude剖面
        ax6 = fig.add_subplot(2, 3, 6)
        speed = np.linalg.norm(vel, axis=1)
        ax6.plot(solution['position'][:, 2], speed, 'k-', linewidth=2)
        ax6.scatter(target_height, target_vel, c='r', s=200, marker='*', 
                   label=f'target: ({target_height}m, {target_vel}m/s)')
        ax6.set_xlabel('altitude (m)')
        ax6.set_ylabel('velocity (m/s)')
        ax6.set_title('velocity-altitude')
        ax6.legend()
        ax6.grid(True)
        
        plt.tight_layout()
        plt.savefig('rocket_optimization_results.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    @staticmethod
    def print_ignition_info(solution, target_height):
        """打印ignition时机相关信息"""
        pos = solution['position']
        thrust = solution['thrust']
        time = solution['time']
        
        # 找到ignition time（thrust首次大于0）
        ignition_idx = np.where(thrust > 1e3)[0][0] if np.any(thrust > 1e3) else 0
        ignition_time = time[ignition_idx]
        ignition_height = pos[ignition_idx, 2]
        
        # 燃烧time
        burn_time = np.sum(thrust > 1e3) * solution['dt']
        
        # fuel consumption
        mass = solution['mass']
        fuel_consumed = mass[0] - mass[-1]
        
        print("\n" + "="*50)
        print("ignition时机优化结果")
        print("="*50)
        print(f"ignition time: {ignition_time:.2f} s")
        print(f"ignitiona ltitude: {ignition_height:.2f} m")
        print(f"time of burn: {burn_time:.2f} s")
        print(f"fuel consumption: {fuel_consumed:.2f} kg")
        print(f"final mass: {mass[-1]:.2f} kg")
        print(f"targetaltitude: {target_height:.2f} m")
        print(f"real altitude: {pos[-1, 2]:.2f} m")
        print(f"altitude error: {abs(pos[-1, 2] - target_height):.4f} m")
        print("="*50 + "\n")

# ==================== 主执行流程 ====================
def main():
    """主函数：演示如何使用火箭ignition时机优化器"""
    
    # 1. 设置火箭参数
    params = RocketParams()
    
    # 2. 定义target状态
    # 例如：在200米altitude时velocity为20 m/s向下
    TARGET_HEIGHT = 200.0  # 米
    TARGET_VELOCITY = 20.0  # m/s，向下
    
    # 3. 创建优化器
    # 调整time步长和步数以确保足够飞行time
    dt = 0.5  # time步长
    total_time = 60.0  # 总模拟time（秒）
    N = int(total_time / dt)
    
    optimizer = RocketTimingOptimizer(params, TARGET_HEIGHT, TARGET_VELOCITY, dt=dt, N=N)
    optimizer.setup_problem()
    
    # 4. 求解
    print(f"开始求解：targetaltitude={TARGET_HEIGHT}m, targetvelocity={TARGET_VELOCITY}m/s")
    solution = optimizer.solve()
    
    if solution is not None:
        # 5. 提取结果
        sol_data = optimizer.extract_solution(solution)
        
        # 6. 可视化
        visualizer = TrajectoryVisualizer()
        visualizer.plot_trajectory(sol_data, TARGET_HEIGHT, TARGET_VELOCITY)
        visualizer.print_ignition_info(sol_data, TARGET_HEIGHT)
        
        return sol_data
    else:
        print("求解失败！")
        return None

if __name__ == "__main__":
    main()