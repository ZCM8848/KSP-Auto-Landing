import krpc
import numpy as np
from internal.utils import create_target_reference_frame
from control import Rocket
from internal import Targets_JNSQ
from tqdm import trange
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# 连接KRPC并获取数据
conn = krpc.connect("sounding")
space_center = conn.space_center
vessel = conn.space_center.active_vessel
trf = create_target_reference_frame(conn, Targets_JNSQ.launchpad)
vessel = Rocket(space_center, vessel, trf)
atd = vessel.vessel.orbit.body.atmosphere_depth

density = []
altitude = []

# 采样大气数据
for alt in trange(0, int(atd), 100, desc='sampling atmosphere'):
    pos = (alt, 0, 0)
    altitude.append(alt)
    density.append(vessel.vessel.orbit.body.atmospheric_density_at_position(pos, trf))

# 转换为numpy数组便于计算
altitude = np.array(altitude)
density = np.array(density)

# ==================== 插值方法 ====================
# 1. 线性插值
rho_linear = interp1d(altitude, density, kind='linear', fill_value='extrapolate')

# 2. 三次样条插值（更平滑，适合制导计算）
rho_cubic = interp1d(altitude, density, kind='cubic', fill_value='extrapolate')

# 生成密集插值点用于绘图（每10米一个点，比采样更密）
alt_fine = np.linspace(0, int(atd), 500)
density_linear = rho_linear(alt_fine)
density_cubic = rho_cubic(alt_fine)

# ==================== 绘图对比 ====================
plt.figure(figsize=(12, 8))

# 主图：半对数坐标
plt.subplot(2, 1, 1)
# 原始采样数据（散点，半透明）
plt.semilogy(altitude, density, 'o', markersize=3, alpha=0.5, color='blue', label='Sampled Data (100m step)')
# 线性插值
plt.semilogy(alt_fine, density_linear, '-', linewidth=1.5, color='red', alpha=0.8, label='Linear Interpolation')
# 三次样条插值
plt.semilogy(alt_fine, density_cubic, '--', linewidth=2, color='green', label='Cubic Spline Interpolation')

plt.xlabel('Altitude (m)')
plt.ylabel('Density (kg/m³) [Log Scale]')
plt.title('Atmospheric Density: Sampling vs Interpolation')
plt.legend(loc='upper right')
plt.grid(True, which='both', linestyle='--', alpha=0.7)
plt.xlim(0, int(atd))

# 子图：插值误差（绝对误差）
plt.subplot(2, 1, 2)
# 在原始采样点上计算插值误差
error_linear = np.abs(rho_linear(altitude) - density) / density * 100  # 百分比误差
error_cubic = np.abs(rho_cubic(altitude) - density) / density * 100

plt.plot(altitude, error_linear, '-', linewidth=1, color='red', alpha=0.7, label='Linear Error')
plt.plot(altitude, error_cubic, '-', linewidth=1, color='green', alpha=0.7, label='Cubic Error')
plt.xlabel('Altitude (m)')
plt.ylabel('Relative Error (%)')
plt.title('Interpolation Error at Sample Points')
plt.legend()
plt.grid(True, linestyle='--', alpha=0.5)
plt.yscale('log')  # 误差也可能跨数量级
plt.xlim(0, int(atd))

plt.tight_layout()
plt.show()

# 保存插值函数供后续使用
print("\n插值函数已创建：")
print("  rho_linear(altitude) - 线性插值（计算快，适合实时控制）")
print("  rho_cubic(altitude)  - 三次样条（更平滑，适合轨迹优化）")
print(f"\n大气层深度: {atd:.0f} m")
print(f"采样点数: {len(altitude)}")
print(f"密度范围: {density.min():.2e} ~ {density.max():.2f} kg/m³")