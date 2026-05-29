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

# ==================== 多项式拟合部分 ====================
# 建议使用5-7阶多项式，大气密度随高度指数下降，多项式可较好近似
degree = 6  # 可调整阶数，越高阶拟合越精确但可能过拟合
coeffs = np.polyfit(altitude, density, degree)
poly = np.poly1d(coeffs)

# 输出多项式表达式
print(f"\n{'='*50}")
print(f"大气密度-高度多项式拟合 (阶数: {degree})")
print(f"{'='*50}")
print(f"ρ(h) = ", end="")
terms = []
for i, coef in enumerate(coeffs):
    power = degree - i
    if abs(coef) > 1e-15:
        coef_str = f"{coef:.6e}"
        if power == 0:
            terms.append(f"{coef_str}")
        elif power == 1:
            terms.append(f"{coef_str}·h")
        else:
            terms.append(f"{coef_str}·h^{power}")
print(" + ".join(terms))

print(f"\n系数数组 (从高次幂到常数项):")
print(f"coeffs = np.array([{', '.join([f'{c:.10e}' for c in coeffs])}])")

# 计算拟合优度 R²
density_pred = poly(altitude)
ss_res = np.sum((density - density_pred)**2)
ss_tot = np.sum((density - np.mean(density))**2)
r_squared = 1 - (ss_res / ss_tot)
print(f"\n拟合优度 R² = {r_squared:.8f}")

# ==================== 可视化部分 ====================
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))

# 图1: 线性坐标显示
ax1.scatter(altitude/1000, density, s=8, alpha=0.6, c='blue', label='KRPC原始数据')
alt_fit = np.linspace(0, altitude.max(), 500)
density_fit = poly(alt_fit)
ax1.plot(alt_fit/1000, density_fit, 'r-', linewidth=2, label=f'多项式拟合 (n={degree})')
ax1.set_xlabel('高度 (km)')
ax1.set_ylabel('密度 (kg/m³)')
ax1.set_title('大气密度-高度分布 (线性坐标)')
ax1.legend()
ax1.grid(True, alpha=0.3)

# 图2: 对数坐标显示（更适合观察大气层指数衰减特性）
ax2.semilogy(altitude/1000, density, 'b.', alpha=0.6, markersize=4, label='KRPC原始数据')
ax2.semilogy(alt_fit/1000, density_fit, 'r-', linewidth=2, label=f'多项式拟合 (n={degree})')
ax2.set_xlabel('高度 (km)')
ax2.set_ylabel('密度 (kg/m³) [对数坐标]')
ax2.set_title('大气密度-高度分布 (对数坐标)')
ax2.legend()
ax2.grid(True, alpha=0.3, which='both')

# 图3: 残差分析
residuals = density - density_pred
ax3.scatter(altitude/1000, residuals, s=8, alpha=0.6, c='green', label='残差')
ax3.axhline(y=0, color='red', linestyle='--', linewidth=1.5)
ax3.set_xlabel('高度 (km)')
ax3.set_ylabel('残差 (kg/m³)')
ax3.set_title(f'拟合残差分布 (RMSE: {np.sqrt(np.mean(residuals**2)):.2e})')
ax3.legend()
ax3.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('atmosphere_density_fit.png', dpi=150, bbox_inches='tight')
plt.show()

# 保存结果供后续使用
np.savez('atmosphere_fit.npz', 
         altitude=altitude, 
         density=density, 
         coeffs=coeffs,
         degree=degree,
         r_squared=r_squared)
print(f"\n✓ 数据已保存至 atmosphere_fit.npz")
print(f"✓ 图像已保存至 atmosphere_density_fit.png")