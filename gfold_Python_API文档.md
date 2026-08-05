# gfold Python API 文档

> 基于 `samutoljamo/g-fold` 仓库源码（`gfold-py`、`gfold-core`）整理，行为描述经 PyPI 0.3.2 实测验证。
> gfold 是燃料最优动力下降制导（G-FOLD）求解器：将软着陆问题无损凸化为二阶锥规划（SOCP），由 Rust 内核 + Clarabel 内点法求解。

## 安装

```bash
pip install gfold
```

预编译 wheel（abi3 稳定 ABI），Rust 内核与 Clarabel 已静态打包，无系统依赖。冷门平台（ARM 嵌入式、musl）可能退化为源码构建，需 Rust 工具链。

## 模块导出

```python
import gfold

gfold.solve          # 求解函数
gfold.Config         # 顶层配置
gfold.Spacecraft     # 飞行器参数
gfold.Environment    # 环境参数
gfold.Solver         # 求解器/时域参数
gfold.Trajectory     # 求解结果（只读）
```

---

## `gfold.solve(config) -> Trajectory`

求解动力下降制导问题。

- **参数**：`config` — `Config` 对象。
- **返回**：`Trajectory` 对象（见下文）。
- **异常**：求解失败（不可行、数值异常等）抛出 `ValueError`，消息为求解器状态，如 `"solver status: PrimalInfeasible"`。
- **行为**：
  - `config.solver.time_of_flight` 为数值 → 单次固定 TOF 求解（实测 N=100 约 25 ms）。
  - `config.solver.time_of_flight` 为 `None` → 自动搜索燃料最优 TOF（13 点粗扫 + 黄金分割，约 34 次内部求解，实测约 700 ms）。

```python
import gfold

traj = gfold.solve(gfold.Config())   # 默认问题 + 自动 TOF 搜索
print(traj.status, traj.final_mass)
```

---

## `gfold.Config(spacecraft, environment, solver)`

顶层配置。三个参数均可省略（用默认值），但一旦传入必须是完整对象。

| 参数 | 类型 | 默认 |
|---|---|---|
| `spacecraft` | `Spacecraft` | `Spacecraft()` |
| `environment` | `Environment` | `Environment()` |
| `solver` | `Solver` | `Solver()` |

**方法**：

- `to_json() -> str`：序列化为 JSON 字符串。
- `Config.from_json(s: str) -> Config`：从 JSON 反序列化（静态方法）。缺失字段自动填默认值（serde `#[serde(default)]`）。

### ⚠️ 重要陷阱：嵌套字段赋值静默失效

PyO3 的 `get_all/set_all` 对嵌套对象**按值返回拷贝**，以下写法不会生效且不报错：

```python
cfg = gfold.Config()
cfg.solver.time_of_flight = 44.63        # ❌ 静默无效
cfg.spacecraft.initial_position = [1,2,3] # ❌ 静默无效

# ✅ 正确：整体替换嵌套对象
cfg.solver = gfold.Solver(100, 44.63, None, None)
# ✅ 或在构造时注入
cfg = gfold.Config(gfold.Spacecraft(initial_position=[450, -330, 2400]),
                   gfold.Environment(), gfold.Solver(100, 44.63, None, None))
```

---

## `gfold.Spacecraft(...)`

飞行器参数。全部为关键字参数，均可省略。

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `wet_mass` | float | 2000.0 | 初始总质量（kg） |
| `fuel` | float | 1700.0 | 可用燃料（kg）；干重 = wet_mass − fuel |
| `real_max_thrust` | float | 24000.0 | 发动机额定最大推力（N） |
| `min_thrust_pct` | float | 0.2 | 最小节流比（推力下限 = 20% × 额定） |
| `max_thrust_pct` | float | 0.8 | 最大节流比（推力上限 = 80% × 额定） |
| `max_velocity` | float | 1000.0 | 速度模上限（m/s，SOC 约束） |
| `initial_position` | list[float]×3 | [450, −330, 2400] | 初始位置（m） |
| `initial_velocity` | list[float]×3 | [−40, 10, −10] | 初始速度（m/s） |
| `target_position` | list[float]×3 | [0, 0, 0] | 目标位置（m） |
| `target_velocity` | list[float]×3 | [0, 0, 0] | 目标速度（m/s） |
| `fuel_consumption` | float | 5e-4 | 燃料消耗率（kg/(N·s)，即 1/Isp 类系数） |

注：动力学为双积分器 + 对数质量，控制量 `u` 是**推力加速度**（m/s²），推力 = u × m。

## `gfold.Environment(gravity, glide_slope_angle_deg, max_angle_deg)`

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `gravity` | list[float]×3 | [0, 0, −3.71] | 重力加速度向量（m/s²，默认火星）。**技巧：稳态风致加速度估计可叠加进此向量** |
| `glide_slope_angle_deg` | float | 0.0 | 滑翔角（度）。0 = 仅要求高度非负；>0 = 位置需在以 +z 为轴的锥内（SOC 约束） |
| `max_angle_deg` | float | 90.0 | 推力指向角上限（度，推力矢量与 +z 轴夹角）。≥180 时该约束省略 |

## `gfold.Solver(n, time_of_flight, tof_min, tof_max)`

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `n` | int | 100 | 离散节点数；变量总数 = 11n。实测耗时近似线性：n=50≈12 ms，n=100≈25 ms，n=200≈55 ms |
| `time_of_flight` | float \| None | None | 固定飞行时间（s）；`None` = 自动搜索燃料最优 TOF |
| `tof_min` | float \| None | None | TOF 搜索下界；`None` = 自动括号 |
| `tof_max` | float \| None | None | TOF 搜索上界；同上 |

**用法建议**：地面离线用 `None` 搜索一次最优 TOF；机载实时循环中固定 TOF（随时间收缩）单次重解。

---

## `gfold.Trajectory`（求解结果，只读属性）

| 属性 | 类型 | 说明 |
|---|---|---|
| `positions` | ndarray (n, 3) | 各节点位置（m） |
| `velocities` | ndarray (n, 3) | 各节点速度（m/s） |
| `u_values` | ndarray (n, 3) | 各节点推力加速度（m/s²） |
| `thrusts` | ndarray (n,) | 各节点推力模（N，= ‖u‖·e^z） |
| `normalized_thrusts` | ndarray (n,) | 推力 / 额定最大推力（0–1） |
| `z_values` | ndarray (n,) | 对数质量（ln kg）；质量 = e^z |
| `s_values` | ndarray (n,) | 推力模松弛变量（无损凸化引入） |
| `time_points` | ndarray (n,) | 时间节点（s），dt = tof / n |
| `objective` | float | 优化目标值 = z_final（末端对数质量，越大越省燃料） |
| `final_mass` | float | 末端质量（kg）= e^z_final |
| `status` | str | 求解器状态，`"Solved"` 或 `"AlmostSolved"` 视为成功 |

注：Python 侧**未暴露** `time_of_flight` 属性（Rust 结构体有，但绑定层未加 getter）。

---

## 完整示例

```python
import numpy as np
import gfold

# 构造配置（必须构造式注入，不能字段级改嵌套属性）
cfg = gfold.Config(
    gfold.Spacecraft(
        wet_mass=2000.0, fuel=1700.0,
        initial_position=[450.0, -330.0, 2400.0],
        initial_velocity=[-40.0, 10.0, -10.0],
    ),
    gfold.Environment(gravity=[0.0, 0.0, -3.71], max_angle_deg=90.0),
    gfold.Solver(n=100, time_of_flight=44.63, tof_min=None, tof_max=None),
)

try:
    traj = gfold.solve(cfg)
except ValueError as e:
    print("求解失败:", e)          # 不可行时抛异常，需有降级逻辑
else:
    u0 = traj.u_values[0]          # 第一段推力加速度（闭环制导取此执行）
    thrust0 = u0 * 2000.0          # × 当前质量 = 推力指令（N）
    print(f"末端质量 {traj.final_mass:.1f} kg, 着陆点 {traj.positions[-1]}")
```

## 异常与边界行为

| 情况 | 行为 |
|---|---|
| TOF 过短物理不可达 | `ValueError: solver status: PrimalInfeasible` |
| TOF 搜索区间全不可行 | `ValueError: infeasible: no feasible time-of-flight in [...]` |
| 求解器数值边缘收敛 | 正常返回，`status == "AlmostSolved"` |
| `max_angle_deg >= 180` | 指向约束自动省略（被推力松弛锥隐含） |
| `glide_slope_angle_deg == 0` | 退化为高度非负的线性约束（不用 SOC） |

## 性能参考（桌面级 CPU 实测）

| 调用 | 耗时 |
|---|---|
| 固定 TOF 单次求解（n=100） | ≈ 25 ms |
| 固定 TOF（n=50 / n=200） | ≈ 12 / 55 ms |
| 完整 TOF 搜索（time_of_flight=None） | ≈ 700 ms |

实时制导循环请使用固定 TOF + 每周期重解模式；不要在循环内使用 `time_of_flight=None`。

---

*依据源码版本：github.com/samutoljamo/g-fold @ 92be5b6（main），实测包版本：PyPI gfold 0.3.2。*
