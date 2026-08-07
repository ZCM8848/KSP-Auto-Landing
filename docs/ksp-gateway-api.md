# KSP 隔离层网关 API

`src/recovery/ksp/` —— 整个应用中**唯一导入 `krpc`** 的模块。上层（simulation / guidance / orchestration / server）只能经由本层公共 API 读写 KSP，禁止直接接触 kRPC。

## 设计约定

- **每船一条独立 kRPC 连接**，各配一个 telemetry 线程；连接彼此独立，RPC 延迟天然并行。
- **读**：kRPC stream 由服务器持续推送，telemetry 线程按 `telemetry_hz` 节拍轮询并**原子地**整体拷贝为冻结快照（`FlightState`）。控制循环读快照 = 纯内存读，µs 级，**零网络往返**。
- **写**：`VesselControls` 网关。姿态指令走服务器端 `AutoPilot`（闭环，无需逐 tick 发杆量）；`throttle` 等每次 setter 是一次 RPC。
- **功能无损**：`VesselHandle.raw` / `VesselControls.raw` 逃逸舱门可直通底层 kRPC 对象。
- 连接生命周期由 `ConnectionManager` 统一管理，注册 `atexit` 兜底关闭。

## 快速开始

```python
from recovery import ConnectionManager

with ConnectionManager(address="127.0.0.1") as km:
    booster = km.add_booster("booster-01", "Booster 1", control_hz=50.0, telemetry_hz=20.0)
    km.register_target("booster-01", lon=-74.4730, lat=-0.1853)   # 必须在 start 前
    km.start()

    state = km.snapshot("booster-01")        # FlightState | None（首帧前为 None）
    booster.controls.apply(
        target_direction=(0.0, -1.0, 0.0),
        reference_frame=km.frame("booster-01", "target"),
        throttle=0.85,
    )

    km.abort_all()                           # 全船急停：油门归零 + AutoPilot 解除
```

> `abort_all()` / `close()` 必须在 `with` 块**内**调用——块退出后连接已关闭，此时再对已关闭连接发指令会抛 `OSError`（WinError 10038）。`abort_all()` 对已关闭连接会安全跳过。

## 生命周期（三种形态）

```python
# ① 上下文管理器（推荐）
with ConnectionManager() as km:
    ...

# ② 显式生命周期
km = ConnectionManager()
km.add_booster(...)
km.start()
try:
    ...
finally:
    km.close()

# ③ atexit 兜底：即使忘了 close()，进程退出也会断开连接
```

## 类参考

### `recovery.ConnectionManager`

聚合所有助推器连接，管理生命周期与全局操作。

```python
ConnectionManager(
    *,
    address: str = "127.0.0.1",
    rpc_port: int = 50000,
    stream_port: int = 50001,
    telemetry_hz: float = 20.0,     # 全局默认遥测频率
)
```

| 方法 | 签名 | 说明 |
|---|---|---|
| `add_booster` | `(booster_id: str, vessel_name: str, *, control_hz=50.0, telemetry_hz=None) -> VesselHandle` | 注册一艘船；按 `vessel_name` 从 `space_center.vessels` 解析，找不到抛 `ValueError`（含可用船名清单）。重复 id 抛 `ValueError`；`start()` 后调用抛 `RuntimeError`。解析失败自动关闭该连接。 |
| `start` | `() -> None` | 启动所有船的 telemetry 线程。 |
| `close` | `() -> None` | 停止所有 telemetry 并关闭连接。幂等。 |
| `vessel` | `(booster_id) -> VesselHandle` | 取句柄；未知 id 抛 `KeyError`。 |
| `snapshot` | `(booster_id) -> FlightState \| None` | 取最新快照；telemetry 未就绪时返回 `None`。 |
| `snapshot_all` | `() -> dict[str, FlightState \| None]` | 全部船的快照。 |
| `frame` | `(booster_id, name="target") -> Any` | 参考系句柄。`"target"`（需先 `register_target`，否则 `KeyError`）或 `"surface"`。 |
| `register_target` | `(booster_id, *, lon: float, lat: float) -> None` | 为该船建立着陆点参考系（见下"坐标系"）。必须 `start()` 前调用，否则 `RuntimeError`。 |
| `abort_all` | `() -> None` | 对每船执行 `cut_thrust()`。 |
| `enable_debug` | `() -> None` | 打开一条共享调试连接（独立于控制连接）。 |
| `disable_debug` | `() -> None` | 关闭调试连接，KSP 自动清除其全部绘图。幂等。 |

`__enter__` / `__exit__` 支持 `with`。`close()` 会顺带 `disable_debug()`。

### `recovery.VesselHandle`

单船句柄（`km.add_booster` / `km.vessel` 的返回值）。

| 成员 | 类型 | 说明 |
|---|---|---|
| `name` | `property -> str` | 船名（注册时解析的那艘）。 |
| `controls` | `property -> VesselControls` | 控制网关。 |
| `raw` | `property -> Any` | **逃逸舱门**：底层 kRPC `Vessel` 对象，绕过快照隔离与安全检查。 |
| `client` | `property -> Any` | **逃逸舱门**：底层 kRPC `Client` 对象。 |
| `snapshot` | `() -> FlightState \| None` | 同 `km.snapshot`。 |
| `frame` | `(name="target") -> Any` | 同 `km.frame`。 |
| `register_target` | `(*, lon, lat) -> None` | 同 `km.register_target`。 |
| `physics_range` | `property -> float`（可读写） | 物理泡半径（米），`RangeManager` 钩子。 |
| `is_controllable` | `() -> bool` | `loaded and not packed`。快照未就绪返回 `False`。 |
| `start` / `close` | `() -> None` | 委托给内部连接。 |
| `control_hz` | 公开属性 `float` | 编排层 Scheduler 使用的目标控制频率（隔离层不自行跑控制循环）。 |

### `recovery.VesselControls`

单船控制网关。`controls.apply(...)` 为推荐入口。

#### `apply(...)` —— 组合指令

全部为关键字参数，传了哪个就写哪个（`None` 表示不动）。

```python
apply(
    *,
    throttle: float | None = None,            # 0..1，越界自动 clamp
    pitch: float | None = None,               # 原始杆量 -1..1
    yaw: float | None = None,
    roll: float | None = None,
    target_direction: tuple[float, float, float] | None = None,  # 机鼻指向（归一化后下发）
    up: tuple[float, float, float] | None = None,               # 背脊/滚转参考（可选）
    roll_angle: float | None = None,          # 绕机鼻额外滚转（可选）
    reference_frame: Any = None,              # target_direction/up 所在参考系
    sas: bool | None = None,
    rcs: bool | None = None,
    legs: bool | None = None,
    gear: bool | None = None,
) -> None
```

- 传入 `target_direction` 时：设置 `auto_pilot.reference_frame`、归一化 `target_direction`、可选 `up`/`roll_angle`，并 `engage` AutoPilot（姿态在**服务器端闭环**）。
- 未传 `reference_frame` 却传了 `target_direction` → `ValueError`。
- `target_direction` 为零向量 → `ValueError`。
- 注意：AutoPilot `engage` 期间 SAS 被服务器强制关闭；客户端断开时 AutoPilot 自动解除。

#### 属性（可读写，除注明外）

- `throttle`（越界 clamp 到 0..1）、`pitch`、`yaw`、`roll`
- `sas`、`rcs`、`legs`、`gear`、`lights`、`brakes`、`abort`
- `auto_pilot_engaged`（只读）
- `target_smoothing_time`（可读写，float）——方向切换的平滑时长（秒），设为 0.2–0.5 时 AutoPilot 将阶梯式方向指令匀速旋转过渡，避免振荡
- `raw` → 底层 kRPC `Control`（逃逸舱门）
- `auto_pilot` → 底层 kRPC `AutoPilot`（逃逸舱门，可细调 `target_smoothing_time` 等）

#### 方法

| 方法 | 签名 | 说明 |
|---|---|---|
| `engage_auto_pilot` | `() -> None` | `auto_pilot.engaged = True` |
| `disengage_auto_pilot` | `() -> None` | `auto_pilot.engaged = False` |
| `activate_next_stage` | `() -> list[Any]` | 分级；返回被抛弃的船对象列表（可供再注册）。 |
| `set_action_group` | `(group: int, state: bool) -> None` | 动作组。 |
| `get_action_group` | `(group: int) -> bool` | |
| `toggle_action_group` | `(group: int) -> None` | |
| `cut_thrust` | `() -> None` | 油门归零 + AutoPilot 解除（急停原语）。 |

### `recovery.FlightState`

冻结数据类，一次原子的遥测快照。`position` / `velocity` / `rotation` 表达在 `frame` 所指参考系内。

| 字段 | 类型 | 含义 |
|---|---|---|
| `ut` / `met` | `float` | 宇宙时 / 任务经过时间（s） |
| `position` / `velocity` | `Vector3` | 目标帧内的位置 / 速度 |
| `velocity_surface` | `Vector3` | 地表参考系速度（地速） |
| `rotation` | `Quaternion` | 姿态（目标帧内） |
| `angular_velocity` | `Vector3` | 角速度（目标帧内，rad/s） |
| `altitude` / `surface_altitude` | `float` | 海拔（ASL）/ 离地高度（AGL） |
| `mass` / `dry_mass` | `float` | 总质量 / 干重（kg） |
| `thrust` | `float` | 当前推力（N） |
| `available_thrust` | `float` | 活跃引擎可用推力（N） |
| `max_thrust` / `max_vacuum_thrust` | `float` | 最大推力 / 真空最大推力（N） |
| `specific_impulse` | `float` | 活跃引擎组合 Isp（s），直读 `Vessel.specific_impulse` 流式推送 |
| `max_acceleration` | `float` | `max_thrust / mass` |
| `throttle` | `float` | 当前油门（0..1） |
| `situation` | `Situation` | 见下 |
| `loaded` / `packed` | `bool` | 物理泡内 / 在轨（on-rails） |
| `landed` | `bool` | `situation ∈ {LANDED, PRE_LAUNCH, SPLASHED}` 的推导 |
| `atmosphere_density` | `float` | 大气密度（kg/m³） |
| `frame` | `Any` | 该快照所在参考系句柄（不透明） |

GFOLD 求解器所需的运动学 + 质量 + 推力 + Isp 数据均在快照内，无需额外 RPC。

### `recovery.Situation`

`StrEnum`：`PRE_LAUNCH`、`ORBITING`、`SUB_ORBITAL`、`ESCAPING`、`FLYING`、`LANDED`、`SPLASHED`、`DOCKED`、`UNKNOWN`。

`Situation.from_krpc(value)` 将 kRPC 枚举转为本类型，无法识别 → `UNKNOWN`。

### `recovery.Vector3` / `recovery.Quaternion`

`NamedTuple`：`Vector3(x, y, z)`；`Quaternion(x, y, z, w)`。

### `recovery.FramePacer`

实时循环节拍器（编排层 Scheduler 使用）。

```python
FramePacer(hz: float)                 # hz 需 > 0，否则 ValueError
pacer.hz = 30.0                        # 运行中可改
pacer.period                           # 1 / hz
pacer.tick() -> float                  # 阻塞到下一帧边界，返回实际 dt
```

帧边界锚定在固定的原点（构造时 `time.monotonic()`）上，长期平均频率精确等于 `hz`；若某 tick 超时越过多个边界则**跳过**而非累积滞后。

## 坐标系

- **`"surface"`**：kRPC 内建地表参考系（测海拔/地速）。
- **`"target"`**：`register_target(lon, lat)` 时由 `create_target_reference_frame` 建立并缓存：`ReferenceFrame.create_relative` 基于天体固连帧做经度/纬度旋转 + 表面高度平移 + 两次 90° 旋转。**轴语义原样继承旧实现**，不做重新解释，由求解器拥有其含义。
- `register_target` 必须在 `start()` 前调用；之后快照的 `position/velocity/rotation` 即表达在 target 帧内（未注册则退回 surface 帧）。
- 约束：AutoPilot 的 `reference_frame` 不能随受控船旋转；target/surface 帧为天体固连，天然满足。

## 线程模型

```
ConnectionManager
 ├─ booster A: krpc.Client[A] ── telemetry thread[A] ── 原子快照[A]
 ├─ booster B: krpc.Client[B] ── telemetry thread[B] ── 原子快照[B]
 └─ ...（每船一连接一线程）
```

- 控制循环（编排层）只读快照 + 经 `VesselControls` 写指令，**不持有任何连接锁**。
- 读快照线程安全（`threading.Lock` 保护）；`snapshot()` 返回冻结对象，无并发修改风险。
- 首次快照前 `snapshot()` 返回 `None`；编排层应轮询就绪。

## 调试绘图（Debug）

所有绘图走**独立调试连接**（`km.enable_debug()` 打开，所有船共用），与控制连接的 RPC 完全隔离。坐标系在调试连接上等价重建。

可用帧名：`"target"`（需先 `register_target`）、`"body"`（天体固连）、`"vessel"`（船体系，原点在质心）、`"surface"`（地表平动）、`"orbital"`（轨道）。

```python
km.enable_debug()            # 打开共享调试连接

# 参考系三轴（x=红, y=绿, z=蓝）
marker = b.debug.reference_frame(frame_name="target", length=10.0)
marker.clear()                               # 移除三轴

# 方向矢量（从参考系原点出发）
b.debug.direction((0, -1, 0), length=30, color=(1, 0.5, 0), thickness=0.5)

# 任意线段
line = b.debug.line((0, 0, 0), (10, 0, 0), color=(0, 0, 1))
line.visible = False                         # 隐藏
line.color = (1, 1, 1)
line.remove()

# gfld 稠密轨迹：首次建 N-1 条 add_line，之后 update 只改 start/end
b.debug.trajectory(gfld_points, name="predicted", frame_name="target", color=(0, 1, 0))
b.debug.trajectory("predicted")              # 按名取回
b.debug.trajectory("predicted").update(new_points)   # 点数量变化时自动重建
b.debug.trajectory("predicted").clear()      # 移除该轨迹
b.debug.trajectories                          # -> dict[str, DebugTrajectory]
b.debug.clear("predicted")                    # 移除某组
b.debug.clear_all()                           # 移除该船全部绘图

km.disable_debug()           # 关闭调试连接，KSP 自动清空所有绘图
```

### `VesselHandle.debug` → `DebugProxy`

| 方法 | 签名 | 说明 |
|---|---|---|
| `reference_frame` | `(*, frame_name="target", length=10.0) -> DebugMarker` | 画参考系三轴；`.clear()` 移除 |
| `direction` | `(direction, *, frame_name, length=10.0, color, thickness) -> DebugLine` | 方向矢量 |
| `line` | `(start, end, *, frame_name, color, thickness) -> DebugLine` | 任意线段 |
| `trajectory` | `(positions, *, name, frame_name, color, thickness) -> DebugTrajectory` | 创建或更新命名轨迹 |
| `trajectory` | `(name: str) -> DebugTrajectory \| None` | 按名取回 |
| `clear` | `(name: str) -> None` | 移除命名轨迹 |
| `clear_all` | `() -> None` | 移除该船全部绘图 |

`DebugLine`：`color` / `visible` / `thickness` 可读写；`set_points`；`remove()` / `clear()`。
`DebugTrajectory`：`update(positions)`（点数不变时仅改 `start`/`end`，零 `add_line`）；`color` / `visible` / `thickness` 可读写；`clear()`。
`DebugMarker`：`visible` 可读写；`clear()`。

> 要求：`frame_name="target"` 前必须先 `km.register_target(...)`，否则 `RuntimeError`；`b.debug` 需先 `km.enable_debug()`，否则 `RuntimeError`。绘图在调试连接上，`positions` 接受 `list[tuple]` / `list[Vector3]` / `numpy.ndarray`。

## 滚转与迎风面控制

`up` 和 `roll_angle` 联合控制船体绕推力轴的旋转——在 Super Heavy 这类栅格翼回收场景中，**保持迎风面始终迎风**至关重要。

### 核心参数

| 参数 | 含义 | 默认行为 |
|---|---|---|
| `up` | 背脊（roof）应指向的参考方向 | 缺省用帧内建 up |
| `roll_angle` | 绕 nose 的额外滚转角 (°) | 缺省不约束滚转 |
| `target_smoothing_time` | 方向切换的平滑时长（s） | 0（瞬时跳变） |

### 滑翔段：迎风面跟踪

nose 倾斜时，`up` 可以从气流方向动态派生，使背脊始终迎风：

```python
def wind_up(nose, velocity=(0, 0, -1)):
    """Project wind-facing direction onto nose-perpendicular plane."""
    belly = (-velocity[0], -velocity[1], -velocity[2])   # 迎风面
    d = nose
    dot = d[0]*belly[0] + d[1]*belly[1] + d[2]*belly[2]
    px, py, pz = belly[0]-dot*d[0], belly[1]-dot*d[1], belly[2]-dot*d[2]
    n2 = px*px + py*py + pz*pz
    if n2 < 1e-12:
        return (1.0, 0.0, 0.0)                           # 鼻平行于气流，fallback
    inv = n2 ** -0.5
    return (-px*inv, -py*inv, -pz*inv)                    # 背脊 = 迎风面反方向

# 锥面扫描——每帧新方向 + 新 up，迎风面自动旋转
for phi in sweep:
    direction = cone(phi)
    controls.apply(
        target_direction=direction,
        reference_frame=frame,
        up=wind_up(direction),
        roll_angle=0.0,
    )
```

nose 绕锥面一圈 → `wind_up` 每帧给出对应方位角下的迎风 `up` → AutoPilot 维持背脊迎风 → 栅格翼始终以最大面积吃风。

### 着陆段：模式切换

当 nose 趋近竖直（与气流方向夹角 < ~20°）时，`wind_up` 的投影退化（鼻平行于速度 → `n2 → 0`），导致 AutoPilot 产生 180° 急滚。**解决方案**：接近奇异时把 `up` 冻结在一个水平方向，随后 nose 穿过竖直也不翻转。

```
滑翔段                      →  鼻接近竖直(<20°)  →  着陆段
up = wind_up(nose)              up = freeze           up = (1, 0, 0)
roll = 0                        roll = freeze          roll = capture_angle
```

切换时机由 `n2 < threshold` 触发（`threshold ≈ sin²(20°) ≈ 0.12`）。切换只需一次 `apply`：

```python
controls.apply(
    target_direction=(0, 0, 1),
    reference_frame=frame,
    up=(1.0, 0.0, 0.0),          # 切到水平基准
    roll_angle=capture_angle,     # 抓塔卡槽角度
)
```

切换后 `up` 永远垂直于 nose（水平 vs 向上），无奇异——滚转精度在着陆末段完全可保证。

## 异常

| 场景 | 异常 |
|---|---|
| 船名未找到 | `ValueError`（附带可用船名） |
| 重复 `booster_id` | `ValueError` |
| `start()` 后 `add_booster` / `register_target` | `RuntimeError` |
| 未知 `booster_id` | `KeyError` |
| `target_direction` 缺 `reference_frame` / 零向量 | `ValueError` |
| 未知帧名 | `KeyError` |

## 性能备注

- 读：快照 = 内存拷贝，µs 级。服务器推流频率决定新鲜度（`telemetry_hz` 只做采样上限）。
- 写：每次 setter 一次 RPC（本机 sub-ms~1ms）。推荐：姿态用 `target_direction`（服务器端闭环），高频循环只写 `throttle`。
- 3-4 船 × 50Hz × 少量属性 ≈ 数百 RPC/s，localhost kRPC 无压力。
- 所有遥测字段均为 kRPC stream 直读（包括 `specific_impulse`）。
- 避免在控制循环内使用 `raw` 做同步 RPC 读（即 `client.md` 中"循环内反复 `vessel.position()`"的反模式）。

## 测试

```bash
D:\miniconda3\envs\KRPC\python.exe -m pytest        # 单测（fake client，无需 KSP）
D:\miniconda3\envs\KRPC\python.exe -m pytest -m live  # 需 KSP + kRPC 运行；船名取 $env:KSP_VESSEL，默认 "Booster 1"
```
