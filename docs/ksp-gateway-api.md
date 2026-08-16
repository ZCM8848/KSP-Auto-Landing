# KSP 隔离层网关 API

`src/recovery/ksp/` —— 整个应用中**唯一导入 `krpc`** 的模块。上层（guidance / control / data 及未来的编排层）只能经由本层公共 API 读写 KSP，禁止直接接触 kRPC。

## 设计约定

- **每船一条独立 kRPC 连接**，各配一个 telemetry 线程；连接彼此独立，RPC 延迟天然并行。
- **读**：kRPC stream 由服务器持续推送，telemetry 线程按 `telemetry_hz` 节拍轮询并**原子地**整体拷贝为冻结快照（`FlightState`）。控制循环读快照 = 纯内存读，µs 级，**零网络往返**。
- **写**：`VesselControls` 网关。姿态有两条路：① 服务器端 `AutoPilot`（传 `target_direction`，闭环，无需逐 tick 发杆量）；② **本地姿态控制**（`LocalAttitudeController.step` 逐 tick 算杆量 → `apply(roll/yaw/pitch)` 写杆量）。`throttle` 等每次 setter 是一次 RPC。
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
| `add_booster` | `(booster_id: str, vessel_name: str, *, control_hz=50.0, telemetry_hz=None) -> VesselHandle` | 注册一艘船；按 `vessel_name` 从 `space_center.vessels` 解析，找不到抛 `VesselNotFound`（含可用船名清单）。重复 id 抛 `DuplicateBooster`；`start()` 后调用抛 `InvalidState`。解析失败自动关闭该连接。 |
| `start` | `() -> None` | 启动所有船的 telemetry 线程。 |
| `close` | `() -> None` | 停止所有 telemetry 并关闭连接。幂等。 |
| `vessel` | `(booster_id) -> VesselHandle` | 取句柄；未知 id 抛 `KeyError`。 |
| `snapshot` | `(booster_id) -> FlightState \| None` | 取最新快照；telemetry 未就绪时返回 `None`。 |
| `snapshot_all` | `() -> dict[str, FlightState \| None]` | 全部船的快照。 |
| `frame` | `(booster_id, name="target") -> Any` | 参考系句柄。`"target"`（需先 `register_target`，否则 `KeyError`）或 `"surface"`。 |
| `register_target` | `(booster_id, *, lon: float, lat: float) -> None` | 为该船建立着陆点参考系（见下"坐标系"）。必须 `start()` 前调用，否则 `InvalidState`。 |
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
| `body_spec` | `property -> BodySpec` | 本船当前天体的行星常数（target 帧内），首次采样后缓存、之后零 RPC；需先 `register_target`。 |
| `sample_predictor_specs` | `(*, mass=None, manual_beta=None, altitude_samples=64) -> (BodySpec, DragSpec)` | 一次性采样：返回缓存 `body_spec` + 新采样的 `DragSpec`；`mass` 默认当前质量。 |
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
| `direction` | `Vector3` | 机鼻（前向）方向，`Vessel.direction(frame)` 流式直读 |
| `bottom_axis` | `Vector3` | 船体 +z（底部）轴在快照帧内，滚转参考（`transform_direction((0,0,1), …)`） |
| `available_reaction_wheel_torque` / `available_rcs_torque` / `available_engine_torque` / `available_control_surface_torque` | `TorquePair` | 各姿态执行器扭矩限值（负/正方向各一个 3 向量，`available_*_torque` 流式直读） |
| `moment_of_inertia` | `Vector3` | 绕质心转动惯量（pitch/roll/yaw 分量） |

GFOLD 求解器所需的运动学 + 质量 + 推力 + Isp 数据均在快照内，无需额外 RPC。

### `recovery.Situation`

`StrEnum`：`PRE_LAUNCH`、`ORBITING`、`SUB_ORBITAL`、`ESCAPING`、`FLYING`、`LANDED`、`SPLASHED`、`DOCKED`、`UNKNOWN`。

`Situation.from_krpc(value)` 将 kRPC 枚举转为本类型，无法识别 → `UNKNOWN`。

### `recovery.Vector3` / `recovery.Quaternion` / `recovery.TorquePair`

`NamedTuple`：`Vector3(x, y, z)`；`Quaternion(x, y, z, w)`；`TorquePair(negative: Vector3, positive: Vector3)`——kRPC `available_*` 属性的负/正方向扭矩对。

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

> 要求：`frame_name="target"` 前必须先 `km.register_target(...)`，否则 `TargetNotRegistered`；`b.debug` 需先 `km.enable_debug()`，否则 `DebugNotEnabled`。绘图在调试连接上，`positions` 接受 `list[tuple]` / `list[Vector3]` / `numpy.ndarray`。

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

## 本地姿态控制（LocalAttitudeController）

`src/recovery/control/local_attitude.py` —— **纯客户端**姿态控制器，与服务器端 `AutoPilot` 二选一。它不接触 kRPC，只吃 `FlightState` 快照 + 目标方向，产出原始杆量，由调用方经 `VesselControls.apply(roll/yaw/pitch)` 写入。控制律**速率无关**（固定 `settling_time`），循环频率完全由调用方（编排层 Scheduler）决定。

### 数据流

```
guidance:  快照 ──► target_dir（指向目标的水平方向）
control:   快照 + target_dir ──► StickCommand(roll, yaw, pitch)   # 纯函数，零 IO
IO:        StickCommand ──► controls.apply(roll/yaw/pitch)        # 写 kRPC 原始杆量
```

### API

```python
from recovery.control import LocalAttitudeController

ctrl = LocalAttitudeController(settling_time=0.5, config_interval=0.5)

s = b.snapshot()
target_dir = (-mx / miss, -my / miss, 0.0)      # 由 guidance 给出
sticks = ctrl.step(s, target_dir)               # StickCommand(roll, yaw, pitch)
b.controls.apply(roll=sticks.roll, yaw=sticks.yaw, pitch=sticks.pitch)

# 滚转策略：roll_target=None（默认）只阻尼滚转角速度；传弧度值则锁到该角度
sticks = ctrl.step(s, target_dir, roll_target=0.0)
```

| 成员 | 签名 | 说明 |
|---|---|---|
| `StickCommand` | `NamedTuple(roll, yaw, pitch)` | 原始杆量，不裁剪（kRPC 在 `Control` 层截到 ±1） |
| `LocalAttitudeController` | `(*, settling_time=0.5, config_interval=0.5)` | 持有 `AutoPilot`；`config_interval` 为 max_acc 重估节流（游戏时间 s） |
| `step` | `(s: FlightState, target_dir, *, roll_target=None) -> StickCommand` | 纯计算：从 `s.direction`/`s.bottom_axis` 推滚转、`s.angular_velocity` 做阻尼、按 `s.ut` 节流重估 max_acc |
| `roll_from_axes` | `(direction, bottom) -> float` | 由鼻向与 +z 轴推滚转角（rad），等价旧 `Rocket.update_ap` 的滚转逻辑 |
| `max_acc_from_snapshot` | `(s: FlightState) -> (roll, yaw, pitch)` | 由扭矩/MoI 估计各轴最大角加速度（rad/s²），等价旧 `_ap_auto_config` |

- 依赖快照字段：`direction` / `bottom_axis` / `angular_velocity` / `available_*_torque` / `moment_of_inertia`（均已入快照，读取零 RPC）。
- 与服务器端 `AutoPilot` 的区别：本控制器逐 tick 输出杆量（速度曲线 + 阻尼，`rot_flag=-1`），`AutoPilot` 则在服务器端做 PID 闭环。
- 完整示例见 `scripts/zem_boosterback_local.py`。

## 异常

所有框架级异常继承自 `recovery.RecoveryError`，同时链入标准异常（`RuntimeError` / `ValueError`），保证向后兼容。

| 场景 | 异常 |
|---|---|
| 船名未找到 | `VesselNotFound`（`ValueError`） |
| 同名冲突（≥2 艘） | `warnings.warn`，不阻断（拾取第一艘） |
| 重复 `booster_id` | `DuplicateBooster`（`ValueError`） |
| `resolve_vessel` 未调用即访问 `vessel`/`controls` | `VesselNotResolved`（`RuntimeError`） |
| `start()` 后 `add_booster` / `register_target` | `InvalidState`（`RuntimeError`） |
| `debug` 未启用 | `DebugNotEnabled`（`RuntimeError`） |
| 未知 `booster_id` | `KeyError` |
| `target_direction` 缺 `reference_frame` / 零向量 | `ValueError` |
| 未知帧名 | `KeyError` |

### 自定义异常类

| 异常 | 基类 | 说明 |
|---|---|---|
| `RecoveryError` | `Exception` | 框架基础异常 |
| `InvalidState` | `RecoveryError`, `RuntimeError` | 生命周期状态非法 |
| `VesselNotResolved` | `RecoveryError`, `RuntimeError` | 船舶未解析 |
| `TargetNotRegistered` | `RecoveryError`, `RuntimeError` | 着陆目标未注册 |
| `DebugNotEnabled` | `RecoveryError`, `RuntimeError` | 调试连接未启用 |
| `VesselNotFound` | `RecoveryError`, `ValueError` | 船名未匹配 |
| `AmbiguousVesselName` | `RecoveryError`, `ValueError` | 同名多艘（异常类，实际以 `warnings.warn` 发出） |
| `DuplicateBooster` | `RecoveryError`, `ValueError` | 助推器 ID 重复 |

## 性能备注

- 读：快照 = 内存拷贝，µs 级。服务器推流频率决定新鲜度（`telemetry_hz` 只做采样上限）。
- 写：每次 setter 一次 RPC（本机 sub-ms~1ms）。推荐：姿态用 `target_direction`（服务器端闭环），高频循环只写 `throttle`。
- 3-4 船 × 50Hz × 少量属性 ≈ 数百 RPC/s，localhost kRPC 无压力。
- 所有遥测字段均为 kRPC stream 直读（包括 `specific_impulse`）。
- 本地姿态控制：每 tick 写 `roll/yaw/pitch` 三杆 = 3 次 RPC（kRPC 无批量杆量接口）；读取全部走快照（零 RPC）。max_acc 重估按游戏时间 0.5s 节流，开销可忽略。
- 避免在控制循环内使用 `raw` 做同步 RPC 读（即 `client.md` 中"循环内反复 `vessel.position()`"的反模式）。

## 制导模块

`src/recovery/guidance/` —— 纯轨迹预测与气动建模层，**完全不接触 kRPC**。所有 KSP I/O 由隔离层的 `recovery.ksp.sampling` 完成，产出纯数据 spec，再交给本层构造模型。

### 分层数据流

```
kRPC 对象 ──→ recovery.ksp.sampling（RPC 采样）──→ recovery.specs（纯数据）
                                                      │
recovery.guidance（纯构造）←──────────────────────────┘
```

### `recovery.specs` —— 纯数据 spec

| spec | 字段 | 说明 |
|---|---|---|
| `BodySpec` | `mu`, `omega`, `body_center`, `body_radius`, `surface_gravity` | 行星常数（target 帧内；`surface_gravity` 为表面重力） |
| `DragSpec` | `ballistic_coefficient`, `density_alts`, `density_vals`, `body_center`, `sea_level_radius` | 大气阻力参数 |

### `recovery.ksp.sampling` —— RPC 采样（唯一做预测 RPC 的模块）

```python
from recovery.ksp.sampling import sample_body_spec, sample_drag_spec

body_spec = sample_body_spec(body, target_frame, lat, lon)
drag_spec = sample_drag_spec(
    body, flight, target_frame,
    mass=vessel.mass,            # 非 FAR 时 β 反算用
    manual_beta=None,            # 强制指定 β，None=自动
    altitude_samples=64,         # 自适应下限 = max(32, depth/500)
)
```

### `recovery.guidance.LandingPredictor`

RK45 数值积分落点预测器，在行星固连（target）参考系中对运动方程积分至地表。

```python
# 纯构造（无 RPC）
predictor = LandingPredictor.from_body_spec(body_spec, aero=drag_model)

# 或直接传标量
predictor = LandingPredictor(
    mu=..., omega=..., body_center=..., body_radius=..., aero=None,
)
```

| 成员 | 签名 | 说明 |
|---|---|---|
| `from_body_spec` | `(spec: BodySpec, aero=None) -> LandingPredictor` | 纯构造，从 `sample_body_spec` 的产物建预测器。 |
| `predict` | `(*, position, velocity, t_max=600, rtol=1e-9, atol=1e-9) -> ImpactResult \| None` | 积分至地表。`t_max` 内未落地返回 `None`。控制循环用 `rtol/atol=1e-6`。 |
| `predict_from` | `(state: FlightState, **kwargs) -> ImpactResult \| None` | 同 `predict`，从快照读取位置/速度。 |

动力学方程：`a = g_μ(r) − 2ω×v − ω×(ω×r) + a_aero(r,v)`。`a_aero` 为可选的气动模型贡献。

### `recovery.guidance.ImpactResult`

```python
@dataclass(frozen=True)
class ImpactResult:
    position: tuple[float, float, float]  # 落点在目标帧坐标 (m)
    time: float                           # 落地剩余时间 (s)
```

### 气动模型（AeroModel 协议）

所有气动模型实现 `acceleration(position, velocity) -> Vec3` 协议：

| 模型 | 来源 | RPC/步？ | 适用场景 |
|---|---|---|---|
| 无（`aero=None`） | — | — | 快速纯弹道预测 |
| `KrpcAeroModel` | `Flight.simulate_aerodynamic_force_at`（攻角 180° / 鼻锥后指） | **是**（每步一发 RPC） | 离线校验、高精度分析 |
| `DragModel` | 一次性采样的密度插值表 + 弹道系数 β | **否** | **高频控制循环（零 RPC/步）** |

### `recovery.guidance.DragModel`

离线阻力模型。由 `sample_drag_spec` 采样的 `DragSpec` 经 `from_spec` 纯构造，之后 `acceleration()` 纯本地计算。

```
a_drag = −½ · ρ(海拔) · |v|² / β · v̂
```

```python
# 方式 ①：手动构造
model = DragModel(
    ballistic_coefficient=5000.0,
    density_fn=lambda h: 1.225 * np.exp(-h / 5600),
    body_center=(0, 0, -600000),
    sea_level_radius=600000,
)

# 方式 ②：从采样 spec 构造（推荐，配合 sample_drag_spec）
model = DragModel.from_spec(drag_spec)
```

| 参数 | 说明 |
|---|---|
| `sample_drag_spec.altitude_samples` | 余弦非均匀分布（低空密、高空疏）。RSS（~140 km 大气）自动 ≥ 128 点。 |
| 弹道系数 β | 三级获取策略：`manual_beta` → FAR `ballistic_coefficient` → `Flight.drag` 反算 |
| 密度模型 | `density_at(altitude)` × N 点 → `np.interp` 线性插值；超出大气深度返回 0 |

### `recovery.guidance.KrpcAeroModel`

每步调用 kRPC 的 `simulate_aerodynamic_force_at`（攻角 180° / 鼻锥反指速度）。适用于离线精度校验。控制循环**不推荐**——单次 `predict` 产生 100–400 次 RPC（总耗时 0.1–2 s）。

```python
aero = KrpcAeroModel(flight=v.flight(target_frame), body=body, mass=mass)
body_spec = sample_body_spec(body, target_frame, lat, lon)
predictor = LandingPredictor.from_body_spec(body_spec, aero=aero)
```

### 典型用法：ZEM 助推回收

```python
with ConnectionManager() as km:
    b = km.add_booster("booster", "SuperHeavy")
    km.register_target("booster", lon=LAUNCHPAD.lon, lat=LAUNCHPAD.lat)
    km.start()

    frame = km.frame("booster", "target")
    body_spec, drag_spec = b.sample_predictor_specs()   # 一次性 RPC；body 取自注册的 target
    predictor = LandingPredictor.from_body_spec(
        body_spec, aero=DragModel.from_spec(drag_spec)
    )

    while True:
        s = b.snapshot()
        r = predictor.predict_from(s, rtol=1e-6, atol=1e-6)
        if r is None:
            continue
        mx, my = r.position[0], r.position[1]
        miss = (mx**2 + my**2) ** 0.5

        # 鼻锥水平指向目标 —— 推力纯水平推回
        if miss < 1.0:
            target_dir = (0.0, 0.0, -1.0)
        else:
            target_dir = (-mx / miss, -my / miss, 0.0)

        b.controls.apply(
            target_direction=target_dir,
            reference_frame=frame,
            throttle=1.0,
        )

        if miss > min_history:
            break  # 局部最小值 —— 助推回收完成
```

完整示例见 `scripts/zem_boosterback.py`。

### `recovery.guidance.gfold` —— G-FOLD 动力下降规划器

纯封装 `gfold` SOCP 求解器，**+z = 天顶**（与 target 帧和 gfold 库一致，重力 `[0, 0, -g]`，推力沿 +z）。`GfoldParams`（frozen dataclass）集中所有可调参数；固定量（质量/燃料/推力/Isp/重力/初始状态）自动从 `FlightState` 快照 + 表面重力 `g0` 推导。

| 成员 | 签名 | 说明 |
|---|---|---|
| `GfoldParams` | frozen dataclass | 可调参数：目标位置/速度、glide-slope 锥、推力指向锥、节流上下限、`max_velocity`、节点数 `n`、TOF（`None`=搜索） |
| `build_config` | `(state, g0, params, *, tof=None, n=None) -> gfold.Config` | 组装求解器配置 |
| `solve` | `(config) -> Trajectory \| None` | 求解；不可行/求解失败返回 `None` |
| `replan` | `(state, g0, params, *, tof) -> Trajectory \| None` | 固定 TOF 单次求解（热循环，~10 ms @ n=50） |
| `solve_optimal` | `(state, g0, params) -> Trajectory \| None` | 燃料最优（内部搜索 TOF，~0.5 s）——点火触发/兜底 |
| `tof_of` | `(traj) -> float` | 反算解出的 TOF |
| `command` | `(traj, *, mass, available_thrust, min_throttle, max_throttle) -> (throttle, nose)` | 轨迹首节点 → `(油门, 鼻锥方向)`（推力加速度方向 = 鼻锥方向；仅在推力朝上的刹车阶段调用） |
| `features_of` | `(state, params) -> list[float]` | 14 维 TOF-Net 特征向量（见下） |

### `recovery.guidance.tofnet` —— TOF-Net TOF 预测器

加载预训练 ONNX 模型，用 `(p_feasible, tf)` 预测**替代 `solve_optimal` 内部的 TOF 搜索**。纯黑盒：14 维原始特征进、`(p_feasible, tf)` 出；归一化已打包进 ONNX 前向。`onnxruntime` 是可选依赖（惰性 import，仅在实例化 `TofPredictor` 时需要）。

```python
from recovery.guidance.gfold import GfoldParams, features_of, replan, solve_optimal
from recovery.guidance.tofnet import TofPredictor

tofnet = TofPredictor()                          # 默认加载包内 assets/tofnet.onnx
p, tf = tofnet.predict(features_of(s, params))   # 微秒级推理
if p >= tofnet.threshold:
    traj = replan(s, g0, params, tof=tf)         # 固定 TOF，跳过搜索
else:
    traj = solve_optimal(s, g0, params)          # fallback：完整 TOF 搜索
```

| 成员 | 说明 |
|---|---|
| `TofPredictor(onnx_path=None, meta_path=None)` | 加载 ONNX + 元数据；默认用 `importlib.resources` 定位 `recovery.guidance/assets/` |
| `predict(features) -> (p_feasible, tf)` | 推理，返回可行概率 + 最优 TOF（秒） |
| `threshold` | 可行性阈值（来自 `.onnx.json` 元数据） |
| `features` | 特征名顺序（契约参考） |

**14 维特征顺序是训练契约**（GFOLD-solver `common/config.py` 的 `FEATURES`）：`x,y,z, vx,vy,vz, dry_mass, fuel, real_max_thrust, min_thrust_pct, max_thrust_pct, fuel_consumption, glide_slope_angle_deg, max_angle_deg`。`features_of` 是唯一产生源；`fuel_consumption` 用标准重力 `G0=9.80665`（Isp 换算），**不是**表面重力 `g0`。

> 注意：该模型是 **Kerbin 专用**（训练时重力 `[0,0,-9.81]` 写死，特征向量不含重力）。换天体需重训或把重力加进特征重训。`onnxruntime` 属可选 extra（`pip install -e .[tofnet]`）。

## 测试

```bash
D:\miniconda3\envs\KRPC\python.exe -m pytest        # 单测（fake client，无需 KSP）
D:\miniconda3\envs\KRPC\python.exe -m pytest -m live  # 需 KSP + kRPC 运行；船名取 $env:KSP_VESSEL，默认 "Booster 1"
```
