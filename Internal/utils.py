import krpc
from tqdm import trange
from numpy import linspace, ndarray, arange
import math
from scipy.interpolate import CubicSpline
from collections import Counter

from control import sin, cos, radians, norm, sqrt, clamp, array, normalize
from .targets import Targets, Targets_JNSQ

_prev_heading_cont = None
# define target reference frame
def create_target_reference_frame(conn:krpc.client.Client, target):
    space_center = conn.space_center
    body = space_center.active_vessel.orbit.body
    body_reference_frame = body.reference_frame
    target_lon = target[0]
    target_lat = target[1]
    # spin around y-axis by -target_lon degrees
    temp_reference_frame = space_center.ReferenceFrame.create_relative(body_reference_frame, rotation=(0., sin(-radians(target_lon / 2)), 0., cos(-radians(target_lon / 2))))
    # spin around z axis by target_lat degrees
    temp_reference_frame = space_center.ReferenceFrame.create_relative(temp_reference_frame, rotation=(0., 0., sin(radians(target_lat / 2)), cos(radians(target_lat / 2))))
    if body.bedrock_height(target_lat, target_lon) < 0:
        target_reference_frame_height = body.equatorial_radius
    else:
        target_reference_frame_height = body.equatorial_radius + body.surface_height(target_lat, target_lon)
    reference_frame = space_center.ReferenceFrame.create_relative(temp_reference_frame, position=(target_reference_frame_height, 0., 0.))
    return reference_frame

def create_solver_reference_frame(conn:krpc.client.Client, target_reference_frame):
    temp_reference_frame = conn.space_center.ReferenceFrame.create_relative(target_reference_frame, rotation=(0., sin(radians(45)), 0., cos(radians(45))))
    temp_reference_frame = conn.space_center.ReferenceFrame.create_relative(temp_reference_frame, rotation=(0., 0., sin(radians(45)), cos(radians(45))))
    return temp_reference_frame


# debug
def draw_reference_frame(conn:krpc.client.Client, reference_frame):
    x_axis = conn.drawing.add_line((10, 0, 0), (0, 0, 0), reference_frame)
    x_axis.color = (1, 0, 0)  # red
    x_axis.thickness = 0.5
    y_axis = conn.drawing.add_line((0, 10, 0), (0, 0, 0), reference_frame)
    y_axis.color = (0, 1, 0)  # green
    y_axis.thickness = 0.5
    z_axis = conn.drawing.add_line((0, 0, 10), (0, 0, 0), reference_frame)
    z_axis.color = (0, 0, 1)  # blue
    z_axis.thickness = 0.5

def draw_direction(conn:krpc.client.Client, direction, reference_frame):
    direction = conn.drawing.add_direction(direction, reference_frame)
    direction.thickness = 0.1

def draw_line(conn:krpc.client.Client, origin, terminal, colour, reference_frame):
    line = conn.drawing.add_line(reference_frame=reference_frame, start=origin, end=terminal)
    line.thickness = 0.1
    line.color = colour
    return line

def draw_trajectory(conn, x, u, reference_frame):
    lines = []
    for i in trange(len(x), desc='drawing trajectory'):
        if i >= 1:
            line_x = draw_line(conn=conn, colour=(255, 255, 255), origin=(x[i - 1, 0], x[i - 1, 1], x[i - 1, 2]),
                      terminal=(x[i, 0], x[i, 1], x[i, 2]),
                      reference_frame=reference_frame)
            line_u = draw_line(conn=conn, colour=(0, 0, 255), origin=(x[i - 1, 0], x[i - 1, 1], x[i - 1, 2]),
                  terminal=(x[i - 1, 0] + u[i - 1, 0], x[i - 1, 1] + u[i - 1, 1], x[i - 1, 2] + u[i - 1, 2]),
                  reference_frame=reference_frame)
            lines.append(line_x)
            lines.append(line_u)
    return lines

def clear_lines(lines):
    for line in lines:
        line.remove()


# other utilities
def get_all_available_vessels(conn:krpc.client.Client):
    return conn.space_center.vessels

def use_JNSQ(conn:krpc.client.Client):
    return conn.space_center.bodies['Kerbin'].atmosphere_depth > 70000

def find_vessel_by_name(conn:krpc.client.Client, name): #This function shouldn't be here, I will move it to another file
    space_center = conn.space_center
    last_matching_vessel = None
    for vessel in space_center.vessels:
        if vessel.name == name:
            last_matching_vessel = vessel
    return last_matching_vessel

def define_targets():
    if use_JNSQ():
        return Targets_JNSQ
    else:
        return Targets

# control utilities
def get_half_rocket_length(rocket):
    vessel_reference_frame = rocket.reference_frame
    part_distance = [norm(part.position(vessel_reference_frame)) for part in rocket.parts.all if part.position(vessel_reference_frame)[1] < 0]
    value_weight_dict = dict(Counter(part_distance))
    total_weight = len(part_distance)
    weighted_sum = sum(value * weight for value, weight in value_weight_dict.items())
    return weighted_sum / total_weight

def landed(rocket):
    legs = rocket.parts.legs
    return all(leg.is_grounded for leg in legs)

def has_legs(rocket):
    return len(rocket.parts.legs) > 0

def ignition_height(rocket, reference_frame, altitude, velocity):
    body = rocket.orbit.body
    g = body.surface_gravity
    current_height = rocket.flight(reference_frame).mean_altitude
    current_velocity = rocket.flight(reference_frame).vertical_speed
    mass = rocket.mass
    available_thrust = rocket.available_thrust
    kinetic_energy_change = 0.5 * mass * (velocity ** 2 - current_velocity ** 2)
    potential_energy_change = mass * g * (altitude - current_height)
    total_energy_change = kinetic_energy_change + potential_energy_change
    ignition_height = total_energy_change / (available_thrust - mass * g)

    return abs(ignition_height)

def descent_throttle(rocket, target_height=0, vt=0):
    body = rocket.orbit.body
    vessel_reference_frame = rocket.reference_frame
    position = rocket.position()[0]
    velocity = rocket.velocity()[0]
    g = body.surface_gravity
    aero_force = rocket.flight(vessel_reference_frame).aerodynamic_force[0]
    mass = rocket.mass
    available_thrust = rocket.available_thrust

    acc = (vt**2 + velocity**2) / (2 * (position - target_height)) + g + aero_force / (mass * g)
    return mass * acc / available_thrust

def impact_point(rocket, reference_frame):
    body = rocket.orbit.body
    g = body.surface_gravity
    position = rocket.position()
    velocity = rocket.velocity()
    terminal_velocity = rocket.flight(reference_frame).terminal_velocity
    estimated_landing_time1 = norm(position) / norm(velocity)
    estimated_landing_time2 = max((velocity[2] - sqrt(velocity[2]**2 + 2 * g * position[2])) / g, (velocity[2] + sqrt(velocity[2]**2 + 2 * g * position[2])) / g)
    ratio = clamp(norm(velocity) / terminal_velocity, 0, 1)
    estimated_landing_time = (ratio) * estimated_landing_time1 + (1 - ratio) * estimated_landing_time2
    estimated_landing_point = position + estimated_landing_time * velocity
    return estimated_landing_point

def roll_controller(heading_deg: float) -> float:
    """
    输入：当前 heading，0–360°
    输出：滚转角，弧度
    解决 heading 在 0/360° 跳变导致的滚转指令跳变
    """
    global _prev_heading_cont

    # 第一次初始化
    if _prev_heading_cont is None:
        _prev_heading_cont = heading_deg

    # ---- 1. 把 heading 展开到连续空间 ----
    # 让本次 heading 与上一帧差值绝对值 < 180°
    d = heading_deg - (_prev_heading_cont % 360)
    if d > 180:
        d -= 360
    elif d < -180:
        d += 360
    heading_cont = _prev_heading_cont + d
    _prev_heading_cont = heading_cont   # 存起来供下一帧用

    # ---- 2. 原公式套在连续角上 ----
    if heading_cont >= 0:
        roll = math.radians(90 + heading_cont)
    else:
        roll = math.radians(90 - heading_cont + 360)

    # 保证落在 [-π, π] 方便后续 PID
    return math.atan2(math.sin(roll), math.cos(roll))

# solver 
def upsample_traj(traj, n=5):
    """
    把一条三维轨迹插值到原来的 n 倍密度（n 为每两个原始点之间新增 n-1 个点）。
    原始采样点全部保留，只在中间插值。

    参数
    ----
    traj : ndarray, shape=(N, 3)
        原始轨迹，可以是位置、速度或加速度。
    n : int
        升采样倍数，n>=1。n=1 时直接返回原轨迹。

    返回
    ----
    traj_dense : ndarray, shape=( (N-1)*n + 1, 3)
        插值后的高密度轨迹。
    """
    if n == 1:
        return traj.copy()

    N = traj.shape[0]
    # 原始“时间戳”就简单地用 0,1,2,...,N-1
    t_old = arange(N)
    # 新的更密时间戳
    t_new = linspace(0, N-1, (N-1)*n + 1)

    # 分别对 x/y/z 做三次样条插值
    cs = CubicSpline(t_old, traj, bc_type='clamped')   #  clamped 保证端点一阶导连续
    traj_dense = cs(t_new)

    return traj_dense

def generate_cubic_with_vertical_end(start_pos, start_vel, target_pos, target_vel, N=100, duration=20.0, blend_ratio=0.1, g=9.81):
    """
    三次样条轨迹，末端平滑过渡到竖直向上（a=0）
    blend_ratio: 末端多少比例段参与过渡（默认 10%）
    """
    t = linspace(0, duration, N)
    g_vec = array([g, 0, 0])

    # 原始三次样条
    spline_x = CubicSpline([0, duration], [start_pos[0], target_pos[0]],
                           bc_type=((1, start_vel[0]), (1, target_vel[0])))
    spline_y = CubicSpline([0, duration], [start_pos[1], target_pos[1]],
                           bc_type=((1, start_vel[1]), (1, target_vel[1])))
    spline_z = CubicSpline([0, duration], [start_pos[2], target_pos[2]],
                           bc_type=((1, start_vel[2]), (1, target_vel[2])))

    x = spline_x(t)
    y = spline_y(t)
    z = spline_z(t)
    vx = spline_x(t, 1)
    vy = spline_y(t, 1)
    vz = spline_z(t, 1)

    # 原始加速度
    ax_raw = spline_x(t, 2)
    ay_raw = spline_y(t, 2)
    az_raw = spline_z(t, 2)

    # 平滑过渡末端加速度 → 0
    k = max(2, int(N * (1 - blend_ratio)))
    alpha = linspace(1, 0, N - k)  # 从1降到0
    ax = ax_raw.copy()
    ay = ay_raw.copy()
    az = az_raw.copy()
    ax[k:] = alpha * ax_raw[k:] + (1 - alpha) * 0
    ay[k:] = alpha * ay_raw[k:] + (1 - alpha) * 0
    az[k:] = alpha * az_raw[k:] + (1 - alpha) * 0

    # 构造输出
    x_full = array([x, y, z, vx, vy, vz])
    g_vec = array(g_vec).reshape(3, 1)
    u_full = array([ax, ay, az]) + g_vec

    return {'x': x_full, 'u': u_full}

def estimate_duration(pos, vel, target_pos, target_vel):
    dist = norm(array(target_pos) - array(pos))
    avg_speed = max(norm(vel), 10)
    return max(dist / avg_speed * 1.2, 5.0)

def find_key_by_shape(data_dict: dict[str, any], target_shape: tuple[int, ...]):
    """
    从字典中查找具有指定shape的ndarray，返回对应的key
    
    Args:
        data_dict: 包含各种值的字典
        target_shape: 目标ndarray的shape，例如 (3, 4) 或 (100,)
    
    Returns:
        str: 找到的第一个匹配的key，如果没有找到则返回None
    """
    for key, value in data_dict.items():
        # 检查值是否为numpy数组
        if isinstance(value, ndarray):
            if value.shape == target_shape:
                return key
    return None