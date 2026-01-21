from internal.utils import create_target_reference_frame
import numpy as np
from math import cos, sin, radians
import krpc  # 确保已安装：pip install krpc
from internal.targets import Targets_JNSQ

# 1. 定义旋转矩阵函数（补充numpy规范）
def R_y(theta):
    """绕Y轴旋转θ度的3×3旋转矩阵（右手坐标系）"""
    theta_rad = radians(theta)
    return np.array([
        [cos(theta_rad), 0, sin(theta_rad)],
        [0, 1, 0],
        [-sin(theta_rad), 0, cos(theta_rad)]
    ])

def R_z(theta):
    """绕Z轴旋转θ度的3×3旋转矩阵（右手坐标系）"""
    theta_rad = radians(theta)
    return np.array([
        [cos(theta_rad), -sin(theta_rad), 0],
        [sin(theta_rad), cos(theta_rad), 0],
        [0, 0, 1]
    ])

# 2. 核心变换函数（brf → trf）
def transform_to_target_frame(conn, body_ip, tgt, trf, brf):
    """
    将天体参考系(brf)中的位置body_ip转换到目标参考系(trf)中
    :param conn: krpc客户端连接
    :param body_ip: brf系下的位置向量（列表/元组/np.array）
    :param tgt: 目标经纬度 [lon, lat]
    :param trf: 目标参考系（krpc.ReferenceFrame对象）
    :param brf: 天体参考系（krpc.ReferenceFrame对象）
    :return: trf系下的位置向量（np.array，3维）
    """
    space_center = conn.space_center
    
    # 步骤1：计算旋转矩阵（先绕Y轴转经度，再绕Z轴转-纬度）
    Ry = R_y(tgt[0])    # 绕Y轴旋转目标经度
    Rz = R_z(-tgt[1])   # 绕Z轴旋转-目标纬度
    R_total = np.dot(Rz, Ry)  # 总旋转矩阵（矩阵乘法，顺序不可乱）
    
    # 步骤2：计算目标参考系原点在brf中的位置
    target_origion = np.array(space_center.transform_position((0,0,0), trf, brf))
    
    # 步骤3：计算body_ip相对于target_origion的相对位置（brf系下）
    # 转为列向量（3×1），符合矩阵乘法规范
    relative_pos_brf = np.array(body_ip) - target_origion
    relative_pos_brf_col = relative_pos_brf.reshape(3, 1)
    
    # 步骤4：旋转变换（核心！用旋转矩阵乘法替代叉乘）
    # 将brf系的相对位置 → trf系的位置
    relative_pos_trf_col = np.dot(R_total, relative_pos_brf_col)
    
    # 步骤5：转回1维数组（方便使用）
    targ_ip = relative_pos_trf_col.flatten()
    
    return targ_ip

# 3. 测试示例（需先启动KSP并开启KRPC服务器）
if __name__ == "__main__":
    # 连接KRPC
    conn = krpc.connect(name="TargetFrameTransform")
    space_center = conn.space_center
    body = space_center.active_vessel.orbit.body
    brf = body.reference_frame  # 天体参考系
    tgt = Targets_JNSQ.launchpad
    trf = create_target_reference_frame(conn, tgt)  # 你的目标参考系函数
    
    # 测试位置：brf系下的任意位置（示例）
    body_ip = np.array([body.equatorial_radius + 1000, 0, 0])
    
    # 执行变换
    targ_ip = transform_to_target_frame(conn, body_ip, tgt, trf, brf)
    print("目标参考系(trf)中的位置：", targ_ip)