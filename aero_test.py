import krpc
import math
import csv
import time

def vector_dot(a, b):
    """计算向量点积"""
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def vector_magnitude(v):
    """计算向量模长"""
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

def angle_between_vectors(v1, v2):
    """计算两个向量之间的夹角（度）"""
    mag1 = vector_magnitude(v1)
    mag2 = vector_magnitude(v2)
    
    if mag1 == 0 or mag2 == 0:
        return 0.0
    
    cos_angle = vector_dot(v1, v2) / (mag1 * mag2)
    cos_angle = max(-1.0, min(1.0, cos_angle))
    angle_rad = math.acos(cos_angle)
    return math.degrees(angle_rad)

def main():
    print("连接到KRPC服务器...")
    conn = None
    try:
        conn = krpc.connect(name='Drag_Lift_Recorder')
        vessel = conn.space_center.active_vessel
        flight = vessel.flight(vessel.orbit.body.reference_frame)
        
        print(f"开始监测航天器: {vessel.name}")
        print("将持续记录数据直到地面高度 < 300米")
        print("-" * 60)
        
        filename = 'angle.csv'
        with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            # 写入CSV头部
            writer.writerow(['Time', 'Altitude(m)', 'Drag_Angle(deg)', 'Lift_Angle(deg)', 
                           'Drag_X', 'Drag_Y', 'Drag_Z', 
                           'Lift_X', 'Lift_Y', 'Lift_Z',
                           'Vel_X', 'Vel_Y', 'Vel_Z'])
            
            record_count = 0
            start_time = time.time()
            
            while True:
                # 获取当前数据
                altitude = flight.surface_altitude
                drag_vector = flight.drag
                lift_vector = flight.lift
                velocity_vector = flight.velocity
                current_time = time.time() - start_time
                
                # 计算夹角
                drag_angle = angle_between_vectors(drag_vector, velocity_vector)
                lift_angle = angle_between_vectors(lift_vector, velocity_vector)
                
                # 写入CSV
                writer.writerow([
                    f'{current_time:.2f}',
                    f'{altitude:.2f}',
                    f'{drag_angle:.4f}',
                    f'{lift_angle:.4f}',
                    f'{drag_vector[0]:.6f}', f'{drag_vector[1]:.6f}', f'{drag_vector[2]:.6f}',
                    f'{lift_vector[0]:.6f}', f'{lift_vector[1]:.6f}', f'{lift_vector[2]:.6f}',
                    f'{velocity_vector[0]:.6f}', f'{velocity_vector[1]:.6f}', f'{velocity_vector[2]:.6f}'
                ])
                
                record_count += 1
                
                # 打印到控制台（每10行打印一次避免刷屏）
                if record_count % 10 == 0 or altitude < 500:
                    print(f"时间:{current_time:6.1f}s | 高度:{altitude:8.1f}m | "
                          f"Drag角度:{drag_angle:6.2f}° | Lift角度:{lift_angle:6.2f}°")
                
                # 检查终止条件
                if altitude < 300:
                    print("-" * 60)
                    print(f"高度 {altitude:.1f}m < 300m，停止记录")
                    print(f"共记录 {record_count} 条数据")
                    break
                
                # 控制采样频率（10Hz）
                # time.sleep(0.01)
                
    except krpc.ConnectionError:
        print("错误：无法连接到KRPC服务器。请确保KSP已运行且KRPC插件已启动。")
    except KeyboardInterrupt:
        print("\n用户手动停止记录")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        if conn:
            conn.close()
            print(f"数据已保存到 angle.csv")

if __name__ == "__main__":
    main()