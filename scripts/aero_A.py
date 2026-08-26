"""朱雀三号一子级 aero 段引导 (boosterback 结束 → landing burn 触发)。

仅实现 aero 段, 三个阶段:
  1. 前置采样: body_spec + drag_spec + lift_model(cl_area 拟合), 构建预测器
  2. coast 等待: 保持油门0, 直到 进入大气(alt<atmosphere_depth) 且 速度向下(vz<0)
  3. aero 主循环: 引擎朝下的横向攻角反馈消 miss + 预测触发判据, 触发则退出

全程熄火(throttle=0)。带 CSV 日志落盘。
用 KRPC 环境运行: PYTHONPATH=src .../envs/KRPC/python.exe aero_A.py
"""
import csv
import os
import sys
import time

import numpy as np

sys.path.insert(0, r"D:\projects\KSP-Auto-Landing\src")

from recovery import ConnectionManager, FramePacer
from recovery.data.targets import LAUNCHPAD_JNSQ
from recovery.guidance import (
    ConstantThrottle,
    ControlSegment,
    ControlledPredictor,
    LiftDragModel,
    RetrogradeNose,
    VirtualControl,
)
from recovery.ksp.sampling import sample_lift_model

# ---- 参数 (放前面方便调) ----
# 目标飞船; 可用环境变量 AERO_VESSEL 覆盖 (如切到 Booster 2 存档)
VESSEL = os.environ.get("AERO_VESSEL", "Booster 2")
TARGET_THROTTLE = 0.95       # landing burn 触发判据的预测油门
ALPHA_MAX_DEG = 25.0         # 攻角上限 (引擎朝下约束)
SIGN = -1.0                   # 横向修正方向: +1=鼻偏"远离目标"(升力对侧推回), -1=翻转
REF_ALT = 40000.0            # 升力采样的参考高度 (m)
REF_SPEED = 500.0            # 升力采样的参考速度 (m/s)
LAT_MAX_MISS = 5000.0        # miss 达到此值 -> α_cmd = α_max (线性比例)
PRED_DT = 0.05               # 预测器定步长
PRED_TMAX = 300.0            # 预测器最大时长
LOG_PATH = "aero_A.csv"
G0 = 9.80665

# dry-run: AERO_DRY=1 时不施加任何控制, 只验证采样/预测/判据, 跑 AERO_DRY_N 帧后退出
DRY_RUN = os.environ.get("AERO_DRY", "0") == "1"
DRY_N = int(os.environ.get("AERO_DRY_N", "30"))


def main() -> None:
    with ConnectionManager(address="127.0.0.1") as km:
        b = km.add_booster("aero", VESSEL)
        km.register_target("aero", lon=LAUNCHPAD_JNSQ.lon, lat=LAUNCHPAD_JNSQ.lat)
        km.start()
        frame = km.frame("aero", "target")
        body = b.raw.orbit.body
        flight = b.raw.flight(frame)

        # ---- 前置一次性采样 (主循环零 RPC) ----
        body_spec, drag_spec = b.sample_predictor_specs()
        mass_now = float(b.raw.mass)
        center = np.asarray(body.position(frame), dtype=float)
        up = -center / float(np.linalg.norm(center))
        ref_pos = tuple(center + up * (body.equatorial_radius + REF_ALT))
        ref_vel = tuple(-up * REF_SPEED)
        cd0, cla, kind = sample_lift_model(
            body, flight, frame,
            position=ref_pos, velocity=ref_vel,
            alpha_max_deg=ALPHA_MAX_DEG,
        )
        print(f"[lift] cd0_area={cd0:.3f}  cl_area={cla:.3f}  k_ind={kind:.3f}  (参考 {REF_ALT/1000:.0f}km/{REF_SPEED:.0f}m/s)")

        lift = LiftDragModel.from_drag_spec(
            drag_spec, mass_ref=mass_now,
            cl_area=cla, k_ind=kind,
            clamp_aoa=np.radians(ALPHA_MAX_DEG),
        )
        predictor = ControlledPredictor.from_body_spec(
            body_spec, aero=lift, dt=PRED_DT, t_max=PRED_TMAX,
        )

        # ---- CSV 日志 ----
        logf = open(LOG_PATH, "w", newline="", encoding="utf-8")
        w = csv.writer(logf)
        w.writerow([
            "ut", "met", "alt", "surf_alt", "vx", "vy", "vz", "spd",
            "rho", "miss", "land_miss", "alpha_cmd", "nose_x", "nose_y", "nose_z",
            "trig_h_vz0", "triggered",
        ])
        logf.flush()
        t_last_flush = time.monotonic()

        def _flush():
            nonlocal t_last_flush
            now = time.monotonic()
            if now - t_last_flush >= 0.5:
                logf.flush()
                t_last_flush = now

        # ---- phase 1: coast 等待 (入大气 且 速度向下) ----
        print("[phase] coast-wait: 等 入大气(alt<atmo) 且 速度向下(vz<0)")
        wait_start = time.monotonic()
        while True:
            time.sleep(0.05)
            s = b.snapshot()
            if s is None:
                continue
            v = np.asarray(s.velocity)
            if s.altitude < s.atmosphere_depth and v[2] < 0.0:
                print(f"[phase] aero 开始  alt={s.altitude:.0f}  vz={v[2]:.1f}")
                break
            if DRY_RUN and time.monotonic() - wait_start > 10.0:
                print("[dry] coast-wait 超时, 强制进入 aero 主循环验证逻辑")
                break

        # ---- phase 2: aero 主循环 ----
        # aero 段全程熄火: 进入时设一次 throttle 0; 主循环锁 20Hz (匹配 telemetry)
        if not DRY_RUN:
            b.controls.apply(throttle=0.0)
        pacer = FramePacer(hz=20)
        n_loop = 0
        last_miss = None
        last_print = 0.0
        while True:
            pacer.tick()
            s = b.snapshot()
            if s is None:
                continue
            n_loop += 1
            v = np.asarray(s.velocity, dtype=float)
            r = np.asarray(s.position, dtype=float)
            spd = float(np.linalg.norm(v))
            vhat = v / spd if spd > 1e-6 else np.array([0.0, 0.0, 1.0])
            miss = float(np.hypot(r[0], r[1]))
            d_miss = (miss - last_miss) if last_miss is not None else float("nan")
            last_miss = miss

            # 落地/低空优雅退出
            if s.landed or s.altitude < 5.0:
                print(f"[aero] 检测到落地/低空, 优雅退出. alt={s.altitude:.0f} miss={miss:.0f}")
                break

            # 触发判据: 预测 Retrograde + TARGET_THROTTLE 点火, 找垂直速度过零处高度
            trig_ctrl = VirtualControl(
                segments=(ControlSegment(
                    throttle=ConstantThrottle(TARGET_THROTTLE),
                    max_thrust=float(s.max_thrust),
                    isp=float(s.specific_impulse),
                    nose=RetrogradeNose(),
                ),),
                dry_mass=float(s.dry_mass), g0=G0,
            )
            tr = predictor.predict(position=r, velocity=v, mass=s.mass, control=trig_ctrl)
            vz = tr.velocities[:, 2]
            idx = int(np.argmax(vz >= 0.0)) if np.any(vz >= 0.0) else -1
            trig_h = float(tr.positions[idx, 2]) if idx > 0 else float("nan")
            # 触发判据: 垂直速度过零处纵轴<=0(点火仍撞地) 或 减速不足必撞地(idx==-1且下落)
            triggered = (idx > 0 and trig_h <= 0.0) or (idx == -1 and vz[0] < 0.0)

            # 横向: 目标 = "点火后落点"水平偏差(垂直速度过零处位置), 命令攻角推其归零
            #  引擎朝下(Retrograde)基础姿态; cl_area<0 -> 升力落鼻偏对侧
            #  无过零落点(idx<=0)时退化为当前位置方向
            alpha_cmd = 0.0
            nose_dir = -vhat  # 默认逆速度 (引擎朝下)
            away = np.array([0.0, 0.0, 0.0])
            if idx > 0:
                land_xy = tr.positions[idx, 0:2]
                land_miss = float(np.hypot(land_xy[0], land_xy[1]))
                target = land_xy
            else:
                land_miss = float(np.hypot(r[0], r[1]))
                target = np.array([r[0], r[1]])
            t_n = float(np.hypot(target[0], target[1]))
            if t_n > 1.0 and spd > 1e-3:
                away = SIGN * np.array([target[0], target[1], 0.0]) / t_n
                alpha_cmd = float(np.clip(
                    ALPHA_MAX_DEG * (land_miss / LAT_MAX_MISS), 0.0, ALPHA_MAX_DEG
                ))
                a_rad = np.radians(alpha_cmd)
                nose_dir = -vhat * float(np.cos(a_rad)) + away * float(np.sin(a_rad))
                nose_dir /= float(np.linalg.norm(nose_dir))

            b.controls.apply(
                target_direction=tuple(float(x) for x in nose_dir),
                reference_frame=frame,
            ) if not DRY_RUN else None

            # 调试输出 (0.5s 一行): miss 变化率 + 水平方位角诊断横向方向
            now = time.monotonic()
            if now - last_print >= 0.5:
                az_v = np.degrees(np.arctan2(v[1], v[0]))
                az_away = np.degrees(np.arctan2(away[1], away[0]))
                az_nose = np.degrees(np.arctan2(nose_dir[1], nose_dir[0]))
                print(
                    f"[aero] met={s.met:6.1f} alt={s.altitude:7.0f} |v|={spd:6.0f} "
                    f"vz={v[2]:7.0f} miss={miss:8.0f} d_miss={d_miss:+8.0f} "
                    f"land_miss={land_miss:8.0f} alpha={alpha_cmd:5.1f} "
                    f"az(v={az_v:6.1f} away={az_away:6.1f} "
                    f"nose={az_nose:6.1f}) trig_h={trig_h:7.0f} trig={int(triggered)}"
                )
                last_print = now

            # 日志
            w.writerow([
                f"{s.ut:.2f}", f"{s.met:.2f}", f"{s.altitude:.1f}", f"{s.surface_altitude:.1f}",
                f"{v[0]:.1f}", f"{v[1]:.1f}", f"{v[2]:.1f}", f"{spd:.1f}",
                f"{s.atmosphere_density:.5f}", f"{miss:.1f}", f"{land_miss:.1f}", f"{alpha_cmd:.2f}",
                f"{nose_dir[0]:.4f}", f"{nose_dir[1]:.4f}", f"{nose_dir[2]:.4f}",
                f"{trig_h:.1f}", f"{int(triggered)}",
            ])
            _flush()

            if triggered:
                print(
                    f"[aero] 触发 landing burn!  trig_h={trig_h:.1f}  "
                    f"alt={s.altitude:.0f}  miss={miss:.1f}"
                )
                break

            if DRY_RUN and n_loop >= DRY_N:
                print(f"[dry] 已完成 {n_loop} 帧验证, 退出 (未施加控制)")
                break

        b.controls.cut_thrust()
        logf.flush()
        logf.close()
        print(f"aero 段结束, 油门归零, 日志 -> {LOG_PATH}")


if __name__ == "__main__":
    main()
