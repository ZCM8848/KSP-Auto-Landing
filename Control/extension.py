from numpy import ndarray
from .control_utils import *
from .auto_pilot import AutoPilot
from time import time, sleep

class Rocket:

    def __init__(self, space_center, vessel, ref) -> None:
        self.space_center = space_center
        self.vessel = vessel
        self.ref = ref
        self.ap = self.get_ap()
        self.ap_config_updated_at = 0

    def get_ap(self) -> AutoPilot:
        return AutoPilot()

    def reset_ref(self, ref):
        self.ref = ref

    def position(self) -> ndarray:
        return array(self.vessel.position(self.ref))

    def velocity(self) -> ndarray:
        return array(self.vessel.velocity(self.ref))

    def _ap_auto_config(self):
        dt = time() - self.ap_config_updated_at
        if dt < 0.5: return
        v = self.vessel
        tor1 = abs(v.available_reaction_wheel_torque[0])
        tor2 = abs(v.available_rcs_torque[0])
        tor3 = abs(v.available_engine_torque[0])
        tor4 = abs(v.available_control_surface_torque[0])
        tor = tor1 + tor2 + tor3 + tor4
        moi = v.moment_of_inertia
        acc = tor / moi
        acc2 = (acc[1], acc[2], acc[0])
        self.ap.update_max_acc(acc2)
        self.ap_config_updated_at = time()

    def update_ap(self, dir, roll: float=None): # roll in radians
        self.last_dir = dir
        self.last_roll = roll
        self._ap_auto_config()
        x = self.vessel.direction(self.ref)
        y = array(self.space_center.transform_direction((0, 0, 1.), self.vessel.reference_frame, self.ref))
        x0 = array((1., 0, 0))
        rot_x = normalize(cross(x, x0))
        rot_x_ang = angle_between(x, x0)
        y0 = rotate(rot_x, y, rot_x_ang)
        ang1 = angle_between(y0, (0, 1, 0))
        ang2 = angle_between(y0, (0, 0, 1))
        cur_roll = ang1
        if ang2 > pi / 2: cur_roll = -cur_roll
        cur = (cur_roll, *x)
        if dir is None:
            dir = x # 如果没有设置目标方向则以当前方向为目标方向，即维持当前方向稳定
        target = (roll, *dir)
        ang_vel = -array(self.vessel.angular_velocity(self.ref))
        ctrl_x, ctrl_y, ctrl_z = self.ap.update(cur, target, ang_vel, rot_flag=-1)
        ctrl = self.vessel.control
        ctrl.roll = ctrl_x
        ctrl.yaw = ctrl_y
        ctrl.pitch = ctrl_z

    def keep_last_ap(self):
        self.update_ap(self.last_dir, self.last_roll)

    def update_ap_and_wait(self, duration, dir, roll=None, dt=0.1):
        t = time()
        while time() - t < duration:
            self.update_ap(dir, roll)
            sleep(dt)

    def release_ap_control(self):
        ctrl = self.vessel.control
        ctrl.roll = 0.
        ctrl.pitch = 0.
        ctrl.yaw = 0.

    def __getattribute__(self, name: str):
        try: attr = super().__getattribute__(name)
        except: attr = getattr(self.vessel, name)
        return attr