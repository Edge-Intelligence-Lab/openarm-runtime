"""Relative WebXR translation for the platform-owned MuJoCo arm."""
import json
import math

from openarm_runtime.arm import FOLLOWER_KD, FOLLOWER_KP
from openarm_runtime.limits import joint_bounds_deg

FRAME_PERIOD = 0.04
WATCHDOG = 0.25


def controller_sample(text):
    if not 0 < len(text) <= 128:
        raise ValueError('无效的手柄数据长度')
    value = json.loads(text)
    if value is None:
        return None
    if (not isinstance(value, list) or len(value) != 3
            or any(type(axis) not in (int, float) or not -20 <= axis <= 20 for axis in value)):
        raise ValueError('无效的手柄位置')
    return tuple(float(axis) for axis in value)


def cartesian_start(chain, joints):
    from openarm_runtime.cartesian import DifferentialIK

    bounds = tuple(joint_bounds_deg('left').values())
    solver = DifferentialIK(chain, lower=[math.radians(a) for a, _ in bounds],
                            upper=[math.radians(b) for _, b in bounds],
                            velocity=[0.35] * 7, posture=joints)
    origin = chain.forward(joints)

    def step(measured, delta, dt):
        target = origin.copy()
        target[:3, 3] += delta
        return solver.step(measured, target, dt)['joints']

    return step


class PicoTeleop:
    def __init__(self, bus, lease, begin):
        self.bus = bus
        self.lease = lease
        self.begin = begin
        self.origin = None
        self.owns_lease = False
        self.enabled = False
        self.blocked = False

    def release(self):
        self.origin = None
        if self.owns_lease:
            try:
                if self.enabled:
                    self.bus.command(1, 'disable')
            finally:
                self.enabled = self.owns_lease = False
                self.lease.release()

    def stop(self):
        self.blocked = True
        self.release()

    def sample(self, position, now):
        if position is None:
            self.release()
            self.blocked = False
            return False
        if self.blocked:
            return False
        if self.origin is None:
            if not self.lease.acquire(blocking=False):
                raise ValueError('仿真机械臂正在被另一个会话占用')
            self.owns_lease = True
            try:
                self.bus.preflight()
                initial = [self.bus.state(mid).position for mid in range(1, 9)]
                self.step = self.begin(initial[:7])
                self.gripper = initial[7]
                self.bus.start_hold(dict(enumerate(initial, 1)), FOLLOWER_KP, FOLLOWER_KD)
                self.enabled = True
                self.origin = self.previous = position
                self.last_command = self.last_sample = now
            except Exception:
                self.release()
                raise
            return True

        if now - self.last_sample >= WATCHDOG or math.dist(position, self.previous) > 0.12:
            self.stop()
            return False
        self.last_sample, self.previous = now, position
        dt = now - self.last_command
        if dt < FRAME_PERIOD:
            return True
        self.last_command = now
        # WebXR: +X right, +Y up, -Z forward. OpenArm: +X forward, +Y left, +Z up.
        x, y, z = (value - origin for value, origin in zip(position, self.origin, strict=True))
        delta = (-z * 0.5, -x * 0.5, y * 0.5)
        scale = max(1., math.hypot(*delta) / 0.15)
        measured = [self.bus.state(mid).position for mid in range(1, 8)]
        joints = self.step(measured, [value / scale for value in delta], min(dt, 0.05))
        self.bus.exchange(dict(enumerate([*joints, self.gripper], 1)), FOLLOWER_KP,
                          FOLLOWER_KD, self.bus.gravity_feedforward())
        return True
