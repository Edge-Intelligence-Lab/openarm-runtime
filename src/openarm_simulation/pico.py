"""Relative WebXR pose tracking for the platform-owned MuJoCo arm."""
import json
import math

from openarm_runtime.arm import FOLLOWER_KD, FOLLOWER_KP
from openarm_runtime.limits import joint_bounds_deg

FRAME_PERIOD = 0.025
WATCHDOG = 0.25
MAX_TRANSLATION = 0.15
MAX_ROTATION = math.radians(60)
JOINT_SPEED = 1.0  # rad/s, only for PICO's simulated arm
BASE_AXES = ((0., -1., 0.), (0., 0., 1.), (-1., 0., 0.))
START_SESSION = object()
RESET_SCENE = object()


def overview_axes(position, target):
    """Map WebXR right/up/back to the overview camera's horizontal frame."""
    dx, dy = (target[i] - position[i] for i in range(2))
    length = math.hypot(dx, dy)
    if not length:
        raise ValueError('Overview camera needs a horizontal viewing direction')
    forward_x, forward_y = dx / length, dy / length
    return ((forward_y, -forward_x, 0.), (0., 0., 1.), (-forward_x, -forward_y, 0.))


def controller_sample(text):
    if not 0 < len(text) <= 600:
        raise ValueError('无效的手柄数据长度')
    value = json.loads(text)
    if isinstance(value, dict) and value.keys() == {'start'} and value['start'] is True:
        return START_SESSION
    if isinstance(value, dict) and value.keys() == {'reset'} and value['reset'] is True:
        return RESET_SCENE
    if isinstance(value, dict) and value.keys() == {'scene'}:
        from .practice_scenes import SCENES

        scene = value['scene']
        if isinstance(scene, str) and scene in SCENES:
            return value
        raise ValueError('无效的练习场景')
    if isinstance(value, dict) and value.keys() == {'left', 'right'}:
        return {side: decode_pose(pose) for side, pose in value.items()}
    if len(text) > 256:
        raise ValueError('无效的手柄数据长度')
    return decode_pose(value)


def decode_pose(value):
    if value is None:
        return None
    if (not isinstance(value, list) or len(value) not in (3, 7, 8)
            or any(type(axis) not in (int, float) or not -20 <= axis <= 20 for axis in value[:3])):
        raise ValueError('无效的手柄位姿')
    quaternion = value[3:7]
    if quaternion and (any(type(axis) not in (int, float) or not -1.01 <= axis <= 1.01
                           for axis in quaternion)
                       or not math.isclose(math.hypot(*quaternion), 1., abs_tol=.01)):
        raise ValueError('无效的手柄朝向')
    if len(value) == 8:
        if type(value[7]) is not bool:
            raise ValueError('无效的夹爪状态')
        return (*map(float, value[:7]), value[7])
    return tuple(float(axis) for axis in value)


def cartesian_start(chain, joints, side='left'):
    import numpy as np
    from scipy.spatial.transform import Rotation
    from openarm_runtime.cartesian import DifferentialIK

    physical = np.deg2rad(list(joint_bounds_deg(side).values()))
    # MuJoCo joint stops are compliant. Project tiny simulated stop penetration
    # to the common model/solver interval without loosening any command limit.
    model_bounds = np.stack((chain.model.lowerPositionLimit[chain.indices],
                             chain.model.upperPositionLimit[chain.indices]), axis=1)
    model_bounds = np.sort((model_bounds - chain.offsets[:, None]) * chain.signs[:, None], axis=1)
    lower = np.maximum(physical[:, 0], model_bounds[:, 0])
    upper = np.minimum(physical[:, 1], model_bounds[:, 1])
    tolerance = math.radians(.75)

    def seed(angles):
        measured = np.asarray(angles, dtype=float)
        invalid = ~np.isfinite(measured) | (measured < lower - tolerance) | (measured > upper + tolerance)
        if invalid.any():
            index = np.flatnonzero(invalid)[0]
            raise ValueError(f'仿真关节 J{index + 1} 明显越限: {measured[index]:.6f} rad')
        return np.clip(measured, lower, upper)

    posture = seed(joints)
    solver = DifferentialIK(chain, lower=lower, upper=upper,
                            velocity=[JOINT_SPEED] * 7, posture=posture,
                            command_margin_rad=math.radians(2))
    origin = chain.forward(posture)

    def step(measured, delta, rotation, dt):
        target = origin.copy()
        target[:3, 3] += delta
        target[:3, :3] = Rotation.from_rotvec(rotation).as_matrix() @ origin[:3, :3]
        return solver.step(seed(measured), target, dt)['joints']

    return step


class PicoTeleop:
    def __init__(self, bus, lease, begin, *, axes=BASE_AXES, open_fingers=None,
                 travel=MAX_TRANSLATION, rotation_limit=MAX_ROTATION):
        self.bus = bus
        self.lease = lease
        self.begin = begin
        self.axes = axes
        self.open_fingers = open_fingers
        self.travel = travel
        self.rotation_limit = rotation_limit
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

    def to_robot_frame(self, vector):
        return tuple(sum(component * axis for component, axis in zip(vector, row, strict=True))
                     for row in zip(*self.axes, strict=True))

    def reanchor(self, pose):
        if self.origin is not None and pose is not None:
            measured = self.bus.position_snapshot()[:7]
            self.step = self.begin(measured)
            self.origin = pose

    def sample(self, pose, now, *, target_pose=None):
        if pose is None:
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
                initial = self.bus.position_snapshot()
                self.step = self.begin(initial[:7])
                self.gripper = initial[7]
                self.bus.start_hold(dict(enumerate(initial, 1)), FOLLOWER_KP, FOLLOWER_KD)
                self.enabled = True
                self.origin = self.previous = pose
                self.last_command = self.last_sample = now
            except Exception:
                self.release()
                raise
            return True

        if (now - self.last_sample >= WATCHDOG or len(pose) != len(self.origin)
                or math.dist(pose[:3], self.previous[:3]) > 0.12):
            self.stop()
            return False
        rotation = (0., 0., 0.)
        target_pose = pose if target_pose is None else target_pose
        if len(pose) >= 7:
            from scipy.spatial.transform import Rotation

            current = Rotation.from_quat(pose[3:7])
            previous = Rotation.from_quat(self.previous[3:7])
            if (current * previous.inv()).magnitude() > MAX_ROTATION:
                self.stop()
                return False
            relative = Rotation.from_quat(target_pose[3:7]) * Rotation.from_quat(self.origin[3:7]).inv()
            ratio = max(1., relative.magnitude() / self.rotation_limit)
            rotation = self.to_robot_frame(relative.as_rotvec() / ratio)
        self.last_sample, self.previous = now, pose
        dt = now - self.last_command
        if dt < FRAME_PERIOD:
            return True
        self.last_command = now
        displacement = tuple(value - origin for value, origin in zip(target_pose[:3], self.origin[:3], strict=True))
        delta = self.to_robot_frame(displacement)
        scale = max(1., math.hypot(*delta) / self.travel)
        measured = self.bus.position_snapshot()[:7]
        joints = self.step(measured, [value / scale for value in delta], rotation, min(dt, 0.05))
        if len(pose) == 8 and self.open_fingers is not None:
            self.gripper = 0. if pose[7] else self.open_fingers
        self.bus.exchange(dict(enumerate([*joints, self.gripper], 1)), FOLLOWER_KP,
                          FOLLOWER_KD, self.bus.gravity_feedforward())
        return True


class PicoHands:
    def __init__(self, arms, *, shared_contact=None):
        self.arms = arms
        self.shared_contact = shared_contact
        self.paired_origins = None

    def sample(self, poses, now):
        if not isinstance(poses, dict):
            side = 'right' if 'right' in self.arms else 'left'
            poses = {side: poses}
        targets = self.paired_targets(poses)
        results = [arm.sample(poses.get(side), now, target_pose=targets.get(side))
                   for side, arm in self.arms.items()]
        return any(results)

    def paired_targets(self, poses):
        if self.shared_contact is None:
            return {}
        holding = all((pose := poses.get(side)) and len(pose) == 8 and pose[7] and arm.enabled
                      for side, arm in self.arms.items())
        if self.paired_origins is None and holding and self.shared_contact():
            self.paired_origins = dict(poses)
            for side, arm in self.arms.items():
                arm.reanchor(poses[side])
        elif self.paired_origins is not None and not holding:
            self.paired_origins = None
            for side, arm in self.arms.items():
                arm.reanchor(poses.get(side))
        if self.paired_origins is None:
            return {}
        movement = tuple(sum(poses[side][axis] - origin[axis]
                             for side, origin in self.paired_origins.items()) / len(self.arms)
                         for axis in range(3))
        return {side: (*(origin[axis] + movement[axis] for axis in range(3)), *origin[3:])
                for side, origin in self.paired_origins.items()}

    def stop(self):
        self.paired_origins = None
        for arm in self.arms.values():
            arm.stop()

    @property
    def blocked_sides(self):
        return [side for side, arm in self.arms.items() if arm.blocked]

    def release(self):
        self.paired_origins = None
        for arm in self.arms.values():
            arm.release()
