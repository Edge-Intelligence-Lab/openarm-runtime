"""Offline Cartesian building blocks. No device access or implicit activation.

Transforms are TCP-to-base; positions are metres, rotations are proper matrices.
This module intentionally does not wire an uncalibrated leader into hardware.
"""
import hashlib
import math
from pathlib import Path
import time

from .gravity import vector as _vector


def vector(value, count, name, *, positive=False):
    """Accept flat NumPy inputs without weakening finite/bool validation."""
    import numpy as np
    if isinstance(value, np.ndarray):
        if value.ndim != 1:
            raise ValueError(f"{name} must be a flat vector")
        value = value.tolist()
    return _vector(value, count, name, positive=positive)


def pose(value):
    import numpy as np
    result = np.array(value, dtype=float, copy=True)
    if (result.shape != (4, 4) or not np.isfinite(result).all()
            or not np.allclose(result[3], [0, 0, 0, 1], atol=1e-9)
            or not np.allclose(result[:3, :3].T @ result[:3, :3], np.eye(3), atol=1e-7)
            or not math.isclose(np.linalg.det(result[:3, :3]), 1., abs_tol=1e-7)):
        raise ValueError("Expected finite rigid TCP-to-base transform")
    return result


class RelativePose:
    """Clutched base-frame deltas, with explicit leader-to-follower axis rotation.

    Translation: p_F0 + scale * A * (p_L - p_L0).
    Rotation: A * R_L * R_L0.T * A.T * R_F0.
    Recreating this object at takeover gives exactly the current follower pose.
    """
    def __init__(self, leader, follower, axes, *, scale=1.):
        import numpy as np
        self.leader, self.follower = pose(leader), pose(follower)
        transform = np.eye(4)
        transform[:3, :3] = axes
        self.axes = pose(transform)[:3, :3]
        self.scale = vector([scale], 1, "translation scale", positive=True)[0]

    def target(self, leader):
        current = pose(leader)
        result = self.follower.copy()
        a = self.axes
        result[:3, 3] += self.scale * a @ (current[:3, 3] - self.leader[:3, 3])
        result[:3, :3] = a @ current[:3, :3] @ self.leader[:3, :3].T @ a.T @ self.follower[:3, :3]
        return result


class KinematicChain:
    """Hash-pinned URDF with explicit scalar joint, encoder and TCP mapping.

    `joints` are in input motor order. Every other movable joint must be locked.
    Model coordinates = signs * input radians + offsets. TCP is a fixed transform
    from the named URDF frame, not a guessed gripper centre.
    """
    def __init__(self, urdf, *, sha256, joints, signs, offsets, locked, frame, tcp):
        import numpy as np
        import pinocchio as pin
        source = Path(urdf).read_bytes()
        if hashlib.sha256(source).hexdigest() != sha256:
            raise ValueError("Kinematics URDF hash mismatch")
        names = tuple(joints)
        if not names or len(set(names)) != len(names):
            raise ValueError("Expected unique ordered joint names")
        self.signs = np.array(vector(signs, len(names), "signs"))
        if not np.isin(self.signs, [-1, 1]).all():
            raise ValueError("Signs must be -1 or +1")
        self.offsets = np.array(vector(offsets, len(names), "offsets"))
        full = pin.buildModelFromXML(source.decode())
        if set(names) & set(locked) or set(names) | set(locked) != set(full.names[1:]):
            raise ValueError("Every movable joint must be mapped or explicitly locked")
        q = pin.neutral(full)
        for name in full.names[1:]:
            joint = full.joints[full.getJointId(name)]
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError("Only bounded scalar joints supported")
            if name in locked:
                q[joint.idx_q] = vector([locked[name]], 1, "locked angle")[0]
                if not full.lowerPositionLimit[joint.idx_q] <= q[joint.idx_q] <= full.upperPositionLimit[joint.idx_q]:
                    raise ValueError("Locked angle outside model limits")
        self.model = pin.buildReducedModel(full, [full.getJointId(n) for n in locked], q) if locked else full
        if not self.model.existFrame(frame):
            raise ValueError("Unknown TCP parent frame")
        self.frame = frame
        self.tcp = pin.SE3(pose(tcp))
        self.indices = np.array([self.model.joints[self.model.getJointId(n)].idx_q for n in names])
        self.data = self.model.createData()
        self.metadata = dict(urdf_sha256=sha256, joints=names, signs=list(signs),
                             offsets=list(offsets), locked=dict(locked), frame=frame,
                             tcp=pose(tcp).tolist())

    def configuration(self, angles):
        import numpy as np
        q = np.empty(self.model.nq)
        q[self.indices] = self.signs * vector(angles, len(self.indices), "joint angles") + self.offsets
        if (q < self.model.lowerPositionLimit - 1e-9).any() or (q > self.model.upperPositionLimit + 1e-9).any():
            raise ValueError("Joint angle outside URDF limits")
        return q

    def forward(self, angles):
        import pinocchio as pin
        pin.framesForwardKinematics(self.model, self.data, self.configuration(angles))
        return (self.data.oMf[self.model.getFrameId(self.frame)] * self.tcp).homogeneous.copy()


class DifferentialIK:
    """One bounded local QP step; output is NOT a hardware authorization.

    Caller provides fresh measured angles each cycle, not a free-running model.
    Barrier objects provide an extension seam, not validated obstacle avoidance.
    Any exception/nonfinite/limit violation rejects the step; never random restart.
    """
    def __init__(self, chain, *, lower, upper, velocity, posture, barriers=()):
        import numpy as np
        import pink
        from pink.tasks import FrameTask, PostureTask
        self.chain = chain
        n = len(chain.indices)
        self.lower = np.array(vector(lower, n, "lower limits"))
        self.upper = np.array(vector(upper, n, "upper limits"))
        self.velocity = np.array(vector(velocity, n, "velocity limits", positive=True))
        if (self.lower >= self.upper).any():
            raise ValueError("Invalid joint interval")
        # Copy: solver limits must not change the FK/leader model.
        self.model = chain.model.copy()
        a = chain.signs * self.lower + chain.offsets
        b = chain.signs * self.upper + chain.offsets
        idx = chain.indices
        self.model.lowerPositionLimit[idx] = np.maximum(self.model.lowerPositionLimit[idx], np.minimum(a, b))
        self.model.upperPositionLimit[idx] = np.minimum(self.model.upperPositionLimit[idx], np.maximum(a, b))
        if (self.model.lowerPositionLimit >= self.model.upperPositionLimit).any():
            raise ValueError("Physical and model limits do not intersect")
        self.model.velocityLimit[idx] = self.velocity
        self.configuration = pink.Configuration(self.model, self.model.createData(), chain.configuration(posture))
        self.configuration.check_limits()
        self.task = FrameTask(chain.frame, position_cost=1., orientation_cost=.3, lm_damping=1.)
        self.posture = PostureTask(cost=1e-3)
        self.posture.set_target(self.configuration.q.copy())
        self.barriers = tuple(barriers)

    def step(self, measured, target, dt):
        import numpy as np
        import pinocchio as pin
        import pink
        dt = vector([dt], 1, "dt", positive=True)[0]
        if dt > .05:
            raise ValueError("Stale control timestep; re-clutch instead")
        measured = np.array(vector(measured, len(self.lower), "measured"))
        if (measured < self.lower).any() or (measured > self.upper).any():
            bad = np.flatnonzero((measured < self.lower) | (measured > self.upper))[0]
            raise ValueError(f"IK seed J{bad+1}={measured[bad]:.12g}rad outside physical limits "
                             f"[{self.lower[bad]:.12g}, {self.upper[bad]:.12g}]")
        self.configuration.update(self.chain.configuration(measured))
        desired = pin.SE3(pose(target))
        self.task.set_target(desired * self.chain.tcp.inverse())
        started = time.perf_counter()
        velocity = pink.solve_ik(self.configuration, [self.task, self.posture], dt,
                                 solver="daqp", damping=1e-6, barriers=self.barriers)
        q = pin.integrate(self.model, self.configuration.q, velocity * dt)
        command = (q[self.chain.indices] - self.chain.offsets) * self.chain.signs
        if (not np.isfinite(command).all()
                or (np.abs(command - measured) > self.velocity * dt + 1e-8).any()
                or (q < self.model.lowerPositionLimit - 1e-8).any()
                or (q > self.model.upperPositionLimit + 1e-8).any()):
            raise RuntimeError("IK output failed independent bounds check")
        # A feasible QP may return e.g. -5e-18 at a zero lower bound. Reject
        # substantive violations above; normalize numerical residue *only on
        # solver output* before this becomes the next cycle's reference.
        q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
        command = np.clip((q[self.chain.indices] - self.chain.offsets) * self.chain.signs,
                          self.lower, self.upper)
        actual = pin.SE3(self.chain.forward(command))
        residual = pin.log6(actual.inverse() * desired).vector
        return dict(joints=command.tolist(), position_error_m=float(np.linalg.norm(actual.translation - desired.translation)),
                    orientation_error_rad=float(np.linalg.norm(residual[3:])),
                    solve_s=time.perf_counter() - started,
                    status="local_step_not_reachability_certificate")
