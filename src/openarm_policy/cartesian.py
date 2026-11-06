"""Feedback-supervised Cartesian execution wrapper around an existing Robot.

Models submit absolute task-frame TCP goals; IK runs locally at every submit,
not once per predicted chunk. Underlying CAN workers still own their watchdogs,
gains, bus leases and disable cleanup. No device is opened by construction.
"""
import math
import time

from .contracts import ActionSpec, SensorSample, StreamSpec, TCP_POSE, finite


class GripperCalibration:
    """Measured monotonic motor-radian / physical-aperture table, either sign."""
    def __init__(self, motor_rad, opening_m):
        import numpy as np
        self.motor = np.asarray(motor_rad, dtype=float)
        self.opening = np.asarray(opening_m, dtype=float)
        if (self.motor.ndim != 1 or len(self.motor) < 2 or self.motor.shape != self.opening.shape
                or not np.isfinite(self.motor).all() or not np.isfinite(self.opening).all()
                or (self.opening < 0).any()):
            raise ValueError("Invalid gripper calibration samples")
        order = np.argsort(self.motor)
        self.motor, self.opening = self.motor[order], self.opening[order]
        change = np.diff(self.opening)
        if not (np.diff(self.motor) > 0).all() or not ((change > 0).all() or (change < 0).all()):
            raise ValueError("Gripper calibration must be strictly monotonic")

    def aperture(self, motor):
        import numpy as np
        if not finite(float(motor)) or not self.motor[0] <= motor <= self.motor[-1]:
            raise ValueError("Gripper feedback outside calibrated motor range")
        return float(np.interp(motor, self.motor, self.opening))

    def angle(self, opening):
        import numpy as np
        if not finite(float(opening)) or not min(self.opening) <= opening <= max(self.opening):
            raise ValueError("Requested aperture outside calibration")
        order = np.argsort(self.opening)
        return float(np.interp(opening, self.opening[order], self.motor[order]))


class CartesianRobot:
    """Single-arm TCP-pose capability; wraps joint-only robots, never relabels them.

    Verified calibration is the default. Explicit operator-only nominal-geometry
    trials keep an uncalibrated label; neither mode certifies collision safety.
    """
    def __init__(self, robot, config, control_hz=20, record=None, *, allow_nominal=False):
        import numpy as np
        from openarm_runtime.cartesian import KinematicChain, pose
        allowed = {"side", "calibration_verified", "chain", "task_from_base", "workspace_m",
                   "gripper", "velocity_rad_s", "gripper_velocity_rad_s", "tracking_limit_rad",
                     "feedback_max_age_s", "calibration_status", "progress"}
        verified = config.get("calibration_verified") is True
        nominal = (config.get("calibration_verified") is False
                   and config.get("calibration_status") == "nominal_model_not_hardware_calibrated")
        if set(config) - allowed or not (verified or (nominal and allow_nominal is True)):
            raise ValueError("Cartesian profile needs verified calibration or explicit nominal-geometry trial opt-in")
        if verified and config.get("calibration_status") == "nominal_model_not_hardware_calibrated":
            raise ValueError("Nominal geometry cannot be labeled verified")
        self.side = config["side"]
        if self.side not in ("left", "right") or tuple(robot.sides) != (self.side,):
            raise ValueError("Cartesian policy currently supports exactly one selected arm")
        if not finite(control_hz) or not 20 <= control_hz <= 200:
            raise ValueError("Cartesian control_hz must be in [20, 200]")
        self.robot, self.config = robot, dict(config)
        self.preparation_spec = robot.spec
        self.period = 1 / control_hz
        self.chain = KinematicChain(**config["chain"])
        if len(self.chain.indices) != 7:
            raise ValueError("Expected seven-joint follower kinematics")
        self.task_from_base = pose(config["task_from_base"])
        self.base_from_task = np.linalg.inv(self.task_from_base)
        self.gripper = GripperCalibration(**config["gripper"])
        if min(self.gripper.motor) < robot.spec.lower[7] or max(self.gripper.motor) > robot.spec.upper[7]:
            raise ValueError("Gripper calibration exceeds physical motor bounds")
        workspace = np.asarray(config["workspace_m"], dtype=float)
        if workspace.shape != (2, 3) or not np.isfinite(workspace).all() or not (workspace[0] < workspace[1]).all():
            raise ValueError("Workspace must be finite [lower_xyz, upper_xyz] in task metres")
        self.speed = config.get("velocity_rad_s", .25)
        self.grip_speed = config.get("gripper_velocity_rad_s", .5)
        self.tracking = config.get("tracking_limit_rad", .15)
        self.max_age = config.get("feedback_max_age_s", .15)
        if (any(not finite(v) or v <= 0 for v in (self.speed, self.grip_speed, self.tracking, self.max_age))
                or self.speed > .5 or self.grip_speed > .5 or self.tracking > .3 or self.max_age > .4):
            raise ValueError("Invalid Cartesian execution limits")
        names = tuple(f"{self.side}.{n}" for n in ("x", "y", "z", "qx", "qy", "qz", "qw", "opening"))
        self.spec = ActionSpec(names, tuple(workspace[0]) + (-1.,)*4 + (float(min(self.gripper.opening)),),
                               tuple(workspace[1]) + (1.,)*4 + (float(max(self.gripper.opening)),), kind=TCP_POSE)
        self.streams = tuple(robot.streams) + tuple(
            StreamSpec(f"{kind}/{self.side}", TCP_POSE, (8,), "m,xyzw,m", "task", self.max_age)
            for kind in ("tcp", "target"))
        self.record = record or (lambda event: None)
        self.solver = self.reference = self.target = None
        self.last_submit = None
        self.dispatch_sequence = 0
        self.preparing = False
        self.progress = None
        if config.get("progress") is not None:
            from .progress import CartesianProgress
            self.progress = CartesianProgress(**config["progress"])
            if self.progress.pause_error + self.speed*self.period >= self.tracking:
                raise ValueError("Progress pause must leave one joint step below the hard tracking limit")
            self.streams += (StreamSpec(f"execution/{self.side}", "waiting_joint_error", (2,),
                                        "bool,rad", "motor", self.max_age),)

    def begin_preparation(self):
        if self.reference is None or self.preparing:
            raise RuntimeError("Cartesian preparation requires a started, idle robot")
        self.preparing = True

    def preparation_pose_error(self, actual, goal):
        """Model FK distance/rotation from supplied measured joints, no CAN reads."""
        import numpy as np
        from scipy.spatial.transform import Rotation
        measured = self.task_from_base @ self.chain.forward(actual[:7])
        target = self.task_from_base @ self.chain.forward(goal[:7])
        return (float(np.linalg.norm(measured[:3, 3]-target[:3, 3])),
                float(Rotation.from_matrix(target[:3, :3].T @ measured[:3, :3]).magnitude()))

    def submit_preparation(self, values, valid_until):
        """Typed joint setup through the same underlying worker/lease, not TCP IK."""
        import numpy as np
        if not self.preparing or self.reference is None:
            raise RuntimeError("Joint preparation is not active")
        self.preparation_spec.validate(values)
        now = time.monotonic()
        if not finite(valid_until) or valid_until <= now:
            raise TimeoutError("Expired preparation command")
        if self.last_submit is not None and now-self.last_submit > max(.25, 3*self.period):
            raise TimeoutError("Preparation submit deadline missed")
        _, measured = self._feedback(self.robot.snapshot())
        joints = np.asarray(values, dtype=float)
        if (max(abs(joints[:7]-self.reference[:7])) > self.speed*self.period+1e-8
                or abs(joints[7]-self.reference[7]) > self.grip_speed*self.period+1e-8):
            raise ValueError("Preparation joint velocity exceeded")
        if max(abs(joints[:7]-measured[:7])) > self.tracking or abs(joints[7]-measured[7]) > .20:
            raise RuntimeError("Preparation commanded lead exceeded")
        target = self._tcp(joints)
        self.spec.validate(tuple(float(v) for v in target))
        self.robot.submit(tuple(float(v) for v in joints), valid_until)
        self.reference, self.target = joints.copy(), target
        self.last_submit = now
        self.dispatch_sequence += 1
        self.record({"event": "cartesian_preparation_dispatch", "time": time.monotonic(),
                     "joint_targets": list(values), "commanded_task_tcp": target.tolist(),
                     "measured_task_tcp": self._tcp(measured).tolist()})

    def finish_preparation(self):
        from openarm_runtime.cartesian import DifferentialIK
        if not self.preparing:
            raise RuntimeError("Preparation is not active")
        _, measured = self._feedback(self.robot.snapshot())
        if max(abs(self.reference[:7]-measured[:7])) > self.tracking or abs(self.reference[7]-measured[7]) > .20:
            raise RuntimeError("Preparation did not finish with bounded tracking")
        # Re-anchor the soft posture task at ready, not the hanging start. Keep
        # the last dispatched configuration/goal to avoid an implicit pose jump.
        self.solver = DifferentialIK(self.chain, lower=self.robot.spec.lower[:7],
            upper=self.robot.spec.upper[:7], velocity=[self.speed]*7, posture=self.reference[:7])
        self.target = self._tcp(self.reference)
        self.preparing = False
        return tuple(float(v) for v in self.target)

    def _feedback(self, samples):
        import numpy as np
        sample = samples.get(f"robot/{self.side}")
        if (sample is None or not sample.valid or sample.error
                or not 0 <= time.monotonic() - sample.received_at <= self.max_age):
            raise RuntimeError("Cartesian robot feedback unavailable/stale")
        state = np.asarray(sample.data, dtype=float)
        if state.shape != (8, 3) or not np.isfinite(state).all():
            raise ValueError("Invalid Cartesian robot feedback")
        measured = state[:, 0]
        self.robot.spec.validate(tuple(float(v) for v in measured))
        return sample, measured

    def _tcp(self, joints):
        import numpy as np
        from scipy.spatial.transform import Rotation
        transform = self.task_from_base @ self.chain.forward(joints[:7])
        return np.r_[transform[:3, 3], Rotation.from_matrix(transform[:3, :3]).as_quat(),
                     self.gripper.aperture(joints[7])]

    def start(self):
        import numpy as np
        from openarm_runtime.cartesian import DifferentialIK
        if self.progress is not None:
            self.progress.waiting_since = None
        self.robot.start()
        try:
            _, measured = self._feedback(self.robot.snapshot())
            self.reference = measured.copy()
            self.target = self._tcp(measured)
            self.spec.validate(tuple(float(v) for v in self.target))
            self.solver = DifferentialIK(self.chain, lower=self.robot.spec.lower[:7],
                upper=self.robot.spec.upper[:7], velocity=[self.speed]*7, posture=measured[:7])
            self.record({"event": "cartesian_context", "time": time.monotonic(),
                         "calibration": self.config, "model": self.chain.metadata})
        except BaseException:
            self.robot.close()
            raise

    def snapshot(self):
        samples = self.robot.snapshot()
        sample, measured = self._feedback(samples)
        samples[f"tcp/{self.side}"] = SensorSample(sample.sequence, sample.received_at,
                                                   tuple(float(v) for v in self._tcp(measured)))
        samples[f"target/{self.side}"] = SensorSample(sample.sequence + self.dispatch_sequence, sample.received_at,
                                                      tuple(float(v) for v in (
                                                          self._tcp(self.reference) if self.progress else self.target)))
        if self.progress is not None:
            samples[f"execution/{self.side}"] = SensorSample(sample.sequence, sample.received_at,
                (float(self.progress.waiting_since is not None), float(max(abs(self.reference[:7]-measured[:7])))))
        return samples

    def submit(self, values, valid_until):
        import numpy as np
        from scipy.spatial.transform import Rotation
        if self.solver is None or self.preparing:
            raise RuntimeError("Cartesian robot not started")
        self.spec.validate(values)
        now = time.monotonic()
        if not finite(valid_until) or valid_until <= now:
            raise TimeoutError("Expired Cartesian command")
        if self.last_submit is not None and now - self.last_submit > max(.25, 3*self.period):
            raise TimeoutError("Cartesian submit deadline missed")
        _, measured = self._feedback(self.robot.snapshot())
        if max(abs(self.reference[:7] - measured[:7])) > self.tracking:
            raise RuntimeError("Cartesian tracking lead exceeded")
        if self.progress is not None and not self.progress.advance(self.reference[:7], measured[:7], now):
            # Hold the existing joint reference: no new IK integration and no
            # target accumulation while feedback catches up. Watchdogs remain live.
            self.robot.submit(tuple(float(v) for v in self.reference), valid_until)
            self.last_submit = now
            self.record({"event": "cartesian_progress_wait", "time": now,
                         "joint_targets": self.reference.tolist(),
                         "measured_task_tcp": self._tcp(measured).tolist()})
            return
        governed = (self.progress.target(self._tcp(self.reference), values)
                    if self.progress is not None else values)
        target = np.eye(4)
        target[:3, 3], target[:3, :3] = governed[:3], Rotation.from_quat(governed[3:7]).as_matrix()
        target_base = self.base_from_task @ target
        # Persistent bounded reference avoids stalling against static friction;
        # independently bound reference-versus-measured error before AND after IK.
        solved = self.solver.step(self.reference[:7], target_base, self.period)
        joints = np.r_[solved["joints"], self.reference[7] + np.clip(
            self.gripper.angle(values[7]) - self.reference[7], -self.grip_speed*self.period, self.grip_speed*self.period)]
        if max(abs(joints[:7] - measured[:7])) > self.tracking or abs(joints[7] - measured[7]) > .20:
            raise RuntimeError("Cartesian commanded lead exceeded")
        realized = self._tcp(joints)
        self.spec.validate(tuple(float(v) for v in realized))
        if time.monotonic() - now > self.period:
            raise TimeoutError("Cartesian IK deadline missed")
        sent = tuple(float(v) for v in joints)
        self.robot.submit(sent, valid_until)
        self.reference, self.target = joints, np.asarray(values, dtype=float).copy()
        self.last_submit = now
        self.dispatch_sequence += 1
        self.record({"event": "cartesian_dispatch", "time": time.monotonic(),
                     "requested_task_tcp": list(values), "governed_task_tcp": list(governed), "joint_targets": sent,
                     "measured_task_tcp": self._tcp(measured).tolist(),
                     "commanded_task_tcp": realized.tolist(), "ik": solved})

    def close(self):
        try:
            self.robot.close()
        finally:
            self.solver = self.reference = self.target = None
            self.preparing = False
