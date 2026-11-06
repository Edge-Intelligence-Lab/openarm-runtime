"""Execution-aware Cartesian reference progression, independent of model/CAN IO."""
import math


class CartesianProgress:
    """Bound task-space lead; pause with hysteresis while actual joints catch up.

    This is an executor governor, not a learned waiting action or collision check.
    The caller retains a persistent joint reference to overcome static friction.
    """
    def __init__(self, translation_step_m=.002, rotation_step_rad=.015,
                 pause_joint_error_rad=.07, resume_joint_error_rad=.04, pause_timeout_s=10.):
        values = (translation_step_m, rotation_step_rad, pause_joint_error_rad,
                  resume_joint_error_rad, pause_timeout_s)
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError("Progress bounds must be positive finite numbers")
        if (translation_step_m > .01 or rotation_step_rad > .1
                or not resume_joint_error_rad < pause_joint_error_rad < .15
                or pause_timeout_s > 30):
            raise ValueError("Invalid progress bounds or hysteresis")
        self.translation, self.rotation = translation_step_m, rotation_step_rad
        self.pause_error, self.resume_error = pause_joint_error_rad, resume_joint_error_rad
        self.timeout = pause_timeout_s
        self.waiting_since = None

    def advance(self, reference, measured, now):
        if len(reference) != 7 or len(measured) != 7:
            raise ValueError("Progress requires seven measured/reference joints")
        differences = [abs(a-b) for a, b in zip(reference, measured)]
        if any(not math.isfinite(v) for v in differences) or not math.isfinite(now):
            raise ValueError("Invalid progress feedback/time")
        error = max(differences)
        if self.waiting_since is None and error >= self.pause_error:
            self.waiting_since = now
        if self.waiting_since is not None:
            if now < self.waiting_since or now-self.waiting_since > self.timeout:
                raise TimeoutError("Cartesian progress failed to settle")
            if error > self.resume_error:
                return False
            self.waiting_since = None
        return True

    def target(self, realized, requested):
        import numpy as np
        from scipy.spatial.transform import Rotation
        realized, requested = np.asarray(realized, dtype=float), np.asarray(requested, dtype=float)
        if realized.shape != (8,) or requested.shape != (8,) or not np.isfinite([realized, requested]).all():
            raise ValueError("Expected finite TCP poses")
        out = requested.copy()
        delta = requested[:3]-realized[:3]
        length = np.linalg.norm(delta)
        out[:3] = realized[:3] + delta * min(1., self.translation/max(length, 1e-12))
        base = Rotation.from_quat(realized[3:7])
        turn = (Rotation.from_quat(requested[3:7])*base.inv()).as_rotvec()
        turn *= min(1., self.rotation/max(np.linalg.norm(turn), 1e-12))
        out[3:7] = (Rotation.from_rotvec(turn)*base).as_quat()
        return out
