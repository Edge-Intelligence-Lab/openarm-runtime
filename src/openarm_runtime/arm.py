"""Persistent whole-arm MIT hold, matching the reference follower gain layout."""

from dataclasses import asdict
from collections import deque
import math
import time

from .telemetry import IDS
from .config import finite
from .motion import SmoothMotion
from .limits import joint_bounds_deg, manual_velocity_rad_s, MAX_MANUAL_SPEED_DEG_S

# J1–J7: Enactic openarm_description OpenArm v2 control_gains.yaml.
# J8 is not specified upstream; retain the existing gripper gains.
FOLLOWER_KP = (70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0, 25.0)
FOLLOWER_KD = (2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5, 0.3)


class ArmHold:
    def __init__(self, bus, cancelled, record, publish, *, tracking=False, manual_side=None, gravity=None, observe=None, integral=None):
        self.bus, self.cancelled, self.record, self.publish = bus, cancelled, record, publish
        self.tracking = tracking
        self.gravity = gravity
        if integral is not None and not tracking:
            raise ValueError('Integral requires explicit tracking ownership')
        from .integral import TrackingIntegral
        self.integral = TrackingIntegral(**integral) if integral is not None else None
        self.observe = observe
        self.manual_bounds = joint_bounds_deg(manual_side) if manual_side is not None else None
        self.motion = None
        if manual_side is not None:
            self.motion = SmoothMotion(joint_bounds_deg(manual_side) | {8: (-30, 30)}, relative_gripper=True)
        self.trace = deque(maxlen=200)
        self.attempted = []
        self.states = {}
        self.base = {}
        self.targets = {}
        self.cycles = 0

    def validate(self, state, enabled=True):
        if not state.fresh(time.monotonic(), 0.4) or state.frame_format != "classic":
            raise RuntimeError(f"ID{state.motor_id}: stale or non-Classic feedback")
        if state.status != int(enabled):
            raise RuntimeError(f"ID{state.motor_id}: status={state.status}, expected={int(enabled)}")
        if any(not math.isfinite(x) for x in (state.position, state.velocity, state.torque)):
            raise RuntimeError("Non-finite feedback")
        if max(state.mos_temperature, state.rotor_temperature) >= 75:
            raise RuntimeError(f"ID{state.motor_id}: temperature limit")
        if not self.tracking and abs(state.velocity) > 2:
            raise RuntimeError(f"ID{state.motor_id}: velocity limit")

    def check_cancelled(self):
        if self.cancelled():
            raise InterruptedError("User stop / browser disconnected")

    def __enter__(self):
        try:
            self.check_cancelled()
            self.bus.prepare_encoder(IDS)
            initial = {}
            for mid in IDS:
                self.check_cancelled()
                state = self.bus.state(mid)
                self.validate(state, enabled=False)
                initial[mid] = state
                if self.motion:
                    self.motion.validate_position(mid, state.position)
                    if abs(state.velocity) > .08:
                        raise RuntimeError(f"ID{mid}: follower must be stationary before enable")
                if self.manual_bounds is not None and mid in self.manual_bounds:
                    lower, upper = self.manual_bounds[mid]
                    finite(math.degrees(state.position), lower-0.5, upper+0.5, "initial_angle_deg")
                self.bus.command(mid, "position", state.position, 0, 0)
            if self.gravity is not None:
                self.gravity.preflight(initial, self.bus.ranges)
                self.record({"event": "gravity_config", **self.gravity.metadata()})
            for mid in IDS:
                self.check_cancelled()
                self.attempted.append(mid)
                self.bus.command(mid, "enable")
                time.sleep(0.02)
            for mid in IDS:
                end = time.monotonic() + 0.3
                while True:
                    self.check_cancelled()
                    state = self.bus.state(mid)
                    if state.status != 0 or time.monotonic() >= end:
                        self.validate(state)
                        break
                    time.sleep(0.02)
                self.states[mid] = state
            self.base = {mid: state.position for mid, state in self.states.items()}
            self.targets = dict(self.base)
            if self.motion:
                self.motion.initialize(self.base)
            self.record({"event": "arm_enabled", "base_rad": self.base,
                         "integral": None if self.integral is None else {
                             "gain_per_s": self.integral.gain,
                             "torque_fraction": self.integral.fraction,
                             "slew_fraction_per_s": self.integral.slew},
                         "kp": FOLLOWER_KP, "kd": FOLLOWER_KD,
                         "sdk_version": self.bus.sdk.__version__,
                         "motion_policy": self.motion.metadata() if self.motion else None})
            # Ramp all gains together while holding freshly read current angles.
            for step in range(1, 11):
                self.tick({mid: 0 for mid in IDS}, 0.5, 0.05, gain_scale=step / 10)
                time.sleep(0.05)
            return self
        except BaseException as exc:
            failures = self.disable()
            self.flush_trace(exc)
            if failures:
                raise RuntimeError(f"{exc}; disable NOT confirmed: {failures}") from exc
            raise

    def __exit__(self, exc_type, exc, tb):
        failures = self.disable()
        self.flush_trace(exc)
        if failures:
            raise RuntimeError(f"Disable NOT confirmed: {failures}; original={exc}") from exc

    def flush_trace(self, exc):
        if self.trace:
            self.record({"event": "motion_trace", "error": str(exc) if exc else None,
                         "samples": list(self.trace)})

    def disable(self):
        if self.integral is not None:
            self.integral.reset()
        errors = []
        # Attempt every disable before checking any reply, even after partial enable.
        for mid in self.attempted:
            try:
                self.bus.command(mid, "disable")
            except Exception as exc:
                errors.append(f"ID{mid} send: {exc}")
        for mid in self.attempted:
            try:
                end = time.monotonic() + 0.3
                while True:
                    state = self.bus.state(mid)
                    if state.status != 1 or time.monotonic() >= end:
                        if state.status != 0:
                            raise RuntimeError(f"status={state.status}")
                        break
                    time.sleep(0.02)
            except Exception as exc:
                errors.append(f"ID{mid} readback: {exc}")
        if self.attempted:
            self.record({"event": "arm_disable", "ids": list(self.attempted),
                         "verified": not errors, "errors": errors})
        self.attempted.clear()
        return errors

    def tick(self, offsets, speed, period, gain_scale=1):
        self.check_cancelled()
        finite(speed, 0.1, MAX_MANUAL_SPEED_DEG_S if self.manual_bounds is not None else 5, "speed_deg_s")
        finite(period, 0.02, 0.1, "period_s")
        finite(gain_scale, 0, 1, "gain_scale")
        if set(offsets) != set(IDS):
            raise ValueError("All eight joint targets required")
        kp = [gain_scale * x for x in FOLLOWER_KP]
        kd = [gain_scale * x for x in FOLLOWER_KD]
        step = math.radians(speed) * period
        targets = {}
        goals = {}
        for mid in IDS:
            self.validate(self.states[mid])
            if self.manual_bounds is not None and mid in self.manual_bounds:
                lower, upper = self.manual_bounds[mid]
                base_deg = math.degrees(self.base[mid])
                # Zero writes have 0.5-degree tolerance. Holding that freshly
                # captured pose must not itself command a move into the range.
                if offsets[mid] == 0 and lower-0.5 <= base_deg <= upper+0.5:
                    offset = 0.0
                else:
                    offset = finite(offsets[mid], lower-base_deg, upper-base_deg, "offset_deg")
            else:
                offset = finite(offsets[mid], -30, 30, "offset_deg")
            goal = self.base[mid] + math.radians(offset)
            if abs(goal) >= self.bus.ranges[mid].position:
                raise ValueError(f"ID{mid}: target outside firmware range")
            goals[mid] = goal
            joint_step = step
            if self.manual_bounds is not None:
                joint_step = min(step, manual_velocity_rad_s(mid, self.bus.ranges[mid].velocity) * period)
            q = self.targets[mid] + max(-joint_step, min(joint_step, goal - self.targets[mid]))
            state = self.states[mid]
            effort = kp[mid - 1] * (q - state.position) - kd[mid - 1] * state.velocity
            if abs(effort) > 0.9 * self.bus.ranges[mid].torque:
                raise RuntimeError(f"ID{mid}: predicted effort near firmware range; error={math.degrees(q-state.position):.2f}°")
            targets[mid] = q
        if self.motion:
            targets = self.motion.step(goals, speed, period)
        self._exchange(targets, kp, kd)

    def track(self, targets):
        """Direct MIT position references; caller clips to robot joint limits.

        Unlike the manual-slider tick, this has no software slew/offset cap or
        inferred-effort rejection. The motor applies its MIT effort saturation.
        """
        if not self.tracking:
            raise RuntimeError("Direct targets require a tracking session")
        self.check_cancelled()
        if set(targets) != set(IDS):
            raise ValueError("All eight joint targets required")
        for mid, goal in targets.items():
            self.validate(self.states[mid])
            if not math.isfinite(goal) or abs(goal) >= self.bus.ranges[mid].position:
                raise ValueError(f"ID{mid}: target outside firmware range")
        self._exchange(dict(targets), FOLLOWER_KP, FOLLOWER_KD)

    def _exchange(self, targets, kp, kd):
        before = self.states
        frame = None
        if self.motion:
            frame = {"time_monotonic": time.monotonic(), "targets_rad": dict(targets),
                     "before": {mid: asdict(s) for mid, s in self.states.items()},
                     "requested_rad": dict(self.motion.last_goals),
                     "target_velocity_rad_s": dict(self.motion.velocities),
                     "issued_kp": kp, "issued_kd": kd}
            self.trace.append(frame)
        try:
            feedforward = None if self.gravity is None else self.gravity.update(self.states)
            if self.integral is not None:
                for state in self.states.values():
                    self.validate(state)
                feedforward = self.integral.update(self.states, targets, kp, kd, self.bus.ranges,
                    feedforward or dict.fromkeys(IDS, 0.), time.monotonic())
            if feedforward is not None:
                for mid in IDS:
                    state = self.states[mid]
                    effort = kp[mid-1] * (targets[mid] - state.position) - kd[mid-1] * state.velocity + feedforward[mid]
                    if abs(effort) > .9 * self.bus.ranges[mid].torque:
                        raise RuntimeError(f"ID{mid}: combined PD/feedforward effort near firmware range")
                if frame is not None:
                    frame["gravity"] = self.gravity.last if self.gravity else None
            if self.motion:
                self.motion.supervise(self.states, targets, self.bus.ranges, kp, kd, time.monotonic(), feedforward=feedforward)
            started = time.monotonic()
            states = (self.bus.exchange(targets, kp, kd) if feedforward is None else
                      self.bus.exchange(targets, kp, kd, feedforward))
            if frame is not None:
                frame["feedback"] = {mid: asdict(s) for mid, s in states.items()}
            if set(states) != set(IDS):
                raise RuntimeError("Incomplete arm feedback")
            for mid, state in states.items():
                if state.motor_id != mid:
                    raise RuntimeError("Feedback ID mismatch")
                self.validate(state)
            if self.motion:
                self.motion.supervise(states, targets, self.bus.ranges, kp, kd, time.monotonic(), after=True, feedforward=feedforward)
        except BaseException as exc:
            if self.integral is not None:
                self.integral.reset()
            if frame is not None:
                frame["error"] = str(exc)
                if hasattr(exc, "feedback"):
                    frame["partial_feedback"] = {mid: asdict(s) for mid, s in exc.feedback.items()}
            raise
        self.states, self.targets = states, targets
        if self.observe is not None:
            self.observe({"dispatch_started_at": started, "exchange_completed_at": time.monotonic(),
                          "targets_rad": dict(targets), "kp": tuple(kp), "kd": tuple(kd),
                          "feedforward_nm": feedforward if feedforward is not None else dict.fromkeys(IDS, 0.0),
                          "observation_before": {mid: asdict(s) for mid, s in before.items()},
                          "feedback": {mid: asdict(s) for mid, s in states.items()},
                          "gravity": self.gravity.last if self.gravity else None})
        if self.integral is not None and self.cycles % 20 == 0:
            self.record({'event': 'integral_sample', 'torque_nm': dict(self.integral.values),
                         'torque_fraction': self.integral.fraction})
        self.publish(states)
        self.cycles += 1
        if self.gravity is not None:
            self.record({"event": "gravity_sample", "time": time.monotonic(),
                         "targets_rad": dict(targets), "gravity": self.gravity.last,
                         "states": {mid: asdict(s) for mid, s in states.items()}})
        if self.cycles % 20 == 0:
            self.record({"event": "hold_sample", "targets_rad": targets,
                         "states": {mid: asdict(s) for mid, s in states.items()}})
