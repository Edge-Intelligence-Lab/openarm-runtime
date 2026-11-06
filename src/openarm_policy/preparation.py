"""Deterministic feedback-gated setup before learned inference, no device access.

Waypoints are absolute joint targets, not prerecorded simulator increments.
Each segment starts at the previous dispatched goal; the first starts at actual
feedback. A minimum-jerk time law bounds velocity/acceleration, then feedback
must settle before the next segment or the policy can take over.
"""
from dataclasses import dataclass
import math

from .contracts import POSITION, finite


def vector(values, size, name):
    if not isinstance(values, (list, tuple)) or len(values) != size or any(not finite(v) for v in values):
        raise ValueError(f"{name} requires {size} finite numbers")
    return tuple(float(v) for v in values)


@dataclass(frozen=True)
class PreparationStep:
    values: tuple[float, ...]
    waypoint: str
    progress: float
    completed: bool
    arrival: dict | None = None


class FixedJointPreparation:
    def __init__(self, side, waypoints, *, velocity_rad_s=.25, acceleration_rad_s2=.75,
                 gripper_velocity_rad_s=.5, gripper_acceleration_rad_s2=1.,
                 settle_tolerance_rad=.015, settle_samples=5, settle_timeout_s=8.,
                 maximum_duration_s=120., start_reference_rad=None, start_tolerance_rad=.75,
                 provenance=None):
        if side not in ("left", "right") or not isinstance(waypoints, list) or not 1 <= len(waypoints) <= 32:
            raise ValueError("Preparation requires one selected arm and 1..32 waypoints")
        for key, value, upper in (("velocity", velocity_rad_s, .5), ("acceleration", acceleration_rad_s2, 5.),
                                  ("gripper velocity", gripper_velocity_rad_s, .5),
                                  ("gripper acceleration", gripper_acceleration_rad_s2, 5.),
                                  ("settle tolerance", settle_tolerance_rad, .15),
                                  ("settle timeout", settle_timeout_s, 60.),
                                  ("maximum duration", maximum_duration_s, 600.),
                                  ("start tolerance", start_tolerance_rad, math.pi)):
            if not finite(value) or not 0 < value <= upper:
                raise ValueError(f"Invalid preparation {key}")
        if type(settle_samples) is not int or not 1 <= settle_samples <= 200:
            raise ValueError("settle_samples must be in [1, 200]")
        self.side = side
        self.waypoints = []
        for item in waypoints:
            if (not isinstance(item, dict) or set(item) - {"name", "joints_rad", "gripper_rad", "tcp_tolerance"}
                    or not isinstance(item.get("name"), str) or not item["name"].strip()):
                raise ValueError("Waypoint needs a name, joints_rad and optional gripper_rad")
            goal = vector(item["joints_rad"], 7, "waypoint joints")
            grip = item.get("gripper_rad")
            if grip is not None and not finite(grip):
                raise ValueError("gripper_rad must be finite or omitted to hold its previous goal")
            tcp = item.get("tcp_tolerance")
            if tcp is not None:
                if not isinstance(tcp, dict) or set(tcp) != {"position_m", "orientation_rad", "gripper_rad"}:
                    raise ValueError("TCP arrival needs position_m, orientation_rad and gripper_rad tolerances")
                for name, upper in (("position_m", .05), ("orientation_rad", .5), ("gripper_rad", .15)):
                    if not finite(tcp[name]) or not 0 < tcp[name] <= upper:
                        raise ValueError(f"Invalid TCP arrival {name}")
                tcp = dict(tcp)
            self.waypoints.append(dict(name=item["name"], joints_rad=goal, gripper_rad=grip,
                                       tcp_tolerance=tcp))
        if len({w["name"] for w in self.waypoints}) != len(self.waypoints):
            raise ValueError("Preparation waypoint names must be unique")
        self.speed = (float(velocity_rad_s),)*7 + (float(gripper_velocity_rad_s),)
        self.acceleration = (float(acceleration_rad_s2),)*7 + (float(gripper_acceleration_rad_s2),)
        self.tolerance, self.stable_count = settle_tolerance_rad, settle_samples
        self.settle_timeout, self.maximum_duration = settle_timeout_s, maximum_duration_s
        self.start_reference = None if start_reference_rad is None else vector(start_reference_rad, 7, "start reference")
        self.start_tolerance = start_tolerance_rad
        self.metadata = dict(kind="fixed_joint_waypoints_v1", side=side, provenance=provenance,
                             learned=False, excluded_from_task_duration=True,
                             waypoints=self.waypoints, velocity_rad_s=list(self.speed),
                             acceleration_rad_s2=list(self.acceleration),
                             settle_tolerance_rad=settle_tolerance_rad, settle_samples=settle_samples,
                             settle_timeout_s=settle_timeout_s, maximum_duration_s=maximum_duration_s,
                             start_reference_rad=self.start_reference, start_tolerance_rad=start_tolerance_rad)
        self.spec = None
        self.started = False
        self.finished = False

    def bind(self, robot, context):
        expected = tuple(f"{self.side}.joint{i}" for i in range(1, 8)) + (f"{self.side}.gripper_motor",)
        self.spec = getattr(robot, "preparation_spec", robot.spec)
        self.pose_error = getattr(robot, "preparation_pose_error", None)
        if any(w["tcp_tolerance"] is not None for w in self.waypoints) and not callable(self.pose_error):
            raise ValueError("TCP arrival requires an explicit robot FK capability")
        if self.spec.kind != POSITION or self.spec.names != expected:
            raise ValueError("Preparation needs an explicit single-arm joint-position capability")
        if (hasattr(robot, "speed") and any(v > robot.speed for v in self.speed[:7])) or (
                hasattr(robot, "grip_speed") and self.speed[7] > robot.grip_speed):
            raise ValueError("Preparation speed exceeds the robot capability")
        for waypoint in self.waypoints:
            # Validate held gripper at both bounds, before any motor startup.
            for grip in ((waypoint["gripper_rad"],) if waypoint["gripper_rad"] is not None
                         else (self.spec.lower[7], self.spec.upper[7])):
                self.spec.validate(waypoint["joints_rad"] + (grip,))
        if not finite(context.control_hz) or not 20 <= context.control_hz <= 200:
            raise ValueError("Preparation requires 20..200Hz control")
        self.period = 1 / context.control_hz

    def measured(self, observation):
        history = observation.streams.get(f"robot/{self.side}", ())
        if not history or not history[-1].valid or history[-1].error:
            raise RuntimeError("Preparation robot feedback missing")
        sample = history[-1]
        if not finite(sample.received_at) or not 0 <= observation.captured_at-sample.received_at <= .15:
            raise RuntimeError("Preparation robot feedback stale")
        values = vector([row[0] for row in sample.data], 8, "measured preparation joints")
        self.spec.validate(values)
        return values

    def begin(self, observation):
        if self.spec is None or self.started:
            raise RuntimeError("Bind once and use a fresh preparation per session")
        actual = self.measured(observation)
        # Already at the final posture? Skip the swing route, but still settle
        # and perform the final gripper action; never swing down again blindly.
        ready = self.waypoints[-1]["joints_rad"]
        at_ready = max(abs(a-b) for a, b in zip(actual[:7], ready)) <= self.tolerance
        if (not at_ready and self.start_reference is not None
                and max(abs(a-b) for a, b in zip(actual[:7], self.start_reference)) > self.start_tolerance):
            raise ValueError("Current pose is outside the declared preparation start region")
        self.index = len(self.waypoints)-1 if at_ready else 0
        self.origin = self.last = actual
        self.started_at = observation.captured_at
        self.started = True
        self._segment()
        return dict(initial_joints_rad=list(actual), skipped_to_ready=at_ready,
                    metadata=self.metadata, planned_motion_s=self.planned_remaining(actual))

    def segment_ticks(self, start, goal):
        duration = self.period
        for a, b, speed, acceleration in zip(start, goal, self.speed, self.acceleration):
            distance = abs(b-a)
            # max s'(u)=1.875, max |s''(u)|=10/sqrt(3) for quintic smoothstep.
            duration = max(duration, 1.875*distance/speed,
                           math.sqrt((10/math.sqrt(3))*distance/acceleration))
        return max(1, math.ceil(duration/self.period))

    def planned_remaining(self, origin):
        duration = 0.
        for waypoint in self.waypoints[self.index:]:
            goal = waypoint["joints_rad"] + (origin[7] if waypoint["gripper_rad"] is None else waypoint["gripper_rad"],)
            duration += self.segment_ticks(origin, goal)*self.period
            origin = goal
        if duration + (len(self.waypoints)-self.index)*self.stable_count*self.period > self.maximum_duration:
            raise ValueError("Preparation plan exceeds maximum_duration_s")
        return duration

    def _segment(self):
        waypoint = self.waypoints[self.index]
        self.goal = waypoint["joints_rad"] + (self.origin[7] if waypoint["gripper_rad"] is None else waypoint["gripper_rad"],)
        self.ticks = self.segment_ticks(self.origin, self.goal)
        self.frame = self.stable = 0
        self.settling_at = None

    def step(self, observation):
        if not self.started or self.finished:
            raise RuntimeError("Preparation has not begun")
        now = observation.captured_at
        if now < self.started_at or now-self.started_at > self.maximum_duration:
            raise TimeoutError("Preparation duration exceeded")
        actual = self.measured(observation)
        if max(abs(a-b) for a, b in zip(actual[:7], self.last[:7])) > .15 or abs(actual[7]-self.last[7]) > .20:
            raise RuntimeError("Preparation tracking error exceeded")
        u = min(1., self.frame/self.ticks)
        blend = u*u*u*(10 + u*(-15 + 6*u))
        command = tuple(a + blend*(b-a) for a, b in zip(self.origin, self.goal))
        self.spec.validate(command)
        if any(abs(a-b) > speed*self.period+1e-9 for a, b, speed in zip(command, self.last, self.speed)):
            raise RuntimeError("Preparation command velocity exceeded")
        self.last = command
        waypoint = self.waypoints[self.index]["name"]
        progress = (self.index + u)/len(self.waypoints)
        completed = False
        arrival = None
        if self.frame >= self.ticks:
            if self.settling_at is None:
                self.settling_at = now
            tcp = self.waypoints[self.index]["tcp_tolerance"]
            if tcp is None:
                error = max(abs(a-b) for a, b in zip(actual, self.goal))
                arrived = error <= self.tolerance
                arrival = dict(kind="joint", max_error_rad=error)
            else:
                position, orientation = self.pose_error(actual, self.goal)
                if any(not finite(v) or v < 0 for v in (position, orientation)):
                    raise RuntimeError("Invalid preparation FK arrival error")
                gripper = abs(actual[7]-self.goal[7])
                arrived = (position <= tcp["position_m"] and orientation <= tcp["orientation_rad"]
                           and gripper <= tcp["gripper_rad"])
                arrival = dict(kind="tcp_fk", position_error_m=position,
                               orientation_error_rad=orientation, gripper_error_rad=gripper)
            arrival["accepted"] = arrived
            self.stable = self.stable+1 if arrived else 0
            if self.stable >= self.stable_count:
                if self.index == len(self.waypoints)-1:
                    completed, progress = True, 1.
                    self.finished = True
                else:
                    self.origin = self.goal
                    self.index += 1
                    self._segment()
            elif now-self.settling_at > self.settle_timeout:
                raise TimeoutError(f"Preparation waypoint failed to settle: {waypoint}")
        else:
            self.frame += 1
        return PreparationStep(command, waypoint, progress, completed, arrival)
