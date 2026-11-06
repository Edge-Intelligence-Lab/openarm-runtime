"""Versioned, model-free contracts. Times are host time.monotonic() seconds."""
from dataclasses import dataclass, field
import math
from typing import Protocol

VERSION = 1
POSITION = "joint_position_rad"
TCP_POSE = "tcp_pose_xyzw_opening_m"


def finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


@dataclass(frozen=True)
class ActionSpec:
    names: tuple[str, ...]
    lower: tuple[float, ...]
    upper: tuple[float, ...]
    kind: str = POSITION
    version: int = VERSION

    def __post_init__(self):
        if self.version != VERSION or self.kind not in (POSITION, TCP_POSE):
            raise ValueError("Unsupported action protocol or kind")
        if not self.names or len(set(self.names)) != len(self.names):
            raise ValueError("Action names must be nonempty and unique")
        if len(self.lower) != len(self.names) or len(self.upper) != len(self.names):
            raise ValueError("Bounds must match action names")
        if any(not finite(a) or not finite(b) or a >= b for a, b in zip(self.lower, self.upper)):
            raise ValueError("Invalid action bounds")
        if self.kind == TCP_POSE and len(self.names) != 8:
            raise ValueError("TCP actions require xyz, xyzw quaternion and opening in metres")

    def validate(self, values):
        if len(values) != len(self.names) or any(
            not finite(v) or not lo <= v <= hi
            for v, lo, hi in zip(values, self.lower, self.upper)
        ):
            raise ValueError("Action must contain all finite, in-range absolute targets")
        if self.kind == TCP_POSE and not math.isclose(sum(v*v for v in values[3:7]), 1., abs_tol=1e-6):
            raise ValueError("TCP quaternion must have unit norm")


@dataclass(frozen=True)
class StreamSpec:
    name: str
    encoding: str
    shape: tuple[int, ...]
    units: str
    frame: str
    max_age_s: float = 0.5
    required: bool = True

    def __post_init__(self):
        if not self.name or not self.encoding or not finite(self.max_age_s) or self.max_age_s <= 0:
            raise ValueError("Invalid stream specification")
        if any(type(n) is not int or n < 1 for n in self.shape):
            raise ValueError("Invalid stream shape")


@dataclass(frozen=True)
class SensorSample:
    sequence: int
    received_at: float
    data: object
    valid: bool = True
    error: str = ""


@dataclass(frozen=True)
class Observation:
    sequence: int
    captured_at: float
    streams: dict[str, tuple[SensorSample, ...]]
    version: int = VERSION


@dataclass(frozen=True)
class Context:
    action: ActionSpec
    streams: tuple[StreamSpec, ...]
    control_hz: float
    task: str = ""
    version: int = VERSION


@dataclass(frozen=True)
class ActionChunk:
    observation_sequence: int
    start_at: float
    period_s: float
    steps: tuple[tuple[float, ...], ...]
    names: tuple[str, ...]
    kind: str = POSITION
    version: int = VERSION
    diagnostics: dict = field(default_factory=dict)

    @property
    def end_at(self):
        return self.start_at + len(self.steps) * self.period_s

    def validate(self, spec, observation, now, max_horizon_s):
        if (self.version != spec.version or self.kind != spec.kind or self.names != spec.names
                or self.observation_sequence != observation.sequence):
            raise ValueError("Action schema/order or observation correlation mismatch")
        if (not finite(self.start_at) or not finite(self.period_s) or self.period_s <= 0
                or self.start_at < observation.captured_at or not self.steps
                or not finite(self.end_at) or self.end_at > observation.captured_at + max_horizon_s):
            raise ValueError("Invalid action time horizon")
        for step in self.steps:
            spec.validate(step)
        if self.end_at <= now:
            raise TimeoutError("Entire action chunk expired during inference")

    def at(self, now):
        if now < self.start_at or now >= self.end_at:
            return None
        return self.steps[min(len(self.steps) - 1, int((now - self.start_at) / self.period_s))]


class Policy(Protocol):
    """Implementations are picklable; load model weights in reset, inside the worker."""
    def reset(self, context: Context) -> None: ...
    def infer(self, observation: Observation) -> ActionChunk: ...
    def close(self) -> None: ...


class Robot(Protocol):
    spec: ActionSpec
    streams: tuple[StreamSpec, ...]

    def start(self) -> None: ...
    def snapshot(self) -> dict[str, SensorSample]: ...
    def submit(self, values: tuple[float, ...], valid_until: float) -> None: ...
    def close(self) -> None: ...
