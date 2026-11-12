"""Episode orchestration; policies never access the robot or own its control loop."""
from dataclasses import dataclass, asdict
from copy import deepcopy
import threading
import time

from .contracts import ActionChunk, Context, finite
from .inference import InferenceWorker
from .observations import ObservationBuffer


@dataclass(frozen=True)
class SessionConfig:
    control_hz: float = 20
    inference_hz: float = 10
    inference_timeout_s: float = 0.4
    reset_timeout_s: float = 60
    source_timeout_s: float = 5
    command_timeout_s: float = 0.5
    max_horizon_s: float = 2
    history: int = 8

    def __post_init__(self):
        for name in ("control_hz", "inference_hz", "inference_timeout_s", "reset_timeout_s",
                     "command_timeout_s", "max_horizon_s", "source_timeout_s"):
            v = getattr(self, name)
            if not finite(v) or v <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if self.control_hz > 200 or self.inference_hz > self.control_hz:
            raise ValueError("Expected inference_hz <= control_hz <= 200")
        if type(self.history) is not int or not 1 <= self.history <= 256:
            raise ValueError("history must be in 1..256")


class PolicySession:
    """One episode. stop/pause ends it and disables; resume is a new reset/episode.

    One inference request is in flight. A new chunk replaces the old chunk at its
    start time; expired prefixes are skipped, never replayed from index zero.
    Future chunks wait while the active chunk continues. A gap holds the last
    target only for command_timeout_s, then terminates the episode.
    """
    def __init__(self, robot, policy, *, sources=(), config=None, task="", recorder=None, preparation=None,
                 return_home=None, on_event=None):
        self.robot, self.policy = robot, policy
        self.sources = tuple(sources)
        self.config = config or SessionConfig()
        self.context = Context(robot.spec, tuple(robot.streams) + tuple(
            s for source in sources for s in source.streams), self.config.control_hz, task,
            execution=deepcopy(getattr(robot, "execution", {})))
        self.buffer = ObservationBuffer(self.context.streams, self.config.history)
        self.recorder = recorder
        self.on_event = on_event
        self.cancel = threading.Event()
        self.state = "idle"
        self.preparation = preparation
        self.return_home = return_home
        self._handoff_hold = None
        if preparation is not None or return_home is not None:
            try:
                for stage in (preparation, return_home):
                    if stage is not None:
                        stage.bind(robot, self.context)
            except BaseException as exc:
                # Constructor validation precedes robot.start(): no motor worker
                # exists, so report a truthful startup rejection, not lost cleanup.
                self._record('ended', state='failed', error=str(exc), cleanup_errors=[],
                             stop_reason='preflight_rejected', hardware_started=False)
                raise

    def stop(self):
        self.cancel.set()

    pause = stop

    def _record(self, event, **data):
        payload = {"event": event, "time": time.monotonic(), **data}
        if self.recorder is not None:
            self.recorder.record(payload)
        if self.on_event is not None:
            self.on_event(payload)

    def _observe(self):
        samples = {}
        for source in self.sources:
            samples.update(source.snapshot())
        # Fetch potentially blocking sources first, then fresh robot feedback.
        # Never change a sample timestamp to hide latency or stale feedback.
        samples.update(self.robot.snapshot())
        return self.buffer.collect(samples)

    def _wait_for_sources(self):
        specs = tuple(s for source in self.sources for s in source.streams)
        probe = ObservationBuffer(specs, 1)
        deadline = time.monotonic() + self.config.source_timeout_s
        while not self.cancel.is_set():
            samples = {}
            for source in self.sources:
                samples.update(source.snapshot())
            try:
                probe.collect(samples)
                return
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
            self.cancel.wait(.02)

    def _prepare(self, *, returning=False):
        stage = self.return_home if returning else self.preparation
        if stage is None or self.cancel.is_set():
            return
        phase = 'return_home' if returning else 'preparation'
        self.state = "returning_home" if returning else "preparing"
        if hasattr(self.robot, "begin_preparation"):
            self.robot.begin_preparation()
        # Returning uses only fresh robot feedback, not cameras or more inference.
        robot_buffer = ObservationBuffer(self.robot.streams, 1) if returning else None
        observe = (lambda: robot_buffer.collect(self.robot.snapshot())) if returning else self._observe
        initial = observe()
        reference = getattr(self.robot, 'reference', None) if returning else None
        plan = stage.begin(initial) if reference is None else stage.begin(initial, commanded_joints=tuple(reference))
        self._record(phase+"_started", phase=phase, plan=plan)
        self._record("observation", phase=phase, observation=initial)
        period = 1 / self.config.control_hz
        previous = time.monotonic()
        last_waypoint = None
        while not self.cancel.is_set():
            tick = time.monotonic()
            if tick-previous > max(.25, 3*period):
                raise TimeoutError(f"{phase} deadline missed")
            previous = tick
            observation = observe()
            if self.cancel.is_set():
                return
            self._record("observation", phase=phase, observation=observation)
            step = stage.step(observation)
            if self.cancel.is_set():
                return
            deadline = time.monotonic() + self.config.command_timeout_s
            submit = getattr(self.robot, "submit_preparation", self.robot.submit)
            submit(step.values, deadline)
            self._record(phase+"_command", phase=phase, values=step.values,
                         names=stage.spec.names, kind=stage.spec.kind,
                         waypoint=step.waypoint, progress=step.progress, valid_until=deadline,
                         arrival=step.arrival)
            if step.waypoint != last_waypoint:
                self._record(phase+"_waypoint", phase=phase, waypoint=step.waypoint)
                last_waypoint = step.waypoint
            if step.completed:
                if returning:
                    self._record('return_home_completed', phase=phase, elapsed_s=tick-initial.captured_at)
                    return
                if hasattr(self.robot, "finish_preparation"):
                    self._handoff_hold = self.robot.finish_preparation()
                else:
                    self._handoff_hold = step.values
                self.robot.spec.validate(self._handoff_hold)
                self._record("preparation_completed", phase="preparation", elapsed_s=tick-initial.captured_at)
                # No setup histories or action queues enter the learned task.
                self.buffer = ObservationBuffer(self.context.streams, self.config.history)
                return
            self.cancel.wait(max(0, period-(time.monotonic()-tick)))

    def run(self, duration_s=None, *, home_only=False):
        if self.state != "idle":
            raise RuntimeError("Create a new session for each episode")
        if duration_s is not None and (not finite(duration_s) or duration_s <= 0):
            raise ValueError("duration_s must be positive")
        if type(home_only) is not bool or home_only and (self.return_home is None or self.sources):
            raise ValueError('Home-only requires a return route and no model/sensor sources')
        worker = None
        primary_error = None
        stop_reason = 'operator_stop'
        self.state = "starting"
        try:
            self._record("context", context=asdict(self.context), config=asdict(self.config))
            if home_only:
                if self.cancel.is_set():
                    return
                self.robot.start()
                self._prepare(returning=True)
                stop_reason = 'home_completed' if not self.cancel.is_set() else 'operator_stop'
                return
            worker = InferenceWorker(self.policy, self.context)
            worker.ready(self.config.reset_timeout_s, self.cancel.is_set)
            self._wait_for_sources()
            if self.cancel.is_set():
                return
            self.robot.start()
            self._prepare()
            if self.cancel.is_set():
                return
            self.state = "running"
            self._record("task_started", phase="task")
            started = last_good = last_tick = time.monotonic()
            active = pending = request = None
            sent_at = 0.0
            next_infer = started
            held = self._handoff_hold
            period = 1 / self.config.control_hz
            while not self.cancel.is_set():
                tick = time.monotonic()
                if duration_s is not None and tick - started >= duration_s:
                    stop_reason = 'duration_reached'
                    self._record('duration_reached', phase='task', elapsed_s=tick-started,
                                 duration_s=duration_s)
                    break
                if tick - last_tick > max(0.25, 3 * period):
                    raise TimeoutError("Session deadline missed")
                last_tick = tick
                observation = self._observe()
                if self.cancel.is_set():
                    break
                self._record("observation", phase="task", observation=observation)
                if request is not None:
                    chunk = worker.result()
                    if chunk is not None:
                        if not isinstance(chunk, ActionChunk):
                            raise TypeError("Policy must return ActionChunk")
                        chunk.validate(self.robot.spec, request, time.monotonic(), self.config.max_horizon_s)
                        pending = chunk
                        self._record("chunk", phase="task", chunk=asdict(chunk), latency_s=time.monotonic() - sent_at)
                        request = None
                    elif tick - sent_at >= self.config.inference_timeout_s:
                        raise TimeoutError("Policy inference timed out")
                now = time.monotonic()
                if pending is not None and pending.start_at <= now:
                    active, pending = pending, None
                values = active.at(now) if active is not None else None
                if values is not None:
                    held, last_good = tuple(values), now
                elif now - last_good >= self.config.command_timeout_s:
                    raise TimeoutError("Action queue starved")
                if held is not None:
                    deadline = last_good + self.config.command_timeout_s
                    self.robot.submit(held, deadline)
                    self._record("command" if active is not None else "handoff_hold",
                                 phase="task" if active is not None else "handoff",
                                 values=held, valid_until=deadline,
                                 observation_sequence=active.observation_sequence if active is not None else None)
                if request is None and now >= next_infer:
                    worker.submit(observation)
                    request, sent_at = observation, time.monotonic()
                    next_infer = sent_at + 1 / self.config.inference_hz
                self.cancel.wait(max(0, period - (time.monotonic() - tick)))
            if stop_reason == 'duration_reached' and not self.cancel.is_set() and self.return_home is not None:
                # Never return from finally: cancellation/faults must disable,
                # not initiate another motion after the operator asked to stop.
                self._prepare(returning=True)
                if self.cancel.is_set():
                    stop_reason = 'operator_stop'
        except BaseException as exc:
            primary_error = exc
            self.state = "failed"
            raise
        finally:
            errors = []
            # Hardware cleanup comes first, even if model cleanup or recording fails.
            for close in (self.robot.close, worker.close if worker else lambda: None):
                try:
                    close()
                except BaseException as exc:
                    errors.append(exc)
            if self.state != "failed":
                self.state = "stopped" if not errors else "failed"
            self._record("ended", state=self.state, error=str(primary_error or ""),
                         stop_reason='error' if primary_error or errors else stop_reason,
                         cleanup_errors=[str(e) for e in errors])
            if errors:
                raise BaseExceptionGroup("Policy session cleanup failed", ([primary_error] if primary_error else []) + errors)
