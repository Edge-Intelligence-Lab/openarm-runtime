"""Simulation and Classic CAN adapters; construction/import never opens hardware."""
from contextlib import ExitStack
import math
import queue
import threading
import time

from .contracts import ActionSpec, SensorSample, StreamSpec, finite


def joint_spec(sides=("left", "right")):
    from openarm_runtime.joints import JOINT_LIMITS
    if not sides or len(set(sides)) != len(sides) or any(s not in JOINT_LIMITS for s in sides):
        raise ValueError("Select unique left/right sides")
    names, lower, upper = [], [], []
    for side in sides:
        for index, (lo, hi) in enumerate(JOINT_LIMITS[side], 1):
            names.append(f"{side}.joint{index}" if index < 8 else f"{side}.gripper_motor")
            lower.append(math.radians(lo))
            upper.append(math.radians(hi))
    return ActionSpec(tuple(names), tuple(lower), tuple(upper))


def state_specs(sides):
    return tuple(StreamSpec(f"robot/{s}", "joint_state", (8, 3), "rad,rad/s,Nm", s, 0.4)
                 for s in sides)


class SimulatedRobot:
    """Ideal position plant for contract tests, not a physics or tracking model."""
    def __init__(self, sides=("left", "right")):
        self.sides = tuple(sides)
        self.spec, self.streams = joint_spec(sides), state_specs(sides)
        self.values = tuple(0.0 for _ in self.spec.names)
        self.started = False
        self.sequence = 0
        self.commands = []

    def start(self):
        if self.started:
            raise RuntimeError("Already started")
        self.started = True

    def snapshot(self):
        if not self.started:
            raise RuntimeError("Robot not started")
        self.sequence += 1
        return {f"robot/{side}": SensorSample(self.sequence, time.monotonic(),
                tuple((q, 0.0, 0.0) for q in self.values[i * 8:(i + 1) * 8]))
                for i, side in enumerate(self.sides)}

    def submit(self, values, valid_until):
        if not self.started or not finite(valid_until) or valid_until <= time.monotonic():
            raise RuntimeError("Robot stopped or action expired")
        self.spec.validate(values)
        self.values = tuple(values)
        self.commands.append((time.monotonic(), self.values))

    def close(self):
        self.started = False


class ClassicRobot:
    """Each bus owns one fixed-rate worker, ArmHold and exclusive bus lease.

    start() explicitly enables/holds the selected arms. All buses preflight before
    any enable. A shared target deadline stops both workers independently of the
    policy/session thread. close() waits for verified disable via ArmHold.
    """
    def __init__(self, profile, sides=("left", "right"), watchdog_s=0.5, record=None, integral=None):
        if not profile.mapping_verified or not finite(watchdog_s) or not 0 < watchdog_s <= 2:
            raise ValueError("Verified profile and watchdog in (0, 2] seconds required")
        self.profile, self.sides = profile, tuple(sides)
        self.spec, self.streams = joint_spec(sides), state_specs(sides)
        self.watchdog_s = watchdog_s
        if integral is not None:
            from openarm_runtime.integral import TrackingIntegral
            TrackingIntegral(**integral)  # Validate before opening devices.
        self.integral = integral
        self.record = record or (lambda event: None)
        self.lock = threading.Lock()
        self.events = queue.Queue(maxsize=256)
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.prepared = threading.Barrier(len(sides), timeout=10)
        self.workers, self.samples, self.holding, self.errors = [], {}, set(), []
        self.targets = None
        self.deadline = None

    def _record(self, side, event):
        # Never perform disk/network I/O on a CAN worker. Full audit queues fail
        # explicitly rather than dropping records or blocking motor cleanup.
        self.events.put_nowait(dict(event, side=side, time=time.monotonic()))

    def _drain_events(self):
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                return
            self.record(event)

    def _run(self, side):
        from openarm_runtime.arm import ArmHold
        from openarm_runtime.config import bus_lease
        from openarm_runtime.control import check_modes
        from openarm_runtime.transport import ClassicBus
        from openarm_runtime.gravity import prepare_gravity

        def publish(states):
            with self.lock:
                old = self.samples.get(f"robot/{side}")
                self.samples[f"robot/{side}"] = SensorSample(
                    1 if old is None else old.sequence + 1,
                    min(s.received_at for s in states.values()),
                    tuple((states[i].position, states[i].velocity, states[i].torque) for i in range(1, 9)))

        def cancelled():
            if self.stop.is_set():
                return True
            with self.lock:
                expired = self.deadline is not None and time.monotonic() >= self.deadline
            if expired:
                raise TimeoutError("Policy command watchdog expired")
            return False

        try:
            with ExitStack() as stack:
                interface = self.profile.interface(side)
                stack.enter_context(bus_lease(interface, exclusive=True))
                bus = ClassicBus(interface, self.profile.feedback_timeout_s)
                stack.callback(bus.close)
                preflight = bus.preflight()
                self._record(side, {"event": "preflight", "motors": preflight})
                gravity = prepare_gravity(self.profile, side, bus, preflight)
                check_modes(bus, True, self.stop.is_set)
                self.prepared.wait()
                with ArmHold(bus, cancelled, lambda e: self._record(side, e), publish,
                             tracking=True, gravity=gravity, integral=self.integral) as control:
                    try:
                        control.track(control.base)
                        with self.lock:
                            self.holding.add(side)
                            if len(self.holding) == len(self.sides):
                                self.deadline = time.monotonic() + self.watchdog_s
                                self.ready.set()
                        period = 1 / self.profile.rate_hz
                        previous = time.monotonic()
                        while not self.stop.is_set():
                            tick = time.monotonic()
                            if tick - previous > max(0.25, 3 * period):
                                raise TimeoutError("CAN control deadline missed")
                            previous = tick
                            with self.lock:
                                values = self.targets
                            offset = self.sides.index(side) * 8
                            targets = control.base if values is None else dict(enumerate(values[offset:offset + 8], 1))
                            control.track(targets)
                            self.stop.wait(max(0, period - (time.monotonic() - tick)))
                    finally:
                        # Cancel the peer before waiting for this arm's disable readback.
                        self.stop.set()
                        self.prepared.abort()
        except BaseException as exc:
            with self.lock:
                if not isinstance(exc, (InterruptedError, threading.BrokenBarrierError)):
                    self.errors.append(f"{side}: {type(exc).__name__}: {exc}")
            self.stop.set()
            self.prepared.abort()
            self.ready.set()

    def start(self):
        if self.workers or self.stop.is_set():
            raise RuntimeError("ClassicRobot is single-session; construct a new instance")
        try:
            for side in self.sides:
                worker = threading.Thread(target=self._run, args=(side,), daemon=True)
                self.workers.append(worker)
                worker.start()
            if not self.ready.wait(15):
                raise TimeoutError("Robot startup timed out")
            self.snapshot()
        except BaseException:
            self.close()
            raise

    def snapshot(self):
        self._drain_events()
        with self.lock:
            if self.errors or self.stop.is_set():
                raise RuntimeError("; ".join(self.errors) or "Robot stopped")
            return dict(self.samples)

    def submit(self, values, valid_until):
        self.spec.validate(values)
        now = time.monotonic()
        if not finite(valid_until) or valid_until <= now:
            raise TimeoutError("Expired command")
        with self.lock:
            if not self.ready.is_set() or self.stop.is_set() or self.errors:
                raise RuntimeError("Robot not ready")
            self.targets = tuple(values)
            self.deadline = min(valid_until, now + self.watchdog_s)

    def close(self):
        self.stop.set()
        self.prepared.abort()
        for worker in self.workers:
            worker.join(15)
        if any(w.is_alive() for w in self.workers):
            raise RuntimeError("CAN worker did not stop")
        self._drain_events()
        if self.errors:
            raise RuntimeError("; ".join(self.errors))
