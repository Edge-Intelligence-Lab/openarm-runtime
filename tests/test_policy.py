"""Policy timing, failures, histories, replay and CAN lifecycle without hardware."""
from contextlib import nullcontext
from dataclasses import replace
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

from openarm_policy import (ActionChunk, ActionSpec, Observation, SensorSample, StreamSpec,
                            ObservationBuffer, SimulatedRobot, ClassicRobot, PolicySession,
                            SessionConfig, EpisodeRecorder, replay_observations)
from openarm_policy.adapters import HoldPolicy
from openarm_policy.contracts import Context
from openarm_runtime.config import Profile


class BrokenPolicy(HoldPolicy):
    def infer(self, observation):
        raise RuntimeError("model failed")


class SlowPolicy(HoldPolicy):
    def infer(self, observation):
        time.sleep(30)
        return super().infer(observation)


class SinglePolicy(HoldPolicy):
    def infer(self, observation):
        chunk = super().infer(observation)
        # Match HoldPolicy's one-second horizon when comparing action shapes.
        # Expiry itself is covered with explicit timestamps in the contract tests.
        return replace(chunk, steps=chunk.steps[:1], period_s=1.0)


class SlowResetPolicy(HoldPolicy):
    def reset(self, context):
        time.sleep(30)


class PolicyTests(unittest.TestCase):
    def test_official_v2_joint_intervals_keep_side_conventions_and_gripper_bounds(self):
        from openarm_runtime.joints import JOINT_LIMITS
        from openarm_policy.robots import joint_spec
        expected = {
            'left': ((-200,80),(-190,10),(-90,90),(0,140),(-90,90),(-45,45),(-90,90),(0,65)),
            'right': ((-80,200),(-10,190),(-90,90),(0,140),(-90,90),(-45,45),(-90,90),(-65,0)),
        }
        for side, intervals in expected.items():
            self.assertEqual(JOINT_LIMITS[side], intervals)
            spec = joint_spec((side,))
            for index, (lower, upper) in enumerate(intervals):
                self.assertAlmostEqual(spec.lower[index], math.radians(lower))
                self.assertAlmostEqual(spec.upper[index], math.radians(upper))
                for boundary in (lower, upper):
                    values = [0.]*8
                    values[index] = math.radians(boundary)
                    spec.validate(tuple(values))
                for outside in (lower-.001, upper+.001):
                    values = [0.]*8
                    values[index] = math.radians(outside)
                    with self.assertRaises(ValueError): spec.validate(tuple(values))

    def test_time_anchored_chunk_skips_expired_prefix(self):
        spec = ActionSpec(("a",), (-1,), (1,))
        obs = Observation(7, 100, {})
        chunk = ActionChunk(7, 100, .1, ((0,), (.2,), (.4,)), ("a",))
        chunk.validate(spec, obs, 100.15, 1)
        self.assertEqual(chunk.at(100.15), (.2,))
        self.assertIsNone(chunk.at(100.31))
        self.assertIsNone(chunk.at(99))
        with self.assertRaises(TimeoutError):
            chunk.validate(spec, obs, 101, 2)

    def test_chunk_rejects_wrong_schema_bounds_and_time(self):
        spec = ActionSpec(("a",), (-1,), (1,))
        obs = Observation(7, 100, {})
        chunk = ActionChunk(7, 100, .1, ((0,),), ("a",))
        for changes in ({"names": ("b",)}, {"observation_sequence": 6}, {"version": 2},
                        {"steps": ((math.nan,),)}, {"steps": ((2,),)}, {"period_s": 0},
                        {"start_at": 99}, {"start_at": 105}, {"kind": "torque"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(chunk, **changes).validate(spec, obs, 100, 1)

    def test_bounded_history_stale_optional_and_owned_payload(self):
        specs = (StreamSpec("q", "vector", (1,), "rad", "arm", .5),
                 StreamSpec("optional", "vector", (1,), "counts", "pad", .1, False))
        buffer = ObservationBuffer(specs, history=2)
        payload = [1.0]
        first = buffer.collect({"q": SensorSample(1, 10, payload)}, 10)
        payload[0] = 9
        self.assertEqual(first.streams["q"][0].data, [1])
        buffer.collect({"q": SensorSample(2, 10.1, [2])}, 10.1)
        last = buffer.collect({"q": SensorSample(3, 10.2, [3])}, 10.2)
        self.assertEqual([s.sequence for s in last.streams["q"]], [2, 3])
        self.assertEqual(last.streams["optional"], ())
        with self.assertRaises(RuntimeError):
            buffer.collect({"q": SensorSample(3, 10.2, [3])}, 11)
        with self.assertRaises(RuntimeError):
            buffer.collect({"q": SensorSample(1, 10.3, [3])}, 10.3)

    def test_shape_and_future_timestamp_rejected(self):
        b = ObservationBuffer((StreamSpec("q", "vector", (2,), "rad", "arm"),))
        with self.assertRaises(ValueError):
            b.collect({"q": SensorSample(1, 10, (1,))}, 10)
        with self.assertRaises(RuntimeError):
            b.collect({"q": SensorSample(1, 11, (1, 2))}, 10)

    def test_single_and_chunk_policies_use_identical_robot(self):
        for policy in (SinglePolicy(), HoldPolicy()):
            with self.subTest(policy=type(policy).__name__):
                class CountingRobot(SimulatedRobot):
                    def submit(self, values, valid_until):
                        super().submit(values, valid_until)
                        if len(self.commands) == 4:
                            session.stop()

                robot = CountingRobot()
                session = PolicySession(robot, policy)
                # Completion is command-driven, not a CI throughput assertion.
                # Keep a bounded failure path if commands never arrive; the
                # session's inference/control watchdogs remain unchanged.
                session.run(5)
                self.assertEqual(session.state, "stopped")
                self.assertFalse(robot.started)
                self.assertEqual(len(robot.commands), 4)
                self.assertTrue(all(not any(v) for _, v in robot.commands))

    def test_policy_exception_and_timeout_close_robot(self):
        for policy, error in ((BrokenPolicy(), RuntimeError), (SlowPolicy(), TimeoutError)):
            with self.subTest(policy=type(policy).__name__):
                robot = SimulatedRobot()
                session = PolicySession(robot, policy, config=SessionConfig(inference_timeout_s=.15))
                started = time.monotonic()
                with self.assertRaises(error):
                    session.run(2)
                self.assertLess(time.monotonic() - started, 4)
                self.assertFalse(robot.started)
                self.assertEqual(session.state, "failed")

    def test_pause_disables_and_session_cannot_reuse_old_chunk(self):
        robot = SimulatedRobot()
        session = PolicySession(robot, HoldPolicy())
        timer = threading.Timer(.5, session.pause)
        timer.start()
        try:
            session.run(3)
        finally:
            timer.join()
        self.assertFalse(robot.started)
        with self.assertRaises(RuntimeError):
            session.run(1)

    def test_stop_during_model_load_never_starts_robot(self):
        robot = SimulatedRobot()
        robot.start = Mock(wraps=robot.start)
        session = PolicySession(robot, SlowResetPolicy())
        timer = threading.Timer(.2, session.stop)
        timer.start()
        started = time.monotonic()
        try:
            session.run(1)
        finally:
            timer.join()
        self.assertLess(time.monotonic() - started, 3)
        robot.start.assert_not_called()

    def test_record_and_replay_observation_and_action_correlation(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "episode"
            with EpisodeRecorder(path) as recorder:
                PolicySession(SimulatedRobot(), HoldPolicy(), recorder=recorder).run(.4)
            observations = list(replay_observations(path))
            self.assertGreater(len(observations), 2)
            events = [json.loads(line) for line in (path / "events.jsonl").read_text().splitlines()]
            commands = [e for e in events if e["event"] == "command"]
            self.assertTrue(commands)
            self.assertIn(commands[0]["observation_sequence"], [o.sequence for o in observations])
            self.assertEqual(events[-1]["state"], "stopped")

    def test_missing_required_camera_prevents_robot_start(self):
        robot = SimulatedRobot()
        robot.start = Mock(wraps=robot.start)
        source = Mock(streams=(StreamSpec("camera/head", "bgr8", (2, 2, 3), "uint8", "head"),))
        source.snapshot.return_value = {}
        session = PolicySession(robot, HoldPolicy(), sources=(source,),
                                config=SessionConfig(source_timeout_s=.05))
        with self.assertRaisesRegex(RuntimeError, "Required stream"):
            session.run(1)
        robot.start.assert_not_called()

    def test_array_recording_is_lossless_and_deduplicated(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest("optional NumPy")
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "arrays"
            image = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
            obs = Observation(1, 10, {"camera/head": (SensorSample(8, 9.9, image),)})
            with EpisodeRecorder(path) as recorder:
                recorder.record({"event": "observation", "observation": obs})
                recorder.record({"event": "observation", "observation": replace(obs, sequence=2)})
            self.assertEqual(len(list(path.glob("*.npy"))), 1)
            replay = list(replay_observations(path))
            np.testing.assert_array_equal(replay[-1].streams["camera/head"][0].data, image)

    def test_recording_failure_still_closes_robot(self):
        class Recorder:
            def record(self, event):
                if event["event"] == "command":
                    raise OSError("disk full")
        robot = SimulatedRobot()
        session = PolicySession(robot, HoldPolicy(), recorder=Recorder())
        with self.assertRaisesRegex(OSError, "disk full"):
            session.run(1)
        self.assertFalse(robot.started)

    def test_tactile_adapter_preserves_timestamp_and_raw_counts(self):
        from openarm_policy import TactileObservation
        from openarm_tactile import Sample
        reader = Mock()
        reader.snapshot.return_value = dict(sequence=4, received_at=123.0, age_s=.1,
            sample=Sample((1, 2, 3), ((0, 0, 2),) * 68), state="live", error=None)
        sample = TactileObservation({"left_pad": reader}).snapshot()["tactile/left_pad"]
        self.assertEqual(sample.received_at, 123)
        self.assertEqual(sample.data[0], (1, 2, 3))
        self.assertEqual(sample.data[1:], ((0, 0, 2),) * 68)

    def test_future_chunk_does_not_replace_current_until_start(self):
        future_starts = []
        class Worker:
            def __init__(self, *args): self.request = None; self.count = 0
            def ready(self, timeout, cancelled=None): pass
            def submit(self, observation): self.request = observation
            def result(self):
                self.count += 1
                q = ((.05 if self.count == 1 else .1),) * 7 + (0,)
                start = self.request.captured_at + (0 if self.count == 1 else .2)
                if self.count > 1:
                    future_starts.append(start)
                return ActionChunk(self.request.sequence, start, .5, (q,), robot.spec.names)
            def close(self): pass
        robot = SimulatedRobot(("left",))
        robot.spec = ActionSpec(robot.spec.names, (-1,) * 8, (1,) * 8)
        session = PolicySession(robot, HoldPolicy(), config=SessionConfig(inference_hz=2))
        # This tests activation timestamps, not host scheduler throughput. The
        # worker is synchronous here, so advance a shared monotonic clock at wait.
        clock = [100.0]
        def advance(seconds):
            clock[0] += seconds
            return False
        with patch("openarm_policy.session.InferenceWorker", Worker), \
             patch("time.monotonic", side_effect=lambda: clock[0]), \
             patch.object(session.cancel, "wait", side_effect=advance):
            session.run(.95)
        qs = [v[0] for _, v in robot.commands]
        self.assertIn(.05, qs)
        self.assertIn(.1, qs)
        self.assertTrue(future_starts)
        first_new_command = next(t for t, values in robot.commands if values[0] == .1)
        self.assertGreaterEqual(first_new_command, future_starts[0])

    def test_classic_all_arm_preflight_before_enable(self):
        robot = ClassicRobot(Profile("can1", "can0", True))
        buses = {s: Mock() for s in ("can0", "can1")}
        buses["can0"].preflight.side_effect = RuntimeError("bad bus")
        with patch("openarm_runtime.config.bus_lease", side_effect=lambda *a, **kw: nullcontext()), \
             patch("openarm_runtime.transport.ClassicBus", side_effect=lambda name, *a: buses[name]), \
             patch("openarm_runtime.control.check_modes"), \
             patch("openarm_runtime.arm.ArmHold") as hold:
            with self.assertRaises(RuntimeError):
                robot.start()
            hold.assert_not_called()
        self.assertFalse(any(w.is_alive() for w in robot.workers))

    def test_can_audit_is_buffered_off_control_thread(self):
        record = Mock()
        robot = ClassicRobot(Profile("can1", "can0", True), record=record)
        robot._record("left", {"event": "arm_disable", "verified": True})
        record.assert_not_called()
        robot.close()
        self.assertEqual(record.call_args.args[0]["event"], "arm_disable")

    def test_model_adapter_keeps_preprocessing_outside_robot(self):
        from openarm_policy import ChunkPolicyAdapter
        robot = SimulatedRobot(("left",))
        robot.start()
        try:
            obs = ObservationBuffer(robot.streams).collect(robot.snapshot())
            encode = Mock(return_value="model-specific input")
            predict = Mock(return_value="normalized model output")
            decode = Mock(return_value=[(0.0,) * 8] * 4)
            adapter = ChunkPolicyAdapter(lambda ctx: predict, encode, decode)
            context = Context(robot.spec, robot.streams, 20)
            adapter.reset(context)
            chunk = adapter.infer(obs)
            chunk.validate(robot.spec, obs, obs.captured_at, 2)
            predict.assert_called_once_with("model-specific input")
            decode.assert_called_once_with("normalized model output", context)
            self.assertEqual(len(chunk.steps), 4)
        finally:
            robot.close()

    def test_classic_watchdog_independent_of_session_and_peer_startup(self):
        robot = ClassicRobot(Profile("can1", "can0", True), watchdog_s=.15)
        first_tick = threading.Event()
        disabled, checked = set(), set()
        test = self

        class Bus:
            def __init__(self, name, *_): self.name = name
            def preflight(self): checked.add(self.name); return {}
            def close(self): pass

        class Hold:
            def __init__(self, bus, cancel, record, publish, **kw):
                self.name, self.cancel, self.publish = bus.name, cancel, publish
                self.base = dict.fromkeys(range(1, 9), 0)
            def __enter__(self):
                test.assertEqual(checked, {"can0", "can1"})
                if self.name == "can0":
                    test.assertTrue(first_tick.wait(2))
                return self
            def track(self, targets):
                if self.cancel(): raise InterruptedError()
                from openarm_runtime.telemetry import MotorState
                self.publish({i: MotorState(i, 0, 0, 0, 20, 20, 1, time.monotonic()) for i in range(1, 9)})
                first_tick.set()
            def __exit__(self, *args): disabled.add(self.name)

        with patch("openarm_runtime.config.bus_lease", side_effect=lambda *a, **kw: nullcontext()), \
             patch("openarm_runtime.transport.ClassicBus", Bus), \
             patch("openarm_runtime.control.check_modes"), patch("openarm_runtime.arm.ArmHold", Hold):
            robot.start()
            self.assertTrue(robot.stop.wait(2), "watchdog did not fire without a policy loop")
            with self.assertRaisesRegex(RuntimeError, "watchdog"):
                robot.close()
        self.assertEqual(disabled, {"can0", "can1"})

    def test_imports_are_passive(self):
        import subprocess
        import sys
        script = """
from unittest.mock import patch
import sys
with patch('socket.socket', side_effect=AssertionError), patch('threading.Thread.start', side_effect=AssertionError):
 import openarm_policy
 assert openarm_policy.SimulatedRobot().started is False
assert not any(k in sys.modules for k in ('torch', 'numpy', 'cv2', 'viser', 'paxini_sdk', 'openarm_can'))
"""
        subprocess.run([sys.executable, "-I", "-c", script], check=True)


if __name__ == "__main__":
    unittest.main()
