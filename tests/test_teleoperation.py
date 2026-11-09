"""Relative takeover and physical-control failure cleanup without hardware."""

import tempfile
import math
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from openarm_leader.mapping import Mapping
from openarm_leader.transport import Sample
from openarm_runtime.config import Profile
from openarm_runtime.teleoperation import Teleoperation, relative_offsets, follower_targets, leader_sample


class TeleoperationTests(unittest.TestCase):
    def test_cartesian_dispatch_and_failure_use_existing_disable_lifecycle(self):
        from test_arm import FakeBus
        from contextlib import nullcontext
        from dataclasses import asdict
        bus = FakeBus()
        bus.preflight = lambda: {mid: {'state': asdict(bus.state(mid))} for mid in range(1, 9)}
        bus.close = Mock()
        reader = Mock(snapshot=Mock(side_effect=lambda: (self.sample(), '', 1)))
        calls = []
        class Adapter:
            metadata = {'mode': 'virtual_leader_fk_constrained_ik'}
            def reset(self, base):
                calls.append(('reset', dict(base)))
            def step(self, targets, states, period):
                calls.append(('step', period))
                if len([x for x in calls if x[0] == 'step']) > 1:
                    raise RuntimeError('IK rejected')
                return dict(targets) | {1: .001}, {}
        session = Teleoperation(Profile('can1', 'can0', True), self.mapping,
                                {'a': reader, 'b': reader}, self.temp.name,
                                cartesian_factory=lambda side: Adapter())
        with patch('openarm_runtime.teleoperation.bus_lease', side_effect=lambda *a, **k: nullcontext()), \
             patch('openarm_runtime.teleoperation.ClassicBus', return_value=bus), \
             patch('openarm_runtime.teleoperation.check_modes'), \
             patch('openarm_runtime.arm.time.sleep'):
            session.start(1, ('left',), mode='cartesian')
            session.worker.join(4)
        self.assertFalse(session.busy)
        self.assertFalse(bus.enabled)
        self.assertIn('IK rejected', session.message)
        self.assertAlmostEqual(bus.last_targets[1], .001)
        self.assertEqual(calls[0][0], 'reset')

    def test_home_alignment_resume_recording_and_disable(self):
        from openarm_collection import Collection
        from test_arm import FakeBus
        from contextlib import nullcontext
        collection = Collection(self.temp.name)
        self.addCleanup(collection.close)
        positions = [2048] * 7 + [2000]
        test = self
        class Reader:
            on_sample = None
            count = 0
            def snapshot(self):
                self.count += 1
                sample = test.sample(positions)
                if self.on_sample:
                    self.on_sample(sample, self.count)
                return sample, "", self.count
        reader = Reader()
        session = Teleoperation(Profile("can1", "can0", True), self.mapping,
                                {"a": reader, "b": reader}, self.temp.name, collection)
        bus = FakeBus()
        bus.positions[1] = math.radians(2)
        bus.preflight = lambda: {}
        bus.close = Mock()
        def wait_for(predicate):
            deadline = time.monotonic() + 4
            while not predicate() and time.monotonic() < deadline:
                threading.Event().wait(.01)
            self.assertTrue(predicate(), session.message)
        with patch("openarm_runtime.teleoperation.bus_lease", side_effect=lambda *a, **kw: nullcontext()), \
             patch("openarm_runtime.teleoperation.ClassicBus", return_value=bus), \
             patch("openarm_runtime.teleoperation.check_modes"), \
             patch("openarm_runtime.arm.time.sleep"):
            try:
                session.start(1, ("right",), home=True)
                wait_for(lambda: session.home.phase == "holding")
                self.assertFalse(collection.ready)
                positions[0] += 200
                session.resume(1)
                wait_for(lambda: collection.ready)
                self.assertAlmostEqual(bus.last_targets[1], 0)
                self.assertAlmostEqual(bus.last_targets[8], 0)
                collection.start("home-test", "operator")
                with self.assertRaisesRegex(RuntimeError, "结束录制"):
                    session.return_zero(1)
                collection.finish()
                session.return_zero(1)
                self.assertFalse(collection.ready)
            finally:
                session.close()
        self.assertFalse(bus.enabled)
        self.assertFalse(session.busy)

    def test_episode_finish_preserves_following_and_records_single_arm(self):
        from openarm_collection import Collection, inspect_episode
        from test_arm import FakeBus
        from contextlib import nullcontext
        collection = Collection(self.temp.name)
        self.addCleanup(collection.close)
        test = self
        class Reader:
            on_sample = None
            count = 0
            def snapshot(self):
                self.count += 1
                sample = test.sample()
                if self.on_sample:
                    self.on_sample(sample, self.count)
                return sample, "", self.count
        reader = Reader()
        session = Teleoperation(Profile("can1", "can0", True), self.mapping,
                                {"a": reader, "b": reader}, self.temp.name, collection)
        bus = FakeBus()
        bus.preflight = lambda: {}
        bus.close = Mock()
        with patch("openarm_runtime.teleoperation.bus_lease", side_effect=lambda *a, **kw: nullcontext()), \
             patch("openarm_runtime.teleoperation.ClassicBus", return_value=bus) as factory, \
             patch("openarm_runtime.teleoperation.check_modes"), \
             patch("openarm_runtime.arm.time.sleep"):
            try:
                session.start(1, ("right",))
                deadline = time.monotonic() + 3
                while not collection.ready and time.monotonic() < deadline:
                    threading.Event().wait(.01)
                self.assertTrue(collection.ready, session.message)
                collection.start("test", "operator")
                threading.Event().wait(.15)
                path = collection.finish()
                self.assertTrue(session.busy)
                self.assertEqual(bus.enabled, set(range(1, 9)))
                self.assertEqual(inspect_episode(path)["state"], "review")
                collection.review("success")
                factory.assert_called_once_with("can0", .04)
            finally:
                session.close()
        self.assertFalse(bus.enabled)

    def test_dual_preflight_failure_enables_neither_arm(self):
        reader = Mock(snapshot=Mock(side_effect=lambda: (self.sample(), "", 1)))
        session = Teleoperation(Profile("can1", "can0", True), self.mapping,
                                {"a": reader, "b": reader}, self.temp.name)
        buses = {name: Mock() for name in ("can0", "can1")}
        buses["can1"].preflight.return_value = {}
        buses["can0"].preflight.side_effect = RuntimeError("right preflight failed")
        from contextlib import nullcontext
        with patch("openarm_runtime.teleoperation.bus_lease", side_effect=lambda *a, **kw: nullcontext()), \
             patch("openarm_runtime.teleoperation.ClassicBus", side_effect=lambda interface, *a: buses[interface]), \
             patch("openarm_runtime.teleoperation.check_modes"), \
             patch("openarm_runtime.teleoperation.ArmHold") as hold:
            session.start(1, ("left", "right"))
            session.worker.join(3)
            hold.assert_not_called()
        self.assertFalse(session.busy)
        self.assertIn("right preflight failed", session.message)
        for bus in buses.values():
            bus.close.assert_called_once()

    def test_dual_arm_keeps_ready_peer_serviced_and_stops_both_on_fault(self):
        reader = Mock(snapshot=Mock(side_effect=lambda: (self.sample(), "", 1)))
        session = Teleoperation(Profile("can1", "can0", True), self.mapping,
                                {"a": reader, "b": reader}, self.temp.name)
        first_tick = threading.Event()
        checked, exited = set(), set()
        buses = {name: Mock(name=name) for name in ("can0", "can1")}
        for name, bus in buses.items():
            bus.interface = name
            def preflight(name=name):
                checked.add(name)
                return {}
            bus.preflight.side_effect = preflight
        test = self
        class Hold:
            def __init__(self, bus, *args, **kwargs):
                self.interface = bus.interface
                self.base = dict.fromkeys(range(1, 9), 0)
            def __enter__(self):
                test.assertEqual(checked, {"can0", "can1"})
                if self.interface == "can0":
                    test.assertTrue(first_tick.wait(2), "Ready arm was not serviced during peer startup")
                return self
            def track(self, targets):
                if self.interface == "can1":
                    first_tick.set()
                else:
                    raise RuntimeError("right feedback failed")
            def __exit__(self, *args):
                exited.add(self.interface)
        from contextlib import nullcontext
        with patch("openarm_runtime.teleoperation.bus_lease", side_effect=lambda *a, **kw: nullcontext()), \
             patch("openarm_runtime.teleoperation.ClassicBus", side_effect=lambda interface, *a: buses[interface]), \
             patch("openarm_runtime.teleoperation.check_modes"), \
             patch("openarm_runtime.teleoperation.ArmHold", Hold):
            session.start(1, ("left", "right"))
            session.worker.join(4)
        self.assertFalse(session.busy)
        self.assertEqual(exited, {"can0", "can1"})
        self.assertIn("right feedback failed", session.message)
        for bus in buses.values():
            bus.close.assert_called_once()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.mapping = Mapping("a", "b", Path(self.temp.name) / "mapping.json")
        for side in ("left", "right"):
            self.mapping.device(side).update(zero=[2048] * 7, open=1000, closed=2000)

    def sample(self, positions=None, errors=(), timestamp=None):
        return Sample(tuple(positions or [2048] * 7 + [1500]), (0,) * 8,
                      time.monotonic() if timestamp is None else timestamp, 0.01, (), errors)

    def test_takeover_zero_and_joint_order_and_gripper_direction(self):
        origin = self.sample()
        for side in ("left", "right"):
            self.assertEqual(set(relative_offsets(self.mapping, side, origin, origin).values()), {0})
        positions = list(origin.positions)
        positions[6] += 100
        positions[7] = 1000
        offsets = relative_offsets(self.mapping, "left", origin, self.sample(positions))
        self.assertAlmostEqual(offsets[6], -100 * 360 / 4096)
        self.assertEqual(offsets[7], 0)
        self.assertEqual(offsets[8], 32.5)
        self.assertEqual(relative_offsets(self.mapping, "right", origin, self.sample(positions))[8], -32.5)

    def test_large_motion_clips_at_joint_bounds_not_takeover_envelope(self):
        offsets = relative_offsets(self.mapping, "right", self.sample(), self.sample([3000] * 8))
        self.assertGreater(abs(offsets[1]), 30)
        targets = follower_targets("right", dict.fromkeys(range(1, 9), 0), offsets)
        self.assertAlmostEqual(math.degrees(targets[1]), -75)
        self.assertAlmostEqual(math.degrees(targets[2]), -9)
        self.assertAlmostEqual(math.degrees(targets[4]), 0)

    def test_stale_and_combined_alarm_stop_physical_control(self):
        for sample in (self.sample(timestamp=0), self.sample(errors=(3,) * 8)):
            with self.assertRaises(RuntimeError):
                leader_sample(Mock(snapshot=Mock(return_value=(sample, "", 1))))

    def test_voltage_only_alarm_is_nonblocking(self):
        sample = Sample((2048,) * 8, (0,) * 8, time.monotonic(), 0.01,
                        (4.8,) * 7 + (3.9,), (0,) * 7 + (1,))
        self.assertIs(leader_sample(Mock(snapshot=Mock(return_value=(sample, "", 1)))), sample)

    def test_fault_message_preserves_combined_alarm_and_voltage(self):
        sample = Sample((2048,) * 8, (0,) * 8, time.monotonic(), 0.01,
                        (4.8,) * 7 + (3.8,), (0,) * 7 + (3,))
        with self.assertRaisesRegex(RuntimeError, r"ID8 舵机告警 0x03，3.8 V"):
            leader_sample(Mock(snapshot=Mock(return_value=(sample, "", 1))))

    def test_control_failure_exits_hold_closes_bus_and_releases_session(self):
        reader = Mock(snapshot=Mock(side_effect=lambda: (self.sample(), "", 1)))
        session = Teleoperation(Profile("can1", "can0", True), self.mapping, {"a": reader, "b": reader}, self.temp.name)
        bus = Mock()
        bus.preflight.return_value = {}
        control = Mock(base=dict.fromkeys(range(1, 9), 0))
        control.track.side_effect = RuntimeError("injected feedback failure")
        from contextlib import nullcontext
        with patch("openarm_runtime.teleoperation.bus_lease", return_value=nullcontext()), \
             patch("openarm_runtime.teleoperation.ClassicBus", return_value=bus), \
             patch("openarm_runtime.teleoperation.check_modes"), \
             patch("openarm_runtime.teleoperation.ArmHold") as hold_factory:
            hold_factory.return_value.__enter__.return_value = control
            hold_factory.return_value.__exit__.return_value = False
            session.start(1, ("right",))
            session.worker.join(3)
            self.assertFalse(session.busy)
            self.assertIn("injected feedback failure", session.message)
            hold_factory.return_value.__exit__.assert_called_once()
            bus.close.assert_called_once()

    def test_preflight_failure_never_enables(self):
        reader = Mock(snapshot=Mock(side_effect=lambda: (self.sample(), "", 1)))
        session = Teleoperation(Profile("can1", "can0", True), self.mapping, {"a": reader, "b": reader}, self.temp.name)
        from contextlib import nullcontext
        with patch("openarm_runtime.teleoperation.bus_lease", return_value=nullcontext()), \
             patch("openarm_runtime.teleoperation.ClassicBus") as bus, \
             patch("openarm_runtime.teleoperation.ArmHold") as hold:
            bus.return_value.preflight.side_effect = RuntimeError("preflight failed")
            session.start(1, ("right",))
            session.worker.join(3)
            hold.assert_not_called()
            bus.return_value.close.assert_called_once()
            self.assertFalse(session.busy)
