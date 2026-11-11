import math
from pathlib import Path
import sys
import time
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

from openarm_runtime.arm import ArmHold, FOLLOWER_KP
from openarm_runtime.panel import MotionSession
from openarm_runtime.config import Profile
from openarm_runtime.transport import ClassicBus
from openarm_runtime.telemetry import IDS, MotorRange, MotorState


class FakeBus:
    def __init__(self):
        self.ranges = {mid: MotorRange(12.5, 30, 28) for mid in IDS}
        self.sdk = SimpleNamespace(__version__="1.4.1")
        self.enabled = set()
        self.commands = []
        self.positions = dict.fromkeys(IDS, 0.0)
        self.fail_enable = None
        self.last_targets = None

    def prepare_encoder(self, ids):
        pass

    def state(self, mid):
        return MotorState(mid, self.positions[mid], 0, 0, 25, 25,
                          int(mid in self.enabled), time.monotonic())

    def command(self, mid, action, *args):
        self.commands.append((mid, action))
        if action == "enable":
            self.enabled.add(mid)
            if mid == self.fail_enable:
                raise OSError("partial send")
        elif action == "disable":
            self.enabled.discard(mid)

    def exchange(self, targets, kp, kd):
        self.last_targets = dict(targets)
        self.positions.update(targets)
        return {mid: self.state(mid) for mid in IDS}


class ArmTests(unittest.TestCase):
    def test_integral_enters_feedforward_and_clears_on_disable(self):
        bus = FakeBus()
        sent = []
        def exchange(targets, kp, kd, ff):
            sent.append(dict(ff))
            return {mid: bus.state(mid) for mid in IDS}
        bus.exchange = exchange
        with patch('openarm_runtime.arm.time.sleep'):
            with ArmHold(bus, lambda: False, Mock(), Mock(), tracking=True, integral={}) as arm:
                arm.integral.previous = time.monotonic()-.05
                arm.track(dict.fromkeys(IDS, .04))
                self.assertGreater(sent[-1][7], 0.)
                self.assertEqual(sent[-1][8], 0.)
                self.assertEqual(arm.targets[7], .04)
            self.assertTrue(all(v == 0 for v in arm.integral.values.values()))
        self.assertFalse(bus.enabled)

    def test_collection_observer_separates_before_command_and_feedback(self):
        bus = FakeBus()
        samples = []
        with patch("openarm_runtime.arm.time.sleep"):
            with ArmHold(bus, lambda: False, Mock(), Mock(), tracking=True, observe=samples.append) as arm:
                arm.track(dict.fromkeys(IDS, .1))
                sample = samples[-1]
                self.assertEqual(sample["observation_before"][1]["position"], 0)
                self.assertEqual(sample["targets_rad"][1], .1)
                self.assertEqual(sample["feedback"][1]["position"], .1)
                self.assertLessEqual(sample["dispatch_started_at"], sample["exchange_completed_at"])
        self.assertFalse(bus.enabled)

    def test_tracking_sends_direct_targets_without_manual_limits(self):
        bus = FakeBus()
        with patch("openarm_runtime.arm.time.sleep"):
            with ArmHold(bus, lambda: False, Mock(), Mock(), tracking=True) as arm:
                arm.states[3] = replace(arm.states[3], velocity=3.0)
                targets = dict.fromkeys(IDS, 0.0)
                targets[3] = math.radians(60)
                arm.track(targets)
                self.assertEqual(bus.last_targets, targets)
                with self.assertRaises(ValueError):
                    arm.track(targets | {3: float("nan")})
        self.assertFalse(bus.enabled)

    def test_tracking_still_stops_on_feedback_loss(self):
        bus = FakeBus()
        with patch("openarm_runtime.arm.time.sleep"), self.assertRaisesRegex(RuntimeError, "Incomplete"):
            with ArmHold(bus, lambda: False, Mock(), Mock(), tracking=True) as arm:
                bus.exchange = lambda *args: {1: bus.state(1)}
                arm.track(dict.fromkeys(IDS, 0.0))
        self.assertFalse(bus.enabled)

    def test_partial_enable_disables_every_attempted_motor(self):
        bus = FakeBus()
        bus.fail_enable = 4
        with patch("openarm_runtime.arm.time.sleep"), self.assertRaises(OSError):
            ArmHold(bus, lambda: False, Mock(), Mock()).__enter__()
        self.assertFalse(bus.enabled)
        self.assertEqual([mid for mid, action in bus.commands if action == "disable"], [1, 2, 3, 4])

    def test_holds_all_joints_and_slew_limits_new_target(self):
        bus = FakeBus()
        with patch("openarm_runtime.arm.time.sleep"):
            with ArmHold(bus, lambda: False, Mock(), Mock()) as arm:
                self.assertEqual(bus.enabled, set(IDS))
                offsets = dict.fromkeys(IDS, 0)
                offsets[3] = 30
                arm.tick(offsets, 1, 0.05)
                self.assertAlmostEqual(bus.last_targets[3], math.radians(0.05))
                self.assertTrue(all(bus.last_targets[mid] == 0 for mid in IDS if mid != 3))
                self.assertEqual(FOLLOWER_KP[2], 70)
        self.assertFalse(bus.enabled)

    def test_cancel_in_hold_disables_whole_arm(self):
        bus = FakeBus()
        cancelled = False
        with patch("openarm_runtime.arm.time.sleep"), self.assertRaises(InterruptedError):
            with ArmHold(bus, lambda: cancelled, Mock(), Mock()) as arm:
                cancelled = True
                arm.tick(dict.fromkeys(IDS, 0), 1, 0.05)
        self.assertFalse(bus.enabled)

    def test_missing_feedback_ends_hold_and_disables_all(self):
        bus = FakeBus()
        with patch("openarm_runtime.arm.time.sleep"), self.assertRaisesRegex(RuntimeError, "Incomplete"):
            with ArmHold(bus, lambda: False, Mock(), Mock()) as arm:
                bus.exchange = lambda *args: {1: bus.state(1)}
                arm.tick(dict.fromkeys(IDS, 0), 1, 0.05)
        self.assertFalse(bus.enabled)

    def test_other_browser_cannot_change_target(self):
        session = MotionSession(Profile("can1", "can0"), {}, "/unused")
        session.ready, session.owner, session.side = True, 1, "right"
        session.base = dict.fromkeys(IDS, 0.0)
        for owner, side in ((2, "right"), (1, "left")):
            with self.assertRaises(RuntimeError):
                session.set_target(owner, side, 1, 30)
        session.set_target(1, "right", 1, 30)
        self.assertEqual(session.offsets[1], 30)

    def test_exchange_collects_out_of_order_replies_without_refresh(self):
        import struct
        bus = ClassicBus.__new__(ClassicBus)
        bus.ranges = {mid: MotorRange(12.5, 30, 10) for mid in IDS}
        bus.command, bus.drain = Mock(), Mock()
        bus.timeout, bus.socket = 0.04, Mock()
        # enabled bit, matching ID, position/velocity/torque midpoint.
        bus.socket.recv.side_effect = [struct.pack("=IB3x8s", mid+16, 8,
            bytes([0x10|mid, 127, 255, 127, 247, 255, 25, 25])) for mid in reversed(IDS)]
        states = bus.exchange(dict.fromkeys(IDS, 0), [20]*8, [1]*8)
        self.assertEqual(set(states), set(IDS))
        self.assertEqual(bus.command.call_count, 8)
        self.assertTrue(all(call.args[1] == "position" for call in bus.command.call_args_list))


if __name__ == "__main__":
    unittest.main()
