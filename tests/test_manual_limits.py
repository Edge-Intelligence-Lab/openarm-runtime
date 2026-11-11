"""Offline motor-limit regressions; never opens real hardware."""
import math
import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

from test_arm import FakeBus
from openarm_runtime.arm import ArmHold
from openarm_runtime.config import Profile
from openarm_runtime.limits import joint_bounds_deg
from openarm_runtime.panel import MotionSession
from openarm_runtime.telemetry import IDS, MotorRange


class ManualLimitTests(unittest.TestCase):
    def arm(self, bus, side='right'):
        return ArmHold(bus, lambda: False, Mock(), Mock(), manual_side=side)

    def test_full_range_targets_are_allowed_and_outside_rejected(self):
        for side in ('left', 'right'):
            bus = FakeBus()
            with patch('openarm_runtime.arm.time.sleep'), self.arm(bus, side) as arm:
                for mid, bounds in joint_bounds_deg(side).items():
                    for angle in bounds:
                        offsets = dict.fromkeys(IDS, 0.0) | {mid: angle}
                        arm.tick(offsets, 5, 0.05)
                        beyond = angle + (-0.01 if angle == bounds[0] else 0.01)
                        before = dict(bus.last_targets)
                        with self.assertRaises(ValueError):
                            arm.tick(offsets | {mid: beyond}, 5, 0.05)
                        self.assertEqual(bus.last_targets, before)
            self.assertFalse(bus.enabled)

    def test_absolute_target_uses_fresh_enable_pose(self):
        session = MotionSession(Profile('can1', 'can0'), {}, '/unused')
        session.ready, session.owner, session.side = True, 1, 'right'
        session.base = dict.fromkeys(IDS, 0.0) | {1: math.radians(150)}
        session.set_target(1, 'right', 1, -80)
        self.assertAlmostEqual(session.offsets[1], -230)
        session.set_target(1, 'right', 1, 200)
        self.assertAlmostEqual(session.offsets[1], 50)
        for invalid in (-80.1, 200.1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                session.set_target(1, 'right', 1, invalid)

    def test_no_load_speed_cannot_be_requested_by_manual_controller(self):
        bus = FakeBus()
        with patch('openarm_runtime.arm.time.sleep'), self.arm(bus) as arm:
            before = dict(bus.last_targets)
            for speed in (5.1, 312, 960, 1200):
                with self.assertRaises(ValueError):
                    arm.tick(dict.fromkeys(IDS, 0.0) | {2: 190}, speed, 0.05)
                self.assertEqual(bus.last_targets, before)
            arm.tick(dict.fromkeys(IDS, 0.0) | {2: 190}, 5, 0.05)
            self.assertAlmostEqual(bus.last_targets[2], math.radians(0.025))

    def test_manual_feedback_keeps_original_velocity_protection(self):
        bus = FakeBus()
        with patch('openarm_runtime.arm.time.sleep'), self.arm(bus) as arm:
            for mid in IDS:
                with self.assertRaisesRegex(RuntimeError, 'velocity limit'):
                    arm.validate(replace(arm.states[mid], velocity=2.01))

    def test_out_of_range_start_rejected_before_enable(self):
        bus = FakeBus()
        bus.positions[1] = math.radians(201)
        with patch('openarm_runtime.arm.time.sleep'), self.assertRaises(ValueError):
            with self.arm(bus):
                pass
        self.assertFalse(any(action == 'enable' for _, action in bus.commands))

    def test_zero_tolerance_holds_without_forcing_movement(self):
        bus = FakeBus()
        bus.positions[4] = math.radians(-0.01)
        with patch('openarm_runtime.arm.time.sleep'), self.arm(bus) as arm:
            self.assertEqual(arm.targets[4], bus.positions[4])
            arm.tick(dict.fromkeys(IDS, 0.0), 5, 0.05)
            self.assertAlmostEqual(bus.positions[4], math.radians(-0.01))

    def test_limits_cannot_bypass_firmware_or_gripper_envelope(self):
        bus = FakeBus()
        with patch('openarm_runtime.arm.time.sleep'), self.arm(bus) as arm:
            bus.ranges[1] = MotorRange(1, 30, 28)
            for offsets in (dict.fromkeys(IDS, 0.0) | {1: 100},
                            dict.fromkeys(IDS, 0.0) | {8: 31}):
                with self.assertRaises(ValueError):
                    arm.tick(offsets, 5, 0.05)
            for speed in (0, 1201, float('nan'), float('inf')):
                with self.assertRaises(ValueError):
                    arm.tick(dict.fromkeys(IDS, 0.0), speed, 0.05)

    def test_speed_owner_and_validation(self):
        session = MotionSession(Profile('can1', 'can0'), {}, '/unused')
        session.set_speed(1, 5)
        self.assertEqual(session.speed, 5)
        session.owner = 1
        with session.lock:
            with self.assertRaises(RuntimeError):
                session.set_speed(2, 4)
            session.set_speed(1, 4)
        with self.assertRaises(ValueError):
            session.set_speed(1, float('nan'))
