"""Offline manual trajectory and supervision tests; no physical transport."""
import math
import random
import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

from test_arm import FakeBus
from openarm_runtime.arm import ArmHold, FOLLOWER_KP, FOLLOWER_KD
from openarm_runtime.motion import SmoothMotion
from openarm_runtime.telemetry import IDS


class MotionTests(unittest.TestCase):
    def motion(self, base=None):
        motion = SmoothMotion(dict.fromkeys(IDS, (-90, 90)))
        motion.initialize(base or dict.fromkeys(IDS, 0.0))
        return motion

    def test_step_target_has_bounded_velocity_acceleration_and_settles(self):
        m = self.motion()
        goals = dict.fromkeys(IDS, math.radians(20))
        last_q = dict(m.positions)
        last_v = dict(m.velocities)
        for _ in range(180):
            q = m.step(goals, 5, .05)
            for mid in IDS:
                v = (q[mid]-last_q[mid])/.05
                self.assertLessEqual(abs(v), math.radians(5)+1e-10)
                self.assertLessEqual(abs(v-last_v[mid]), math.radians(10)*.05+1e-10)
                self.assertLessEqual(q[mid], goals[mid]+1e-10)
                last_v[mid] = v
            last_q = q
        self.assertAlmostEqual(q[1], goals[1], places=6)
        self.assertLess(abs(m.velocities[1]), 1e-6)

    def test_reversals_and_lower_speed_brake_without_position_jump(self):
        m = self.motion()
        random.seed(17)
        for i in range(2000):
            if i % 10 == 0:
                goal = math.radians(random.choice((-90, -10, 0, 10, 90)))
            previous_q, previous_v = dict(m.positions), dict(m.velocities)
            q = m.step(dict.fromkeys(IDS, goal), 5 if i % 100 < 50 else .1, .05)
            for mid in IDS:
                self.assertLessEqual(abs(q[mid]-previous_q[mid]), math.radians(5)*.05+1e-10)
                self.assertLessEqual(abs(m.velocities[mid]-previous_v[mid]), math.radians(10)*.05+1e-10)
                self.assertTrue(-math.pi/2 <= q[mid] <= math.pi/2)

    def test_braking_at_both_joint_boundaries(self):
        m = self.motion(dict.fromkeys(IDS, math.radians(89)))
        for angle in (90, -90, 90):
            for _ in range(850):
                q = m.step(dict.fromkeys(IDS, math.radians(angle)), 5, .05)
                self.assertTrue(all(-math.pi/2-1e-10 <= v <= math.pi/2+1e-10 for v in q.values()))
            self.assertAlmostEqual(q[1], math.radians(angle), places=6)

    def test_nonstationary_manual_arm_is_not_enabled(self):
        bus = FakeBus()
        original = bus.state
        bus.state = lambda mid: replace(original(mid), velocity=.2)
        with patch('openarm_runtime.arm.time.sleep'), self.assertRaisesRegex(RuntimeError, 'stationary'):
            with ArmHold(bus, lambda: False, Mock(), Mock(), manual_side='right'):
                pass
        self.assertFalse(any(action=='enable' for _, action in bus.commands))

    def test_initial_pose_and_nonfinite_target_rejected(self):
        m = self.motion()
        for value in (float('nan'), float('inf'), math.radians(91)):
            with self.assertRaises(ValueError):
                m.step(dict.fromkeys(IDS, value), 5, .05)
        with self.assertRaises(ValueError):
            m.initialize(dict.fromkeys(IDS, math.radians(91)))

    def test_torque_uses_motor_rating_not_firmware_encoding(self):
        bus = FakeBus()
        m = self.motion()
        states = {mid: bus.state(mid) for mid in IDS}
        for mid, torque in ((1, 20.1), (3, 9.1), (5, 3.1)):
            with self.assertRaisesRegex(RuntimeError, 'measured torque'):
                m.supervise(states | {mid: replace(states[mid], torque=torque)},
                            dict.fromkeys(IDS, 0.), bus.ranges, FOLLOWER_KP, FOLLOWER_KD, 0)
        # At official gains, 4 deg J3 error plus 5 Nm feedforward exceeds its 9 Nm rating.
        with self.assertRaisesRegex(RuntimeError, 'predicted torque'):
            m.supervise(states, dict.fromkeys(IDS, 0.) | {3: math.radians(4)},
                        bus.ranges, FOLLOWER_KP, FOLLOWER_KD, 0, feedforward={3: 5.0})

    def test_tracking_and_sustained_stall_are_distinct_and_stop(self):
        bus, m = FakeBus(), self.motion()
        states = {mid: bus.state(mid) for mid in IDS}
        targets = dict.fromkeys(IDS, 0.) | {1: math.radians(6)}
        with self.assertRaisesRegex(RuntimeError, 'tracking error'):
            m.supervise(states, targets, bus.ranges, FOLLOWER_KP, FOLLOWER_KD, 0)
        targets[1] = math.radians(3)
        m.supervise(states, targets, bus.ranges, FOLLOWER_KP, FOLLOWER_KD, 10, after=True)
        m.supervise(states, targets, bus.ranges, FOLLOWER_KP, FOLLOWER_KD, 10.9, after=True)
        with self.assertRaisesRegex(RuntimeError, 'stalled'):
            m.supervise(states, targets, bus.ranges, FOLLOWER_KP, FOLLOWER_KD, 11.1, after=True)
        states[1] = replace(states[1], position=targets[1])
        m.supervise(states, targets, bus.ranges, FOLLOWER_KP, FOLLOWER_KD, 12, after=True)
        self.assertIsNone(m.stall_since[1])

    def test_trace_contains_fault_frame_and_is_written_after_disable(self):
        bus = FakeBus()
        events = []
        def record(event):
            if event['event'] == 'motion_trace':
                self.assertFalse(bus.enabled)
            events.append(event)
        with patch('openarm_runtime.arm.time.sleep'):
            with self.assertRaisesRegex(RuntimeError, 'status=14'):
                with ArmHold(bus, lambda: False, record, Mock(), manual_side='right') as arm:
                    original = bus.exchange
                    def fail(*args):
                        states = original(*args)
                        states[2] = replace(states[2], status=14)
                        return states
                    bus.exchange = fail
                    arm.tick(dict.fromkeys(IDS, 0.), 5, .05)
        trace = next(e for e in events if e['event']=='motion_trace')
        self.assertEqual(trace['samples'][-1]['feedback'][2]['status'], 14)
        self.assertIn('status=14', trace['samples'][-1]['error'])
        self.assertFalse(bus.enabled)
