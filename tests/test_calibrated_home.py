"""Existing motor-zero semantics and home-only lifecycle; no physical devices."""
from dataclasses import replace
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from openarm_policy import Context, Observation, PolicySession, SensorSample, SimulatedRobot
from openarm_policy.calibrated_home import CalibratedZeroReturn


def observation(values, now=100., received=None):
    received = now if received is None else received
    return Observation(1, now, {'robot/'+s: (SensorSample(1, received,
        tuple((q, 0., 0.) for q in values[i*8:i*8+8])),)
        for i, s in enumerate(('left', 'right'))})


class Clock:
    def __init__(self): self.now = 100.
    def wait(self, dt): self.now += max(.001, dt); return False


class CalibratedHomeTests(unittest.TestCase):
    initial = (.3, -.4, .4, .5, .2, -.2, .3, .4,
               -.2, .3, -.3, .4, -.2, .2, -.3, -.4)

    def bound(self):
        robot = SimulatedRobot(('left', 'right'))
        stage = CalibratedZeroReturn()
        stage.bind(robot, Context(robot.spec, robot.streams, 20))
        return robot, stage

    def test_dual_arm_zero_holds_grippers_and_reuses_existing_speed(self):
        robot, stage = self.bound()
        robot.values = self.initial
        events = []; recorder = Mock(); recorder.record.side_effect = events.append
        session = PolicySession(robot, None, return_home=stage, recorder=recorder)
        clock = Clock()
        with patch('time.monotonic', side_effect=lambda: clock.now), \
             patch.object(session.cancel, 'wait', side_effect=clock.wait), \
             patch('openarm_policy.session.InferenceWorker', side_effect=AssertionError('model')):
            session.run(home_only=True)
        self.assertFalse(robot.started)
        self.assertEqual(robot.values, (0.,)*7+(.4,)+(0.,)*7+(-.4,))
        previous = self.initial
        for _, commands in robot.commands:
            self.assertEqual((commands[7], commands[15]), (.4, -.4))
            for i in (*range(7), *range(8, 15)):
                self.assertLessEqual(abs(commands[i]-previous[i]), math.radians(5)/20+1e-9)
            previous = commands
        self.assertIn('return_home_completed', [e['event'] for e in events])
        self.assertEqual(events[-1]['stop_reason'], 'home_completed')
        self.assertEqual(events[-1]['cleanup_errors'], [])

    def test_missing_stale_and_nonfinite_feedback_are_rejected(self):
        for bad in (replace(observation(self.initial), streams={}),
                    observation(self.initial, received=99.),
                    observation((float('nan'),)+self.initial[1:])):
            robot, stage = self.bound()
            with self.assertRaises((RuntimeError, ValueError)):
                stage.begin(bad)
            self.assertFalse(robot.started)

    def test_single_arm_and_cartesian_contracts_cannot_be_used(self):
        robot = SimulatedRobot(('left',))
        with self.assertRaises(ValueError):
            CalibratedZeroReturn().bind(robot, Context(robot.spec, robot.streams, 20))

    def test_zero_stop_quantization_bounds_commands_without_rewriting_feedback(self):
        robot, stage = self.bound()
        values = list(self.initial); values[3] = values[11] = -.0001907377737087046
        obs = observation(values)
        stage.begin(obs)
        commands = stage.step(observation(values, now=100.05)).values
        self.assertEqual((commands[3], commands[11]), (0., 0.))
        self.assertEqual(obs.streams['robot/left'][-1].data[3][0], values[3])
        robot.spec.validate(commands)

    def test_tracking_failure_prevents_further_motion(self):
        _, stage = self.bound(); stage.begin(observation(self.initial))
        changed = list(self.initial); changed[0] -= .2
        with self.assertRaisesRegex(RuntimeError, 'tracking'):
            stage.step(observation(changed, now=100.05))

    def test_arrival_timeout_is_not_removed_by_manual_stop_policy_mode(self):
        _, stage = self.bound(); stage.begin(observation(self.initial))
        values = self.initial
        for frame in range(1, 600):
            now = 100.+frame*.05
            if now >= stage.home.started+stage.home.duration:
                # Within tracking protection, but outside the established 5deg
                # arrival tolerance: must time out, not claim home completion.
                values = (.09,)+(0.,)*6+(.4,)+(0.,)*7+(-.4,)
            try:
                step = stage.step(observation(values, now=now))
            except RuntimeError as exc:
                self.assertIn('回零未收敛', str(exc))
                self.assertGreater(now-stage.home.started, stage.home.duration+stage.home.settle_timeout_s)
                break
            self.assertFalse(step.completed)
            values = step.values
        else:
            self.fail('Unsettled return did not time out')

    def test_failure_on_either_arm_is_cleaned_up_without_completed_event(self):
        robot, stage = self.bound(); robot.values = self.initial
        events = []; recorder = Mock(); recorder.record.side_effect = events.append
        session = PolicySession(robot, None, return_home=stage, recorder=recorder)
        with patch.object(robot, 'submit', side_effect=RuntimeError('right feedback fault')), \
             patch('openarm_policy.session.InferenceWorker', side_effect=AssertionError('model')):
            with self.assertRaisesRegex(RuntimeError, 'right feedback fault'):
                session.run(home_only=True)
        self.assertFalse(robot.started)
        self.assertNotIn('return_home_completed', [e['event'] for e in events])
        self.assertEqual(events[-1]['stop_reason'], 'error')

    def test_both_arm_preflight_still_precedes_any_enable(self):
        from openarm_policy.robots import ClassicRobot
        from openarm_runtime.config import Profile
        profile = Profile(left='can1', right='can0', mapping_verified=True)
        buses = {'can1': Mock(), 'can0': Mock()}
        buses['can0'].preflight.side_effect = RuntimeError('right bus failed')
        robot = ClassicRobot(profile, ('left', 'right'))
        with patch('openarm_runtime.transport.ClassicBus', side_effect=lambda interface, timeout: buses[interface]), \
             patch('openarm_runtime.config.bus_lease') as lease, \
             patch('openarm_runtime.control.check_modes'), \
             patch('openarm_runtime.gravity.prepare_gravity'), \
             patch('openarm_runtime.arm.ArmHold') as hold:
            lease.return_value.__enter__.return_value = None
            with self.assertRaises(RuntimeError): robot.start()
        hold.assert_not_called()
        self.assertFalse(any(worker.is_alive() for worker in robot.workers))

    def test_operator_stop_never_restarts_or_completes_return(self):
        robot, stage = self.bound(); robot.values = self.initial
        recorder = Mock()
        session = PolicySession(robot, None, return_home=stage, recorder=recorder)
        original = robot.submit
        def submit(values, deadline):
            original(values, deadline); session.stop()
        with patch.object(robot, 'submit', side_effect=submit), \
             patch('openarm_policy.session.InferenceWorker', side_effect=AssertionError('model')):
            session.run(home_only=True)
        self.assertFalse(robot.started)
        events = [c.args[0] for c in recorder.record.call_args_list]
        self.assertNotIn('return_home_completed', [e['event'] for e in events])
        self.assertEqual(events[-1]['stop_reason'], 'operator_stop')

    def test_cli_has_no_model_profile_and_no_inference_or_camera_construction(self):
        from openarm_policy.cli import run
        args = SimpleNamespace(profile=None, robot='simulate', sides=['left', 'right'],
            calibrated_zero=True, home_only=False, auto_return_home=False, record=None,
            until_stopped=False, policy='does.not.exist:Predictor', duration=10)
        with patch('openarm_policy.cli.PolicySession') as factory, \
             patch('openarm_policy.cli.ImportedPolicy', side_effect=AssertionError('model')), \
             patch('openarm_policy.cli.HTTPVideoObservation', side_effect=AssertionError('camera')):
            run(args)
        self.assertIsNone(factory.call_args.args[1])
        self.assertIsInstance(factory.call_args.kwargs['return_home'], CalibratedZeroReturn)
        factory.return_value.run.assert_called_once_with(None, home_only=True)
        args.sides = ['left']
        with self.assertRaises(ValueError): run(args)
