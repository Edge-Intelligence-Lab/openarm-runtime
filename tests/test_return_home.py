"""Natural-hanging return: synthetic feedback only, no devices or camera inputs."""
from dataclasses import replace
from contextlib import redirect_stderr
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from openarm_policy import Context, Observation, SensorSample, SimulatedRobot, PolicySession, ActionChunk
from openarm_policy.preparation import FixedJointPreparation, reverse_preparation


def prep():
    return FixedJointPreparation('left', [
        dict(name='swing', joints_rad=[0, -.01, 0, .02, 0, 0, 0]),
        dict(name='ready', joints_rad=[.02, -.01, 0, .03, 0, 0, 0]),
        dict(name='open', joints_rad=[.02, -.01, 0, .03, 0, 0, 0], gripper_rad=.4)],
        velocity_rad_s=.5, acceleration_rad_s2=5., settle_samples=1)


def options(**changes):
    return dict(kind='reverse_preparation_v1', home_joints_rad=[0, 0, 0, .02, 0, 0, 0], **changes)


def obs(values, now=10.):
    return Observation(1, now, {'robot/left': (SensorSample(1, now, tuple((q, 0., 0.) for q in values)),)})


class Clock:
    def __init__(self): self.now = 100.
    def wait(self, duration): self.now += max(.001, duration); return False


class Worker:
    def __init__(self, policy, context): self.context, self.pending, self.calls = context, None, 0
    def ready(self, *args): pass
    def submit(self, observation):
        self.calls += 1
        values = tuple(v[0] for v in observation.streams['robot/left'][-1].data)
        self.pending = ActionChunk(observation.sequence, observation.captured_at, .05,
                                  (values,)*20, self.context.action.names)
    def result(self):
        result, self.pending = self.pending, None
        return result
    def close(self): pass


class ReturnRouteTests(unittest.TestCase):
    def bind(self, stage):
        robot = SimulatedRobot(('left',))
        stage.bind(robot, Context(robot.spec, robot.streams, 20))
        return robot

    def test_reverse_joint_route_and_gripper_commands_never_replayed(self):
        stage = reverse_preparation(prep(), options())
        self.bind(stage)
        self.assertEqual([w['joints_rad'] for w in stage.waypoints], [
            (.02, -.01, 0, .03, 0, 0, 0), (0, -.01, 0, .02, 0, 0, 0), (0, 0, 0, .02, 0, 0, 0)])
        self.assertTrue(all(w['gripper_rad'] is None for w in stage.waypoints))
        actual = (.03, -.01, 0., .035, 0., 0., 0., .1)
        stage.begin(obs(actual))
        for frame in range(2000):
            step = stage.step(obs(actual, 10+frame*.05))
            self.assertAlmostEqual(step.values[7], .1)
            self.assertLessEqual(max(abs(a-b) for a, b in zip(step.values, actual)), .02500001)
            actual = step.values
            if step.completed: break
        else: self.fail('return did not finish')
        self.assertEqual(actual[:7], tuple(options()['home_joints_rad']))

    def test_explicit_home_and_bounded_route_required_before_start(self):
        for bad in (None, {}, dict(kind='unknown'), dict(options(), home_joints_rad=[0]*6),
                    dict(options(), home_joints_rad=[float('nan')]*7), options(velocity_rad_s=.6)):
            with self.assertRaises(ValueError): reverse_preparation(prep(), bad)
        stage = reverse_preparation(prep(), dict(options(), home_joints_rad=[10]*7))
        with self.assertRaises(ValueError): self.bind(stage)

    def test_commanded_reference_handoff_never_snaps_to_measured(self):
        stage = reverse_preparation(prep(), options())
        self.bind(stage)
        measured = (.02, -.01, 0., .03, 0., 0., 0., .1)
        reference = (.025, -.01, 0., .03, 0., 0., 0., .11)
        stage.begin(obs(measured), commanded_joints=reference)
        self.assertEqual(stage.step(obs(measured)).values, reference)

    def test_stale_feedback_and_bad_tracking_still_reject_return(self):
        stage = reverse_preparation(prep(), options()); self.bind(stage)
        with self.assertRaises(RuntimeError): stage.begin(replace(obs((0.,)*8), captured_at=11))
        with self.assertRaises(RuntimeError): stage.begin(obs((0.,)*8), commanded_joints=(.2, 0, 0, 0, 0, 0, 0, 0))


class ReturnSessionTests(unittest.TestCase):
    def test_unlimited_session_passes_old_web_limit_until_operator_stops(self):
        robot, session, events = self.session()
        clock = Clock()
        original = robot.submit
        started = clock.now
        def submit(values, deadline):
            original(values, deadline)
            if clock.now - started > 31:
                session.stop()
        with patch('time.monotonic', side_effect=lambda: clock.now), \
             patch.object(session.cancel, 'wait', side_effect=clock.wait), \
             patch('openarm_policy.session.InferenceWorker', Worker), \
             patch.object(robot, 'submit', side_effect=submit):
            session.run(None)
        self.assertGreater(clock.now - started, 31)
        self.assertFalse(robot.started)
        names = [e['event'] for e in events]
        self.assertNotIn('duration_reached', names)
        self.assertNotIn('return_home_started', names)
        self.assertEqual(events[-1]['stop_reason'], 'operator_stop')

    def test_cli_manual_stop_flag_reaches_session_without_default_ten_seconds(self):
        from openarm_policy.cli import run
        args = SimpleNamespace(profile=None, robot='simulate', sides=['left'], home_only=False,
            auto_return_home=False, until_stopped=True, record=None,
            policy='openarm_policy.adapters:HoldPolicy', duration=10)
        with patch('openarm_policy.cli.PolicySession') as factory:
            run(args)
        factory.return_value.run.assert_called_once_with(None, home_only=False)
        args.auto_return_home = True
        with patch('openarm_policy.cli.PolicySession') as factory:
            with self.assertRaisesRegex(ValueError, 'Manual-stop'):
                run(args)
        factory.assert_not_called()

    def test_cli_parser_keeps_manual_and_explicit_timed_modes_exclusive(self):
        from openarm_runtime.__main__ import main
        with patch('sys.argv', ['openarm-runtime', 'policy', '--robot', 'simulate', '--until-stopped']), \
             patch('openarm_policy.cli.run') as run:
            main()
        self.assertTrue(run.call_args.args[0].until_stopped)
        with patch('sys.argv', ['openarm-runtime', 'policy', '--robot', 'simulate',
                                '--until-stopped', '--duration', '2']), \
             patch('openarm_policy.cli.run') as run, redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                main()
        self.assertEqual(error.exception.code, 2)
        run.assert_not_called()
    def session(self, *, preparation=None):
        robot = SimulatedRobot(('left',)); robot.values = (.02, -.01, 0., .03, 0., 0., 0., .1)
        events = []; recorder = Mock(); recorder.record.side_effect = events.append
        session = PolicySession(robot, None, preparation=preparation,
                                return_home=reverse_preparation(prep(), options()), recorder=recorder)
        return robot, session, events

    def execute(self, session, *, home_only=False):
        clock = Clock()
        with patch('time.monotonic', side_effect=lambda: clock.now), \
             patch.object(session.cancel, 'wait', side_effect=clock.wait), \
             patch('openarm_policy.session.InferenceWorker', Worker):
            session.run(None if home_only else .3, home_only=home_only)

    def test_normal_duration_returns_before_disable_without_more_inference(self):
        robot, session, events = self.session(preparation=prep())
        original = robot.submit
        def submit(values, deadline):
            original(values, deadline)
            if session.state == 'returning_home':
                self.assertEqual(events[-1]['phase'], 'return_home')
        with patch.object(robot, 'submit', side_effect=submit), patch.object(robot, 'start', wraps=robot.start) as start:
            self.execute(session)
        self.assertEqual(start.call_count, 1)
        names = [e['event'] for e in events]
        self.assertLess(names.index('duration_reached'), names.index('return_home_started'))
        self.assertLess(names.index('return_home_completed'), names.index('ended'))
        self.assertEqual(events[-1]['stop_reason'], 'duration_reached')
        self.assertFalse(robot.started)
        self.assertEqual(robot.values[:7], tuple(options()['home_joints_rad']))
        self.assertAlmostEqual(robot.values[7], .4)  # hold task's final grip, not reverse opening
        after = events[names.index('return_home_started'):]
        self.assertFalse(any(e['event'] in ('chunk', 'command') for e in after))

    def test_home_only_never_constructs_inference_worker(self):
        robot, session, events = self.session()
        clock = Clock()
        with patch('time.monotonic', side_effect=lambda: clock.now), \
             patch.object(session.cancel, 'wait', side_effect=clock.wait), \
             patch('openarm_policy.session.InferenceWorker', side_effect=AssertionError('inference')):
            session.run(home_only=True)
        self.assertEqual(events[-1]['stop_reason'], 'home_completed')
        self.assertNotIn('task_started', [e['event'] for e in events])
        self.assertFalse(robot.started)

    def test_operator_stop_never_initiates_return(self):
        robot, session, events = self.session()
        original = robot.submit
        def submit(values, deadline): original(values, deadline); session.stop()
        with patch.object(robot, 'submit', side_effect=submit): self.execute(session)
        self.assertNotIn('return_home_started', [e['event'] for e in events])
        self.assertEqual(events[-1]['stop_reason'], 'operator_stop')

    def test_error_never_initiates_return(self):
        robot, session, events = self.session()
        with patch.object(robot, 'submit', side_effect=ValueError('bad feedback')):
            with self.assertRaises(ValueError): self.execute(session)
        self.assertNotIn('return_home_started', [e['event'] for e in events])
        self.assertFalse(robot.started)
        self.assertEqual(events[-1]['stop_reason'], 'error')

    def test_stop_during_return_cancels_without_finishing_or_reenabling(self):
        robot, session, events = self.session()
        original = robot.submit
        def submit(values, deadline): original(values, deadline); session.stop()
        with patch.object(robot, 'submit', side_effect=submit): self.execute(session, home_only=True)
        self.assertNotIn('return_home_completed', [e['event'] for e in events])
        self.assertEqual(events[-1]['stop_reason'], 'operator_stop')
        self.assertFalse(robot.started)

    def test_home_only_cli_ignores_model_and_live_sources(self):
        from openarm_policy.cli import run
        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory)/'profile.json'
            data = dict(preparation=dict(kind='fixed_joint_waypoints_v1', side='left',
                waypoints=[dict(name='ready', joints_rad=[0]*7)]), return_home=options(),
                video_http=dict(url='http://127.0.0.1:5003', names=['head']), rerun_recording=True)
            profile.write_text(json.dumps(data))
            args = SimpleNamespace(profile=profile, robot='simulate', sides=['left'], home_only=True,
                auto_return_home=False, record=None, policy='does.not.exist:Predictor', duration=1)
            with patch('openarm_policy.cli.PolicySession') as factory, \
                 patch('openarm_policy.cli.ImportedPolicy', side_effect=AssertionError('model')), \
                 patch('openarm_policy.cli.HTTPVideoObservation', side_effect=AssertionError('camera')):
                run(args)
            self.assertIsNone(factory.call_args.args[1])
            self.assertEqual(factory.call_args.kwargs['sources'], [])
            self.assertIsNone(factory.call_args.kwargs['preparation'])
            factory.return_value.run.assert_called_once_with(None, home_only=True)
