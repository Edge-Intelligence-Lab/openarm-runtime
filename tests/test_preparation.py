"""Fixed setup and learned-policy handoff, only synthetic plants or mocks."""
from dataclasses import replace
import math
import json
import time
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from openarm_policy import Context, Observation, SensorSample, SimulatedRobot, PolicySession, SessionConfig, ActionChunk
from openarm_policy.preparation import FixedJointPreparation


def observation(values, now=10.):
    return Observation(1, now, {"robot/left": (SensorSample(1, now, tuple((q, 0., 0.) for q in values)),)})


def recipe(**changes):
    options = dict(side="left", waypoints=[
        dict(name="swing", joints_rad=[0, -.1, 0, .02, 0, 0, 0]),
        dict(name="raise", joints_rad=[0, -.1, 0, .2, 0, 0, 0]),
        dict(name="ready", joints_rad=[.05, -.05, 0, .15, 0, 0, 0]),
        dict(name="open", joints_rad=[.05, -.05, 0, .15, 0, 0, 0], gripper_rad=.4)],
        settle_samples=2, start_reference_rad=[0]*7)
    options.update(changes)
    return FixedJointPreparation(**options)


class PreparationTests(unittest.TestCase):
    def test_startup_rejection_records_reason_without_starting_robot(self):
        robot = SimulatedRobot(('left',))
        robot.speed = .25
        recorder = Mock()
        with self.assertRaisesRegex(ValueError, 'speed exceeds'):
            PolicySession(robot, Mock(), recorder=recorder, preparation=recipe(velocity_rad_s=.5))
        self.assertFalse(robot.started)
        event = recorder.record.call_args.args[0]
        self.assertEqual(event['event'], 'ended')
        self.assertEqual(event['stop_reason'], 'preflight_rejected')
        self.assertFalse(event['hardware_started'])
        self.assertEqual(event['cleanup_errors'], [])

    def bind(self, stage):
        robot = SimulatedRobot(("left",))
        stage.bind(robot, Context(robot.spec, robot.streams, 20))
        return robot

    def test_nonzero_start_no_jump_gripper_order_and_bounded_dynamics(self):
        stage = recipe()
        self.bind(stage)
        actual = (.03, .04, .1, .03, -.05, .02, .01, .02)
        plan = stage.begin(observation(actual))
        self.assertFalse(plan["skipped_to_ready"])
        previous = actual
        previous_velocity = (0.,)*8
        names = []
        for frame in range(5000):
            step = stage.step(observation(actual, 10+frame*.05))
            if frame == 0:
                self.assertEqual(step.values, actual)
            velocity = tuple((a-b)/.05 for a, b in zip(step.values, previous))
            self.assertTrue(all(abs(v) <= limit+1e-8 for v, limit in zip(velocity, stage.speed)))
            self.assertTrue(all(abs(v-p)/.05 <= limit+1e-7 for v, p, limit in
                                zip(velocity, previous_velocity, stage.acceleration)))
            if step.waypoint != "open":
                self.assertAlmostEqual(step.values[7], .02)
            if not names or names[-1] != step.waypoint:
                names.append(step.waypoint)
            previous, previous_velocity, actual = step.values, velocity, step.values
            if step.completed:
                break
        else:
            self.fail("Preparation did not complete")
        self.assertEqual(names, ["swing", "raise", "ready", "open"])
        self.assertAlmostEqual(actual[7], .4)
        self.assertEqual(step.progress, 1)
        with self.assertRaises(RuntimeError): stage.step(observation(actual, 100))

    def test_ready_pose_skips_swing_but_still_opens_and_settles(self):
        stage = recipe()
        self.bind(stage)
        actual = (.05, -.05, 0, .15, 0, 0, 0, .02)
        self.assertTrue(stage.begin(observation(actual))["skipped_to_ready"])
        self.assertEqual(stage.step(observation(actual)).waypoint, "open")

    def test_not_ready_until_real_feedback_settles(self):
        stage = recipe(waypoints=[dict(name="small", joints_rad=[.02, 0, 0, .02, 0, 0, 0])],
                       settle_timeout_s=.1, settle_tolerance_rad=.001)
        self.bind(stage)
        actual = (0.,)*8
        stage.begin(observation(actual))
        completed = False
        with self.assertRaises(TimeoutError):
            for frame in range(200):
                completed |= stage.step(observation(actual, 10+frame*.05)).completed
        self.assertFalse(completed)

    def test_upright_arrival_accepts_small_bias_but_not_large_error(self):
        options = json.loads((Path(__file__).resolve().parents[1]/
                              "examples/upright-preparation.example.json").read_text())
        options.pop("kind")
        options["waypoints"] = options["waypoints"][:1]
        options["start_reference_rad"] = None
        self.assertAlmostEqual(options["settle_tolerance_rad"], math.radians(3))
        self.assertEqual(options["settle_timeout_s"], 20)
        for error_deg, accepted in ((2.28, True), (4.8, False)):
            stage = FixedJointPreparation(**options)
            self.bind(stage)
            actual = list(options["waypoints"][0]["joints_rad"]) + [.01]
            actual[4] += math.radians(error_deg)
            stage.begin(observation(actual))
            if accepted:
                for frame in range(100):
                    step = stage.step(observation(actual, 10+frame*.05))
                    if step.completed:
                        self.assertEqual(stage.stable, 5)
                        break
                else:
                    self.fail("Acceptable preparation bias never settled")
            else:
                with self.assertRaises(TimeoutError):
                    for frame in range(500):
                        self.assertFalse(stage.step(observation(actual, 10+frame*.05)).completed)

    def test_stale_tracking_far_start_and_timeouts_reject(self):
        stage = recipe()
        self.bind(stage)
        with self.assertRaises(ValueError): stage.begin(observation((1., 0., 0., 0., 0., 0., 0., 0.)))
        stage = recipe(); self.bind(stage); stage.begin(observation((0.,)*8))
        stale = replace(observation((0.,)*8), captured_at=10.2)
        with self.assertRaises(RuntimeError): stage.step(stale)
        with self.assertRaises(RuntimeError): stage.step(observation((.2, 0, 0, 0, 0, 0, 0, 0), 10.05))
        with self.assertRaises(TimeoutError): stage.step(observation((0.,)*8, 131))

    def tcp_stage(self, error, *, grip=0.):
        stage = recipe(waypoints=[dict(name="ready", joints_rad=[.07, 0, 0, .02, 0, 0, 0],
                                       gripper_rad=grip, tcp_tolerance=dict(
                                           position_m=.02, orientation_rad=math.radians(10), gripper_rad=.05))],
                       settle_tolerance_rad=.01, settle_samples=5, settle_timeout_s=20.)
        robot = SimulatedRobot(("left",))
        robot.preparation_pose_error = error
        stage.bind(robot, Context(robot.spec, robot.streams, 20))
        stage.begin(observation((0.,)*8))
        return stage

    def test_tcp_arrival_not_per_joint_precision_and_waits_longer_than_eight_seconds(self):
        error = Mock(return_value=(.03, 0.))
        stage = self.tcp_stage(error)
        for frame in range(500):
            if frame == 220:  # feedback reaches the task region after 11 seconds
                error.return_value = (.01, math.radians(5))
            step = stage.step(observation((0.,)*8, 10+frame*.05))
            if step.completed:
                self.assertGreaterEqual(frame, 224)
                self.assertEqual(stage.stable, 5)
                self.assertEqual(step.arrival["kind"], "tcp_fk")
                break
        else:
            self.fail("FK arrival failed despite acceptable TCP pose")

    def test_tcp_position_orientation_and_gripper_must_all_arrive(self):
        for error, grip in (((.021, 0.), 0.), ((0., math.radians(11)), 0.), ((0., 0.), .06)):
            stage = self.tcp_stage(Mock(return_value=error), grip=grip)
            with self.assertRaises(TimeoutError):
                for frame in range(500):
                    self.assertFalse(stage.step(observation((0.,)*8, 10+frame*.05)).completed)

    def test_tcp_missing_capability_and_invalid_error_reject(self):
        stage = self.tcp_stage(Mock(return_value=(float("nan"), 0.)))
        with self.assertRaises(RuntimeError):
            for frame in range(100):
                stage.step(observation((0.,)*8, 10+frame*.05))
        stage = recipe(waypoints=[dict(name="ready", joints_rad=[0]*7, tcp_tolerance=dict(
                                      position_m=.02, orientation_rad=.1, gripper_rad=.05))])
        with self.assertRaises(ValueError):
            self.bind(stage)

    def test_invalid_configuration_or_capability_fails_before_start(self):
        for change in (dict(velocity_rad_s=1), dict(settle_samples=0), dict(acceleration_rad_s2=math.nan),
                       dict(waypoints=[]), dict(waypoints=[dict(name="x", joints_rad=[0]*7, typo=True)])):
            with self.subTest(change=change), self.assertRaises(ValueError): recipe(**change)
        stage = recipe(waypoints=[dict(name="bad", joints_rad=[10]*7)])
        robot = SimulatedRobot(("left",))
        with self.assertRaises(ValueError): stage.bind(robot, Context(robot.spec, robot.streams, 20))
        self.assertFalse(robot.started)
        with self.assertRaises(ValueError):
            recipe().bind(SimulatedRobot(), Context(robot.spec, robot.streams, 20))


class Recorder:
    def __init__(self):
        self.events = []

    def record(self, event):
        self.events.append(event)


class ImmediateWorker:
    """Imitates asynchronous policy transport, returns measured-pose hold only."""
    def __init__(self, policy, context):
        self.context = context
        self.pending = None
        self.requests = []
        self.closed = False

    def ready(self, *args):
        pass

    def submit(self, obs):
        self.requests.append(obs)
        values = tuple(row[0] for row in obs.streams["robot/left"][-1].data)
        self.pending = ActionChunk(obs.sequence, obs.captured_at, .05, (values,)*20, self.context.action.names)

    def result(self):
        result, self.pending = self.pending, None
        return result

    def close(self):
        self.closed = True


class PreparationSessionTests(unittest.TestCase):
    def small_stage(self):
        return recipe(waypoints=[dict(name="ready", joints_rad=[.01, 0, 0, .02, 0, 0, 0]),
                                 dict(name="open", joints_rad=[.01, 0, 0, .02, 0, 0, 0], gripper_rad=.02)],
                      velocity_rad_s=.5, acceleration_rad_s2=5., gripper_acceleration_rad_s2=5.,
                      settle_samples=1)

    def test_setup_then_inference_once_started_no_enable_gap_and_separate_records(self):
        robot = SimulatedRobot(("left",))
        worker = ImmediateWorker(None, Context(robot.spec, robot.streams, 20))
        recorder = Recorder()
        session = PolicySession(robot, None, preparation=self.small_stage(), recorder=recorder)
        original = robot.submit
        task_commands = []

        def submit(values, deadline):
            original(values, deadline)
            if session.state == "preparing":
                self.assertFalse(worker.requests)
            else:
                task_commands.append(values)
                if len(task_commands) == 4:
                    session.stop()

        with patch.object(robot, "start", wraps=robot.start) as start, \
                patch.object(robot, "submit", side_effect=submit), \
                patch("openarm_policy.session.InferenceWorker", return_value=worker):
            session.run(3)
        self.assertEqual(start.call_count, 1)
        self.assertFalse(robot.started)
        self.assertTrue(worker.closed)
        events = [e["event"] for e in recorder.events]
        self.assertLess(events.index("preparation_completed"), events.index("task_started"))
        self.assertIn("handoff_hold", events)
        self.assertTrue(worker.requests)
        first_state = worker.requests[0].streams["robot/left"][-1].data
        self.assertAlmostEqual(first_state[0][0], .01)
        self.assertEqual(len(worker.requests[0].streams["robot/left"]), 1)
        self.assertEqual(session.state, "stopped")

    def test_setup_failure_never_submits_model_job_and_closes_robot(self):
        robot = SimulatedRobot(("left",))
        stage = self.small_stage()
        worker = ImmediateWorker(None, Context(robot.spec, robot.streams, 20))
        session = PolicySession(robot, None, preparation=stage)
        with patch.object(stage, "step", side_effect=RuntimeError("setup failure")), \
                patch("openarm_policy.session.InferenceWorker", return_value=worker):
            with self.assertRaises(RuntimeError): session.run(2)
        self.assertFalse(worker.requests)
        self.assertFalse(robot.started)
        self.assertTrue(worker.closed)

    def test_cancel_during_setup_never_starts_task(self):
        robot = SimulatedRobot(("left",))
        worker = ImmediateWorker(None, Context(robot.spec, robot.streams, 20))
        session = PolicySession(robot, None, preparation=self.small_stage())
        original = robot.submit

        def submit(values, deadline):
            original(values, deadline)
            session.stop()

        with patch.object(robot, "submit", side_effect=submit), \
                patch("openarm_policy.session.InferenceWorker", return_value=worker):
            session.run(2)
        self.assertFalse(worker.requests)
        self.assertFalse(robot.started)
        self.assertEqual(session.state, "stopped")

    def test_phase_filtered_replay_excludes_setup_and_supports_old_episodes(self):
        from openarm_policy.recording import EpisodeRecorder, replay_observations
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"episode"
            with EpisodeRecorder(path) as recorder:
                recorder.record(dict(event="observation", phase="preparation", observation=observation((0.,)*8, 10)))
                recorder.record(dict(event="observation", phase="task", observation=observation((0.,)*8, 11)))
                recorder.record(dict(event="observation", observation=observation((0.,)*8, 12)))
            self.assertEqual([o.captured_at for o in replay_observations(path, phase="task")], [11, 12])
            self.assertEqual([o.captured_at for o in replay_observations(path, phase="preparation")], [10])
            self.assertEqual(len(list(replay_observations(path))), 3)


if __name__ == "__main__":
    unittest.main()
