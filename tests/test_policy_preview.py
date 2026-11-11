"""Real simulated sessions and record/UI boundaries; never access physical devices."""
from dataclasses import asdict
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from openarm_platform.config import load_config
from openarm_platform.policy import PolicyPreview, PolicyTrace
from openarm_platform.policy_view import chart_data, joint_rows
from openarm_platform.proxy import routes
from openarm_policy.contracts import Context
from openarm_policy.robots import joint_spec, state_specs


def context():
    return asdict(Context(joint_spec(('right',)), state_specs(('right',)), 20))


def events():
    return [dict(event='context', time=100., context=context()),
            dict(event='observation', time=101., observation=dict(captured_at=101., streams={
                'robot/right': [dict(data=[[.1, 0, 0]]*8, received_at=101., sequence=1)]})),
            dict(event='command', time=101.1, values=[.2]*8),
            dict(event='chunk', time=101.1, latency_s=.025),
            dict(event='ended', time=102., state='stopped', error='', cleanup_errors=[])]


class PreviewTests(unittest.TestCase):
    def test_internal_page_is_not_a_hardware_service_or_proxy(self):
        config = load_config()
        page = next(p for p in config.pages if p.id == 'policy')
        self.assertEqual(page.path, '/policy')
        self.assertIsNone(page.port)
        self.assertNotIn(page, config.services)
        self.assertNotIn('/apps/policy', routes(config))

    def test_recorded_feedback_never_substitutes_submitted_target(self):
        trace = PolicyTrace()
        for event in events():
            trace.record(event)
        snapshot = trace.snapshot()
        target, feedback = chart_data(snapshot, 'right.joint1')
        self.assertEqual(len(target), 1)
        self.assertAlmostEqual(target[0][0], 1.1)
        self.assertAlmostEqual(target[0][1], math.degrees(.2))
        self.assertEqual(feedback, [[1., math.degrees(.1)]])
        self.assertEqual(snapshot['latency_ms'], 25)
        self.assertEqual(joint_rows(snapshot, 1.)[0]['target'], '—')
        row = joint_rows(snapshot, 2.)[0]
        self.assertEqual(row['feedback'], '+5.73')
        self.assertEqual(row['target'], '+11.46')
        self.assertEqual(row['error'], '+5.73')

    def test_nonposition_actions_are_not_mislabelled_as_joint_angles(self):
        event = events()[0]
        event['context']['action']['kind'] = 'joint_torque_nm'
        with self.assertRaises(ValueError):
            PolicyTrace().record(event)

    def test_replay_does_not_open_devices_or_inference(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episode = root/'trial'
            episode.mkdir()
            (episode/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events()))
            preview = PolicyPreview(root)
            with patch('openarm_policy.robots.ClassicRobot', side_effect=AssertionError('physical')), \
                 patch('openarm_policy.inference.InferenceWorker', side_effect=AssertionError('inference')):
                preview.load('trial')
            self.assertEqual(preview.episodes(), ['trial'])
            self.assertEqual(preview.snapshot()['commands'], 1)
            self.assertEqual(preview.snapshot()['state'], 'stopped')
            self.assertIsNone(preview.worker)
            preview.close()

    def test_malformed_and_escaped_records_keep_previous_view(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            preview = PolicyPreview(root)
            (root/'bad').mkdir()
            (root/'bad/events.jsonl').write_text('{bad json')
            for name in ('../outside', '/tmp', 'bad'):
                with self.subTest(name=name), self.assertRaises(ValueError):
                    preview.load(name)
                self.assertFalse(preview.loading)
                self.assertEqual(preview.snapshot()['source'], '等待策略')
            (root/'alias').symlink_to('/tmp', target_is_directory=True)
            self.assertNotIn('alias', preview.episodes())

    def test_array_reference_cannot_escape_episode(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'trial').mkdir()
            data = events()
            data[1]['observation']['streams']['robot/right'][0]['data'] = {'array': '../outside.npy'}
            (root/'trial/events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in data))
            with self.assertRaisesRegex(ValueError, '数组路径'):
                PolicyPreview(root).load('trial')

    def test_incomplete_episode_is_labelled_and_samples_are_bounded(self):
        trace = PolicyTrace()
        trace.record(events()[0])
        for i in range(2100):
            trace.record(dict(event='command', time=100+i/20, values=[0]*8))
        self.assertEqual(len(trace.snapshot()['points']), 2000)
        self.assertEqual(trace.snapshot()['commands'], 2100)
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory)/'partial'
            p.mkdir()
            (p/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events()[:-1]))
            preview = PolicyPreview(directory)
            preview.load('partial')
            self.assertEqual(preview.snapshot()['state'], 'incomplete')

    def test_real_simulation_emits_chunks_and_stops_without_physical_backend(self):
        with tempfile.TemporaryDirectory() as directory:
            preview = PolicyPreview(directory)
            try:
                with patch('openarm_policy.robots.ClassicRobot', side_effect=AssertionError('physical')):
                    preview.start('sine', 1)
                    with self.assertRaises(RuntimeError):
                        preview.start('hold', 1)
                    preview.worker.join(timeout=8)
                snapshot = preview.snapshot()
                self.assertFalse(snapshot['busy'])
                self.assertEqual(snapshot['state'], 'stopped', snapshot['error'])
                self.assertGreater(snapshot['chunks'], 0)
                self.assertGreater(snapshot['commands'], 0)
                self.assertTrue(chart_data(snapshot, 'left.joint1')[0])
            finally:
                preview.close()

    def test_shutdown_cancels_a_running_simulation(self):
        with tempfile.TemporaryDirectory() as directory:
            preview = PolicyPreview(directory)
            preview.start('hold', 60)
            preview.close()
            self.assertFalse(preview.worker.is_alive())
            self.assertTrue(preview.session.cancel.is_set())
            with self.assertRaises(RuntimeError):
                preview.start()

    def test_manual_gravity_combines_with_existing_rated_effort_guard(self):
        from test_arm import FakeBus
        from openarm_runtime.arm import ArmHold
        from openarm_runtime.telemetry import IDS
        gravity = Mock()
        gravity.metadata.return_value = {}
        gravity.last = {}
        gravity.update.return_value = dict.fromkeys(IDS, 0.) | {5: 3.1}
        bus = FakeBus()
        with patch('openarm_runtime.arm.time.sleep'), self.assertRaisesRegex(RuntimeError, 'predicted torque'):
            with ArmHold(bus, lambda: False, Mock(), Mock(), manual_side='right', gravity=gravity):
                pass
        self.assertFalse(bus.enabled)
        gravity.preflight.assert_called_once()


@unittest.skipUnless(importlib.util.find_spec('nicegui'), 'Requires platform extra')
class PreviewGuiTests(unittest.TestCase):
    def test_view_loads_empty_and_renders_trace_with_separate_target_feedback(self):
        from nicegui import Client, ui
        from openarm_platform.policy_view import PolicyView
        with tempfile.TemporaryDirectory() as directory:
            preview = PolicyPreview(directory)
            client = Client(ui.page('/policy-test'))
            try:
                with client:
                    view = PolicyView(preview)
                    for event in events():
                        preview.trace.record(event)
                    view.render()
                    self.assertEqual(view.joint.value, 'right.joint1')
                    self.assertEqual(len(view.table.rows), 8)
                    self.assertNotEqual(view.chart.options['series'][0]['data'], view.chart.options['series'][1]['data'])
                    self.assertTrue(view.start_button.enabled)
                    self.assertFalse(view.stop_button.enabled)
            finally:
                client.delete()
                preview.close()
