"""Model deployment lifecycle tests; no cameras, CAN SDK or real robot."""
import json
import importlib.util
import os
from pathlib import Path
import tempfile
import time
import socket
import threading
import unittest
from unittest.mock import Mock, patch

from openarm_platform.model_workspace import ModelWorkspace, catalogue
from openarm_platform.config import Page
from openarm_platform.services import ServiceManager


class Process:
    def __init__(self):
        self.code = None
        self.terminated = self.killed = False

    def poll(self):
        return self.code

    def terminate(self):
        self.terminated = True

    def kill(self):
        self.killed = True
        self.code = -9

    def wait(self, timeout=None):
        self.code = 0 if self.code is None else self.code
        return self.code


class ModelWorkspaceTests(unittest.TestCase):
    def calibrated_machine(self):
        self.write('robot.json', dict(format='classic', bitrate=1000000,
                                     left='can1', right='can0', mapping_verified=True, rate_hz=20))
        self.item.update(robot='classic', robot_config='robot.json')
        self.write_catalogue()
        self.workspace.refresh()

    def test_calibrated_dual_return_uses_machine_not_model_or_policy(self):
        self.calibrated_machine()
        (self.root/'model.json').unlink()
        (self.root/'policy.json').unlink()
        self.assertTrue(self.workspace.snapshot()['can_return_zero'])
        child = Process()
        with patch.object(self.workspace, '_client', side_effect=AssertionError('model')), \
             patch.object(self.workspace, '_validate_home', side_effect=AssertionError('model route')), \
             patch('openarm_platform.model_workspace.subprocess.Popen', return_value=child) as launch, \
             patch('openarm_platform.model_workspace.threading.Thread'):
            self.workspace.return_zero(confirmed=True, owner='browser-one')
        command = launch.call_args.args[0]
        self.assertIn('--calibrated-zero', command)
        self.assertEqual(command[command.index('--sides')+1:command.index('--sides')+3], ['left', 'right'])
        for option in ('--profile', '--policy', '--auto-return-home', '--duration'):
            self.assertNotIn(option, command)
        self.assertIsNone(self.workspace.server)
        self.assertEqual(self.workspace.state, 'unloaded')
        deployment = json.loads((self.workspace.run_root/'deployment.json').read_text())
        self.assertIsNone(deployment['identity'])
        self.assertIsNone(deployment['sample_test'])
        self.assertEqual(deployment['sides'], ['left', 'right'])
        self.assertEqual(list(deployment['configuration_hashes']), [str(self.root/'robot.json')])
        self.finish(child)

    def test_dual_return_requires_owner_confirmation_mapping_and_clean_idle(self):
        self.calibrated_machine()
        for kwargs in ({}, {'confirmed': True}, {'confirmed': False, 'owner': 'browser-one'}):
            with self.assertRaises(ValueError): self.workspace.return_zero(**kwargs)
        self.workspace.execution = 'failed'
        with self.assertRaises(ValueError):
            self.workspace.return_zero(confirmed=True, owner='browser-one')
        self.workspace.execution = 'idle'
        self.write('robot.json', dict(left='can1', right='can0', mapping_verified=False))
        self.assertFalse(self.workspace.snapshot()['can_return_zero'])
        with self.assertRaises(ValueError):
            self.workspace.return_zero(confirmed=True, owner='browser-one')
        self.manager.claim_motion.assert_not_called()

    def test_dual_return_never_runs_on_readonly_or_simulation_platform(self):
        self.calibrated_machine()
        for readonly, simulation in ((True, False), (False, True)):
            self.workspace.readonly, self.workspace.simulation = readonly, simulation
            self.assertFalse(self.workspace.snapshot()['can_return_zero'])
            with self.assertRaises(ValueError):
                self.workspace.return_zero(confirmed=True, owner='browser-one')
        self.manager.claim_motion.assert_not_called()

    def test_robot_profile_change_during_home_handover_prevents_launch(self):
        self.calibrated_machine()
        self.manager.claim_motion.side_effect = lambda token: self.write('robot.json', dict(changed=True))
        with patch('openarm_platform.model_workspace.subprocess.Popen') as launch:
            with self.assertRaisesRegex(ValueError, '变化'):
                self.workspace.return_zero(confirmed=True, owner='browser-one')
        launch.assert_not_called()
        self.manager.release_motion.assert_called_once()

    def home_profile(self):
        self.policy['preparation'] = dict(kind='fixed_joint_waypoints_v1', side='left',
            waypoints=[dict(name='swing', joints_rad=[0, -.1, 0, .03, 0, 0, 0]),
                       dict(name='ready', joints_rad=[.1, -.1, 0, .1, 0, 0, 0])])
        self.policy['return_home'] = dict(kind='reverse_preparation_v1',
            home_joints_rad=[0, 0, 0, .03, 0, 0, 0])
        self.write('policy.json', self.policy)

    def test_home_only_from_unloaded_page_uses_motion_lease_but_never_model(self):
        self.home_profile()
        child = Process()
        with patch.object(self.workspace, '_client', side_effect=AssertionError('model')), \
             patch('openarm_platform.model_workspace.subprocess.Popen', return_value=child) as launch, \
             patch('openarm_platform.model_workspace.threading.Thread'):
            self.workspace.return_home('sample-model', confirmed=True, owner='browser-one')
        command = launch.call_args.args[0]
        self.assertIn('--home-only', command)
        self.assertNotIn('--auto-return-home', command)
        self.assertEqual(self.workspace.operation_kind, 'home')
        self.assertEqual(self.workspace.state, 'unloaded')
        self.manager.claim_motion.assert_called_once()
        self.finish(child)
        self.assertEqual(self.workspace.execution, 'completed')

    def test_home_requires_configuration_confirmation_and_cleanup(self):
        with self.assertRaises(ValueError):
            self.workspace.return_home('sample-model', confirmed=True, owner='browser-one')
        self.home_profile()
        with self.assertRaises(ValueError):
            self.workspace.return_home('sample-model', owner='browser-one')
        self.workspace.execution = 'failed'
        with self.assertRaisesRegex(ValueError, '清理'):
            self.workspace.return_home('sample-model', confirmed=True, owner='browser-one')
        self.manager.claim_motion.assert_not_called()
        self.workspace.execution = 'idle'

    def test_auto_return_is_explicit_and_missing_route_rejects_before_claim(self):
        self.tested()
        with patch.object(self.workspace, '_client', return_value=self.client):
            with self.assertRaises(ValueError):
                self.workspace.start(1, confirmed=True, owner='browser-one', auto_return_home=True)
        self.manager.claim_motion.assert_not_called()
        self.workspace.unload()
        self.home_profile()
        self.server = Process()
        self.tested()
        child = Process()
        with patch.object(self.workspace, '_client', return_value=self.client), \
             patch('openarm_platform.model_workspace.subprocess.Popen', return_value=child) as launch, \
             patch('openarm_platform.model_workspace.threading.Thread'):
            self.workspace.start(1, confirmed=True, owner='browser-one', auto_return_home=True)
        self.assertIn('--auto-return-home', launch.call_args.args[0])
        self.finish(child)

    def test_default_start_has_no_duration_and_stop_preserves_cleanup_lease(self):
        self.tested()
        child = Process()
        with patch.object(self.workspace, '_client', return_value=self.client), \
             patch('openarm_platform.model_workspace.subprocess.Popen', return_value=child) as launch, \
             patch('openarm_platform.model_workspace.threading.Thread'):
            self.workspace.start(confirmed=True, owner='browser-one')
        command = launch.call_args.args[0]
        self.assertIn('--until-stopped', command)
        self.assertNotIn('--duration', command)
        self.assertNotIn('--auto-return-home', command)
        deployment = json.loads((self.workspace.run_root/'deployment.json').read_text())
        self.assertIsNone(deployment['duration_s'])
        self.assertFalse(deployment['auto_return_home'])
        self.workspace.stop(owner='another-browser')
        self.assertFalse(child.terminated)
        self.workspace.stop(owner='browser-one')
        self.assertTrue(child.terminated)
        self.assertFalse(child.killed)
        self.manager.release_motion.assert_not_called()
        self.finish(child)
        self.manager.release_motion.assert_called_once()

    def test_manual_stop_rejects_automatic_return_before_motion_claim(self):
        self.tested()
        with self.assertRaisesRegex(ValueError, '手动停止'):
            self.workspace.start(confirmed=True, owner='browser-one', auto_return_home=True)
        self.manager.claim_motion.assert_not_called()

    def test_home_watcher_does_not_require_predictor_server(self):
        self.home_profile()
        child = Process()
        with patch('openarm_platform.model_workspace.subprocess.Popen', return_value=child), \
             patch('openarm_platform.model_workspace.threading.Thread'):
            self.workspace.return_home('sample-model', confirmed=True, owner='browser-one')
        self.assertIsNone(self.workspace.server)
        child.code = 0
        with patch.object(self.workspace, '_finished') as finished:
            self.workspace._watch()
        self.assertFalse(child.terminated)
        finished.assert_called_once()
        self.finish(child)

    def test_return_home_readonly_and_simulation_platform_guards_precede_motion(self):
        self.home_profile()
        self.workspace.readonly = True
        with self.assertRaises(ValueError):
            self.workspace.return_home('sample-model', confirmed=True, owner='browser-one')
        self.workspace.readonly = False
        self.workspace.simulation = True
        from dataclasses import replace
        self.workspace.entries['sample-model'] = replace(self.workspace.entries['sample-model'],
            robot='classic', robot_config=self.root/'model.json')
        with self.assertRaisesRegex(ValueError, '仿真平台'):
            self.workspace.return_home('sample-model', confirmed=True, owner='browser-one')
        self.manager.claim_motion.assert_not_called()

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name).resolve()
        self.identity = {'family': 'test-predictor', 'revision': 'frozen-1'}
        self.write('model.json', {'factory': 'tests.fake:Predictor'})
        self.policy = {'policy_kwargs': {'model': {
            'factory': 'openarm_policy.models.http:NumpyHTTP',
            'kwargs': {'url': 'http://127.0.0.1:18765/', 'expected_identity': self.identity}}}}
        self.write('policy.json', self.policy)
        self.item = dict(id='sample-model', label='测试模型', model_profile='model.json',
                         identity=self.identity, output_shape=[2, 8], robot='simulate',
                         policy_profile='policy.json', probe_inputs=None)
        self.catalogue = self.root / 'catalog.json'
        self.write_catalogue()
        self.manager = Mock()
        self.workspace = ModelWorkspace(self.catalogue, self.root / 'records', self.manager)
        self.addCleanup(self.workspace.close)
        self.server = Process()
        self.client = Mock()

    def write(self, name, data):
        (self.root / name).write_text(json.dumps(data))

    def write_catalogue(self, entries=None):
        self.write('catalog.json', dict(schema='openarm.model-catalog.v1', port=18765,
                                       models=[self.item] if entries is None else entries))

    def load(self):
        with patch('openarm_platform.model_workspace.tcp_reachable', return_value=False), \
             patch('openarm_platform.model_workspace.subprocess.Popen', return_value=self.server) as launch, \
             patch.object(self.workspace, '_client', return_value=self.client):
            self.workspace.load('sample-model')
        return launch

    def tested(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest('Requires optional NumPy fixture support')
        np.savez(self.root / 'probe.npz', state=np.zeros((1, 8)))
        self.item['probe_inputs'] = 'probe.npz'
        self.write_catalogue()
        self.workspace.refresh()
        self.client.predict.return_value = np.zeros((2, 8))
        self.load()

    def test_load_automatically_checks_sample_once_and_is_ready(self):
        self.tested()
        self.client.predict.assert_called_once()
        self.assertEqual(self.workspace.state, 'tested')
        self.assertTrue(self.workspace.snapshot()['can_start'])
        self.manager.claim_motion.assert_not_called()

    def test_automatic_check_rejects_wrong_shape_and_nonfinite_output(self):
        import numpy as np
        np.savez(self.root / 'probe.npz', state=np.zeros((1, 8)))
        self.item['probe_inputs'] = 'probe.npz'
        self.write_catalogue()
        self.workspace.refresh()
        for output in (np.zeros((1, 8)), np.full((2, 8), np.nan), np.full((2, 8), np.inf)):
            with self.subTest(output=output.shape):
                self.server = Process()
                self.client.predict.return_value = output
                with self.assertRaisesRegex(ValueError, '协议'):
                    self.load()
                self.assertEqual(self.workspace.state, 'error')
                self.assertIsNone(self.workspace.server)
                self.assertTrue(self.server.terminated)
                self.assertFalse(self.workspace.snapshot()['can_start'])
        self.manager.claim_motion.assert_not_called()

    def test_stop_during_automatic_check_cannot_publish_readiness(self):
        import numpy as np
        def cancel_prediction(inputs):
            self.assertEqual(self.workspace.state, 'checking')
            self.assertFalse(self.workspace.snapshot()['can_return_home'])
            self.workspace.stop()
            return np.zeros((2, 8))
        self.client.predict.side_effect = cancel_prediction
        with self.assertRaisesRegex(RuntimeError, '取消'):
            self.tested()
        self.assertIsNone(self.workspace.test_result)
        self.assertIsNone(self.workspace.server)
        self.assertFalse(self.workspace.snapshot()['can_start'])
        self.manager.claim_motion.assert_not_called()

    def test_model_replacement_rejects_active_motion_before_closing_predictor(self):
        child, _ = self.start()
        from dataclasses import replace
        self.workspace.entries['other'] = replace(self.workspace.entry, id='other')
        with self.assertRaisesRegex(ValueError, '停止执行'):
            self.workspace.load('other')
        self.assertFalse(self.server.terminated)
        self.finish(child)

    def test_idle_model_replacement_closes_only_old_owned_predictor(self):
        self.write_catalogue([self.item, {**self.item, 'id': 'other'}])
        self.workspace.refresh()
        self.load()
        old = self.server
        replacement = Process()
        with patch('openarm_platform.model_workspace.tcp_reachable', return_value=False), \
             patch('openarm_platform.model_workspace.subprocess.Popen', return_value=replacement), \
             patch.object(self.workspace, '_client', return_value=self.client):
            self.workspace.load('other')
        self.assertTrue(old.terminated)
        self.assertFalse(replacement.terminated)
        self.assertEqual(self.workspace.entry.id, 'other')
        self.manager.claim_motion.assert_not_called()

    def start(self):
        self.tested()
        child = Process()
        # Keep the monitor deterministic; test its completion transition directly.
        with patch.object(self.workspace, '_client', return_value=self.client), \
             patch('openarm_platform.model_workspace.subprocess.Popen', return_value=child) as launch, \
             patch('openarm_platform.model_workspace.threading.Thread'):
            self.workspace.start(1, confirmed=True, owner='browser-one')
        return child, launch

    def finish(self, child, *, clean=True):
        episode = self.workspace.run_root / 'episode'
        episode.mkdir()
        (episode / 'summary.json').write_text(json.dumps(dict(complete=True,
            termination={'cleanup_errors': [] if clean else ['disable not verified']})))
        child.code = 0
        self.workspace._finished()

    def test_construction_refresh_snapshot_are_passive_and_paths_private(self):
        with patch('subprocess.Popen', side_effect=AssertionError('child')):
            state = self.workspace.refresh()
        self.assertEqual(state['state'], 'unloaded')
        self.assertFalse(state['can_start'])
        self.assertNotIn('inference_python', state['models'][0])
        self.assertNotIn('model_profile', state['models'][0])
        self.manager.claim_motion.assert_not_called()

    def test_catalogue_rejects_unknown_commands_duplicate_ids_and_bad_bounds(self):
        for changes in ({'command': ['sh']}, {'id': '../outside'}, {'output_shape': [True, 7]},
                        {'max_duration_s': float('nan')}, {'inference_env': {'LD_PRELOAD': 'evil'}}):
            with self.subTest(changes=changes):
                self.write_catalogue([{**self.item, **changes}])
                with self.assertRaises(ValueError):
                    catalogue(self.catalogue)
        self.write_catalogue([self.item, self.item])
        with self.assertRaises(ValueError):
            catalogue(self.catalogue)

    def test_load_launches_only_predictor_never_policy_or_motion(self):
        launch = self.load()
        command = launch.call_args.args[0]
        self.assertIn('model-serve', command)
        self.assertNotIn('policy', command)
        self.assertFalse(self.workspace.snapshot()['can_start'])
        self.manager.claim_motion.assert_not_called()
        with self.assertRaises(ValueError):
            self.workspace.load('sample-model')
        with self.assertRaises(ValueError):
            self.workspace.load('/some/arbitrary/path')

    def test_occupied_model_port_is_never_adopted(self):
        with patch('openarm_platform.model_workspace.tcp_reachable', return_value=True), \
             patch('subprocess.Popen') as launch:
            with self.assertRaisesRegex(ValueError, '外部'):
                self.workspace.load('sample-model')
        launch.assert_not_called()

    def test_identity_mismatch_closes_only_owned_inference(self):
        with patch('openarm_platform.model_workspace.tcp_reachable', return_value=False), \
             patch('subprocess.Popen', return_value=self.server), \
             patch.object(self.workspace, '_client', side_effect=ValueError('identity mismatch')):
            with self.assertRaisesRegex(ValueError, 'identity'):
                self.workspace.load('sample-model')
        self.assertTrue(self.server.terminated)
        self.assertIsNone(self.workspace.server)
        self.manager.claim_motion.assert_not_called()

    def test_sample_prediction_does_not_activate_hardware_or_retain_actions(self):
        self.tested()
        state = self.workspace.snapshot()
        self.assertTrue(state['can_start'])
        self.assertEqual(state['sample_test']['shape'], [2, 8])
        self.assertNotIn('actions', state['sample_test'])
        self.manager.claim_motion.assert_not_called()

    def test_missing_sample_cannot_be_silently_treated_as_passed(self):
        self.load()
        with self.assertRaisesRegex(ValueError, '样例'):
            self.workspace.test()
        self.assertFalse(self.workspace.snapshot()['can_start'])

    def test_changed_configuration_invalidates_preflight_before_motion(self):
        self.tested()
        self.write('policy.json', {'changed': True})
        with self.assertRaisesRegex(ValueError, '变化'):
            self.workspace.start(1, confirmed=True, owner='browser-one')
        self.manager.claim_motion.assert_not_called()

    def test_start_requires_confirmation_owner_test_and_bounded_duration(self):
        self.load()
        with self.assertRaises(ValueError):
            self.workspace.start(1, confirmed=True, owner='browser-one')
        self.workspace.unload()
        self.server = Process()
        self.tested()
        for kwargs in ({'confirmed': False, 'owner': 'a'}, {'confirmed': True, 'owner': None}):
            with self.assertRaises(ValueError):
                self.workspace.start(1, **kwargs)
        for duration in (0, -1, float('nan'), True, 31):
            with self.assertRaises(ValueError):
                self.workspace.start(duration, confirmed=True, owner='a')
        self.manager.claim_motion.assert_not_called()

    def test_execution_profile_cannot_point_at_another_model_server(self):
        self.policy['policy_kwargs']['model']['kwargs']['expected_identity'] = {'family': 'wrong'}
        self.write('policy.json', self.policy)
        self.tested()
        with self.assertRaisesRegex(ValueError, 'expected_identity'):
            self.workspace.start(1, confirmed=True, owner='browser-one')
        self.manager.claim_motion.assert_not_called()

    def test_nominal_cartesian_geometry_is_not_promoted_to_verified(self):
        self.write('robot.json', {})
        self.item.update(robot='classic', robot_config='robot.json')
        self.policy['cartesian'] = {'calibration_verified': False}
        self.write('policy.json', self.policy)
        self.tested()
        with self.assertRaisesRegex(ValueError, '几何'):
            self.workspace.start(1, confirmed=True, owner='browser-one')
        self.manager.claim_motion.assert_not_called()

    def test_nominal_opt_in_only_accepts_explicitly_declared_research_geometry(self):
        self.write('robot.json', {})
        self.item.update(robot='classic', robot_config='robot.json')
        self.policy['cartesian'] = {'calibration_verified': False,
            'calibration_status': 'nominal_model_not_hardware_calibrated'}
        self.write('policy.json', self.policy)
        self.tested()
        self.assertTrue(self.workspace.snapshot()['requires_nominal_opt_in'])
        with self.assertRaisesRegex(ValueError, '几何'):
            self.workspace.start(1, confirmed=True, owner='browser-one')
        self.workspace._execution_options(allow_nominal=True)
        self.assertFalse(json.loads((self.root / 'policy.json').read_text())['cartesian']['calibration_verified'])

    def test_start_reuses_cli_summary_recording_and_keeps_loaded_model(self):
        child, launch = self.start()
        command = launch.call_args.args[0]
        self.assertIn('policy', command)
        self.assertIn('openarm_policy.models.policy:ModelPolicy', command)
        self.assertIn('--record-mode', command)
        self.assertEqual(command[-1], 'summary')
        self.manager.claim_motion.assert_called_once()
        with self.assertRaises(ValueError):
            self.workspace.unload()
        with self.assertRaises(ValueError):
            self.workspace.load('sample-model')
        self.workspace.stop(owner='other-browser')
        self.assertFalse(child.terminated)
        self.workspace.stop(owner='browser-one')
        self.assertTrue(child.terminated)
        self.assertFalse(child.killed)
        self.finish(child)
        self.assertFalse(self.server.terminated)
        self.assertEqual(self.workspace.snapshot()['execution'], 'completed')
        self.assertTrue(self.manager.release_motion.call_args.kwargs['clean'])

    def test_missing_or_failed_cleanup_blocks_motion_handover(self):
        child, _ = self.start()
        self.finish(child, clean=False)
        self.assertEqual(self.workspace.snapshot()['execution'], 'failed')
        self.assertFalse(self.manager.release_motion.call_args.kwargs['clean'])

    def test_policy_failure_and_verified_cleanup_are_distinct(self):
        child, _ = self.start()
        episode = self.workspace.run_root / 'episode'
        episode.mkdir()
        (episode/'summary.json').write_text(json.dumps(dict(complete=True,
            termination={'cleanup_errors': [], 'state': 'failed', 'error': 'safety stop'})))
        child.code = 1
        self.workspace._finished()
        self.assertEqual(self.workspace.snapshot()['execution'], 'stopped_error')
        self.assertEqual(self.workspace.snapshot()['error'], 'safety stop（退出清理已确认）')
        self.assertTrue(self.manager.release_motion.call_args.kwargs['clean'])

    def test_unverified_cleanup_preserves_original_failure_and_motion_lock(self):
        child, _ = self.start()
        episode = self.workspace.run_root / 'episode'
        episode.mkdir()
        reason = 'left: RuntimeError: Expected healthy Classic 1 Mbps'
        (episode / 'summary.json').write_text(json.dumps(dict(complete=True,
            termination={'cleanup_errors': [reason], 'error': reason})))
        child.code = 1
        self.workspace._finished()
        state = self.workspace.snapshot()
        self.assertIn(reason, state['error'])
        self.assertIn('未确认正常清理', state['error'])
        self.assertEqual(state['execution'], 'failed')
        self.assertFalse(state['can_start'])
        self.assertFalse(self.manager.release_motion.call_args.kwargs['clean'])

    def test_slow_control_handover_keeps_status_and_cancel_responsive(self):
        self.tested()
        entered, release = threading.Event(), threading.Event()
        errors = []
        self.manager.claim_motion.side_effect = lambda *_: (entered.set(), release.wait(5))

        def start():
            try:
                self.workspace.start(1, confirmed=True, owner='browser-one')
            except BaseException as exc:
                errors.append(exc)

        with patch.object(self.workspace, '_client', return_value=self.client), \
             patch('openarm_platform.model_workspace.subprocess.Popen') as launch:
            worker = threading.Thread(target=start)
            worker.start()
            try:
                self.assertTrue(entered.wait(2))
                snapshots = []
                reader = threading.Thread(target=lambda: snapshots.append(self.workspace.snapshot()))
                reader.start()
                reader.join(timeout=2)
                self.assertFalse(reader.is_alive(), 'Status must not wait for peer shutdown')
                self.assertTrue(snapshots[0]['running'])
                self.assertFalse(snapshots[0]['can_start'])
                self.workspace.stop(owner='browser-one')
            finally:
                release.set()
                worker.join(timeout=3)
            self.assertFalse(worker.is_alive())
            launch.assert_not_called()
        self.assertFalse(errors)
        self.assertTrue(self.manager.release_motion.call_args.kwargs['clean'])

    def test_predictor_death_is_not_reported_as_ready(self):
        self.tested()
        self.server.code = 1
        state = self.workspace.snapshot()
        self.assertEqual(state['state'], 'error')
        self.assertFalse(state['can_start'])

    def test_input_preview_uses_declared_sources_and_current_model_only(self):
        self.policy['video_http'] = {'url': 'http://127.0.0.1:5003', 'names': ['head', 'left', 'right']}
        self.policy['policy_kwargs']['observation'] = {'kwargs': {'overview': 'camera/head', 'wrist': 'camera/left'}}
        self.write('policy.json', self.policy)
        self.workspace.video_preview_port = 5003
        self.assertEqual(self.workspace.input_status('sample-model')['cameras'], ['head', 'left'])
        self.workspace.video_preview_port = 5999
        self.assertEqual(self.workspace.input_status('sample-model')['cameras'], [])

    def test_readonly_mode_does_not_load_or_execute(self):
        self.workspace.readonly = True
        for operation in (lambda: self.workspace.load('sample-model'),
                          self.workspace.test, self.workspace.refresh, self.workspace.unload):
            with self.assertRaises(ValueError):
                operation()


class MotionLeaseTests(unittest.TestCase):
    def manager(self):
        manager = ServiceManager([Page('teleop', '遥操', 8890, exclusive_group='motion')], '/unused')
        manager._started = True
        return manager

    @patch('openarm_platform.services.tcp_reachable', return_value=False)
    def test_claim_prevents_peer_activation_until_clean_release(self, _):
        manager = self.manager()
        manager.claim_motion('selected-policy')
        with self.assertRaisesRegex(ValueError, '停止模型'):
            manager.activate('teleop')
        with self.assertRaisesRegex(RuntimeError, '尚未确认停止'):
            manager.close()
        with self.assertRaises(ValueError):
            manager.release_motion('not-owner', clean=True)
        manager.release_motion('selected-policy', clean=True)
        self.assertIsNone(manager._motion_owner)

    @patch('openarm_platform.services.tcp_reachable', return_value=True)
    def test_claim_never_stops_external_controller(self, _):
        manager = self.manager()
        with patch.object(manager, '_stop_services') as stop:
            with self.assertRaisesRegex(ValueError, '外部'):
                manager.claim_motion('selected-policy')
        stop.assert_not_called()

    @patch('openarm_platform.services.tcp_reachable', return_value=False)
    def test_failed_cleanup_blocks_subsequent_motion(self, _):
        manager = self.manager()
        manager.claim_motion('selected-policy')
        manager.release_motion('selected-policy', clean=False)
        with self.assertRaises(ValueError):
            manager.claim_motion('next-policy')
        with self.assertRaises(ValueError):
            manager.activate('teleop')


@unittest.skipUnless(importlib.util.find_spec('fastapi') and importlib.util.find_spec('aiohttp'),
                     'Requires optional platform dependencies')
class ModelCatalogueApiTests(unittest.TestCase):
    def test_input_image_and_recording_only_use_current_episode(self):
        from starlette.testclient import TestClient
        from openarm_platform.app import create_app
        from openarm_platform.config import load_config
        manager = Mock()
        manager.status.return_value = {}
        app = create_app(load_config(), manager, attach_only=True, with_ui=False)
        workspace = app.state.model_workspace
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)/'policy-fixture'
            images = root/'episode/model-inputs'
            images.mkdir(parents=True)
            (images/'image.png').write_bytes(b'png-fixture')
            (root/'episode/recording.rrd').write_bytes(b'rrd-fixture')
            with TestClient(app, base_url='http://127.0.0.1:8888') as client:
                self.assertEqual(client.get('/api/models/input-image/image').status_code, 404)
                workspace.run_root = root
                image = client.get('/api/models/input-image/image')
                self.assertEqual(image.content, b'png-fixture')
                self.assertEqual(image.headers['cache-control'], 'no-store')
                stream = client.get('/api/models/input-stream/image?run=policy-fixture')
                self.assertIn('multipart/x-mixed-replace', stream.headers['content-type'])
                self.assertIn(b'png-fixture', stream.content)
                self.assertEqual(stream.content.count(b'--frame'), 1)
                self.assertEqual(client.get('/api/models/input-stream/image?run=wrong-run').status_code, 404)
                self.assertEqual(client.get('/api/models/input-image/invalid.key').status_code, 404)
                self.assertEqual(client.get('/api/models/recording').content, b'rrd-fixture')
                workspace.process = Mock()
                self.assertEqual(client.get('/api/models/recording').status_code, 404)
                workspace.process = None

    def test_get_models_is_passive_and_attach_only_stays_readonly(self):
        from starlette.testclient import TestClient
        from openarm_platform.app import create_app
        from openarm_platform.config import load_config
        manager = Mock()
        manager.status.return_value = {}
        with patch('subprocess.Popen', side_effect=AssertionError('GET must not launch a child')):
            with TestClient(create_app(load_config(), manager, attach_only=True, with_ui=False),
                            base_url='http://127.0.0.1:8888') as client:
                response = client.get('/api/models')
                self.assertEqual(response.status_code, 200)
                state = response.json()
                self.assertTrue(state['readonly'])
                self.assertEqual(state['models'], [])
                self.assertFalse(state['running'])
                manager.start.assert_not_called()
                manager.claim_motion.assert_not_called()


class ModelDeploymentIntegrationTests(unittest.TestCase):
    def test_two_model_families_share_loading_and_simulated_execution(self):
        """Real model server, HTTP predictor, spawned inference and policy CLI.

        Neither model imports a hardware library. Only SimulatedRobot executes.
        """
        try:
            import numpy as np
        except ImportError:
            self.skipTest('Requires optional NumPy model support')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            (root / 'deployment_fixture.py').write_text('''
import numpy as np
class Predictor:
    def __init__(self, family): self.metadata = {'family': family, 'revision': 'fixture-1'}
    def reset(self, context): pass
    def predict(self, inputs): return np.zeros((2,8), dtype=np.float32)
    def close(self): pass
class OtherPredictor:
    def __init__(self, family): self.metadata = {'family': family, 'revision': 'fixture-1'}
    def reset(self, context): pass
    def predict(self, inputs):
        assert inputs['features'].shape == (1,4)
        return [[0.0]*8, [0.0]*8]
    def close(self): pass
class Encoder:
    def reset(self, context): pass
    def encode(self, observation): return {'state':np.zeros((1,8))}
class OtherEncoder:
    def reset(self, context): pass
    def encode(self, observation): return {'features':np.zeros((1,4))}
class Decoder:
    def reset(self, context): pass
    def decode(self, output, observation): return output
''')
            with socket.socket() as sock:
                sock.bind(('127.0.0.1', 0))
                port = sock.getsockname()[1]
            env = {'PYTHONPATH': os.pathsep.join([str(Path(__file__).resolve().parents[1] / 'src'), str(root),
                                               os.environ.get('PYTHONPATH', '')])}
            with_rerun = importlib.util.find_spec('rerun') is not None
            entries = []
            for family in ('vision-fixture', 'other-format-fixture'):
                identity = {'family': family, 'revision': 'fixture-1'}
                other = family == 'other-format-fixture'
                np.savez(root / (family + '-sample.npz'),
                         **({'features':np.zeros((1,4))} if other else {'state':np.zeros((1,8))}))
                (root / (family + '.json')).write_text(json.dumps({
                    'factory': 'deployment_fixture:OtherPredictor' if other else 'deployment_fixture:Predictor',
                    'kwargs': {'family': family}}))
                (root / (family + '-policy.json')).write_text(json.dumps({'rerun_recording': with_rerun, 'policy_kwargs': {
                    'diagnostic_inputs': with_rerun,
                    'model': {'factory': 'openarm_policy.models.http:NumpyHTTP', 'kwargs': {
                        'url': f'http://127.0.0.1:{port}/', 'expected_identity': identity}},
                    'observation': {'factory': 'deployment_fixture:OtherEncoder' if other else 'deployment_fixture:Encoder'},
                    'action': {'factory': 'deployment_fixture:Decoder'}}}))
                entries.append(dict(id=family, label=family, identity=identity,
                    model_profile=family + '.json', policy_profile=family + '-policy.json',
                    probe_inputs=family + '-sample.npz', output_shape=[2,8], robot='simulate',
                    inference_env=env, runtime_env=env))
            catalog = root / 'catalog.json'
            catalog.write_text(json.dumps(dict(schema='openarm.model-catalog.v1', port=port, models=entries)))
            manager = ServiceManager([], root / 'logs').start()
            workspace = ModelWorkspace(catalog, root / 'records', manager, simulation=True)
            try:
                for name in ('vision-fixture', 'other-format-fixture'):
                    with self.subTest(family=name):
                        workspace.load(name)
                        self.assertIsNone(manager._motion_owner)
                        workspace.test()
                        workspace.start(.25, confirmed=True, owner='test-browser')
                        deadline = time.monotonic() + 20
                        while workspace.snapshot()['running'] and time.monotonic() < deadline:
                            time.sleep(.05)
                        self.assertEqual(workspace.snapshot()['execution'], 'completed',
                            (workspace.run_root / 'console.log').read_text())
                        self.assertIsNone(manager._motion_owner)
                        report = json.loads((workspace.run_root / 'episode/summary.json').read_text())
                        self.assertTrue(report['complete'])
                        self.assertIsNone(report['task_success'])
                        self.assertFalse((workspace.run_root / 'episode/events.jsonl').exists())
                        if with_rerun:
                            self.assertTrue((workspace.run_root / 'episode/recording.rrd').is_file())
                            self.assertTrue(json.loads((workspace.run_root / 'episode/rerun-status.json').read_text())['complete'])
                        workspace.unload()
            finally:
                workspace.close()
                manager.close()


if __name__ == '__main__':
    unittest.main()
