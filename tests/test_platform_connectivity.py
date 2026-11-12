"""CAN reconnect supervision without CAN sockets, system changes or motors."""
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
import importlib.util
from unittest.mock import Mock, patch

from openarm_platform.connectivity import CANConnectivity
from openarm_runtime.telemetry import Telemetry, MotorState


def healthy_link():
    return dict(flags=['UP'], mtu=16, linkinfo=dict(info_data=dict(state='ERROR-ACTIVE',
        bittiming=dict(bitrate=1000000), ctrlmode=[], restart_ms=0, berr_counter=dict(tx=0, rx=0))))


class Connection:
    def __init__(self, interface, frame_format):
        self.interface, self.frame_format = interface, frame_format
        self.shared, self.stop = Telemetry(), threading.Event()
        self.started = self.closed = False
    def restart(self): self.started = True
    def close(self): self.closed = True; self.stop.set()


class ConnectivityTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(); self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)/'robot.json'
        self.path.write_text(json.dumps(dict(format='classic', bitrate=1000000,
            left='can1', right='can0', mapping_verified=True)))
        self.entry = SimpleNamespace(robot='classic', robot_config=self.path, sides=('left',))
        self.manager = Mock(); self.manager.motion_active.return_value = False
        self.monitor = CANConnectivity(self.manager)
        self.addCleanup(self.monitor.close)
        self.monitor.configure(self.entry)
        self.patch_reader = patch('openarm_runtime.telemetry.ArmConnection', Connection)
        self.patch_reader.start(); self.addCleanup(self.patch_reader.stop)
        self.patch_link = patch('openarm_runtime.telemetry.inspect_link', return_value=healthy_link())
        self.link = self.patch_link.start(); self.addCleanup(self.patch_link.stop)

    def fill(self, *, age=0., status=0):
        connection = self.monitor.connections['left']
        connection.shared.states = {i: MotorState(i, 0., 0., 0., 24, 24, status,
            time.monotonic()-age) for i in range(1, 9)}
        return connection

    def test_construction_is_passive_and_missing_device_remains_waiting(self):
        self.assertFalse(self.monitor.connections)
        self.link.side_effect = FileNotFoundError('CAN unplugged')
        self.monitor.tick()
        self.assertEqual(self.monitor.snapshot()['state'], 'waiting')
        self.assertIn('CAN unplugged', self.monitor.snapshot()['error'])
        self.assertTrue(self.monitor.connections['left'].started)
        self.assertEqual(self.monitor.connections['left'].frame_format, 'classic')

    def test_recovery_only_marks_ready_and_never_claims_motion(self):
        self.monitor.tick()
        self.assertEqual(self.monitor.snapshot()['state'], 'waiting')
        self.fill()
        self.monitor.tick()
        self.assertEqual(self.monitor.snapshot()['state'], 'ready')
        self.manager.claim_motion.assert_not_called()
        self.manager.activate.assert_not_called()

    def test_stale_feedback_or_enabled_motor_never_marks_ready(self):
        self.monitor.tick()
        for values in (dict(age=1.), dict(status=1), dict(status=14)):
            self.fill(**values)
            self.monitor.tick()
            self.assertNotEqual(self.monitor.snapshot()['state'], 'ready')

    def test_busy_control_closes_probe_without_reconnecting(self):
        self.monitor.tick(); connection = self.fill()
        self.manager.motion_active.return_value = True
        self.monitor.tick()
        self.assertTrue(connection.closed)
        self.assertFalse(self.monitor.connections)
        self.assertEqual(self.monitor.snapshot()['state'], 'busy')
        self.monitor.tick()
        self.assertFalse(self.monitor.connections)
        self.manager.motion_active.return_value = False
        self.monitor.tick()
        self.assertIsNot(self.monitor.connections['left'], connection)

    def test_motion_handover_epoch_prevents_probe_racing_back_in(self):
        self.monitor.tick(); connection = self.fill()
        def handover(): self.monitor.pause(); return False
        self.manager.motion_active.side_effect = handover
        self.monitor.tick()
        self.assertTrue(connection.closed)
        self.assertFalse(self.monitor.connections)

    def test_waiting_blocks_start_and_configuration_change_invalidates_ready(self):
        self.monitor.enabled = True
        with self.assertRaises(ValueError): self.monitor.require_ready(self.entry)
        self.monitor.tick(); self.fill(); self.monitor.tick()
        self.monitor.require_ready(self.entry)
        entry = SimpleNamespace(robot='classic', robot_config=self.path, sides=('right',))
        with self.assertRaises(ValueError): self.monitor.require_ready(entry)
        self.assertEqual(self.monitor.snapshot()['state'], 'waiting')

    def test_unverified_configuration_is_visible_without_creating_readers(self):
        data = json.loads(self.path.read_text()); data['mapping_verified'] = False
        self.path.write_text(json.dumps(data))
        self.monitor.tick()
        self.assertEqual(self.monitor.snapshot()['state'], 'blocked')
        self.assertFalse(self.monitor.connections)

    def test_snapshot_never_waits_for_io_lock_or_exposes_mutable_state(self):
        self.monitor.tick()
        acquired, release = threading.Event(), threading.Event()
        def hold():
            with self.monitor.lock:
                acquired.set(); release.wait(2.)
        worker = threading.Thread(target=hold); worker.start(); acquired.wait(1.)
        try:
            snapshot = self.monitor.snapshot()
            self.assertEqual(snapshot['state'], 'waiting')
            snapshot['sides'].clear()
            self.assertTrue(self.monitor.snapshot()['sides'])
        finally:
            release.set(); worker.join(2.)


@unittest.skipUnless(importlib.util.find_spec('fastapi') and importlib.util.find_spec('aiohttp'), 'install platform extra')
class ConnectivityApiTests(unittest.TestCase):
    def test_missing_can_does_not_abort_web_and_recovery_never_starts_motion(self):
        from dataclasses import replace
        from starlette.testclient import TestClient
        from openarm_platform import load_config
        from openarm_platform.app import create_app
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'robot.json').write_text(json.dumps(dict(format='classic', bitrate=1000000,
                left='can1', right='can0', mapping_verified=True)))
            (root/'predictor.json').write_text('{}')
            (root/'catalog.json').write_text(json.dumps(dict(schema='openarm.model-catalog.v1', port=18765,
                models=[dict(id='registered', label='Robot', identity={'revision':'test'}, output_shape=[100,7],
                             robot='classic', robot_config='robot.json', model_profile='predictor.json')])) )
            config = replace(load_config(), model_catalog=root/'catalog.json', policy_record_root=root/'records')
            manager = Mock(); manager.motion_active.return_value = False; manager.status.return_value = {}
            app = create_app(config, manager, with_ui=False)
            with patch('openarm_runtime.telemetry.ArmConnection', Connection), \
                 patch('openarm_runtime.telemetry.inspect_link', return_value=healthy_link()), \
                 patch('subprocess.Popen', side_effect=AssertionError('motion/model process')):
                with TestClient(app, base_url='http://127.0.0.1:8888') as client:
                    monitor = app.state.model_workspace.connectivity
                    monitor.tick()
                    self.assertEqual(client.get('/api/config').status_code, 200)
                    status = client.get('/api/models')
                    self.assertEqual(status.status_code, 200)
                    self.assertEqual(status.json()['connection']['state'], 'waiting')
                    monitor.connections['left'].shared.states = {i: MotorState(i,0,0,0,24,24,0,
                        time.monotonic()) for i in range(1,9)}
                    monitor.tick()
                    self.assertEqual(client.get('/api/models').json()['connection']['state'], 'ready')
                    with patch.object(app.state.model_workspace, 'snapshot', side_effect=OSError('temporary file replacement')):
                        unavailable = client.get('/api/models')
                        self.assertEqual(unavailable.status_code, 503)
                        self.assertTrue(unavailable.json()['retryable'])
                    self.assertEqual(client.get('/api/models').status_code, 200)
                    manager.claim_motion.assert_not_called()
                    manager.activate.assert_not_called()
            self.assertFalse(app.state.model_workspace.connectivity.connections)
