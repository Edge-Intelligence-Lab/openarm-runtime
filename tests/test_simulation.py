"""Explicit simulator boundaries; no hardware transports or external models."""
import json
import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from openarm_platform.config import load_config


class SimulationConfigurationTests(unittest.TestCase):
    def profile(self, root, **changes):
        data = dict(simulation_profile='simulation.json', log_dir='outputs/logs',
            calibration_root='outputs/calibration', collection_root='outputs/collection',
            policy_record_root='outputs/policy')
        data.update(changes)
        path = Path(root)/'platform.json'
        path.write_text(json.dumps(data))
        return load_config(path)

    def test_simulation_replaces_hardware_pages_with_builtin_routes(self):
        with tempfile.TemporaryDirectory() as root:
            config = self.profile(root)
        self.assertEqual([p.id for p in config.services], ['vision'])
        for page in config.pages[:3]:
            self.assertIsNone(page.port)
            self.assertEqual(page.path, '/simulation/'+page.id)
            self.assertFalse(page.command)

    def test_external_and_auxiliary_commands_are_rejected(self):
        for changes in (dict(services={'control': {'command': ['hardware']}}),
                        dict(services={'vision': {'command': ['camera']}}),
                        dict(support={'aux_hardware': {'port': 13000}})):
            with self.subTest(changes=changes), tempfile.TemporaryDirectory() as root:
                with self.assertRaisesRegex(ValueError, 'forbids'):
                    self.profile(root, **changes)

    def test_simulation_requires_explicit_output_paths(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'platform.json'
            path.write_text(json.dumps(dict(simulation_profile='simulation.json')))
            with self.assertRaisesRegex(ValueError, 'isolated'):
                load_config(path)

    def test_simulation_import_does_not_load_hardware(self):
        import subprocess
        import sys
        result = subprocess.run([sys.executable, '-c',
            "import sys; "
            "sys.addaudithook(lambda event,args: (_ for _ in ()).throw(AssertionError('socket opened')) "
            "if event=='socket.__new__' else None); "
            "import openarm_simulation.backend; "
            "assert 'mujoco' not in sys.modules"], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


@unittest.skipUnless(all(importlib.util.find_spec(n) for n in ('mujoco','cv2','scipy')),
                     'simulation and calibration extras')
class SimulationDeviceTests(unittest.TestCase):
    def world(self, root):
        from openarm_simulation.world import SimulationWorld
        joints = ''.join(f'<body name="link{i}" pos="0 0 .03"><joint name="j{i}"/>'
            '<geom type="sphere" size=".01" mass=".1" contype="0" conaffinity="0"/>'
            for i in range(7))
        fingers = ''.join(f'<body name="finger{i}"><joint name="f{i}"/>'
            '<geom type="sphere" size=".01" mass=".01" contype="0" conaffinity="0"/></body>'
            for i in range(2))
        xml = '<mujoco><worldbody><body name="robot">'+joints+'<body name="tool">'+fingers+\
            '</body>'+'</body>'*7+'</body></worldbody></mujoco>'
        source = Path(root)/'scene.xml'
        source.write_text(xml)
        camera = dict(position_m=[1,1,1],target_m=[0,0,.2],fovy_deg=45,width=320,height=240,fps=5)
        config = dict(initial_rad=[0.]*7+[.4],tool_from_board=[
            [1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],cameras={'head':camera,'overview':camera})
        setup = dict(kinematics=dict(joints=[f'j{i}' for i in range(7)],signs=[1]*7,offsets=[0]*7),
            board_spec=dict(squares_x=6,squares_y=4,square_size_m=.02,marker_size_m=.015,dictionary='DICT_4X4_50'))
        config['geometry']=dict(model_xml=str(source),model_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                robot_roots=['robot'],tool_body='tool',gripper_joints={'f0':[.7,0],'f1':[.7,0]},fixed_joints={})
        return SimulationWorld(config,setup)

    def test_bus_lifecycle_feedback_and_single_owner_lease(self):
        from openarm_simulation.devices import SimulationBus, SimulationCapture
        with tempfile.TemporaryDirectory() as root:
            world = self.world(root)
            bus = SimulationBus(world)
            self.assertEqual(len(bus.preflight()),8)
            self.assertTrue(world.lease.acquire(blocking=False))
            self.assertFalse(world.lease.acquire(blocking=False))
            world.lease.release()
            with self.assertRaisesRegex(RuntimeError,'disabled'):
                bus.exchange(dict(enumerate([0.]*8,1)),[1.]*8,[1.]*8)
            bus.command(1,'enable')
            self.assertEqual(bus.state(1).status,1)
            with self.assertRaisesRegex(ValueError,'already enabled'):
                bus.preflight()
            capture = SimulationCapture(world)
            self.assertTrue(all(c.calibration_identity.startswith('mujoco:') for c in capture.configs))
            self.assertNotEqual(capture.configs[0].calibration_identity,world.source_id)
            world.insert_board()
            self.assertTrue(world.state()['board_inserted'])
            world.start()
            world.close()
            self.assertFalse(world.state()['enabled'])
            self.assertFalse(world.physics.is_alive())
            self.assertEqual(bus.state(1).status,0)

    def test_insertion_requires_open_enabled_gripper(self):
        with tempfile.TemporaryDirectory() as root:
            world = self.world(root)
            with self.assertRaises(ValueError):world.insert_board()
            world.enabled = True
            world._set_pose([0.]*8)
            with self.assertRaises(ValueError):world.insert_board()
            world.close()

    def test_camera_mode_and_timing_cannot_cross_simulation_boundary(self):
        from unittest.mock import Mock
        from openarm_platform.calibration_source import CalibrationCamera
        from openarm_runtime.stationary_calibration import digest
        evidence={'source':'test'}
        acquisition=dict(semantics='qualified_acquisition_interval',qualification_evidence=evidence,
            qualification_evidence_sha256=digest(evidence),png_sha256='1'*64,
            earliest_s=9.9,latest_s=9.95,qualification='synthetic_test')
        meta=dict(acquisition=acquisition,png_sha256='1'*64,received_at_s=10.,simulation=True)
        with tempfile.TemporaryDirectory() as root:
            auto=CalibrationCamera(15003,{'camera_name':'head'},{},simulation=False,clock=lambda:10.)
            auto.fetch=lambda *a:(b'png',meta)
            with patch('openarm_platform.calibration_source.decode_frame',return_value=None):
                with self.assertRaisesRegex(ValueError,'禁止混用'):auto.read()
                auto.simulation=True
                meta['simulation']=False
                with self.assertRaisesRegex(ValueError,'禁止混用'):auto.read()
                meta['simulation']=True
                acquisition['qualification']='v4l2_monotonic_capture'
                with self.assertRaisesRegex(ValueError,'来源时序'):auto.read()
