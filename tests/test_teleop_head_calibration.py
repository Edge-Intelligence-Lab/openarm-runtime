"""Passive synthetic feedback only; no serial/CAN or physical following."""
import copy
import importlib.util
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

NUMERIC=all(importlib.util.find_spec(n) for n in ('numpy','scipy','cv2'))


@unittest.skipUnless(NUMERIC,'calibration extras')
class TeleopHeadTests(unittest.TestCase):
    def setUp(self):
        from test_stationary_calibration import StationaryCalibrationTests
        from openarm_platform.teleop_calibration import TeleopHeadCalibration
        self.data,self.chain,self.X,self.Y=StationaryCalibrationTests().fixture()
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.now=0.
        with patch('openarm_platform.teleop_calibration.load_setup',return_value=(
                self.data,SimpleNamespace(make_gravity=lambda *a,**k:object()),self.chain,{'simulation':False})):
            self.cal=TeleopHeadCalibration('fixture',self.temp.name,5003,clock=lambda:self.now)

    def feed(self,sample,*,owner='owner',generation=('run',0),grip=None,following=True):
        for row in sample['stationary_evidence']['feedback']:
            feedback={i:dict(status=row['status'][i-1],position=row['positions_rad'][i-1],
                velocity=row['velocities_rad_s'][i-1],received_at=row['received_at_s'][i-1]) for i in range(1,9)}
            self.now=row['observed_at_s']
            self.cal.observe('left',dict(feedback=feedback,targets_rad={8:.1 if grip is None else grip},
                exchange_completed_at=self.now,gravity={'mode':'active'}),owner=owner,generation=generation,following=following)

    def test_operator_capture_keeps_12_4_roles_and_solves_without_motion(self):
        import numpy as np
        self.feed(self.data['samples'][0]);self.cal.begin('owner',mounted=True)
        with patch('openarm_platform.teleop_calibration.board_fits_image',return_value=True):
            for i,original in enumerate(self.data['samples']):
                if i:self.feed(original)
                obs={k:copy.deepcopy(v) for k,v in original.items() if k not in
                    ('stationary_evidence','measured_joints_rad','waypoint_id','role')}
                obs['acquisition']=original['stationary_evidence']['acquisition']
                self.cal.camera.read=lambda o=obs:(b'original-png',o,{'synthetic_fixture':True})
                self.assertEqual(self.cal.capture('owner'),i+1)
        self.cal.ended()  # Complete group can be solved after operator stops.
        result=self.cal.solve('owner')
        np.testing.assert_allclose(result['transform'],self.X,atol=1e-8)
        self.assertFalse(result['installed'])
        self.assertEqual(result['metrics']['validation']['samples'],4)
        self.assertTrue((self.cal.directory/'session.json').is_file())
        self.assertEqual((self.cal.directory/self.cal.samples[0]['artifact_path']/'image.png').read_bytes(),b'original-png')

    def test_open_then_reclose_or_retakeover_latches_group_invalid(self):
        self.feed(self.data['samples'][0]);self.cal.begin('owner',mounted=True)
        self.feed(self.data['samples'][0],grip=.12)
        self.feed(self.data['samples'][0],grip=.1)
        self.assertIn('夹爪',self.cal.status()['error'])
        with self.assertRaises(ValueError):self.cal.capture('owner')
        self.cal.begin('owner',mounted=True)
        self.feed(self.data['samples'][0],generation=('run',1))
        self.assertIn('接管',self.cal.status()['error'])

    def test_mount_holds_only_gripper_through_leader_jitter_until_stop(self):
        from openarm_runtime.teleoperation import Teleoperation
        sample=self.data['samples'][0]
        self.feed(sample)
        session=SimpleNamespace(head_calibration=self.cal)
        received=[]
        def track(targets):
            received.append(dict(targets))
            self.feed(sample,grip=targets[8])
        control=SimpleNamespace(track=track)
        targets={i:.01*i for i in range(1,9)}
        targets[8]=.104397147  # two actual leader encoder ticks above .1
        Teleoperation._track(session,'left',control,targets)
        self.assertEqual(received[-1],targets)  # mounting is still operator-driven
        self.cal.begin('owner',mounted=True)
        held=self.cal.grip_command
        for target in (.1,.12,.02):
            targets[8]=target
            Teleoperation._track(session,'left',control,targets)
            self.assertEqual(received[-1][8],held)
            self.assertEqual({i:received[-1][i] for i in range(1,8)},
                             {i:targets[i] for i in range(1,8)})
            self.assertIsNone(self.cal.status()['error'])
            self.assertEqual(self.cal.status()['count'],0)  # never automatic capture
        self.assertEqual(targets[8],.02)  # caller's mapping remains untouched
        self.assertEqual(self.cal.hold_targets('right',targets),targets)
        self.cal.error='test group rejected'
        self.assertEqual(self.cal.hold_targets('left',targets)[8],held)
        self.cal.ended()
        self.assertEqual(self.cal.status()['error'],'test group rejected')
        self.assertFalse(self.cal.status()['gripper_held'])
        self.assertEqual(self.cal.hold_targets('left',targets),targets)

    def test_real_measured_grip_change_still_invalidates_held_group(self):
        sample=copy.deepcopy(self.data['samples'][0])
        self.feed(sample);self.cal.begin('owner',mounted=True)
        for row in sample['stationary_evidence']['feedback']:
            row['positions_rad'][7]+=.004
        self.feed(sample)
        self.assertIn('实测开度',self.cal.status()['error'])
        self.assertTrue(self.cal.status()['gripper_held'])
        with self.assertRaises(ValueError):self.cal.capture('owner')

    def test_mount_snapshot_waits_for_inflight_exchange(self):
        import threading
        from openarm_runtime.teleoperation import Teleoperation
        sample=self.data['samples'][0];self.feed(sample)
        entered=threading.Event();release=threading.Event();mounted=threading.Event()
        errors=[]
        def track(targets):
            entered.set()
            if not release.wait(2):raise RuntimeError('test exchange timed out')
            self.feed(sample,grip=targets[8])
        def mount():
            try:self.cal.begin('owner',mounted=True)
            except Exception as exc:errors.append(exc)
            finally:mounted.set()
        worker=threading.Thread(target=lambda:Teleoperation._track(
            SimpleNamespace(head_calibration=self.cal),'left',SimpleNamespace(track=track),{8:.12}))
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            starter=threading.Thread(target=mount);starter.start()
            self.assertFalse(mounted.wait(.05))
        finally:
            release.set();worker.join(2)
        starter.join(2)
        self.assertEqual(errors,[])
        self.assertTrue(mounted.is_set())
        self.assertEqual(self.cal.grip_command,.12)

    def test_requires_owner_fresh_feedback_and_explicit_fixture_confirmation(self):
        with self.assertRaises(ValueError):self.cal.begin('owner',mounted=True)
        self.feed(self.data['samples'][0])
        with self.assertRaises(ValueError):self.cal.begin('other',mounted=True)
        with self.assertRaises(ValueError):self.cal.begin('owner')
        self.cal.begin('owner',mounted=True)
        with self.assertRaises(ValueError):self.cal.begin('other',mounted=True)
        self.assertIsNone(self.cal.status()['error'])
        self.now+=.2
        with self.assertRaises(ValueError):self.cal.begin('owner',mounted=True)

    def test_manual_protocol_cannot_reclassify_validation(self):
        from openarm_runtime.stationary_calibration import fit_stationary,digest
        data=copy.deepcopy(self.data)
        route=data['route'];route['schema']='openarm.calibration-manual-route.v1'
        route['protocol']=dict(schema='openarm.manual-head-protocol.v1',slots=[
            {'id':p['id'],'role':p['role']} for p in route['waypoints']])
        data['manual_protocol_sha256']=digest(route['protocol']);data['route_sha256']=digest(route)
        fit_stationary(data,self.chain)
        route['waypoints'][0]['role']='validation';data['route_sha256']=digest(route)
        with self.assertRaisesRegex(ValueError,'12/4'):fit_stationary(data,self.chain)


class TeleopConfigurationTests(unittest.TestCase):
    def test_platform_wires_existing_service_without_a_second_controller(self):
        import json
        from openarm_platform.config import load_config
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'platform.json'
            path.write_text(json.dumps(dict(teleop_calibration_profile='head.json',
                calibration_root='evidence',services={'teleop':{'command':['python','-m','openarm_runtime',
                    '--config','robot.json','leader','--follow','--collection-profile','collection.json','--port','8890']},'vision':{'port':15003}})))
            config=load_config(path)
            command=next(p.command for p in config.pages if p.id=='teleop')
            self.assertNotIn('--head-calibration-profile',command)
            self.assertIn('--collection-profile',command)
            head=next(p for p in config.support if p.id=='head_alignment')
            self.assertEqual(head.command.count('--head-calibration-profile'),1)
            self.assertEqual(head.command[head.command.index('--head-calibration-root')+1],str(Path(root)/'evidence/teleop-head'))
            self.assertEqual(head.command[head.command.index('--head-video-port')+1],'15003')
            self.assertNotIn('--collection-profile',head.command)
            self.assertEqual(head.command[head.command.index('--port')+1],'8894')
            self.assertEqual(head.exclusive_group,'motion')
            self.assertTrue(head.lazy)
            from openarm_platform.proxy import routes
            self.assertEqual(routes(config)['/apps/head_alignment'],8894)
            from openarm_platform.services import ServiceManager
            with patch('openarm_platform.services.tcp_reachable',return_value=False):
                manager=ServiceManager(config.services,Path(root)/'logs')
                with patch.object(manager,'_launch') as launch:
                    manager.start()
                    self.assertNotIn('head_alignment',[c.args[0].page.id for c in launch.call_args_list])

    def test_simulation_profile_cannot_open_physical_leader_calibration(self):
        import json
        from openarm_platform.config import load_config
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'platform.json'
            path.write_text(json.dumps(dict(teleop_calibration_profile='head.json',simulation_profile='sim.json')))
            with self.assertRaisesRegex(ValueError,'simulation'):load_config(path)
