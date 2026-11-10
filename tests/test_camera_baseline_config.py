"""Published configuration integrity only; no device or scientific dependencies."""
import hashlib
import json
import math
from pathlib import Path
import unittest


class HeadCameraBaselineTests(unittest.TestCase):
    def setUp(self):
        self.root=Path(__file__).resolve().parents[1]/'configs/cameras'
        self.selection=json.loads((self.root/'current-head.json').read_text())
        self.path=(self.root/self.selection['path']).resolve()
        self.data=json.loads(self.path.read_text())

    def test_selected_version_is_immutable_and_installation_specific(self):
        self.assertTrue(self.path.is_relative_to(self.root.resolve()))
        self.assertEqual(self.selection['sha256'],hashlib.sha256(self.path.read_bytes()).hexdigest())
        self.assertEqual(self.data['version'],self.selection['version'])
        self.assertEqual(self.data['installation_id'],self.selection['installation_id'])
        self.assertEqual(self.data['scope'],'this_installation_only_not_factory_calibration')
        self.assertFalse(self.selection['automatic_device_selection'])
        self.assertFalse(self.selection['automatic_hardware_activation'])
        self.assertFalse(self.data['automatically_activate_hardware'])
        self.assertFalse(self.data['independently_physically_verified'])

    def test_intrinsics_and_transform_conventions_are_complete(self):
        c=self.data['camera'];e=self.data['extrinsics']
        self.assertEqual(c['image_size'],[1280,720])
        self.assertEqual(c['distortion_order'],['k1','k2','p1','p2','k3'])
        self.assertEqual(len(c['distortion']),5)
        self.assertEqual(c['matrix'][2],[0,0,1])
        self.assertEqual(e['transform_name'],'base_from_camera')
        self.assertEqual(e['units'],'m')
        self.assertEqual(e['camera_axes'],dict(x='right',y='down',z='forward'))
        for T in (e['base_from_camera'],self.data['calibration_fixture']['tool_from_board']):
            self.assertEqual(T[3],[0,0,0,1])
            R=[row[:3] for row in T[:3]]
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(sum(R[k][i]*R[k][j] for k in range(3)),float(i==j),places=10)
            det=sum(R[0][i]*(R[1][(i+1)%3]*R[2][(i+2)%3]-R[1][(i+2)%3]*R[2][(i+1)%3]) for i in range(3))
            self.assertAlmostEqual(det,1.,places=10)
        self.assertEqual(e['mujoco_optical_rotation'],[[1,0,0],[0,-1,0],[0,0,-1]])

    def test_source_kinematics_and_validation_roles_are_preserved(self):
        robot=self.data['robot']
        raw=json.dumps(robot['kinematics'],sort_keys=True,separators=(',',':'),allow_nan=False).encode()
        self.assertEqual(hashlib.sha256(raw).hexdigest(),robot['kinematics_sha256'])
        v=self.data['validation']
        self.assertEqual(v['fit']['samples'],12)
        self.assertEqual(v['validation']['samples'],4)
        for role in ('fit','validation'):
            self.assertLess(v[role]['rms_translation_m'],v['rms_acceptance_limits']['translation_m'])
            self.assertLess(v[role]['rms_rotation_rad'],v['rms_acceptance_limits']['rotation_rad'])
        self.assertEqual(self.data['provenance']['group_id'],'ef9692cd4c164350ac70b9c4b7de5466')
        self.assertFalse(v['exposure_timestamp_known'])
        self.assertGreater(len(self.data['limitations']),0)

    def test_published_data_has_no_machine_paths_network_addresses_or_nonfinite_values(self):
        def check(value):
            if isinstance(value,dict):
                for v in value.values():check(v)
            elif isinstance(value,list):
                for v in value:check(v)
            elif isinstance(value,str):
                for forbidden in ('/home/','/Users/','yunzhen','bing-amd','/dev/','127.0.0.1','100.124.','192.168.','10.203.'):
                    self.assertNotIn(forbidden,value)
            elif isinstance(value,(int,float)):
                self.assertTrue(math.isfinite(value))
        check(self.data)

    def test_huggingface_lock_pins_an_immutable_model_asset_inventory(self):
        lock=json.loads((self.root/'huggingface-head.lock.json').read_text())
        self.assertEqual(lock['schema'],'tacpilot.artifact-lock.v1')
        self.assertEqual(lock['repo_type'],'model')
        self.assertEqual(lock['repo_id'],'tacpilot/tacpilot')
        self.assertRegex(lock['revision'],r'^[0-9a-f]{40}$')
        self.assertRegex(lock['manifest']['sha256'],r'^[0-9a-f]{64}$')
        self.assertEqual(lock['manifest']['path'],'releases/v0.3.1/manifest.json')


if __name__=='__main__':unittest.main()
