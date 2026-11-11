"""Synthetic, hardware-free regression tests for the separate stationary contract."""
import copy
import importlib.util
import unittest

NUMERIC = all(importlib.util.find_spec(n) for n in ('numpy', 'scipy', 'cv2'))


class SyntheticChain:
    metadata = {'urdf_sha256': '0'*64, 'frame': 'synthetic_tool', 'signs': [1]*7}

    def forward(self, joints):
        import numpy as np
        from scipy.spatial.transform import Rotation
        T = np.eye(4)
        T[:3, :3] = Rotation.from_rotvec(joints[:3]).as_matrix()
        T[:3, 3] = joints[3:6]
        return T


@unittest.skipUnless(NUMERIC, 'optional scientific/camera libraries')
class StationaryCalibrationTests(unittest.TestCase):
    def fixture(self):
        import numpy as np
        from scipy.spatial.transform import Rotation
        from openarm_runtime.stationary_calibration import digest
        rng = np.random.default_rng(29)
        X, Y = np.eye(4), np.eye(4)
        X[:3, :3] = Rotation.from_rotvec([.12, -.24, .2]).as_matrix()
        X[:3, 3] = [.1, -.2, .3]
        Y[:3, :3] = Rotation.from_rotvec([-.1, .15, .07]).as_matrix()
        Y[:3, 3] = [.04, .01, .08]
        chain = SyntheticChain()
        samples, points = [], []
        for index in range(16):
            q = rng.uniform(-.4, .4, 6).tolist()+[0., .1]
            role = 'fit' if index < 12 else 'validation'
            point = dict(id=f'pose-{index}', role=role, joints_rad=q)
            points.append(point)
            image_hash = digest({'image': index})
            start = 10+index*10
            trace = [dict(sequence=j+1, source='measured_feedback', status=[1]*8,
                positions_rad=list(q), velocities_rad_s=[0.]*8,
                observed_at_s=start-.7+j*.05+.008,
                received_at_s=[start-.7+j*.05+k*.001 for k in range(8)])
                for j in range(24)]
            C = np.linalg.inv(X)@chain.forward(q[:7])@Y
            sample = dict(waypoint_id=point['id'], role=role, png_sha256=image_hash,
                frame_sequence=index+1, camera_identity=dict(camera_name='synthetic',
                    source_id='synthetic-run', width=1280, height=720, simulation=True),
                measured_joints_rad=list(q), camera_from_board=C.tolist(), reprojection_rms_px=.2,
                stationary_evidence=dict(schema='openarm.stationary-bracket.v1',
                    acquisition=dict(clock='host_monotonic', semantics='qualified_acquisition_interval',
                        qualification='synthetic_test', png_sha256=image_hash,
                        qualification_evidence_sha256=digest({'source': 'synthetic_fixture'}),
                        earliest_s=start, latest_s=start+.1, host_received_at_s=start+.12),
                    feedback=trace))
            samples.append(sample)
        route = dict(schema='openarm.calibration-route.v1', side='left',
                     start_rad=[0.]*7+[.1], waypoints=points)
        data = dict(schema='openarm.stationary-head-observations.v1', units='m',
            joint_state_source='measured_feedback', kinematics_sha256=digest(chain.metadata),
            camera_name='synthetic', base_frame='synthetic_base', tool_frame='synthetic_tool',
            intrinsics_sha256='1'*64, route_sha256=digest(route), route=route, samples=samples)
        return data, chain, X, Y

    def test_recovers_both_unknown_transforms_without_fabricated_exposure(self):
        import numpy as np
        from openarm_runtime.stationary_calibration import fit_stationary
        data, chain, X, Y = self.fixture()
        original = copy.deepcopy(data)
        result = fit_stationary(data, chain)
        np.testing.assert_allclose(result['transform'], X, atol=1e-8)
        np.testing.assert_allclose(result['tool_from_board'], Y, atol=1e-8)
        self.assertEqual(result['metrics']['fit']['samples'], 12)
        self.assertEqual(result['metrics']['validation']['samples'], 4)
        self.assertFalse(result['installed'])
        self.assertTrue(result['synthetic'])
        self.assertEqual(data, original)
        self.assertNotIn('max_pair_skew_s', result)
        self.assertNotIn('image_captured_at_s', data['samples'][0])

    def test_validation_data_cannot_change_fit(self):
        import numpy as np
        from openarm_runtime.stationary_calibration import fit_stationary
        data, chain, _, _ = self.fixture()
        baseline = fit_stationary(data, chain)
        for sample in data['samples'][12:]:
            sample['camera_from_board'][0][3] += .001
        result = fit_stationary(data, chain)
        np.testing.assert_array_equal(result['transform'], baseline['transform'])
        np.testing.assert_array_equal(result['tool_from_board'], baseline['tool_from_board'])
        self.assertGreater(result['metrics']['validation']['rms_translation_m'], .0005)

    def test_v2_speed_bias_preserved_but_real_motion_and_stale_receipts_rejected(self):
        from openarm_runtime.stationary_calibration import validate_stationary
        sample = self.fixture()[0]['samples'][0]
        for row in sample['stationary_evidence']['feedback']:
            row['velocities_rad_s'] = [.033]*8
        with self.assertRaisesRegex(ValueError, 'moved'):
            validate_stationary(sample)  # Historical v1 is not reinterpreted.
        sample['stationary_evidence']['schema'] = 'openarm.stationary-bracket.v2'
        result = validate_stationary(sample)
        self.assertEqual(result['stationarity_method'], 'position_envelope')
        self.assertEqual(result['max_velocity_rad_s'], .033)
        moved = copy.deepcopy(sample)
        moved['stationary_evidence']['feedback'][10]['positions_rad'][0] += .004
        with self.assertRaisesRegex(ValueError, 'moved'):
            validate_stationary(moved)
        stale = copy.deepcopy(sample)
        stale['stationary_evidence']['feedback'][10]['received_at_s'] = stale['stationary_evidence']['feedback'][9]['received_at_s']
        with self.assertRaises(ValueError):
            validate_stationary(stale)

    def test_bad_holdout_rejects_without_outlier_removal(self):
        from openarm_runtime.stationary_calibration import fit_stationary
        data, chain, _, _ = self.fixture()
        data['samples'][-1]['camera_from_board'][0][3] += .1
        with self.assertRaisesRegex(ValueError, 'validation:'):
            fit_stationary(data, chain)
        self.assertEqual(len(data['samples']), 16)

    def test_false_timing_and_bad_feedback_rejected(self):
        from openarm_runtime.stationary_calibration import validate_stationary
        original = self.fixture()[0]['samples'][0]
        edits = [
            lambda s: s['stationary_evidence']['acquisition'].update(semantics='host_received_not_exposure'),
            lambda s: s['stationary_evidence']['acquisition'].update(qualification='operator_guessed_delay'),
            lambda s: s['stationary_evidence']['acquisition'].update(png_sha256='2'*64),
            lambda s: s['stationary_evidence']['acquisition'].update(latest_s=float('nan')),
            lambda s: s['stationary_evidence']['acquisition'].update(earliest_s=True),
            lambda s: s['stationary_evidence']['acquisition'].update(earliest_s=0.),
            lambda s: s['stationary_evidence']['feedback'][10]['positions_rad'].__setitem__(7, .11),
            lambda s: s['stationary_evidence']['feedback'][10]['velocities_rad_s'].__setitem__(0, .02),
            lambda s: s['stationary_evidence']['feedback'][10]['received_at_s'].__setitem__(0, 0.),
            lambda s: s['stationary_evidence']['feedback'][10].update(sequence=1),
            lambda s: s['stationary_evidence']['feedback'][10].update(source='commanded_reference'),
            lambda s: s['stationary_evidence']['feedback'][10].update(status=[0]*8),
            lambda s: s['stationary_evidence']['feedback'][0]['received_at_s'].__setitem__(7, 10.),
            lambda s: s['stationary_evidence']['feedback'][-1]['received_at_s'].__setitem__(7, 10.1),
            lambda s: s['measured_joints_rad'].__setitem__(0, .6),
        ]
        for edit in edits:
            sample = copy.deepcopy(original)
            edit(sample)
            with self.subTest(edit=edits.index(edit)), self.assertRaises(ValueError):
                validate_stationary(sample)

    def test_identity_roles_duplicates_and_fk_mapping_rejected(self):
        from openarm_runtime.stationary_calibration import fit_stationary
        original, chain, _, _ = self.fixture()
        edits = [
            lambda d: d.update(kinematics_sha256='f'*64),
            lambda d: d.update(joint_state_source='commanded_reference'),
            lambda d: d['samples'][-1].update(role='fit'),
            lambda d: d['samples'].reverse(),
            lambda d: d['samples'][1].update(png_sha256=d['samples'][0]['png_sha256']),
            lambda d: d['samples'][1].update(frame_sequence=1),
            lambda d: d['samples'][1]['camera_identity'].update(source_id='reconnected'),
            lambda d: d['samples'][1].update(reprojection_rms_px=2.),
            lambda d: d['route']['waypoints'][0].update(role='validation'),
        ]
        for edit in edits:
            data = copy.deepcopy(original)
            edit(data)
            with self.subTest(edit=edits.index(edit)), self.assertRaises(ValueError):
                fit_stationary(data, chain)

    def test_underexcited_measured_fk_rejects(self):
        from openarm_runtime.stationary_calibration import digest, fit_stationary
        data, chain, _, _ = self.fixture()
        for sample in data['samples']:
            sample['measured_joints_rad'][:3] = [0.]*3
            for row in sample['stationary_evidence']['feedback']:
                row['positions_rad'][:3] = [0.]*3
        for point in data['route']['waypoints']:
            point['joints_rad'][:3] = [0.]*3
        data['route_sha256'] = digest(data['route'])
        with self.assertRaisesRegex(ValueError, 'two axes'):
            fit_stationary(data, chain)

    def test_old_synchronized_contract_still_rejects_receipt_only_input(self):
        from openarm_runtime.camera_calibration import fit
        data = self.fixture()[0]
        with self.assertRaises(ValueError):
            fit(data)
