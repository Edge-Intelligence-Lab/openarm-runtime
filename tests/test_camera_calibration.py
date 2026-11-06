import importlib.util
import unittest

NUMERIC = all(importlib.util.find_spec(n) for n in ('numpy', 'scipy', 'cv2'))


@unittest.skipUnless(NUMERIC, 'optional scientific/camera libraries')
class CalibrationTests(unittest.TestCase):
    def observations(self, mode):
        import numpy as np
        from scipy.spatial.transform import Rotation
        rng = np.random.default_rng(18)
        def pose(r, p):
            T = np.eye(4);T[:3, :3] = Rotation.from_rotvec(r).as_matrix();T[:3, 3] = p
            return T
        X = pose([.2, -.15, .3], [.05, -.04, .08])
        Y = pose([.1, .05, -.2], [.2, .2, 1.])
        samples = []
        for i in range(16):
            B = pose(rng.uniform(-.6, .6, 3), rng.uniform(-.3, .3, 3)+[0, 0, .8])
            C = np.linalg.inv(X)@np.linalg.inv(B)@Y if mode == 'wrist' else np.linalg.inv(X)@B@Y
            samples.append(dict(base_from_tool=B.tolist(), camera_from_board=C.tolist(), reprojection_rms_px=.1,
                                image_captured_at_s=10+i, robot_received_at_s=10+i+.01))
        return dict(schema='openarm.camera-calibration.observations.v1', mode=mode, units='m',
                    joint_state_source='measured_feedback', base_frame='base', tool_frame='tcp',
                    camera_name='fixture', robot_model_sha256='0'*64, intrinsics_source='synthetic', samples=samples), X

    def test_both_camera_installations_recover_known_extrinsics(self):
        import numpy as np
        from openarm_runtime.camera_calibration import fit
        for mode in ('head', 'wrist'):
            data, expected = self.observations(mode)
            result = fit(data)
            np.testing.assert_allclose(result['transform'], expected, atol=1e-8)
            self.assertEqual(result['calibration_status'], 'estimated_not_independently_verified')
            self.assertLess(result['rms_translation_m'], 1e-8)

    def test_degenerate_unsynchronized_and_commanded_poses_reject(self):
        from openarm_runtime.camera_calibration import fit
        data, _ = self.observations('wrist');data['joint_state_source'] = 'commanded_reference'
        with self.assertRaises(ValueError):fit(data)
        data, _ = self.observations('wrist');data['samples'] = [data['samples'][0]]*16
        with self.assertRaises(ValueError):fit(data)
        data, _ = self.observations('head');data['samples'][0]['robot_received_at_s'] += 1
        with self.assertRaises(ValueError):fit(data)
        data, _ = self.observations('wrist');data['samples'][0]['reprojection_rms_px'] = 2.
        with self.assertRaises(ValueError):fit(data)

    def test_metric_pnp_board_pose_and_reprojection(self):
        import cv2
        import numpy as np
        from openarm_runtime.camera_calibration import board_pose
        points = np.zeros((9*6, 3));points[:, :2] = np.mgrid[:9, :6].T.reshape(-1, 2)*.025
        K = np.array([[600., 0, 320], [0, 600, 240], [0, 0, 1.]])
        r, t = np.array([.1, -.2, .1]), np.array([-.1, -.05, .8])
        pixels = cv2.projectPoints(points, r, t, K, np.zeros(5))[0]
        pose, rms = board_pose(pixels, K, np.zeros(5), [9, 6], .025)
        np.testing.assert_allclose(pose[:3, 3], t, atol=1e-7)
        np.testing.assert_allclose(pose[:3, :3], cv2.Rodrigues(r)[0], atol=1e-7)
        self.assertLess(rms, 1e-5)

    def test_indexed_charuco_detection_preserves_board_ids(self):
        import cv2
        import numpy as np
        from openarm_runtime.camera_calibration import charuco_pose
        if not hasattr(getattr(cv2, 'aruco', None), 'CharucoDetector'):
            self.skipTest('optional OpenCV aruco support')
        spec = dict(squares_x=7, squares_y=5, square_size_m=.025,
                    marker_size_m=.018, dictionary='DICT_4X4_50')
        board = cv2.aruco.CharucoBoard((7, 5), .025, .018,
                                     cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
        image = np.full((600, 800), 255, dtype=np.uint8)
        image[100:500, 120:680] = board.generateImage((560, 400))
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        K = np.array([[800., 0, 400], [0, 800., 300], [0, 0, 1.]])
        result = charuco_pose(image, K, np.zeros(5), spec)
        self.assertEqual(len(result['corner_ids']), 24)
        self.assertLess(result['reprojection_rms_px'], .2)
        np.testing.assert_allclose(np.asarray(result['camera_from_board'])[:3, 3],
                                   [-.0875, -.0625, .25], atol=.002)
        with self.assertRaises(ValueError):charuco_pose(np.full_like(image, 255), K, np.zeros(5), spec)
