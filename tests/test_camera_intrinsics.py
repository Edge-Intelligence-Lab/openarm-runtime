"""Known-camera synthetic projections and real indexed detection, never devices."""
import copy
import importlib.util
import unittest

AVAILABLE = all(importlib.util.find_spec(n) for n in ('cv2', 'numpy', 'scipy'))
SPEC = dict(squares_x=7, squares_y=5, square_size_m=.025,
            marker_size_m=.018, dictionary='DICT_4X4_50')


@unittest.skipUnless(AVAILABLE, 'calibration extra')
class IntrinsicsTests(unittest.TestCase):
    def synthetic(self):
        import cv2
        import numpy as np
        from openarm_runtime.camera_intrinsics import board
        K = np.array([[780.,0,400],[0,800.,300],[0,0,1.]])
        objects = board(SPEC).getChessboardCorners()
        rng = np.random.default_rng(9)
        samples = []
        for i in range(16):
            r = rng.uniform(-.6,.6,3)
            t = np.array([-.08,-.05,rng.uniform(.45,.8)])
            pixels = cv2.projectPoints(objects, r, t, K, np.zeros(5))[0].reshape(-1,2)
            samples.append(dict(id=str(i), role='fit' if i<12 else 'validation',
                detection=dict(objects=objects.tolist(),pixels=pixels.tolist())))
        return samples,K

    def test_recovers_known_intrinsics_and_holds_out_views(self):
        import numpy as np
        from openarm_runtime.camera_intrinsics import solve
        samples,K = self.synthetic()
        result = solve(samples,[800,600])
        np.testing.assert_allclose(result['matrix'],K,atol=.02)
        self.assertLess(result['validation_rms_px'],.001)
        changed = copy.deepcopy(samples)
        changed[-1]['detection']['pixels'][0][0] += 20
        other = solve(changed,[800,600])
        np.testing.assert_array_equal(result['matrix'],other['matrix'])
        self.assertGreater(other['validation_rms_px'],result['validation_rms_px'])
        self.assertTrue(set(result['fit_ids']).isdisjoint(result['validation_ids']))
        self.assertEqual(result['calibration_status'],'estimated_not_independently_verified')

    def test_requires_fit_validation_and_tilt_diversity(self):
        from openarm_runtime.camera_intrinsics import solve
        samples,_ = self.synthetic()
        for data in (samples[:8], [dict(s,role='validation') for s in samples],
                     [dict(samples[0],id=str(i),role='fit' if i<12 else 'validation') for i in range(16)]):
            with self.assertRaises(ValueError):
                solve(data,[800,600])

    def test_indexed_detector_and_explicit_board_parameters(self):
        import cv2
        import numpy as np
        from openarm_runtime.camera_intrinsics import board, detect
        image = np.full((600,800),255,np.uint8)
        image[100:500,120:680] = board(SPEC).generateImage((560,400))
        result = detect(cv2.cvtColor(image,cv2.COLOR_GRAY2RGB),SPEC)
        self.assertEqual(len(result['corner_ids']),24)
        self.assertGreater(result['coverage_fraction'],.1)
        for spec in ({},dict(SPEC,square_size_m=True),dict(SPEC,dictionary='guess')):
            with self.assertRaises(ValueError): board(spec)
        with self.assertRaises(ValueError): detect(np.zeros((600,800,3),np.uint8),SPEC)

    def test_plain_10_by_7_checkerboard_detects_54_metric_corners(self):
        import cv2
        import numpy as np
        from openarm_runtime.camera_intrinsics import board, detect
        spec = dict(type='checkerboard',squares_x=10,squares_y=7,square_size_m=.015)
        image = np.full((520,700),255,np.uint8)
        for y in range(7):
            for x in range(10):
                image[70+y*50:70+(y+1)*50,100+x*50:100+(x+1)*50] = 255*((x+y)%2)
        for frame in (image, cv2.rotate(image,cv2.ROTATE_180), cv2.rotate(image,cv2.ROTATE_90_CLOCKWISE)):
            result = detect(cv2.cvtColor(frame,cv2.COLOR_GRAY2RGB),spec)
            self.assertEqual(len(result['pixels']),54)
            self.assertEqual(len(set(result['corner_ids'])),54)
            self.assertEqual(result['board_frame_status'],'per_image_grid_ambiguous_for_extrinsics')
            points = np.asarray(result['objects'])
            np.testing.assert_allclose(points.max(axis=0),[.12,.075,0],atol=1e-8)
        self.assertEqual(board(spec).getChessboardCorners().shape,(54,3))
        for bad in (dict(spec,square_size_m=True), dict(spec,squares_x=10.5),
                    dict(spec,square_size_m=float('nan')),dict(spec,square_size_m=0),
                    dict(spec,marker_size_m=.01),dict(spec,type='unknown')):
            with self.assertRaises(ValueError):board(bad)
        with self.assertRaises(ValueError):detect(np.zeros((520,700,3),np.uint8),spec)

    def test_checkerboard_intrinsics_preserve_holdout_and_metric_grid(self):
        import cv2
        import numpy as np
        from openarm_runtime.camera_intrinsics import board, solve
        spec = dict(type='checkerboard',squares_x=10,squares_y=7,square_size_m=.015)
        points = board(spec).getChessboardCorners()
        K = np.array([[780.,0,400],[0,800.,300],[0,0,1.]])
        samples=[]
        rng=np.random.default_rng(19)
        for i in range(16):
            pixels=cv2.projectPoints(points,rng.uniform(-.6,.6,3),
                np.array([-.06,-.04,rng.uniform(.3,.6)]),K,np.zeros(5))[0].reshape(-1,2)
            samples.append(dict(id=str(i),role='fit' if i<12 else 'validation',
                detection=dict(objects=points.tolist(),pixels=pixels.tolist())))
        result=solve(samples,[800,600])
        np.testing.assert_allclose(result['matrix'],K,atol=.02)
        changed=copy.deepcopy(samples);changed[-1]['detection']['pixels'][0][0]+=20
        other=solve(changed,[800,600])
        np.testing.assert_array_equal(result['matrix'],other['matrix'])
        self.assertGreater(other['validation_rms_px'],result['validation_rms_px'])
