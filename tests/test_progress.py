import math
import unittest

from openarm_policy.progress import CartesianProgress


class ProgressTests(unittest.TestCase):
    def test_hysteresis_hold_resume_and_timeout(self):
        p = CartesianProgress()
        self.assertFalse(p.advance([.08]*7, [0]*7, 10))
        self.assertFalse(p.advance([.05]*7, [0]*7, 11))
        self.assertTrue(p.advance([.03]*7, [0]*7, 12))
        self.assertIsNone(p.waiting_since)
        self.assertFalse(p.advance([.08]*7, [0]*7, 13))
        with self.assertRaises(TimeoutError):p.advance([.08]*7, [0]*7, 24)

    def test_invalid_bounds_reject(self):
        for kwargs in ({'pause_joint_error_rad': .2}, {'translation_step_m': .1},
                       {'resume_joint_error_rad': .09}, {'pause_timeout_s': math.nan}):
            with self.assertRaises(ValueError):CartesianProgress(**kwargs)
        with self.assertRaises(ValueError):CartesianProgress().advance([0]*7, [0]*6+[math.nan], 0)

    def test_timeout_names_limiting_axis_without_changing_resume_boundary(self):
        p = CartesianProgress()
        self.assertFalse(p.advance([0.]*6+[.08], [0.]*7, 10.))
        with self.assertRaisesRegex(TimeoutError,
                r'Cartesian progress failed to settle: J7 error=0.042000 rad, resume<=0.040000 rad'):
            p.advance([0.]*6+[.042], [0.]*7, 20.1)
        self.assertEqual(p.resume_error, .04)
        self.assertEqual(p.timeout, 10.)

    def test_target_bounds_and_quaternion_sign_equivalence(self):
        try:
            import numpy as np
            from scipy.spatial.transform import Rotation
        except ImportError:
            self.skipTest('optional numeric dependencies')
        p = CartesianProgress()
        base = [0., 0., 0., 0., 0., 0., 1., .1]
        goal = [1., 0., 0., *Rotation.from_rotvec([0., 0., 1.]).as_quat(), .05]
        target = p.target(base, goal)
        self.assertAlmostEqual(np.linalg.norm(target[:3]), .002)
        self.assertAlmostEqual(Rotation.from_quat(target[3:7]).magnitude(), .015)
        same = p.target(base, [0., 0., 0., 0., 0., 0., -1., .1])
        self.assertAlmostEqual(Rotation.from_quat(same[3:7]).magnitude(), 0)
