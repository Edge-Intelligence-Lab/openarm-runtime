import unittest
from types import SimpleNamespace as NS

from openarm_runtime.integral import TrackingIntegral


class IntegralTests(unittest.TestCase):
    def setUp(self):
        self.i = TrackingIntegral()
        self.states = {j: NS(position=0., velocity=0.) for j in range(1, 9)}
        self.targets = dict.fromkeys(range(1, 9), .04)
        self.ranges = {j: NS(torque=10.) for j in range(1, 9)}
        self.ff = dict.fromkeys(range(1, 9), 0.)

    def update(self, now, ff=None):
        return self.i.update(self.states, self.targets, [10.]*8, [.5]*8,
                             self.ranges, self.ff if ff is None else ff, now)

    def test_static_error_builds_torque_and_caps_without_windup(self):
        for n in range(1001):
            out = self.update(n*.05)
        self.assertAlmostEqual(out[7], 2.5)
        self.assertEqual(out[8], 0.)
        self.assertAlmostEqual(self.i.values[7], 2.5)

    def test_slew_combined_headroom_and_error_reversal(self):
        self.update(0.)
        out = self.update(.05)
        self.assertLessEqual(out[7], .05)
        ff = dict.fromkeys(range(1, 9), 8.4)
        for n in range(2, 102):
            out = self.update(n*.05, ff)
        self.assertLessEqual(out[7]+.4, 8.9+1e-10)
        before = self.i.values[7]
        self.targets[7] = -.04
        self.update(5.1, ff)
        self.assertLess(self.i.values[7], before)

    def test_motion_large_error_deadline_and_disable_reset(self):
        self.update(0.); self.update(.05)
        self.states[7].velocity = .2
        self.assertEqual(self.update(.1)[7], 0.)
        self.targets[6] = .2
        self.assertEqual(self.update(.15)[6], 0.)
        with self.assertRaises(RuntimeError):
            self.update(1.)
        self.assertTrue(all(v == 0 for v in self.i.values.values()))
        self.i.reset()
        self.assertIsNone(self.i.previous)

    def test_invalid_configuration(self):
        for options in ({'torque_fraction': 1.}, {'gain_per_s': float('nan')},
                        {'slew_fraction_per_s': 0}, {'gain_per_s': True}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                TrackingIntegral(**options)
