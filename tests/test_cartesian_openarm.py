"""Opt-in official follower checks: OPENARM_CARTESIAN_URDF=/.../v2.urdf.

No mesh files, devices, calibration claims or automatic asset downloads.
"""
import hashlib
import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from test_cartesian import AVAILABLE


@unittest.skipUnless(AVAILABLE and os.environ.get('OPENARM_CARTESIAN_URDF'),
                     'install Cartesian extra and set official URDF path')
class OfficialOpenArmTests(unittest.TestCase):
    def test_left_arm_tracking_limits_and_latency(self):
        import numpy as np
        from openarm_runtime.cartesian import KinematicChain, DifferentialIK
        path = Path(os.environ['OPENARM_CARTESIAN_URDF'])
        raw = path.read_bytes()
        self.assertEqual(hashlib.sha1(f'blob {len(raw)}\0'.encode()+raw).hexdigest(),
                         '2e99c84d467b41964f704a1c1bbc4134aa2c30b9')
        names = [f'openarm_left_joint{i}' for i in range(1, 8)]
        xml = ET.fromstring(raw)
        locked = {j.get('name'): 0. for j in xml.findall('joint')
                  if j.get('type') != 'fixed' and j.get('name') not in names}
        chain = KinematicChain(path, sha256=hashlib.sha256(raw).hexdigest(),
                               joints=names, signs=[1]*7, offsets=[0]*7, locked=locked,
                               frame='openarm_left_ee_base_link', tcp=np.eye(4))
        lo = chain.model.lowerPositionLimit[chain.indices]
        hi = chain.model.upperPositionLimit[chain.indices]
        q0 = np.clip(np.array([.1, .2, .1, .5, .1, .1, .1]), lo+.01, hi-.01)
        solver = DifferentialIK(chain, lower=lo.tolist(), upper=hi.tolist(),
                                velocity=[.5]*7, posture=q0.tolist())
        target = chain.forward(np.clip(q0 + [.03, .04, -.02, .05, .02, -.03, .02], lo+.01, hi-.01))
        q = q0.copy(); timings = []
        for _ in range(500):
            result = solver.step(q, target, .01)
            next_q = np.array(result['joints'])
            self.assertLessEqual(np.max(np.abs(next_q-q)), .00500001)
            self.assertTrue((next_q >= lo-1e-8).all() and (next_q <= hi+1e-8).all())
            q = next_q; timings.append(result['solve_s'])
        self.assertLess(result['position_error_m'], .002)
        self.assertLess(result['orientation_error_rad'], .01)
        print('\nOFFICIAL_OPENARM_OFFLINE', dict(position_m=result['position_error_m'],
              orientation_rad=result['orientation_error_rad'], median_ms=float(np.median(timings)*1000),
              p99_ms=float(np.percentile(timings, 99)*1000), max_ms=max(timings)*1000))
