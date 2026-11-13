"""Small offline action-boundary checks for the published foundation adapter."""
import json
from pathlib import Path
import tempfile
import unittest


class FoundationChecks(unittest.TestCase):
    def test_gripper_only_and_world_delta(self):
        import numpy as np
        from openarm_policy.contracts import Observation, SensorSample
        from openarm_policy.models.foundation import FoundationActions
        contract = dict(schema='tacpilot.upright-normalized-delta.v1', dimension=7,
            translation_scale_m=.02, rotation_scale_rad=.1,
            translation_frame='world; relative to previous target',
            rotation_frame='world; left multiplication of previous target rotation',
            gripper='absolute normalized -1 open, +1 closed', control_hz=20,
            open_finger_target_rad=.7854, closed_finger_target_rad=-.1, executor='PaxiniTaskEnv.step')
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/'action.json';p.write_text(json.dumps(contract))
            decoder = FoundationActions(p, .005, .15, [[-1, .15], [1, .005]])
            obs = Observation(1, 10., {'target/left': (SensorSample(1, 9.9, [.25,.2,1.04,0,0,0,1,.15]),)})
            raw = np.zeros((100, 7));raw[:, 0] = .1;raw[:, 6] = -1.0176
            result = decoder.decode(raw, obs)
            self.assertEqual(len(result), 25)
            np.testing.assert_allclose(result[-1][:3], [.3, .2, 1.04])
            self.assertEqual(result[0][7], .15)
            self.assertEqual(raw[0, 6], -1.0176)
            raw[0, 0] = 1.001
            with self.assertRaises(ValueError):decoder.decode(raw, obs)
            raw[0, 0] = float('nan')
            with self.assertRaises(ValueError):decoder.decode(raw, obs)


if __name__ == '__main__':
    unittest.main()
