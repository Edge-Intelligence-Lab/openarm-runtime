"""Real SDK round-trip from synthetic observations; never opens hardware."""
import importlib.util
import json
from pathlib import Path
import tempfile
import time
import unittest


@unittest.skipUnless(importlib.util.find_spec('rerun'), 'Optional recording extra')
class RerunPolicyTests(unittest.TestCase):
    def test_recording_preserves_inputs_sources_commands_and_cleanup(self):
        import numpy as np
        import rerun.chunk
        from PIL import Image
        from openarm_policy.rerun_recording import RerunEpisodeRecorder
        from openarm_policy.contracts import Observation, SensorSample
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)/'episode'
            now = time.monotonic()
            encoded = np.zeros((1, 32, 32, 3), dtype=np.uint8)
            encoded[0, 0, 0] = [13, 79, 201]
            with RerunEpisodeRecorder(root) as recorder:
                recorder.record({'event': 'retention_context', 'profile': {'fixture': True}})
                recorder.record({'event': 'preparation_started', 'time': now, 'phase': 'preparation'})
                obs = Observation(4, now, {
                    'camera/head': (SensorSample(2, now-.01, encoded[0, ..., ::-1]),),
                    'robot/left': (SensorSample(7, now, [[0., 0., 0.]]*8),)})
                recorder.record({'event': 'observation', 'time': now, 'observation': obs})
                recorder.record({'event': 'chunk', 'time': now+.1, 'latency_s': .1, 'chunk': {
                    'observation_sequence': 4, 'steps': [[0.]*8], 'diagnostics': {
                        'encoded_inputs': {'image': encoded, 'state': np.zeros((1, 8))},
                        'raw_actions': [[.1]*7]}}})
                recorder.record({'event': 'command', 'time': now+.12, 'values': [0.]*8})
                recorder.record({'event': 'ended', 'time': now+.2, 'state': 'stopped',
                                 'stop_reason': 'duration_reached', 'error': '', 'cleanup_errors': []})
            self.assertTrue(json.loads((root/'rerun-status.json').read_text())['complete'])
            self.assertTrue(json.loads((root/'summary.json').read_text())['complete'])
            manifest = json.loads((root/'model-inputs/inputs.json').read_text())
            self.assertEqual(manifest['observation_sequence'], 4)
            np.testing.assert_array_equal(np.asarray(Image.open(root/'model-inputs/image.png')), encoded[0])
            reader = rerun.chunk.RrdReader(root/'recording.rrd')
            self.assertTrue(list(reader.recordings()))

    def test_invalid_image_reports_incomplete_recording(self):
        import numpy as np
        from openarm_policy.rerun_recording import RerunEpisodeRecorder
        from openarm_policy.contracts import Observation, SensorSample
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)/'episode'
            recorder = RerunEpisodeRecorder(root)
            now = time.monotonic()
            recorder.record({'event': 'observation', 'time': now, 'observation': Observation(
                1, now, {'camera/head': (SensorSample(1, now, np.zeros((2, 2))),)})})
            with self.assertRaisesRegex(RuntimeError, 'incomplete'):
                recorder.close()
            self.assertFalse(json.loads((root/'rerun-status.json').read_text())['complete'])


if __name__ == '__main__':
    unittest.main()
