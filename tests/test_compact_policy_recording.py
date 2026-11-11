"""Summary-only policy receipts retain cleanup, never frames or full actions."""
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace

from openarm_policy.recording import CompactEpisodeRecorder


class CompactRecordingTests(unittest.TestCase):
    def test_high_rate_control_samples_do_not_evict_phase_events(self):
        with tempfile.TemporaryDirectory() as folder:
            with CompactEpisodeRecorder(Path(folder)/'episode') as rec:
                rec.record({'event': 'task_started'})
                for n in range(100):
                    rec.record({'event': 'gravity_sample', 'value': n})
                    rec.record({'event': 'integral_sample', 'value': n})
            value = json.loads((Path(folder)/'episode/summary.json').read_text())
            self.assertEqual(value['lifecycle'][0]['event'], 'task_started')
            self.assertEqual(value['last_control_samples']['gravity_sample']['value'], 99)
    def test_no_images_actions_or_replay_sidecars_and_unknown_task_success(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)/"episode"
            with CompactEpisodeRecorder(root) as recorder:
                recorder.record({"event": "context", "context": {"task": "synthetic"}})
                for i in range(100):
                    # Opaque objects would fail serialization if observations were retained.
                    recorder.record({"event": "observation", "observation": SimpleNamespace(sequence=i, streams=object())})
                    recorder.record({"event": "command", "values": object()})
                    recorder.record({"event": "chunk", "latency_s": .02, "chunk": {
                        "steps": object(), "diagnostics": {"raw_actions": object(),
                            "model": {"model_sha256": "fixture"}}}})
                recorder.record({"event": "ended", "state": "stopped", "error": "", "cleanup_errors": []})
            self.assertEqual([p.name for p in root.iterdir()], ["summary.json"])
            value = json.loads((root/"summary.json").read_text())
            self.assertIsNone(value["task_success"])
            self.assertTrue(value["complete"])
            self.assertEqual(value["model"]["model"]["model_sha256"], "fixture")
            self.assertEqual(value["event_counts"]["observation"], 100)
            self.assertEqual(value["inference_latency_s"]["count"], 100)
            self.assertNotIn("raw_actions", (root/"summary.json").read_text())
            self.assertLess((root/"summary.json").stat().st_size, 4096)

    def test_bounded_lifecycle_last_state_and_failed_cleanup_preserved(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)/"episode"
            with CompactEpisodeRecorder(root) as recorder:
                for i in range(100):
                    recorder.record({"event": "cartesian_dispatch", "measured_task_tcp": [i]})
                    recorder.record({"event": "preparation_waypoint", "waypoint": i})
                recorder.record({"event": "ended", "state": "failed", "error": "stale feedback",
                                 "cleanup_errors": ["disable verification failed"]})
            value = json.loads((root/"summary.json").read_text())
            self.assertLessEqual(len(value["lifecycle"]), 32)
            self.assertEqual(value["last_cartesian_state"]["measured_task_tcp"], [99])
            self.assertEqual(value["termination"]["cleanup_errors"], ["disable verification failed"])
            self.assertEqual(value["termination"]["error"], "stale feedback")
            self.assertIsNone(value["task_success"])

    def test_close_without_terminal_event_is_incomplete(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)/"episode"
            with CompactEpisodeRecorder(root):
                pass
            self.assertFalse(json.loads((root/"summary.json").read_text())["complete"])
            with self.assertRaises(FileExistsError): CompactEpisodeRecorder(root)


if __name__ == "__main__":
    unittest.main()
