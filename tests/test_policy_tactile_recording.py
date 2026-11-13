"""Native-rate tactile sidecars without hardware or policy input changes."""
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from openarm_policy.recording import record_tactile
from openarm_tactile import Sample


class Reader:
    def __init__(self, **kwargs):
        self.device = kwargs.get('device')
        self.device_id, self.baudrate, self.simulate = 2, 921600, kwargs.get('simulate', False)
        self.on_sample = None
        self.state, self.error, self.sequence = 'waiting', None, 0
        self.received_at = None

    def snapshot(self):
        return dict(state=self.state, error=self.error, sequence=self.sequence,
                    received_at=self.received_at, age_s=0., rate_hz=60.)

    def emit(self, value):
        self.sequence += 1
        self.received_at = time.monotonic()
        sample = Sample((value, -value, value), ((value, -value, value),)*68)
        self.on_sample(sample, self.sequence, self.received_at)

    def __enter__(self):
        self.state = 'live'
        if self.on_sample is not None:
            self.emit(3)
        return self

    def __exit__(self, *_):
        self.state = 'closed'


class TactileSidecarTests(unittest.TestCase):
    def options(self):
        return {name: dict(device='fixture-'+name, device_id=2,
                           identity=dict(serial='fixture-'+name, mount='unverified'))
                for name in ('pad_a', 'pad_b')}

    def test_native_callbacks_preserve_every_frame_and_raw_signs(self):
        readers = {name: Reader(**options) for name, options in self.options().items()}
        events = Mock()
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)/'episode';root.mkdir()
            with record_tactile(root, readers, self.options(), events) as episode:
                for i in range(1, 8):
                    for reader in readers.values():
                        reader.emit(i)
                for reader in readers.values():
                    reader.state = 'closed'
            receipt = json.loads((root/'tactile.json').read_text())
            self.assertTrue(receipt['complete'])
            self.assertFalse(receipt['model_input'])
            rows = [json.loads(line) for line in (episode.path/'samples.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows), 14)
            for name in readers:
                stream = [r for r in rows if r['stream'] == 'tactile/'+name]
                self.assertEqual([r['sequence'] for r in stream], list(range(1, 8)))
                self.assertEqual(stream[-1]['data'], [[7, -7, 7]]*69)
                self.assertEqual(stream[-1]['timestamp'], readers[name].received_at)
                self.assertIsNone(readers[name].on_sample)
            self.assertEqual(episode.manifest['streams']['tactile/pad_a']['units'], 'counts')
            self.assertEqual(events.record.call_count, 2)

    def test_capture_failure_keeps_raw_data_but_marks_incomplete(self):
        options = self.options()
        readers = {name: Reader(**value) for name, value in options.items()}
        with tempfile.TemporaryDirectory() as d:
            with record_tactile(d, readers, options) as episode:
                for reader in readers.values():reader.emit(2)
                readers['pad_b'].error = 'fixture serial timeout'
            receipt = json.loads((Path(d)/'tactile.json').read_text())
            self.assertFalse(receipt['complete'])
            self.assertIn('fixture serial timeout', ' '.join(receipt['quality']['problems']))
            self.assertEqual(len((episode.path/'samples.jsonl').read_text().splitlines()), 2)

    def test_no_samples_are_missing_not_fabricated_zeros(self):
        options = self.options()
        readers = {name: Reader(**value) for name, value in options.items()}
        with tempfile.TemporaryDirectory() as d:
            with record_tactile(d, readers, options) as episode:pass
            self.assertFalse(json.loads((Path(d)/'tactile.json').read_text())['complete'])
            self.assertEqual((episode.path/'samples.jsonl').read_text(), '')

    def test_identity_and_existing_sink_are_checked_before_starting_writer(self):
        options = self.options()
        readers = {name: Reader(**value) for name, value in options.items()}
        with tempfile.TemporaryDirectory() as d:
            readers['pad_a'].on_sample = lambda *_: None
            with self.assertRaisesRegex(ValueError, 'recording sink'):
                with record_tactile(d, readers, options):pass
            readers['pad_a'].on_sample = None
            del options['pad_a']['identity']
            with self.assertRaisesRegex(ValueError, 'identity'):
                with record_tactile(d, readers, options):pass
            self.assertEqual(list(Path(d).iterdir()), [])

    def test_summary_policy_can_record_native_tactile_without_adding_model_streams(self):
        from openarm_policy.cli import run
        from openarm_policy.observations import TactileObservation
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            profile = root/'profile.json'
            (root/'tactile.json').write_text(json.dumps(dict(tactile=self.options())))
            profile.write_text(json.dumps(dict(tactile_profile='tactile.json', tactile_recording=True)))
            args = SimpleNamespace(profile=profile, robot='simulate', sides=['left'],
                                   record=root/'episode', record_mode='summary', policy='fixture:Policy', duration=1)
            with patch('openarm_tactile.TactileReader', Reader), \
                 patch('openarm_policy.cli.PolicySession') as session:
                run(args)
            self.assertFalse(any(isinstance(s, TactileObservation)
                                 for s in session.call_args.kwargs['sources']))
            receipt = json.loads((args.record/'tactile.json').read_text())
            self.assertTrue(receipt['complete'])
            manifest = json.loads((args.record/receipt['path']/'manifest.json').read_text())
            self.assertTrue(manifest['streams']['tactile/pad_a']['simulated'])

    def test_unready_sidecar_rejects_before_policy_session_or_robot_start(self):
        from openarm_policy.cli import run

        class FailedReader(Reader):
            def __enter__(self):
                self.state, self.error = 'error', 'fixture unplugged'
                return self

        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            profile = root/'profile.json'
            profile.write_text(json.dumps(dict(tactile=self.options(), tactile_recording=True)))
            args = SimpleNamespace(profile=profile, robot='simulate', sides=['left'],
                                   record=root/'episode', record_mode='summary', policy='fixture:Policy', duration=1)
            with patch('openarm_tactile.TactileReader', FailedReader), \
                 patch('openarm_policy.cli.PolicySession') as session:
                with self.assertRaisesRegex(RuntimeError, 'robot was not started'):run(args)
            session.assert_not_called()
            self.assertFalse(json.loads((args.record/'tactile.json').read_text())['complete'])


if __name__ == '__main__':
    unittest.main()
