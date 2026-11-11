"""Camera source references only; no network, devices or scientific dependencies."""
import json
from pathlib import Path, PurePosixPath
import unittest


class CameraReferenceTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parents[1] / 'configs/cameras'
        self.reference = json.loads((self.root / 'current.json').read_text())

    def test_reference_pins_hf_repository_commit_and_path(self):
        ref = self.reference
        self.assertEqual(ref['schema'], 'openarm.camera-references.v1')
        self.assertEqual(ref['repository'], 'tacpilot/tacpilot')
        self.assertEqual(ref['repo_type'], 'model')
        self.assertRegex(ref['revision'], r'^[0-9a-f]{40}$')
        self.assertEqual(set(ref['cameras']), {'head', 'wrist'})
        for camera, entry in ref['cameras'].items():
            with self.subTest(camera=camera):
                self.assertEqual(set(entry), {'path'})
                path = PurePosixPath(entry['path'])
                self.assertFalse(path.is_absolute())
                self.assertNotIn('..', path.parts)
                self.assertEqual(str(path), f'assets/simulation/cameras/{camera}/calibration.json')

    def test_reference_does_not_activate_hardware(self):
        self.assertIs(self.reference['automatic_device_selection'], False)
        self.assertIs(self.reference['automatic_hardware_activation'], False)

    def test_no_duplicate_parameters_or_per_file_metadata(self):
        self.assertEqual(set(self.reference), {
            'schema', 'cameras', 'repository', 'repo_type',
            'revision', 'automatic_device_selection',
            'automatic_hardware_activation',
        })
        self.assertEqual(list(self.root.rglob('*.json')), [self.root / 'current.json'])
        for entry in self.reference['cameras'].values():
            self.assertEqual(set(entry), {'path'})
            self.assertIsInstance(entry['path'], str)


if __name__ == '__main__':
    unittest.main()
