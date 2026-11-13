"""Offline checks for persisted operator mapping; no sensor dependencies."""
import json
from pathlib import Path
import tempfile
import unittest

from openarm_tactile.profile import load, save_positions
from openarm_tactile.workspace import page


class ProfileTests(unittest.TestCase):
    def fixture(self):
        return {'tactile': {f'pad_{i}': {'device': f'/fixture/serial-{i}', 'device_id': 2,
                'identity': {'serial': f'serial-{i}', 'mount': 'unverified'}} for i in range(4)}}

    def test_roundtrip_keeps_devices_and_persists_all_four_positions(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'tactile.json'
            data = self.fixture()
            path.write_text(json.dumps(data))
            positions = dict(zip([f'serial-{i}' for i in range(4)],
                                 ['left_pad_1', 'left_pad_2', 'right_pad_1', 'right_pad_2']))
            save_positions(path, positions)
            result = load(path)
            for name, options in result['tactile'].items():
                self.assertEqual(options['device'], data['tactile'][name]['device'])
                self.assertEqual(options['device_id'], 2)
                self.assertEqual(options['identity']['mount'], positions[options['identity']['serial']])
            html = page(result, 8891).decode()
            self.assertEqual(html.count('<iframe '), 4)
            self.assertEqual(html.count(' selected'), 4)
            self.assertIn('8904', html)
            self.assertEqual(len(list(Path(d).iterdir())), 1)

    def test_invalid_mapping_does_not_modify_file(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'tactile.json'
            path.write_text(json.dumps(self.fixture()))
            original = path.read_bytes()
            for mapping in ({}, {f'serial-{i}': 'left_pad_1' for i in range(4)},
                            {f'serial-{i}': '../invalid' for i in range(4)}):
                with self.assertRaises(ValueError):
                    save_positions(path, mapping)
                self.assertEqual(path.read_bytes(), original)

    def test_unassigned_is_not_a_guessed_physical_mapping(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'tactile.json'
            path.write_text(json.dumps(self.fixture()))
            save_positions(path, {f'serial-{i}': 'unassigned' for i in range(4)})
            self.assertTrue(all(o['identity']['mount'] == 'physical_pad_assignment_unverified'
                                for o in load(path)['tactile'].values()))
