import copy
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from openarm_video.timed_rgb import acquisition_metadata


class TimedRGBTests(unittest.TestCase):
    def header(self):
        return dict(flags=0x2000, timestamp_s=10., period_s=1/30, exposure_100us=200,
                    exposure_readable=True, driver_version=1, kernel_sequence=20)

    def test_kernel_time_not_dequeue_time_and_retained_qualification(self):
        h = self.header()
        result = acquisition_metadata(h, 10.1, 'a'*64)
        self.assertEqual(result['latest_s'], 10.)
        self.assertAlmostEqual(result['earliest_s'], 10.-.02-1/30)
        self.assertFalse(result['qualification_evidence']['exposure_timestamp_known'])
        encoded = json.dumps(result['qualification_evidence'], sort_keys=True, separators=(',', ':')).encode()
        self.assertEqual(hashlib.sha256(encoded).hexdigest(), result['qualification_evidence_sha256'])
        self.assertEqual(h, self.header())

    def test_unknown_copy_future_stale_and_unbounded_eof_exposure_rejected(self):
        for changes in (dict(flags=0), dict(flags=0x4000), dict(flags=0x22000),
                        dict(timestamp_s=11), dict(timestamp_s=9), dict(timestamp_s=float('nan')),
                        dict(period_s=0), dict(exposure_100us=2001), dict(exposure_readable=False)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                acquisition_metadata(dict(self.header(), **changes), 10.1, 'a'*64)

    def test_astra_monotonic_soe_without_readable_integration_uses_full_receive_upper_bound(self):
        header = dict(self.header(), flags=0x12001, exposure_readable=False, exposure_100us=0)
        result = acquisition_metadata(header, 10.1, 'a'*64)
        self.assertEqual(result['earliest_s'], 10.)
        self.assertEqual(result['latest_s'], 10.1)
        self.assertEqual(result['qualification_evidence']['timestamp_source'], 'SOE')
        self.assertEqual(result['qualification_evidence']['interval_rule'], 'SOE_to_complete_host_receive')

    def test_configuration_opt_in_and_import_no_process_or_camera(self):
        from openarm_video.camera import CameraConfig
        self.assertEqual(CameraConfig('head').capture_backend, 'opencv')
        self.assertEqual(CameraConfig('head', capture_backend='v4l2_timestamped').capture_backend, 'v4l2_timestamped')
        with self.assertRaises(ValueError):
            CameraConfig('depth', kind='depth', capture_backend='v4l2_timestamped')
        with patch('subprocess.Popen', side_effect=AssertionError('process')):
            import importlib
            import openarm_video.timed_rgb as module
            importlib.reload(module)
