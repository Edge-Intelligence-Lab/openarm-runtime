"""Borrow lossless native video without opening cameras or trusting previews."""
import hashlib
import importlib.util
import io
import json
import unittest
from unittest.mock import Mock, patch

from openarm_policy.observations import HTTPVideoObservation


@unittest.skipUnless(importlib.util.find_spec('cv2') and importlib.util.find_spec('numpy'),
                     'Requires optional video dependencies')
class NativePolicyVideoTests(unittest.TestCase):
    def setUp(self):
        import cv2
        import numpy as np
        self.image = np.zeros((720, 1280, 3), dtype=np.uint8)
        self.image[12, 34] = [1, 20, 255]
        ok, data = cv2.imencode('.png', self.image)
        self.assertTrue(ok)
        self.png = data.tobytes()
        self.meta = dict(schema='openarm.calibration-frame.v1', camera_name='head',
            image_geometry='native_no_transform', pixel_format='bgr8', simulation=False,
            sequence=1, age_s=0, width=1280, height=720, source_id='capture-generation',
            camera_identity='installed-head', png_sha256=hashlib.sha256(self.png).hexdigest())

    def response(self, changes=None, body=None):
        response = io.BytesIO(self.png if body is None else body)
        response.headers = {'X-Calibration-Metadata': json.dumps({**self.meta, **(changes or {})})}
        return response

    def source(self, changes=None, **kwargs):
        opener = Mock()
        opener.open.side_effect = lambda *a, **k: self.response(changes)
        with patch('urllib.request.build_opener', return_value=opener):
            return HTTPVideoObservation('http://127.0.0.1:5003', ['head'], native=True, **kwargs), opener

    def test_native_preserves_original_shape_bgr_bytes_and_provenance(self):
        import numpy as np
        source, opener = self.source(identities={'head': 'installed-head'})
        sample = source.snapshot()['camera/head']
        np.testing.assert_array_equal(sample.data, self.image)
        self.assertEqual(sample.data.shape, (720, 1280, 3))
        self.assertTrue(source.metadata['lossless'])
        self.assertFalse(source.metadata['hardware_synchronized'])
        self.assertEqual(source.metadata['transport'], 'http_native_png')
        self.assertTrue(all('/calibration/frame/head.png' in c.args[0] for c in opener.open.call_args_list))

    def test_invalid_or_resized_stale_and_changed_identity_frames_fail_closed(self):
        for change in ({'schema':'wrong'}, {'camera_name':'left'},
                       {'image_geometry':'preview_resized'}, {'age_s':1.},
                       {'source_id':''}, {'width':960}, {'width':True},
                       {'height':100000}, {'sequence':True}, {'simulation':True},
                       {'png_sha256':'0'*64}, {'camera_identity':'different-head'}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.source(change, identities={'head':'installed-head'})

    def test_generation_or_mode_change_requires_new_observation_session(self):
        source, opener = self.source()
        opener.open.side_effect = lambda *a, **k: self.response({'source_id':'new-capture-generation'})
        with self.assertRaisesRegex(ValueError, 'generation'):
            source.snapshot()

    def test_original_age_is_not_replaced_with_http_arrival_time(self):
        source, _ = self.source({'age_s':.1})
        import time
        sample = source.snapshot()['camera/head']
        self.assertGreaterEqual(time.monotonic()-sample.received_at, .1)

    def test_native_flag_is_explicit_and_identities_cannot_name_other_streams(self):
        for kwargs in ({'native':'true'}, {'native':True,'identities':{'left':'anything'}}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                HTTPVideoObservation('http://127.0.0.1:5003',['head'],**kwargs)


if __name__ == '__main__':
    unittest.main()
