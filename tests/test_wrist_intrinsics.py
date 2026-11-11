"""Wrist UI reuses intrinsic solving without robot ownership or head evidence."""
import asyncio
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from openarm_platform.camera_calibration import CalibrationWorkspace, document
from test_platform_camera_calibration import AVAILABLE, PLATFORM, SPEC, frame


@unittest.skipUnless(AVAILABLE, 'calibration extra')
class WristWorkspaceTests(unittest.TestCase):
    def test_current_six_by_four_charuco_board_detection(self):
        import cv2
        import numpy as np
        from openarm_runtime.camera_intrinsics import board, detect
        spec = dict(type='charuco', squares_x=6, squares_y=4, square_size_m=.0225,
                    marker_size_m=.0165, dictionary='DICT_4X4_50')
        image = np.full((600,800),255,np.uint8)
        image[100:500,100:700] = board(spec).generateImage((600,400))
        result = detect(cv2.cvtColor(image,cv2.COLOR_GRAY2RGB),spec)
        self.assertEqual(len(result['corner_ids']),15)
        self.assertEqual(result['board_frame_status'],'indexed_charuco_board')

    def test_wrist_scope_identity_and_independent_head_evidence(self):
        with tempfile.TemporaryDirectory() as root:
            head = CalibrationWorkspace(root, 5003, fetch=lambda *a: frame())
            wrist = CalibrationWorkspace(Path(root)/'wrist-intrinsics', 5003,
                allowed_cameras=('left','right'), intrinsics_only=True,
                fetch=lambda *a: frame(camera_name='left', camera_identity='left-lens'))
            with self.assertRaises(ValueError): wrist.start('a', 'head', SPEC)
            self.assertFalse(Path(root, 'wrist-intrinsics').exists())
            head.start('a', 'head', SPEC)
            wrist.start('a', 'left', SPEC)
            head.capture('a', 'fit'); wrist.capture('a', 'fit')
            self.assertNotEqual(head.session_id, wrist.session_id)
            self.assertEqual(head.samples[0]['metadata']['camera_name'], 'head')
            self.assertEqual(wrist.samples[0]['metadata']['camera_name'], 'left')
            with self.assertRaisesRegex(ValueError, '不接受头部外参'):
                wrist.import_head('a', b'{}')
            with self.assertRaises(ValueError): wrist.start('a', 'right', SPEC)
            wrist.result = dict(matrix=[[700,0,400],[0,700,300],[0,0,1]], distortion=[0]*5)
            wrist.state = 'estimated'
            saved = wrist.save('a', confirmed=True)
            candidate = document((wrist.root/saved).read_bytes())
            self.assertEqual(candidate['camera_name'], 'left')
            self.assertTrue(candidate['intrinsics_only'])
            self.assertIsNone(candidate['head_extrinsics'])
            self.assertFalse(candidate['installed'])
            wrist.start('a', 'right', SPEC)
            with self.assertRaisesRegex(ValueError, '相机名称不匹配'):
                wrist.capture('a', 'fit')
            self.assertEqual(wrist.samples, [])
            self.assertEqual(head.state, 'collecting')

    def test_undistortion_uses_hashed_evidence_without_fetching_or_devices(self):
        import cv2
        import numpy as np
        with tempfile.TemporaryDirectory() as root:
            work = CalibrationWorkspace(root, 5003, intrinsics_only=True,
                allowed_cameras=('left','right'), fetch=lambda *a: frame(camera_name='left'))
            work.start('a', 'left', SPEC); sample = work.capture('a', 'fit')
            work.result = dict(matrix=[[700,0,400],[0,710,300],[0,0,1]], distortion=[.1,-.02,0,0,0])
            work.state = 'estimated'
            with (patch.object(work, 'fetch', side_effect=AssertionError('must use evidence')),
                  patch('cv2.VideoCapture', side_effect=AssertionError('hardware'))):
                images = work.preview_intrinsics('a')
            self.assertEqual(hashlib.sha256(images['original']).hexdigest(), sample['metadata']['png_sha256'])
            before = cv2.imdecode(np.frombuffer(images['original'], np.uint8), cv2.IMREAD_COLOR)
            after = cv2.imdecode(np.frombuffer(images['corrected'], np.uint8), cv2.IMREAD_COLOR)
            np.testing.assert_array_equal(after, cv2.undistort(before, np.array(work.result['matrix'], float),
                np.array(work.result['distortion'], float), None, np.array(work.result['matrix'], float)))
            self.assertFalse(work.busy)
            with self.assertRaises(ValueError): work.preview_intrinsics('other')
            (work.root/work.session_id/sample['id']/'image.png').write_bytes(images['original']+b'changed')
            with self.assertRaises(ValueError): work.preview_intrinsics('a')


@unittest.skipUnless(PLATFORM and AVAILABLE, 'platform/calibration extras')
class WristViewTests(unittest.IsolatedAsyncioTestCase):
    async def test_camera_tabs_defaults_no_motion_and_explicit_save(self):
        from nicegui import Client, ui
        from openarm_platform.camera_calibration_view import build_camera_calibration_view
        client = Client(ui.page('/wrist-test'))
        try:
            with tempfile.TemporaryDirectory() as root:
                work = CalibrationWorkspace(root, 5003, fetch=lambda *a: frame(camera_name='left'))
                with client, ui.column(): view = build_camera_calibration_view(work)
                wrist = view.wrist_view
                await wrist.refresh()
                self.assertEqual(view.camera_panels.value, 'head')
                self.assertEqual(wrist.camera.value, 'left')
                self.assertEqual(wrist.camera.options, {'left':'左腕相机','right':'右腕相机'})
                self.assertEqual((wrist.board_type.value,wrist.nx.value,wrist.ny.value,wrist.square.value),
                                 ('charuco',6,4,.0225))
                self.assertEqual((wrist.marker.value,wrist.dictionary.value),(.0165,'DICT_4X4_50'))
                self.assertTrue(wrist.marker.visible)
                self.assertEqual((view.board_type.value,view.nx.value,view.ny.value,view.square.value),
                                 (wrist.board_type.value,wrist.nx.value,wrist.ny.value,wrist.square.value))
                self.assertIsNone(wrist.confirm)
                self.assertFalse(wrist.import_panel.visible)
                self.assertFalse(wrist.previewing)
                self.assertFalse(work.wrist_calibration.root.exists())
                await wrist.start()
                self.assertEqual(work.state, 'idle')
                self.assertEqual(work.wrist_calibration.state, 'collecting')
                self.assertFalse(wrist.camera.enabled)
                self.assertIn('/left.mjpg', wrist.image.props['src'])
                detection=dict(corner_ids=list(range(54)),center_normalized=[.5,.5],
                    coverage_fraction=.3,sharpness=100,objects=[],pixels=[])
                with patch('openarm_runtime.camera_intrinsics.detect', return_value=detection):
                    await wrist.capture()
                work.wrist_calibration.result=dict(matrix=[[700,0,400],[0,700,300],[0,0,1]],
                    distortion=[0]*5,image_size=[800,600],fit_rms_px=.2,validation_rms_px=.3)
                work.wrist_calibration.state='estimated'
                await wrist.refresh()
                self.assertIn('fx 700.00', wrist.parameters.text)
                self.assertTrue(wrist.save_button.enabled)
                await wrist.preview_result()
                self.assertTrue(wrist.result_images.visible)
                await wrist.save()
                self.assertEqual(work.wrist_calibration.state,'saved')
                self.assertFalse(wrist.save_button.enabled)
                self.assertEqual(work.samples,[])
                work.close()
                self.assertTrue(work.wrist_calibration.closed)
        finally:
            client.delete()
            await asyncio.sleep(0)
