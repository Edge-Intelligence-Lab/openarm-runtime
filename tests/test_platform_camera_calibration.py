"""Owner, evidence, native-frame and UI contracts with software fixtures only."""
import asyncio
import copy
from dataclasses import replace
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

from openarm_platform.camera_calibration import (CalibrationWorkspace, MAX_PNG,
    atomic_json, borrow_frame, decode_frame, document)
from openarm_platform import load_config

AVAILABLE = all(importlib.util.find_spec(n) for n in ('cv2','numpy','scipy','flask'))
PLATFORM = importlib.util.find_spec('nicegui') is not None
SPEC = dict(squares_x=7,squares_y=5,square_size_m=.025,marker_size_m=.018,dictionary='DICT_4X4_50')


def frame(sequence=1, **changes):
    import cv2
    import numpy as np
    from openarm_runtime.camera_intrinsics import board
    image = np.full((600,800,3),255,np.uint8)
    image[100:500,120:680] = cv2.cvtColor(board(SPEC).generateImage((560,400)),cv2.COLOR_GRAY2BGR)
    raw = cv2.imencode('.png',image)[1].tobytes()
    metadata = dict(schema='openarm.calibration-frame.v1',camera_name='head',camera_identity='fixture-serial/lens',
        source_id='source-1',sequence=sequence,width=800,height=600,pixel_format='bgr8',
        received_at_s=12.,timestamp_semantics='host_received_not_exposure',age_s=.01,
        image_geometry='native_no_transform',mode=dict(width=800,height=600,fps=30),
        simulation=True,png_sha256=hashlib.sha256(raw).hexdigest())
    metadata.update(changes)
    return raw,metadata


@unittest.skipUnless(AVAILABLE,'calibration extra')
class WorkspaceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.now = 10.
        self.packet = frame()
        self.work = CalibrationWorkspace(self.temp.name,5003,clock=lambda:self.now,fetch=lambda *a:self.packet)
        self.work.start('a','head',SPEC)

    def test_capture_native_evidence_and_refuse_duplicate_or_identity_change(self):
        sample = self.work.capture('a','fit')
        directory = Path(self.temp.name)/self.work.session_id/sample['id']
        self.assertEqual((directory/'image.png').read_bytes(),self.packet[0])
        stored = json.loads((directory/'sample.json').read_text())
        self.assertEqual(stored['metadata'],self.packet[1])
        self.assertEqual(stored['role'],'fit')
        with self.assertRaises(ValueError): self.work.capture('a','fit')
        for changes in (dict(source_id='new'),dict(camera_identity='other'),dict(mode=dict(width=800,height=600,fps=60))):
            self.packet = frame(2,**changes)
            with self.assertRaises(ValueError):self.work.capture('a','validation')
        self.assertEqual(len(self.work.samples),1)

    def test_owner_cancel_lease_and_late_result(self):
        with self.assertRaises(ValueError):self.work.start('b','head',SPEC)
        with self.assertRaises(ValueError):self.work.capture('b','fit')
        with self.assertRaises(ValueError):self.work.cancel('b')
        entered,release = threading.Event(),threading.Event()
        def fetch(*a):
            entered.set(); release.wait(3)
            return self.packet
        self.work.fetch=fetch
        errors=[]
        def run():
            try:self.work.capture('a','fit')
            except ValueError as exc:errors.append(str(exc))
        worker=threading.Thread(target=run)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            self.work.cancel('a')
            self.assertEqual(self.work.state,'cancelled')
        finally:
            release.set();worker.join(3)
        self.assertTrue(errors)
        self.assertEqual(self.work.samples,[])
        self.work.start('b','head',SPEC)
        self.now += 121
        with self.assertRaises(ValueError):self.work.capture('b','fit')
        self.assertEqual(self.work.state,'cancelled')

    def test_solve_save_invalidate_and_immutable_result(self):
        self.work.capture('a','fit')
        with patch('openarm_runtime.camera_intrinsics.solve',return_value=dict(calibration_status='estimated_not_independently_verified')):
            self.work.solve('a')
        with self.assertRaises(ValueError):self.work.save('a')
        relative=self.work.save('a',confirmed=True)
        value=document((Path(self.temp.name)/relative).read_bytes())
        self.assertFalse(value['installed'])
        with self.assertRaises(ValueError):self.work.save('a',confirmed=True)
        with self.assertRaises(FileExistsError):atomic_json(Path(self.temp.name)/relative,{})
        self.work.start('a','head',SPEC)
        self.work.capture('a','fit')
        self.work.result={'old':True}
        self.packet=frame(2)
        self.work.capture('a','validation')
        self.assertIsNone(self.work.result)

    def test_reject_paths_payloads_and_replaced_evidence_directory(self):
        with self.assertRaises(ValueError):self.work.start('b','../head',SPEC)
        for raw in (b'{} '*(700_000),b'{"x":NaN}',b'{"x":1,"x":2}',b'[]'):
            with self.assertRaises(ValueError):document(raw)
        raw,meta=frame()
        for changes in (dict(width=400),dict(age_s=2),dict(png_sha256='0'*64),dict(timestamp_semantics='exposure')):
            with self.assertRaises(ValueError):decode_frame(raw,dict(meta,**changes))
        giant=raw[:16]+struct.pack('>II',100000,100000)+raw[24:]
        with patch('cv2.imdecode',side_effect=AssertionError('must bound before decode')):
            with self.assertRaises(ValueError):decode_frame(giant,meta)
        with self.assertRaises(ValueError):decode_frame(b'x'*(MAX_PNG+1),meta)
        path=Path(self.temp.name)/self.work.session_id
        moved=path.with_name('moved')
        path.rename(moved)
        path.symlink_to(moved,target_is_directory=True)
        with self.assertRaises(ValueError):self.work.capture('a','fit')

    def test_import_head_is_bounded_and_stays_unverified(self):
        from test_camera_calibration import CalibrationTests
        observations,_=CalibrationTests().observations('head')
        observations['camera_name']='head'
        result=self.work.import_head('a',json.dumps(observations).encode())
        self.assertEqual(result['calibration_status'],'estimated_not_independently_verified')
        self.assertEqual(result['provenance_boundary'],'operator_supplied_not_runtime_verified')
        self.assertFalse(self.work.status('a')['realtime_extrinsics'])
        original=Path(self.temp.name)/self.work.session_id/result['evidence_id']/'observations.json'
        self.assertEqual(hashlib.sha256(original.read_bytes()).hexdigest(),result['observations_sha256'])
        observations['mode']='wrist'
        with self.assertRaises(ValueError):self.work.import_head('a',json.dumps(observations).encode())

    def test_checkerboard_capture_records_local_grid_and_blocks_external_pose(self):
        import cv2
        import numpy as np
        self.work.cancel('a')
        spec=dict(type='checkerboard',squares_x=10,squares_y=7,square_size_m=.015)
        self.work.start('a','head',spec)
        image=np.full((600,800,3),255,np.uint8)
        for y in range(7):
            for x in range(10):
                image[90+y*50:90+(y+1)*50,150+x*50:150+(x+1)*50]=255*((x+y)%2)
        raw=cv2.imencode('.png',image)[1].tobytes()
        self.packet=(raw,dict(self.packet[1],png_sha256=hashlib.sha256(raw).hexdigest()))
        sample=self.work.capture('a','validation')
        self.assertEqual(len(sample['detection']['pixels']),54)
        self.assertEqual(sample['detection']['board_frame_status'],'per_image_grid_ambiguous_for_extrinsics')
        session=document((Path(self.temp.name)/self.work.session_id/'session.json').read_bytes())
        self.assertEqual(session['board_spec'],spec)
        with patch('openarm_runtime.camera_calibration.fit',side_effect=AssertionError('must not fit')):
            with self.assertRaisesRegex(ValueError,'固定板坐标方向'):
                self.work.import_head('a',b'{}')
        self.assertFalse(self.work.busy)
        self.assertFalse(self.work.status('a')['realtime_extrinsics'])

    def test_count_and_storage_budgets_and_capture_failure_leave_no_sample(self):
        self.work.samples=[dict(metadata=dict(sequence=i)) for i in range(64)]
        with self.assertRaises(ValueError):self.work.capture('a','fit')
        self.work.samples=[]
        self.work.bytes=256*1024**2
        with self.assertRaises(ValueError):self.work.capture('a','fit')
        self.assertFalse(self.work.busy)

    def test_tampered_evidence_and_failed_write_cannot_create_candidate(self):
        sample=self.work.capture('a','fit')
        self.work.result={'calibration_status':'estimated_not_independently_verified'}
        path=Path(self.temp.name)/self.work.session_id/sample['id']/'sample.json'
        path.write_text('{}')
        with self.assertRaises(ValueError):self.work.save('a',confirmed=True)
        self.assertFalse((path.parent.parent/'candidate.json').exists())
        self.work.cancel('a');self.work.start('a','head',SPEC)
        with patch('openarm_platform.camera_calibration.atomic_bytes',side_effect=OSError('disk full')):
            with self.assertRaises(OSError):self.work.capture('a','fit')
        self.assertEqual(self.work.samples,[])
        self.assertFalse(self.work.busy)


@unittest.skipUnless(AVAILABLE,'calibration/video extra')
class NativeFrameTests(unittest.TestCase):
    def test_same_snapshot_full_resolution_and_no_device_start(self):
        import cv2
        import numpy as np
        from openarm_video import CameraConfig,Frame
        from openarm_video.stream import create_app
        image=np.arange(600*800*3,dtype=np.uint8).reshape(600,800,3)
        sample=Frame(image,time.monotonic(),19,source_id='generation')
        capture=Mock()
        capture.configs=[CameraConfig('head',preview_width=400,preview_height=300,calibration_identity='fixture')]
        capture.stale_after=3
        capture.simulate=True
        capture.snapshot.return_value=(sample,'',19)
        client=create_app(capture).test_client()
        response=client.get('/calibration/frame/head.png')
        self.assertEqual(response.status_code,200)
        metadata=json.loads(response.headers['X-Calibration-Metadata'])
        self.assertEqual(metadata['sequence'],19)
        np.testing.assert_array_equal(cv2.cvtColor(decode_frame(response.data,metadata),cv2.COLOR_RGB2BGR),image)
        self.assertEqual(client.get('/calibration/frame/missing.png').status_code,404)
        capture.start.assert_not_called()
        capture.close.assert_not_called()
        capture.configs=[CameraConfig('head')]
        self.assertEqual(create_app(capture).test_client().get('/calibration/frame/head.png').status_code,409)

    def test_borrowing_fake_http_service_never_opens_a_device_and_rejects_redirects(self):
        from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
        raw,metadata=frame()
        class Handler(BaseHTTPRequestHandler):
            redirect=False
            def log_message(self,*args):pass
            def do_GET(self):
                if self.redirect:
                    self.send_response(302);self.send_header('Location','http://example.invalid/');self.end_headers()
                    return
                self.send_response(200)
                self.send_header('Content-Type','image/png')
                self.send_header('X-Calibration-Metadata',json.dumps(metadata))
                self.end_headers();self.wfile.write(raw)
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
        try:
            with patch('cv2.VideoCapture',side_effect=AssertionError('hardware')):
                received,meta=borrow_frame(server.server_port,'head')
                self.assertEqual(received,raw)
                self.assertEqual(meta['sequence'],metadata['sequence'])
                decode_frame(received,meta)
                Handler.redirect=True
                with self.assertRaises(ValueError):borrow_frame(server.server_port,'head')
                with self.assertRaises(ValueError):borrow_frame(server.server_port,'../outside')
        finally:
            server.shutdown();server.server_close();worker.join(2)


class CalibrationConfigTests(unittest.TestCase):
    def test_new_modules_import_without_optional_science_or_devices(self):
        code="""
import builtins
original=builtins.__import__
def guarded(name,*args,**kwargs):
    if name.split('.')[0] in {'cv2','numpy','scipy','serial','openarm_can','nicegui','flask'}:
        raise AssertionError(name)
    return original(name,*args,**kwargs)
builtins.__import__=guarded
import openarm_runtime.camera_intrinsics
import openarm_platform.camera_calibration
import openarm_platform.camera_calibration_view
"""
        subprocess.run([sys.executable,'-I','-c',code],check=True)

    def test_builtin_configuration_does_not_create_artifacts_or_service(self):
        with tempfile.TemporaryDirectory() as root:
            profile=Path(root)/'platform.json'
            profile.write_text(json.dumps({'calibration_root':'evidence'}))
            config=load_config(profile)
            self.assertEqual(config.calibration_root,Path(root)/'evidence')
            self.assertFalse(config.calibration_root.exists())
            page=next(p for p in config.pages if p.id=='camera_calibration')
            self.assertEqual(page.path,'/camera-calibration')
            self.assertNotIn(page,config.services)


@unittest.skipUnless(PLATFORM,'platform extra')
class CalibrationViewTests(unittest.IsolatedAsyncioTestCase):
    async def test_ui_renders_passively_and_navigation_available_in_attach_only(self):
        from nicegui import ui
        from openarm_platform.camera_calibration_view import build_camera_calibration_view
        with tempfile.TemporaryDirectory() as root:
            work=CalibrationWorkspace(Path(root)/'not-created',5003)
            with patch('cv2.VideoCapture',side_effect=AssertionError('hardware')) if AVAILABLE else patch('socket.socket'):
                with ui.column():
                    view=build_camera_calibration_view(work)
                    await view.refresh()
                self.assertEqual(work.state,'idle')
                self.assertFalse(work.root.exists())
                self.assertFalse(view.previewing)
                await view.toggle_preview()
                self.assertTrue(view.previewing)
                self.assertTrue(view.image.props['src'].startswith('/apps/vision/stream/head.mjpg?'))
                await view.toggle_preview()
                self.assertNotIn('src',view.image.props)
                self.assertFalse(work.root.exists())
                self.assertEqual(view.board_type.value,'checkerboard')
                self.assertFalse(view.marker.visible)
                self.assertFalse(view.import_upload.enabled)
                view.board_type.value='charuco'
                view.board_fields()
                self.assertTrue(view.marker.visible)
                self.assertTrue(view.import_upload.enabled)
        from starlette.testclient import TestClient
        from openarm_platform.app import create_app
        manager=Mock();manager.status.return_value={}
        with TestClient(create_app(load_config(),manager,attach_only=True,with_ui=False),base_url='http://127.0.0.1:8888') as client:
            result=client.post('/api/select',json={'id':'camera_calibration'},headers={'Origin':'http://127.0.0.1:8888'})
            self.assertTrue(result.json()['services']['camera_calibration']['reachable'])
            manager.start.assert_not_called();manager.activate.assert_not_called()
