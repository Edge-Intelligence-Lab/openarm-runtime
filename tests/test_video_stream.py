"""Offline finite-frame HTTP contracts; no camera or native SDK is opened."""

from dataclasses import dataclass
import importlib.util
import subprocess
import sys
import time
import unittest
from unittest.mock import Mock, patch

from openarm_video.stream import create_app, encode_frame, serve


VIDEO_AVAILABLE = all(importlib.util.find_spec(name) for name in ("flask", "numpy", "cv2"))


@dataclass(frozen=True)
class FakeFrame:
    image: object
    received_at: float
    sequence: int = 7
    pixel_format: str = "bgr8"

    def fresh(self, now, max_age=3):
        return 0 <= now - self.received_at < max_age


@dataclass(frozen=True)
class FakeConfig:
    name: str
    kind: str = "rgb"
    width: int = 4
    height: int = 2
    fps: int = 30
    preview_width: int | None = None
    preview_height: int | None = None


class FakeCapture:
    def __init__(self, frames, configs=None, simulate=False):
        self.frames = frames
        self.configs = tuple(configs or [FakeConfig(name, "depth" if frame.pixel_format == "depth_mm" else "rgb")
                                       for name, frame in frames.items()])
        self.simulate = simulate
        self.stale_after = 3
        self.errors = {}
        self.start = Mock()
        self.close = Mock()

    def snapshot(self, name):
        return self.frames.get(name), self.errors.get(name, ""), 1

    def status(self):
        return {config.name: {"status": "Disconnected" if self.errors.get(config.name) else
                             "Online" if self.frames[config.name].fresh(time.monotonic()) else "Stale"}
                for config in self.configs}


class StreamImportTests(unittest.TestCase):
    def test_stream_import_does_not_require_optional_packages(self):
        code = """
import builtins
original = builtins.__import__
def guarded(name, *args, **kwargs):
    if name.split('.')[0] in {'cv2', 'numpy', 'flask', 'werkzeug'}:
        raise AssertionError('optional import: ' + name)
    return original(name, *args, **kwargs)
builtins.__import__ = guarded
import openarm_video.stream
"""
        subprocess.run([sys.executable, "-c", code], check=True, capture_output=True)


@unittest.skipUnless(VIDEO_AVAILABLE, "Install the video extra for HTTP/image tests")
class VideoStreamTests(unittest.TestCase):
    def setUp(self):
        import numpy as np
        self.np = np
        self.depth = np.array([[0, 300, 1000, 65535], [4000, 2000, 100, 0]], dtype=np.uint16)
        self.capture = FakeCapture({
            "head": FakeFrame(np.full((2, 4, 3), 180, dtype=np.uint8), time.monotonic()),
            "range_sensor": FakeFrame(self.depth, time.monotonic(), pixel_format="depth_mm"),
        })
        self.app = create_app(self.capture)
        self.client = self.app.test_client()

    def decode(self, response):
        import cv2
        return cv2.imdecode(self.np.frombuffer(response.data, self.np.uint8), cv2.IMREAD_UNCHANGED)

    def test_create_app_is_passive_and_resources_serve(self):
        self.capture.start.assert_not_called()
        self.capture.close.assert_not_called()
        response = self.client.get("/")
        self.assertEqual(response.status_code, 200)
        self.assertIn(b'/static/viewer.js', response.data)
        javascript = self.client.get("/static/viewer.js")
        self.assertEqual(javascript.status_code, 200)
        self.assertIn(b"/frame/", javascript.data)
        self.assertNotIn(b"/video/", javascript.data)
        self.assertEqual(self.client.get("/static/missing.js").status_code, 404)

    def test_custom_camera_names_are_returned_without_machine_paths(self):
        config = self.client.get("/config").json
        self.assertEqual([c["id"] for c in config["cameras"]], ["head", "range_sensor"])
        self.assertFalse(config["simulation"])
        self.assertNotIn("device", config["cameras"][0])
        status = self.client.get("/status").json
        self.assertEqual(status["cameras"]["range_sensor"], "Online")
        self.assertEqual(status["depth"]["valid_percent"], 75)
        self.assertEqual(status["depths"]["range_sensor"]["min_mm"], 100)

    def test_rgb_is_finite_jpeg_and_snapshot_owned_by_capture(self):
        response = self.client.get("/frame/head.jpg")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.mimetype, "image/jpeg")
        self.assertEqual(response.content_length, len(response.data))
        self.assertEqual(response.headers["Cache-Control"], "no-store")
        self.assertEqual(response.headers["X-Frame-Sequence"], "7")
        self.assertEqual(self.decode(response).shape, (2, 4, 3))
        self.capture.start.assert_not_called()

    def test_rgb_preview_size_does_not_change_source(self):
        self.capture.configs = (FakeConfig("head", preview_width=2, preview_height=1),)
        response = create_app(self.capture).test_client().get("/frame/head.jpg")
        self.assertEqual(self.decode(response).shape, (1, 2, 3))
        self.assertEqual(self.capture.frames["head"].image.shape, (2, 4, 3))

    def test_mjpeg_reuses_capture_skips_duplicates_and_stops_on_failure(self):
        import cv2
        self.capture.configs = (FakeConfig('head',preview_width=2,preview_height=1),)
        client=create_app(self.capture).test_client()
        ticks=[]
        def advance(_delay):
            ticks.append(1)
            if len(ticks)==2:
                self.capture.frames['head']=FakeFrame(self.capture.frames['head'].image,time.monotonic(),sequence=8)
            elif len(ticks)==3:
                self.capture.errors['head']='disconnected'
        with patch('openarm_video.stream.time.sleep',side_effect=advance), \
             patch('openarm_video.stream.encode_frame',wraps=encode_frame) as encoded:
            response=client.get('/stream/head.mjpg',buffered=False)
            self.assertEqual(response.mimetype,'multipart/x-mixed-replace')
            self.assertEqual(response.headers['X-Accel-Buffering'],'no')
            stream=iter(response.response)
            first,second=next(stream),next(stream)
            self.assertIn(b'X-Frame-Sequence: 7\r\n',first)
            self.assertIn(b'X-Frame-Sequence: 8\r\n',second)
            jpeg=first.split(b'\r\n\r\n',1)[1][:-2]
            image=cv2.imdecode(self.np.frombuffer(jpeg,self.np.uint8),cv2.IMREAD_COLOR)
            self.assertEqual(image.shape,(1,2,3))
            with self.assertRaises(StopIteration):next(stream)
            response.close()
            self.assertEqual(encoded.call_count,2)
        self.assertEqual(self.capture.frames['head'].image.shape,(2,4,3))
        self.capture.start.assert_not_called()
        self.capture.close.assert_not_called()

    def test_mjpeg_unknown_depth_and_stale_sources_are_rejected(self):
        for path in ['/stream/missing.mjpg','/stream/range_sensor.mjpg']:
            self.assertEqual(self.client.get(path).status_code,404)
        self.capture.errors['head']='unplugged'
        self.assertEqual(self.client.get('/stream/head.mjpg').status_code,503)

    def test_raw_depth_png_preserves_uint16_values(self):
        response = self.client.get("/depth/range_sensor.png?raw=1")
        self.assertEqual(response.status_code, 200)
        self.assertIn("attachment", response.headers["Content-Disposition"])
        decoded = self.decode(response)
        self.assertEqual(decoded.dtype, self.np.uint16)
        self.np.testing.assert_array_equal(decoded, self.depth)
        alias = self.client.get("/depth.png?raw=1")
        self.np.testing.assert_array_equal(self.decode(alias), self.depth)

    def test_depth_preview_marks_invalid_pixels_black(self):
        response = self.client.get("/depth/range_sensor.png?near=0.3&far=4")
        preview = self.decode(response)
        self.assertEqual(preview.shape, (2, 4, 3))
        self.assertTrue(self.np.all(preview[self.depth == 0] == 0))
        self.assertTrue(self.np.any(preview[self.depth > 0] != 0))
        self.np.testing.assert_array_equal(self.capture.frames["range_sensor"].image, self.depth)

    def test_depth_bounds_and_wrong_sources_are_rejected(self):
        for query in ("near=4&far=1", "near=nan", "far=inf", "near=bad", "far=21"):
            with self.subTest(query=query):
                self.assertEqual(self.client.get("/depth.png?" + query).status_code, 400)
        for path in ("/frame/missing.jpg", "/frame/range_sensor.jpg", "/depth/head.png"):
            with self.subTest(path=path):
                self.assertEqual(self.client.get(path).status_code, 404)

    def test_stale_or_failed_capture_returns_503_not_old_images(self):
        self.capture.frames["head"] = FakeFrame(self.capture.frames["head"].image, time.monotonic() - 10)
        self.assertEqual(self.client.get("/frame/head.jpg").status_code, 503)
        self.capture.errors["range_sensor"] = "disconnected"
        self.assertEqual(self.client.get("/depth.png").status_code, 503)
        self.assertNotIn("valid_percent", self.client.get("/status").json["depth"])

    def test_all_zero_depth_and_simulation_are_explicit(self):
        self.capture.frames["range_sensor"] = FakeFrame(self.np.zeros_like(self.depth), time.monotonic(), pixel_format="depth_mm")
        self.capture.simulate = True
        status = self.client.get("/status").json
        self.assertEqual(status["depth"]["valid_percent"], 0)
        self.assertIsNone(status["depth"]["median_mm"])
        self.assertTrue(status["simulation"])
        self.assertTrue(self.client.get("/config").json["simulation"])

    def test_multiple_depth_names_and_missing_depth(self):
        self.capture.frames["second"] = self.capture.frames["range_sensor"]
        self.capture.configs += (FakeConfig("second", "depth"),)
        client = create_app(self.capture).test_client()
        self.assertEqual(client.get("/depth/second.png?raw=1").status_code, 200)
        self.assertEqual(set(client.get("/status").json["depths"]), {"range_sensor", "second"})
        self.capture.configs = (FakeConfig("head"),)
        self.assertEqual(create_app(self.capture).test_client().get("/depth.png").status_code, 404)

    def test_encoding_rejects_wrong_depth_units_and_dtype(self):
        frame = FakeFrame(self.depth.astype(self.np.float32), time.monotonic(), pixel_format="depth_mm")
        with self.assertRaisesRegex(ValueError, "uint16"):
            encode_frame(frame)
        frame = FakeFrame(self.depth, time.monotonic(), pixel_format="shift")
        with self.assertRaisesRegex(ValueError, "Unsupported pixel format"):
            encode_frame(frame)

    def test_serve_binds_before_start_and_cleans_up_on_interrupt(self):
        events = []
        server = Mock(server_port=5003)
        server.serve_forever.side_effect = KeyboardInterrupt()
        server.server_close.side_effect = lambda: events.append("server_close")
        self.capture.start.side_effect = lambda: events.append("start")
        self.capture.close.side_effect = lambda: events.append("capture_close")
        def bind(*args, **kwargs):
            events.append("bind")
            return server
        with patch("werkzeug.serving.make_server", side_effect=bind):
            with self.assertRaises(KeyboardInterrupt):
                serve(self.capture)
        self.assertEqual(events, ["bind", "start", "capture_close", "server_close"])

    def test_bind_failure_does_not_open_hardware(self):
        with patch("werkzeug.serving.make_server", side_effect=OSError("in use")):
            with self.assertRaisesRegex(OSError, "in use"):
                serve(self.capture)
        self.capture.start.assert_not_called()
        self.capture.close.assert_not_called()

    def test_capture_start_and_close_failures_still_close_server(self):
        server = Mock(server_port=5003)
        self.capture.start.side_effect = RuntimeError("start failed")
        with patch("werkzeug.serving.make_server", return_value=server):
            with self.assertRaisesRegex(RuntimeError, "start failed"):
                serve(self.capture)
        self.capture.close.assert_called_once()
        server.server_close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
