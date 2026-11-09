"""Optional finite-frame HTTP presentation; importing this module opens nothing.

``create_app`` borrows a capture group; the caller starts and closes it. ``serve``
owns the group's lifecycle for one blocking server session. No camera is opened
by an HTTP request, and no multipart response is sent to the browser.
"""

from dataclasses import replace
from importlib.resources import files
import math
import time
import hashlib
import json


def _dependencies():
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise RuntimeError('Install video dependencies: pip install "openarm-runtime[video]"') from exc
    return cv2, np


def encode_frame(frame, *, raw=False, near=0.3, far=4.0):
    """Return ``(bytes, MIME type)`` for BGR JPEG or millimetre-depth PNG.

    Depth frames must contain a 2-D uint16 array, with zero meaning invalid.
    ``raw=True`` preserves those values exactly. Preview bounds are metres;
    values outside the bounds saturate to the colour-map endpoints.
    """
    cv2, np = _dependencies()
    image = frame.image
    if frame.pixel_format == "depth_mm":
        if not isinstance(image, np.ndarray) or image.ndim != 2 or image.dtype != np.uint16:
            raise ValueError("Depth frames must be 2-D uint16 millimetres")
        if raw:
            output = image
        else:
            if not (math.isfinite(near) and math.isfinite(far) and 0 <= near < far <= 20):
                raise ValueError("Expected 0 <= near < far <= 20 metres")
            scaled = np.clip((image.astype(np.float32) - near * 1000) * 255 /
                             ((far - near) * 1000), 0, 255).astype(np.uint8)
            output = cv2.applyColorMap(255 - scaled, cv2.COLORMAP_TURBO)
            output[image == 0] = 0
        extension, mime, params = ".png", "image/png", []
    elif frame.pixel_format == "bgr8":
        if raw:
            raise ValueError("Raw download is available for depth frames only")
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("RGB frames must be HxWx3 uint8 BGR")
        output = image
        extension, mime, params = ".jpg", "image/jpeg", [cv2.IMWRITE_JPEG_QUALITY, 80]
    else:
        raise ValueError(f"Unsupported pixel format: {frame.pixel_format}")
    ok, encoded = cv2.imencode(extension, output, params)
    if not ok:
        raise RuntimeError("Image encoding failed")
    return encoded.tobytes(), mime


def _depth_stats(frame):
    _, np = _dependencies()
    values = frame.image[frame.image > 0]
    return {
        "width": int(frame.image.shape[1]), "height": int(frame.image.shape[0]),
        "valid_percent": round(float(values.size / frame.image.size * 100), 2),
        "min_mm": int(values.min()) if values.size else None,
        "median_mm": int(np.median(values)) if values.size else None,
        "max_mm": int(values.max()) if values.size else None,
    }


def create_app(capture):
    """Build a Flask application around an existing, caller-owned capture group.

    No threads or devices are started. Static assets come from package resources,
    so this works from an installed wheel outside the source checkout.
    """
    try:
        from flask import Flask, Response, jsonify, request
    except ImportError as exc:
        raise RuntimeError('Install video dependencies: pip install "openarm-runtime[video]"') from exc
    _dependencies()
    app = Flask(__name__, static_folder=None)
    configs = {config.name: config for config in capture.configs}
    depth_names = [name for name, config in configs.items() if config.kind == "depth"]
    stale_after = getattr(capture, "stale_after", 3.0)

    def current(name):
        frame, error, _count = capture.snapshot(name)
        if error or frame is None or not frame.fresh(time.monotonic(), stale_after):
            return None
        return frame

    @app.get("/")
    def index():
        content = files("openarm_video").joinpath("static/index.html").read_text(encoding="utf-8")
        return Response(content, content_type="text/html; charset=utf-8", headers={"Cache-Control": "no-cache"})

    @app.get("/static/viewer.js")
    def javascript():
        content = files("openarm_video").joinpath("static/viewer.js").read_text(encoding="utf-8")
        return Response(content, content_type="application/javascript; charset=utf-8", headers={"Cache-Control": "no-cache"})

    @app.get("/config")
    def configuration():
        return jsonify(cameras=[{
            "id": config.name, "kind": config.kind, "width": config.width,
            "height": config.height, "fps": config.fps,
        } for config in configs.values()], simulation=bool(getattr(capture, "simulate", False)))

    @app.get("/status")
    def status():
        details = capture.status()
        states = {name: details[name]["status"] for name in configs}
        depths = {}
        for name in depth_names:
            info = {"status": states[name]}
            frame = current(name)
            if frame is not None and frame.pixel_format == "depth_mm":
                info.update(_depth_stats(frame))
            depths[name] = info
        return jsonify(cameras=states, details=details, depths=depths,
                       depth=depths.get(depth_names[0], {}) if depth_names else {},
                       simulation=bool(getattr(capture, "simulate", False)))

    def picture(name, *, depth=False):
        config = configs.get(name)
        if config is None or (config.kind == "depth") != depth:
            return jsonify(error="Camera not found"), 404
        raw = request.args.get("raw", "0") == "1" if depth else False
        try:
            near, far = float(request.args.get("near", "0.3")), float(request.args.get("far", "4"))
            if depth and not raw and not (math.isfinite(near) and math.isfinite(far) and 0 <= near < far <= 20):
                raise ValueError()
        except ValueError:
            return jsonify(error="Expected 0 <= near < far <= 20 metres"), 400
        frame = current(name)
        if frame is None:
            return jsonify(error="No fresh camera frame"), 503
        if not depth and config.preview_width is not None and config.preview_height is not None:
            cv2, _ = _dependencies()
            image = cv2.resize(frame.image, (config.preview_width, config.preview_height))
            frame = replace(frame, image=image)
        data, mime = encode_frame(frame, raw=raw, near=near, far=far)
        headers = {"Cache-Control": "no-store", "X-Frame-Sequence": str(frame.sequence),
                   "X-Frame-Age-Ms": str(round((time.monotonic() - frame.received_at) * 1000))}
        if raw:
            headers["Content-Disposition"] = f'attachment; filename="{name}-depth-mm.png"'
        return Response(data, content_type=mime, headers=headers)

    @app.get("/frame/<name>.jpg")
    def rgb(name):
        return picture(name)

    @app.get('/calibration/frame/<name>.png')
    def calibration_frame(name):
        # One borrowed snapshot supplies BOTH pixels and metadata; no device open,
        # preview resize, crop, undistortion or timestamp relabelling is allowed.
        config = configs.get(name)
        if config is None or config.kind != 'rgb':
            return jsonify(error='RGB camera not found'), 404
        frame = current(name)
        if frame is None:
            return jsonify(error='No fresh camera frame'), 503
        identity = getattr(config, 'calibration_identity', None)
        source_id = getattr(frame, 'source_id', '')
        if not identity or not source_id:
            return jsonify(error='Configure calibration_identity; source must publish a generation'), 409
        cv2, np = _dependencies()
        image = frame.image
        if (frame.pixel_format != 'bgr8' or not isinstance(image, np.ndarray)
                or image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3
                or max(image.shape[:2]) > 4096 or image.size > 24_000_000):
            return jsonify(error='Calibration frame exceeds supported shape'), 413
        ok, encoded = cv2.imencode('.png', image)
        if not ok:
            return jsonify(error='PNG encoding failed'), 503
        data = encoded.tobytes()
        if len(data) > 24_000_000:
            return jsonify(error='Calibration PNG exceeds limit'), 413
        metadata = dict(schema='openarm.calibration-frame.v1', camera_name=name,
            camera_identity=identity, source_id=source_id, sequence=frame.sequence,
            width=int(image.shape[1]), height=int(image.shape[0]), pixel_format='bgr8',
            received_at_s=frame.received_at, timestamp_semantics='host_received_not_exposure',
            clock_domain='host_monotonic',
            age_s=time.monotonic()-frame.received_at, image_geometry='native_no_transform',
            mode=dict(width=config.width, height=config.height, fps=config.fps),
            simulation=bool(getattr(capture, 'simulate', False)),
            png_sha256=hashlib.sha256(data).hexdigest())
        return Response(data, content_type='image/png', headers={
            'Cache-Control':'no-store', 'X-Calibration-Metadata':json.dumps(metadata, ensure_ascii=True)})

    @app.get("/depth/<name>.png")
    def depth(name):
        return picture(name, depth=True)

    @app.get("/depth.png")
    def default_depth():
        if not depth_names:
            return jsonify(error="No depth camera configured"), 404
        return picture(depth_names[0], depth=True)

    return app


def serve(capture, host="127.0.0.1", port=5003):
    """Bind a server, start capture, then block until interrupted; always clean up.

    Bind failures do not open hardware. The capture group is closed on startup or
    server errors as well as KeyboardInterrupt. Use create_app instead to embed
    the UI in an application that owns an already-running capture group.
    """
    app = create_app(capture)
    from werkzeug.serving import WSGIRequestHandler, make_server

    class QuietFramesHandler(WSGIRequestHandler):
        def log_request(self, code="-", size="-"):
            if str(code) == "200" and self.path.startswith(("/frame/", "/depth/", "/depth.png")):
                return
            super().log_request(code, size)

    server = make_server(host, port, app, threaded=True, request_handler=QuietFramesHandler)
    try:
        capture.start()
        print(f"OpenArm video: http://{host}:{server.server_port}", flush=True)
        server.serve_forever()
    finally:
        try:
            capture.close()
        finally:
            server.server_close()
