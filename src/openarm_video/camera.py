"""Camera configuration and synchronous sources; importing does not load OpenCV."""

from dataclasses import dataclass
import hashlib
import math
import os
from pathlib import Path
import re
import sys
import tempfile
import threading
import time


@dataclass(frozen=True)
class CameraConfig:
    """Caller-owned device selection and capture/preview settings.

    ``device`` is the RGB device path. Depth uses an explicitly prepared native
    ``depth_helper`` and optional OpenNI ``uri``. Preview dimensions only affect
    HTTP encoding; Python callers receive full capture-resolution frames.
    """

    name: str
    device: str = ""
    kind: str = "rgb"
    width: int = 640
    height: int = 480
    fps: float = 30
    preview_width: int | None = None
    preview_height: int | None = None
    depth_helper: str | None = None
    uri: str | None = None

    calibration_identity: str | None = None
    capture_backend: str = 'opencv'

    def __post_init__(self):
        if self.capture_backend not in ('opencv', 'v4l2_timestamped'):
            raise ValueError('Unknown capture_backend')
        if self.kind != 'rgb' and self.capture_backend != 'opencv':
            raise ValueError('Timestamped V4L2 backend is RGB only')
        if not isinstance(self.name, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_-]*", self.name):
            raise ValueError("Camera name must start with a letter and contain only letters, digits, _ or -")
        if self.kind not in ("rgb", "depth"):
            raise ValueError("Camera kind must be 'rgb' or 'depth'")
        if not isinstance(self.device, str):
            raise ValueError("Camera device must be a path string")
        for name in ("width", "height"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"Camera {name} must be a positive integer")
        if isinstance(self.fps, bool) or not isinstance(self.fps, (int, float)) or not math.isfinite(self.fps) or self.fps <= 0:
            raise ValueError("Camera fps must be finite and positive")
        if self.kind == "depth" and (self.width > 4096 or self.height > 4096
                                     or self.fps > 120 or int(self.fps) != self.fps):
            raise ValueError("Depth mode requires width/height <= 4096 and an integer fps <= 120")
        if (self.preview_width is None) != (self.preview_height is None):
            raise ValueError("Set both preview_width and preview_height, or neither")
        for name in ("preview_width", "preview_height"):
            value = getattr(self, name)
            if value is not None and (isinstance(value, bool) or not isinstance(value, int) or value <= 0):
                raise ValueError(f"Camera {name} must be a positive integer")
        for name in ("depth_helper", "uri", "calibration_identity"):
            value = getattr(self, name)
            if value is not None and (not isinstance(value, str) or not value.strip()):
                raise ValueError(f"Camera {name} must be a nonempty string or None")
        if self.calibration_identity is not None and (len(self.calibration_identity) > 256 or not self.calibration_identity.isprintable()):
            raise ValueError('Camera calibration_identity must be printable and at most 256 characters')


@dataclass(frozen=True)
class Frame:
    """Owned image sample: BGR uint8 or depth uint16 millimetres (zero is invalid).

    ``received_at`` uses ``time.monotonic()`` and is not a wall-clock timestamp.
    Consumers should treat ``image`` as read-only; copy before modifying it.
    """

    image: object
    received_at: float
    sequence: int = 0
    pixel_format: str = "bgr8"
    source_id: str = ""
    acquisition: dict | None = None

    def fresh(self, now, max_age=3):
        return 0 <= now - self.received_at < max_age


_LEASE_LOCK = threading.Lock()
_LEASES = set()


class _DeviceLease:
    """Process-local and, on POSIX, cross-process advisory video ownership."""

    def __init__(self, key):
        self.key, self.file = str(key), None
        self._closed = False
        with _LEASE_LOCK:
            if self.key in _LEASES:
                raise RuntimeError(f"Camera is already in use by this process: {key}")
            _LEASES.add(self.key)
        try:
            try:
                import fcntl
            except ImportError:
                return  # Non-POSIX users retain in-process protection.
            identity = hashlib.sha256(self.key.encode()).hexdigest()
            uid = os.getuid() if hasattr(os, "getuid") else "user"
            path = Path(tempfile.gettempdir()) / f"openarm-video-{uid}-{identity}.lock"
            self.file = path.open("a")
            try:
                fcntl.flock(self.file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError(f"Camera is already in use by another process: {key}") from exc
        except BaseException:
            self.close()
            raise

    def close(self):
        with _LEASE_LOCK:
            if self._closed:
                return
            self._closed = True
            if self.file is not None:
                self.file.close()
                self.file = None
            _LEASES.discard(self.key)


class OpenCVCamera:
    """One RGB capture handle. Constructing opens hardware; importing does not."""

    def __init__(self, config):
        if config.kind != "rgb" or not config.device:
            raise ValueError("RGB capture requires kind='rgb' and an explicit device path")
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("RGB capture needs the video extra: pip install -e '.[video]'") from exc
        self.config, self._cv2 = config, cv2
        self._cap = None
        self._io_lock = threading.Lock()
        self._lease = _DeviceLease(str(Path(config.device).expanduser().resolve()))
        self._count = 0
        try:
            backend = cv2.CAP_V4L2 if sys.platform.startswith("linux") else cv2.CAP_ANY
            self._cap = cv2.VideoCapture(str(Path(config.device).expanduser()), backend)
            if not self._cap.isOpened():
                raise OSError(f"Cannot open camera: {config.device}")
            self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
            self._cap.set(cv2.CAP_PROP_FPS, config.fps)
        except BaseException:
            self.close()
            raise

    def read(self):
        with self._io_lock:
            if self._cap is None:
                raise OSError("Camera is closed")
            ok, image = self._cap.read()
            if not ok or image is None:
                raise OSError(f"Camera frame read failed: {self.config.device}")
            self._count += 1
            return Frame(image.copy(), time.monotonic(), self._count)

    def close(self):
        # Avoid concurrent native read()/release(), which is unsafe in OpenCV.
        if not self._io_lock.acquire(timeout=1):
            raise RuntimeError(f"Camera read did not return; cannot safely close {self.config.device}")
        try:
            if self._cap is not None:
                self._cap.release()
                self._cap = None
            self._lease.close()
        finally:
            self._io_lock.release()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()


class SimulationCamera:
    """Explicit software RGB/depth source. Does not open any camera or SDK."""

    def __init__(self, config):
        try:
            import numpy as np
        except ImportError as exc:
            raise RuntimeError("Video simulation needs the video extra: pip install -e '.[video]'") from exc
        self.config, self._np = config, np
        self._count = 0
        self._closed = False

    def read(self):
        if self._closed:
            raise OSError("Simulation camera is closed")
        np, c = self._np, self.config
        self._count += 1
        x = np.arange(c.width, dtype=np.uint32)[None, :]
        y = np.arange(c.height, dtype=np.uint32)[:, None]
        if c.kind == "depth":
            image = (500 + (x * 4 + y * 2 + self._count * 7) % 3500).astype(np.uint16)
            image[(x + y) % 31 == 0] = 0
            pixel_format = "depth_mm"
        else:
            image = np.empty((c.height, c.width, 3), dtype=np.uint8)
            image[:, :, 0] = (x + self._count * 5) % 256
            image[:, :, 1] = (y + self._count * 3) % 256
            image[:, :, 2] = (x + y + self._count * 7) % 256
            pixel_format = "bgr8"
        return Frame(image, time.monotonic(), self._count, pixel_format)

    def close(self):
        self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
