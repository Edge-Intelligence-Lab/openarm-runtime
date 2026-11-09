"""Background camera lifecycle, reconnects and latest-frame snapshots."""

from dataclasses import replace
import math
from pathlib import Path
import threading
import time
import uuid

from .camera import CameraConfig, Frame, OpenCVCamera, SimulationCamera


def _source(config):
    if config.kind == "depth":
        from .depth import OpenNIDepthCamera
        return OpenNIDepthCamera(config)
    return OpenCVCamera(config)


class CameraReader:
    """Passive until started. Each instance owns one camera and reconnect loop.

    ``snapshot()`` retains the last frame through a failure, with a nonempty error.
    Use both the error and frame freshness before consuming a sample. Readers are
    one-shot: create a new reader after ``close()``.
    """

    def __init__(self, config, *, source_factory=None, stale_after=3, retry_interval=1, simulate=False):
        if not isinstance(config, CameraConfig):
            raise TypeError("config must be CameraConfig")
        for name, value in (("stale_after", stale_after), ("retry_interval", retry_interval)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if simulate and source_factory is not None:
            raise ValueError("Choose simulate=True or source_factory, not both")
        self.config, self.simulate = config, bool(simulate)
        self.stale_after, self.retry_interval = stale_after, retry_interval
        self._factory = SimulationCamera if simulate else source_factory or _source
        self._lock = threading.Lock()
        self._lifecycle = threading.Lock()
        self._cleanup_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._source = None
        self._frame = None
        self._count = 0
        self._error = ""
        self._cleanup_error = ""
        self._closed = False
        self.on_sample = None  # Optional nonblocking acquisition sink; borrows this owner.

    def start(self):
        with self._lifecycle:
            if self._closed:
                raise RuntimeError("Reader is closed; create a new CameraReader")
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name=f"video-{self.config.name}", daemon=True)
                self._thread.start()
        return self

    def snapshot(self):
        with self._lock:
            return self._frame, self._error, self._count

    def status(self):
        frame, error, count = self.snapshot()
        if self._closed and self._source is None and (self._thread is None or not self._thread.is_alive()):
            state = "Stopped"
        elif error:
            state = "Disconnected"
        elif frame is None:
            state = "Starting" if self._thread is not None else "Stopped"
        else:
            state = "Online" if frame.fresh(time.monotonic(), self.stale_after) else "Stale"
        shape = getattr(frame.image, "shape", ()) if frame else ()
        return {
            "status": state, "error": error, "count": count,
            "kind": self.config.kind, "simulation": self.simulate,
            "width": int(shape[1]) if len(shape) >= 2 else self.config.width,
            "height": int(shape[0]) if len(shape) >= 2 else self.config.height,
            "frame_age_s": max(0, time.monotonic() - frame.received_at) if frame else None,
        }

    def _run(self):
        while not self._stop.is_set():
            source = None
            try:
                source = self._factory(self.config)
                source_id = uuid.uuid4().hex
                with self._lock:
                    self._source = source
                while not self._stop.is_set():
                    started = time.monotonic()
                    sample = source.read()
                    if not isinstance(sample, Frame):
                        raise TypeError("Camera source read() must return Frame")
                    if self._stop.is_set():
                        break
                    with self._lock:
                        self._count += 1
                        self._frame = replace(sample, sequence=self._count, source_id=source_id)
                        self._error = ""
                    if self.on_sample is not None:
                        self.on_sample(self._frame, self._count)
                    self._stop.wait(max(0, 1 / self.config.fps - (time.monotonic() - started)))
            except Exception as exc:
                if not self._stop.is_set():
                    with self._lock:
                        self._error = str(exc)
            finally:
                if source is not None:
                    self._close_source(source)
            self._stop.wait(self.retry_interval)

    def _close_source(self, source, *, retry=False):
        # The caller and worker can reach cleanup together. Close only once and
        # retain ownership after failure, including the source's device lease.
        with self._cleanup_lock:
            with self._lock:
                if self._source is not source or (self._cleanup_error and not retry):
                    return
            try:
                source.close()
            except Exception as exc:
                with self._lock:
                    self._cleanup_error = str(exc)
                    self._error = f"Camera cleanup failed: {exc}"
                self._stop.set()
            else:
                with self._lock:
                    self._source = None
                    if self._cleanup_error:
                        self._error = ""
                        self._cleanup_error = ""

    def close(self):
        """Stop capture; failed source cleanup stays owned for a later retry."""
        with self._lifecycle:
            self._closed = True
            self._stop.set()
            with self._lock:
                source = self._source
                retry = bool(self._cleanup_error)
            if source is not None:
                # A new failure remains visible for this call; only an explicit
                # later close retries it, never the exiting worker.
                self._close_source(source, retry=retry)
            if self._thread is not None:
                self._thread.join(timeout=3)
                if self._thread.is_alive():
                    raise RuntimeError(f"Camera reader did not stop: {self.config.name}")
            if self._cleanup_error:
                raise RuntimeError(f"Camera cleanup failed: {self._cleanup_error}")

    def __enter__(self):
        return self.start()

    def __exit__(self, *_exc):
        self.close()


class VideoCapture:
    """Own a named set of readers; constructing does not access hardware."""

    def __init__(self, configs, *, source_factory=None, stale_after=3, simulate=False):
        self.configs = tuple(configs)
        if not self.configs:
            raise ValueError("At least one camera is required")
        if not all(isinstance(config, CameraConfig) for config in self.configs):
            raise TypeError("configs must contain CameraConfig objects")
        if len({config.name for config in self.configs}) != len(self.configs):
            raise ValueError("Camera names must be unique")
        if not simulate and source_factory is None:
            for config in self.configs:
                if config.kind == "rgb" and not config.device:
                    raise ValueError(f"RGB camera {config.name} requires an explicit device path")
                if config.kind == "depth" and not config.depth_helper:
                    raise ValueError(f"Depth camera {config.name} requires depth_helper")
            identities = [str(Path(config.device).expanduser().resolve()) for config in self.configs
                          if config.kind == "rgb" and config.device]
            if len(set(identities)) != len(identities):
                raise ValueError("Each RGB camera must select a different device")
        self.simulate = bool(simulate)
        self.stale_after = stale_after
        self.readers = {config.name: CameraReader(config, source_factory=source_factory,
                                                stale_after=stale_after, simulate=simulate)
                        for config in self.configs}

    def start(self):
        try:
            for reader in self.readers.values():
                reader.start()
        except BaseException:
            self.close()
            raise
        return self

    def snapshot(self, name):
        return self.readers[name].snapshot()

    def status(self):
        return {name: reader.status() for name, reader in self.readers.items()}

    def close(self):
        errors = []
        for name, reader in reversed(tuple(self.readers.items())):
            try:
                reader.close()
            except Exception as exc:
                errors.append(f"{name}: {exc}")
        if errors:
            raise RuntimeError("; ".join(errors))

    def __enter__(self):
        return self.start()

    def __exit__(self, *_exc):
        self.close()
