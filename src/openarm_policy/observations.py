"""Bounded histories and adapters borrowing existing capture owners."""
from collections import deque
import copy
import time

from .contracts import Observation, SensorSample, StreamSpec, finite


class ObservationBuffer:
    def __init__(self, specs, history=8):
        self.specs = tuple(specs)
        if type(history) is not int or history < 1 or len({s.name for s in specs}) != len(specs):
            raise ValueError("History must be positive and stream names unique")
        self.buffers = {s.name: deque(maxlen=history) for s in specs}
        self.sequence = 0

    def collect(self, samples, now=None):
        now = time.monotonic() if now is None else now
        for spec in self.specs:
            sample = samples.get(spec.name)
            good = (sample is not None and sample.valid and not sample.error
                    and finite(sample.received_at) and 0 <= now - sample.received_at <= spec.max_age_s)
            if not good:
                if spec.required:
                    raise RuntimeError(f"Required stream unavailable/stale: {spec.name}")
                self.buffers[spec.name].clear()
                continue
            if type(sample.sequence) is not int or sample.sequence < 0:
                raise ValueError(f"Invalid source sequence: {spec.name}")
            buf = self.buffers[spec.name]
            if buf and (sample.sequence < buf[-1].sequence or
                        (sample.sequence != buf[-1].sequence and sample.received_at < buf[-1].received_at)):
                raise RuntimeError(f"Stream restarted or timestamp regressed: {spec.name}")
            if not buf or sample.sequence != buf[-1].sequence:
                shape = getattr(sample.data, "shape", None)
                if shape is None:
                    def matches(value, dimensions):
                        if not dimensions:
                            return finite(value)
                        return isinstance(value, (tuple, list)) and len(value) == dimensions[0] and all(
                            matches(v, dimensions[1:]) for v in value)
                    if not matches(sample.data, spec.shape):
                        raise ValueError(f"Invalid payload shape/values: {spec.name}")
                elif tuple(shape) != spec.shape:
                    raise ValueError(f"Invalid payload shape: {spec.name}")
                if shape is not None and spec.encoding in ("bgr8", "depth_mm"):
                    expected = "uint8" if spec.encoding == "bgr8" else "uint16"
                    if getattr(getattr(sample.data, "dtype", None), "name", None) != expected:
                        raise ValueError(f"Invalid image dtype: {spec.name}")
                buf.append(copy.deepcopy(sample))
        self.sequence += 1
        # History contains only causal samples; consumers choose alignment/interpolation.
        return Observation(self.sequence, now, {k: tuple(v) for k, v in self.buffers.items()})


class VideoObservation:
    """Borrow a started VideoCapture. No open/close, resize or color conversion."""
    def __init__(self, capture, prefix="camera", required=True, max_age_s=0.5):
        self.capture = capture
        self.names = {c.name: f"{prefix}/{c.name}" for c in capture.configs}
        self.streams = tuple(StreamSpec(self.names[c.name], "depth_mm" if c.kind == "depth" else "bgr8",
                                       (c.height, c.width) if c.kind == "depth" else (c.height, c.width, 3),
                                       "mm" if c.kind == "depth" else "uint8", c.name,
                                       max_age_s, required) for c in capture.configs)

    def snapshot(self):
        result = {}
        for name, key in self.names.items():
            frame, error, count = self.capture.snapshot(name)
            if frame is not None:
                result[key] = SensorSample(count, frame.received_at, frame.image, not bool(error), str(error or ""))
        return result


class HTTPVideoObservation:
    """Borrow an existing loopback video service, without opening camera handles.

    Uses the service's sequence/age headers, not HTTP arrival as exposure time.
    These are explicitly lossy JPEG preview frames, potentially resized by their
    owner, not lossless native acquisition or hardware-synchronized observations.
    """
    def __init__(self, url, names, *, max_age_s=.5, timeout_s=.25, max_bytes=2_000_000):
        from urllib.parse import urlsplit
        from urllib.request import build_opener, ProxyHandler, HTTPRedirectHandler
        address = urlsplit(url)
        if (address.scheme != "http" or address.hostname not in ("127.0.0.1", "localhost", "::1")
                or address.username or address.password or address.query or address.fragment
                or not finite(timeout_s) or timeout_s <= 0 or not finite(max_age_s) or max_age_s <= 0
                or type(max_bytes) is not int or not 1 <= max_bytes <= 8_000_000):
            raise ValueError("Expected bounded loopback camera service")
        import re
        if (not names or len(set(names)) != len(names)
                or any(not isinstance(n, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", n) for n in names)):
            raise ValueError("Camera names must be unique simple identifiers")

        class NoRedirect(HTTPRedirectHandler):
            def redirect_request(self, *args, **kwargs):
                raise ValueError("Camera redirects are not allowed")

        self.url, self.names = url.rstrip("/"), tuple(names)
        self.max_age, self.timeout, self.limit = max_age_s, timeout_s, max_bytes
        self.opener = build_opener(ProxyHandler({}), NoRedirect())
        initial = self.snapshot()
        self.streams = tuple(StreamSpec(f"camera/{name}", "bgr8", initial[f"camera/{name}"].data.shape,
                                        "uint8", name, max_age_s, True) for name in self.names)
        self.metadata = dict(transport="http_jpeg_preview", url=self.url, cameras=list(self.names),
                             shapes={k: list(v.data.shape) for k, v in initial.items()},
                             hardware_synchronized=False, lossless=False)

    def snapshot(self):
        import cv2
        import numpy as np
        result = {}
        for name in self.names:
            started = time.monotonic()
            with self.opener.open(f"{self.url}/frame/{name}.jpg", timeout=self.timeout) as response:
                sequence = int(response.headers["X-Frame-Sequence"])
                age = float(response.headers["X-Frame-Age-Ms"]) / 1000
                body = response.read(self.limit + 1)
            # Conservative bound: include the whole request, rather than making
            # an older source frame appear fresh just because HTTP returned now.
            received_at = started - age
            if (sequence < 0 or not finite(age) or age < 0
                    or time.monotonic() - received_at > self.max_age or len(body) > self.limit):
                raise RuntimeError(f"Stale/invalid/bounded camera response: {name}")
            image = cv2.imdecode(np.frombuffer(body, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None or image.ndim != 3 or image.shape[2] != 3:
                raise ValueError(f"Invalid JPEG camera payload: {name}")
            result[f"camera/{name}"] = SensorSample(sequence, received_at, image)
        return result


class TactileObservation:
    """Borrow named TactileReaders. Counts remain raw; never implicitly zero."""
    def __init__(self, readers, required=True, max_age_s=0.5):
        self.readers = dict(readers)
        self.streams = tuple(StreamSpec(f"tactile/{name}", "xyz_counts", (69, 3), "counts",
                                       name, max_age_s, required) for name in readers)

    def snapshot(self):
        result = {}
        for name, reader in self.readers.items():
            state = reader.snapshot()
            if state["sample"] is not None:
                sample = state["sample"]
                result[f"tactile/{name}"] = SensorSample(
                    state["sequence"], state["received_at"],
                    (sample.resultant, *sample.forces), state["state"] == "live", state["error"] or "")
        return result
