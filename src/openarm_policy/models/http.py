"""Bounded loopback NPZ transport for separately managed inference environments.

Useful when Torch/LeRobot dependencies must not replace the live robot/camera
environment. The model service owns its lifecycle; this adapter owns no SSH or
subprocess. Remote GPUs can be reached through an operator-managed SSH tunnel.
"""
import io
import json
from urllib.parse import urlsplit, urlunsplit
from urllib.request import Request, build_opener, ProxyHandler, HTTPRedirectHandler


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        raise ValueError("Inference redirects are not allowed")


class NumpyHTTP:
    def __init__(self, url, timeout_s=.3, max_response_bytes=1_000_000, expected_model_sha256=None):
        from ..contracts import finite
        address = urlsplit(url)
        if (address.scheme != "http" or address.hostname not in ("127.0.0.1", "::1", "localhost")
                or address.username or address.password or address.fragment
                or not finite(timeout_s) or timeout_s <= 0
                or type(max_response_bytes) is not int or not 1 <= max_response_bytes <= 8_000_000):
            raise ValueError("Inference transport requires bounded loopback HTTP")
        self.url, self.timeout = url, timeout_s
        self.limit = max_response_bytes
        self.expected_hash = expected_model_sha256

    def reset(self, context):
        self.opener = build_opener(ProxyHandler({}), _NoRedirect())
        address = urlsplit(self.url)
        health = urlunsplit((address.scheme, address.netloc, "/health", "", ""))
        with self.opener.open(health, timeout=self.timeout) as response:
            data = response.read(self.limit + 1)
        if len(data) > self.limit:
            raise ValueError("Inference health response exceeds transport bound")
        info = json.loads(data)
        if info.get("ready") is not True or not isinstance(info.get("model"), dict):
            raise ValueError("Model service is not ready")
        if self.expected_hash is not None and info["model"].get("model_sha256") != self.expected_hash:
            raise ValueError("Inference service model SHA256 mismatch")
        self.metadata = {"transport": "npz_loopback_http", "url": self.url, "model": info["model"]}

    def predict(self, inputs):
        import numpy as np
        payload = io.BytesIO()
        if any(np.asarray(v).dtype.hasobject for v in inputs.values()):
            raise ValueError("Object arrays are not allowed in model transport")
        np.savez_compressed(payload, **inputs)
        body = payload.getvalue()
        if len(body) > 2_000_000:
            raise ValueError("Inference input exceeds transport bound")
        request = Request(self.url, data=body, headers={"Content-Type": "application/octet-stream"})
        with self.opener.open(request, timeout=self.timeout) as response:
            data = response.read(self.limit + 1)
        if len(data) > self.limit:
            raise ValueError("Inference response exceeds transport bound")
        output = np.asarray(json.loads(data)["actions"], dtype=float)
        if output.ndim != 2 or not np.isfinite(output).all():
            raise ValueError("Expected finite H x A HTTP model output")
        return output

    def close(self):
        pass
