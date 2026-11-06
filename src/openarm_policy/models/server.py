"""Optional loopback inference service; never imports or opens robot devices."""
import io
import json
from http.server import BaseHTTPRequestHandler, HTTPServer

from .policy import component


def serve(spec, port):
    """Serve a context-independent predictor (reset(None), predict(dict), close)."""
    import numpy as np
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError("Invalid model service port")
    model = component(spec)
    try:
        model.reset(None)

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path != "/health":
                    self.send_error(404)
                    return
                self.send_json({"ready": True, "model": getattr(model, "metadata", {})})

            def send_json(self, value):
                body = json.dumps(value, allow_nan=False).encode()
                if len(body) > 1_000_000:
                    raise ValueError("Model response exceeds transport bound")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                try:
                    size = int(self.headers.get("Content-Length", 0))
                    if not 0 < size <= 2_000_000:
                        raise ValueError("Invalid model request size")
                    with np.load(io.BytesIO(self.rfile.read(size)), allow_pickle=False) as data:
                        inputs = {key: data[key] for key in data.files}
                    output = np.asarray(model.predict(inputs))
                    if output.ndim != 2 or not np.isfinite(output).all():
                        raise ValueError("Model must return finite H x A actions")
                    self.send_json({"actions": output.tolist()})
                except Exception as exc:
                    self.send_error(400, str(exc))

            def log_message(self, *_):
                pass

        with HTTPServer(("127.0.0.1", port), Handler) as server:
            server.timeout = 5
            print(json.dumps({"model_ready": True, "port": port, "model": getattr(model, "metadata", {})}), flush=True)
            server.serve_forever()
    finally:
        model.close()
