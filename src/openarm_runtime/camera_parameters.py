"""Bundled camera baselines and caller-provided replacements."""

import json
from pathlib import Path


def load_camera_parameters(camera: str = "head", path: Path | str | None = None) -> dict:
    """Load the included baseline, or a local file with the same camera schema."""
    schemas = {
        "head": "openarm.camera-baseline.v1",
        "wrist": "openarm.wrist-intrinsics.v1",
    }
    if camera not in schemas:
        raise ValueError("Camera must be head or wrist")
    source = (Path(__file__).resolve().parent / "calibration" / f"{camera}.json"
              if path is None else Path(path).expanduser())
    data = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema") != schemas[camera]:
        raise ValueError(f"Expected {schemas[camera]} in {source}")
    return data
