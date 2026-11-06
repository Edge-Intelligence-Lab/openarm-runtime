"""Shared, explicit machine configuration and cooperating-process bus ownership."""

from contextlib import contextmanager
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re


def finite(value: float, lower: float, upper: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError(f"{name} must be finite and in [{lower}, {upper}]")
    return value


@dataclass(frozen=True)
class Profile:
    left: str
    right: str
    mapping_verified: bool = False
    rate_hz: float = 20.0
    feedback_timeout_s: float = 0.04
    gravity: dict | None = None

    def __post_init__(self):
        if self.left == self.right:
            raise ValueError("Left and right must use distinct interfaces")
        for interface in (self.left, self.right):
            if not re.fullmatch(r"[A-Za-z0-9_-]{1,15}", interface):
                raise ValueError("Invalid SocketCAN interface name")
        if type(self.mapping_verified) is not bool:
            raise ValueError("mapping_verified must be a JSON boolean")
        finite(self.rate_hz, 10, 50, "rate_hz")
        finite(self.feedback_timeout_s, 0.005, 0.1, "feedback_timeout_s")
        if self.gravity is not None and (not isinstance(self.gravity, dict)
                or set(self.gravity) - {"left", "right"}
                or any(not isinstance(p, str) or not p for p in self.gravity.values())):
            raise ValueError("gravity maps left/right to gravity profile paths")

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text())
        if data.pop("format", None) != "classic" or data.pop("bitrate", None) != 1_000_000:
            raise ValueError("Only Classic CAN 1 Mbps is supported")
        # Old commissioning profiles carried a manual flag. The panel checks each
        # motor itself; retaining an old machine file must not bypass that check.
        data.pop("mit_mode_verified", None)
        if isinstance(data.get("gravity"), dict):
            data["gravity"] = {side: str((Path(path).expanduser().resolve().parent / value).resolve())
                               for side, value in data["gravity"].items()}
        return cls(**data)

    def make_gravity(self, side):
        self.interface(side)
        if not self.gravity or side not in self.gravity:
            return None
        from .gravity import load_gravity
        return load_gravity(self.gravity[side], side=side)

    def interface(self, side: str):
        if side not in ("left", "right"):
            raise ValueError("Select left or right explicitly")
        return getattr(self, side)


def require_classic_link(link: dict):
    info = link.get("linkinfo", {}).get("info_data", {})
    if (
        "UP" not in link.get("flags", [])
        or link.get("mtu") != 16
        or "FD" in info.get("ctrlmode", [])
        or info.get("state") != "ERROR-ACTIVE"
        or info.get("bittiming", {}).get("bitrate") != 1_000_000
        or info.get("restart_ms", 0) != 0
        or any(info.get("berr_counter", {}).values())
    ):
        raise RuntimeError("Expected healthy Classic 1 Mbps, fd off, restart-ms 0; no automatic reconfiguration")


def require_classic_read_link(link: dict):
    """Permit bounded reads to clear historical warnings, never reconfigure a bus.

    Motion still uses require_classic_link, including its zero-error requirement.
    Passive, bus-off, down and unknown states cannot start a read attempt.
    """
    info = link.get("linkinfo", {}).get("info_data", {})
    if info.get("state") not in ("ERROR-ACTIVE", "ERROR-WARNING"):
        raise RuntimeError("Classic read requires ERROR-ACTIVE or ERROR-WARNING; no automatic restart")
    normalized = {**link, "linkinfo": {**link.get("linkinfo", {}), "info_data": {
        **info, "state": "ERROR-ACTIVE", "berr_counter": {"tx": 0, "rx": 0},
    }}}
    require_classic_link(normalized)


@contextmanager
def bus_lease(interface: str, exclusive: bool):
    """Advisory lock shared by our viewer/check/control processes, not third-party tools."""
    import fcntl

    if not re.fullmatch(r"[A-Za-z0-9_-]{1,15}", interface):
        raise ValueError("Invalid interface")
    # Keep the same namespace during extraction so old and new clients exclude
    # each other. Renaming only one would allow two controllers on the same bus.
    root = Path.home() / ".cache" / "bimanual-tactile" / "can-locks"
    root.mkdir(parents=True, exist_ok=True)
    with (root / f"{interface}.lock").open("a+") as handle:
        try:
            fcntl.flock(handle, (fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH) | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f"{interface} in use; stop its viewer/check before motion") from exc
        try:
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)
