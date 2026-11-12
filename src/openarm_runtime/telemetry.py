"""Strictly bounded SocketCAN reads. This module has no motor-control commands."""

from __future__ import annotations

import json
import math
import socket
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field

IDS = tuple(range(1, 9))
RANGE_PARAMETERS = (21, 22, 23)  # Firmware ranges, not URDF mechanical limits.
PARAMETERS = (*RANGE_PARAMETERS, 35)  # can_br is read for protocol preflight.
STATUS = {
    0: "未使能",
    1: "已使能",
    8: "过压",
    9: "欠压",
    10: "过流",
    11: "MOS 过热",
    12: "线圈过热",
    13: "通信丢失",
    14: "过载",
}


def read_frame(motor_id: int, parameter: int | None = None, frame_format: str = "classic") -> bytes:
    if type(motor_id) is not int or motor_id not in IDS:
        raise ValueError("Only motor IDs 1..8 are allowed")
    if parameter is not None and (type(parameter) is not int or parameter not in PARAMETERS):
        raise ValueError("Only PMAX/VMAX/TMAX/can_br reads are allowed")
    data = bytes(
        (
            motor_id,
            0,
            0xCC if parameter is None else 0x33,
            0 if parameter is None else parameter,
            0,
            0,
            0,
            0,
        )
    )
    if frame_format == "classic":
        return struct.pack("=IB3x8s", 0x7FF, 8, data)
    if frame_format == "fd":
        return struct.pack("=IBB2x64s", 0x7FF, 8, 1, data)  # CANFD_BRS
    raise ValueError("Frame format must be classic or fd")


ALLOWED_FRAMES = frozenset(read_frame(i, rid) for i in IDS for rid in (None, *PARAMETERS))
ALLOWED_FD_FRAMES = frozenset(read_frame(i, rid, "fd") for i in IDS for rid in (None, *PARAMETERS))


def inspect_link(interface: str) -> dict:
    result = subprocess.run(
        ["ip", "-details", "-json", "link", "show", interface],
        check=True,
        capture_output=True,
        text=True,
        timeout=2,
    )
    return json.loads(result.stdout)[0]


def select_format(requested: str, link: dict, baud_codes: dict[int, int]) -> str:
    """Validate configuration, never change it or sweep baud rates."""
    if requested not in ("auto", "classic", "fd", "compat"):
        raise ValueError("CAN format must be auto, classic, fd or compat")
    if set(baud_codes) != set(IDS):
        raise ValueError("Need can_br replies from all eight motors")
    codes = set(baud_codes.values())
    if not codes <= {4, 9}:
        raise ValueError(f"Unsupported can_br values: {baud_codes}")
    if len(codes) != 1 and requested != "compat":
        raise ValueError(f"Mixed motor can_br settings: {baud_codes}; no automatic writes")
    code = next(iter(codes))
    if code not in (4, 9):
        raise ValueError(f"Unsupported can_br={code}; supported profiles are 1M Classic / 1M+5M FD")
    expected = "fd" if code == 9 else "classic"
    if requested not in ("auto", "compat") and requested != expected:
        raise ValueError(f"Requested {requested} but motor can_br={code} requires {expected}")
    info = link.get("linkinfo", {}).get("info_data", {})
    if "UP" not in link.get("flags", []) or info.get("state") in ("BUS-OFF", "STOPPED", "SLEEPING"):
        raise ValueError("CAN interface is down or bus-off; no automatic restart")
    if info.get("bittiming", {}).get("bitrate") != 1_000_000:
        raise ValueError("Expected host arbitration bitrate 1000000")
    if (expected == "fd" or (requested == "compat" and 9 in codes)) and (
        "FD" not in info.get("ctrlmode", [])
        or info.get("data_bittiming", {}).get("bitrate") != 5_000_000
    ):
        raise ValueError("Motor FD profile requires host FD enabled with data bitrate 5000000")
    # Compatibility is an explicit read-only mode: Classic requests, either RX format.
    # Register values alone do not establish that every motor accepts FD requests.
    return "classic" if requested == "compat" else expected


@dataclass(frozen=True)
class MotorRange:
    position: float
    velocity: float
    torque: float

    def __post_init__(self):
        if any(
            not math.isfinite(x) or not 0 < x < 100_000
            for x in (self.position, self.velocity, self.torque)
        ):
            raise ValueError("Invalid firmware decoding range")


@dataclass(frozen=True)
class MotorState:
    motor_id: int
    position: float
    velocity: float
    torque: float
    mos_temperature: int
    rotor_temperature: int
    status: int
    received_at: float
    frame_format: str = "classic"

    def fresh(self, now: float, timeout: float = 0.4) -> bool:
        return 0 <= now - self.received_at <= timeout


def model_ready(
    states: dict[int, MotorState], now: float, error: str = "", gripper: bool = False
) -> bool:
    ids = IDS if gripper else range(1, 8)
    return not error and all(i in states and states[i].fresh(now) for i in ids)


def decode_state(
    can_id: int,
    data: bytes,
    motor_id: int,
    limits: MotorRange,
    received_at: float,
    frame_format: str = "classic",
) -> MotorState | None:
    if can_id != motor_id + 0x10 or len(data) != 8 or data[0] & 15 != motor_id:
        return None
    # Parameter replies are not joint states, even though the first byte can match.
    if data[0] == motor_id and data[1] == 0 and data[2] in (0x33, 0x55):
        return None
    q = int.from_bytes(data[1:3], "big")
    v = (data[3] << 4) | (data[4] >> 4)
    tau = ((data[4] & 15) << 8) | data[5]
    return MotorState(
        motor_id,
        (q / 65535 * 2 - 1) * limits.position,
        (v / 4095 * 2 - 1) * limits.velocity,
        (tau / 4095 * 2 - 1) * limits.torque,
        data[6],
        data[7],
        data[0] >> 4,
        received_at,
        frame_format,
    )


class ReadOnlyCAN:
    frame_format = "classic"

    def __init__(self, interface: str):
        if sys.platform != "linux":
            raise RuntimeError("SocketCAN telemetry requires Linux")
        self.socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            filters = b"".join(struct.pack("=II", i + 0x10, 0xC00007FF) for i in IDS)
            self.socket.setsockopt(socket.SOL_CAN_RAW, 1, filters)
            self.socket.setsockopt(socket.SOL_CAN_RAW, 2, struct.pack("=I", 0x1FFFFFFF))
            self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            self.socket.bind((interface,))
        except BaseException:
            self.close()
            raise

    def close(self):
        self.socket.close()

    def _send(self, frame: bytes):
        allowed = ALLOWED_FD_FRAMES if self.frame_format == "fd" else ALLOWED_FRAMES
        if frame not in allowed:
            raise ValueError("Outbound packet rejected: viewer allows only reads")
        # Avoid back-to-back parameter requests outrunning motor processing.
        time.sleep(max(0, 0.02 - (time.monotonic() - getattr(self, "_last_send", 0))))
        self.socket.settimeout(0.05)
        if self.socket.send(frame) != len(frame):
            raise OSError("Incomplete CAN frame write")
        self._last_send = time.monotonic()

    def _responses(self, timeout: float):
        end = time.monotonic() + timeout
        count = 0
        while count < 128 and time.monotonic() < end:
            self.socket.settimeout(max(0.0001, end - time.monotonic()))
            try:
                frame = self.socket.recv(72)
            except socket.timeout:
                return
            count += 1
            if len(frame) not in (16, 72):
                continue
            can_id, size = struct.unpack_from("=IB", frame)
            if size > 8:
                continue
            if len(frame) == 72 and frame[5] & ~0x07:
                continue
            data = frame[8 : 8 + size]
            if can_id & 0x20000000:
                # A recovery-to-active event is not a new fault. All other
                # controller/error events stop polling; there is no auto-reset.
                if can_id == 0x20000004 and len(data) >= 2 and data[1] == 0x40:
                    continue
                raise RuntimeError(f"CAN error {can_id:#x}: {data.hex()}; polling stopped")
            yield can_id, data, "fd" if len(frame) == 72 else "classic"

    def read_parameter(self, motor_id: int, rid: int) -> float | int:
        for attempt in range(2):
            value = self._read_parameter_once(motor_id, rid)
            if value is not None:
                return value
            if attempt == 0:
                time.sleep(0.02)
        raise RuntimeError(f"ID {motor_id}: no reply for read-only RID {rid}")

    def _read_parameter_once(self, motor_id: int, rid: int) -> float | int | None:
        self._send(read_frame(motor_id, rid, self.frame_format))
        for can_id, data, _format in self._responses(0.35):
            if (
                can_id == motor_id + 0x10
                and len(data) == 8
                and data[:2] == bytes((motor_id, 0))
                and data[2] == 0x33
                and data[3] == rid
            ):
                return (
                    int.from_bytes(data[4:], "little")
                    if rid == 35
                    else struct.unpack("<f", data[4:])[0]
                )
        return None

    def read_range(self, motor_id: int) -> MotorRange:
        return MotorRange(*(self.read_parameter(motor_id, rid) for rid in RANGE_PARAMETERS))

    def refresh(self, motor_id: int, limits: MotorRange) -> MotorState | None:
        for _attempt in range(2):
            self._send(read_frame(motor_id, frame_format=self.frame_format))
            for can_id, data, rx_format in self._responses(0.04):
                state = decode_state(can_id, data, motor_id, limits, time.monotonic(), rx_format)
                if state is not None:
                    return state
        return None


@dataclass
class Telemetry:
    lock: threading.Lock = field(default_factory=threading.Lock)
    states: dict[int, MotorState] = field(default_factory=dict)
    ranges: dict[int, MotorRange] = field(default_factory=dict)
    error: str = ""
    message: str = "正在读取电机解码量程…"
    responses: int = 0
    missed: int = 0
    baud_codes: dict[int, int] = field(default_factory=dict)
    frame_format: str = "preflight"
    requested_format: str = "auto"
    link: dict = field(default_factory=dict)
    rx_classic: int = 0
    rx_fd: int = 0
    connection_attempts: int = 0
    retry_delay_s: float = 0.

    def snapshot(self):
        with self.lock:
            return (dict(self.states), self.error, self.message, self.responses, self.missed)

    def metadata(self):
        with self.lock:
            return {
                "baud_codes": dict(self.baud_codes),
                "format": self.frame_format,
                "requested_format": self.requested_format,
                "link": self.link,
                "rx_classic": self.rx_classic,
                "rx_fd": self.rx_fd,
                "connection_attempts": self.connection_attempts,
                "retry_delay_s": self.retry_delay_s,
            }


def poll(
    interface: str,
    shared: Telemetry,
    stop: threading.Event,
    rate: float = 20,
    frame_format: str = "auto",
    max_cycles: int | None = None,
):
    from contextlib import ExitStack
    from openarm_runtime.config import bus_lease, require_classic_read_link

    reader = None
    ownership = ExitStack()
    try:
        ownership.enter_context(bus_lease(interface, exclusive=False))
        host = inspect_link(interface)
        if frame_format == "classic":
            require_classic_read_link(host)
        reader = ReadOnlyCAN(interface)
        with shared.lock:
            shared.baud_codes.clear()
            shared.ranges.clear()
            shared.frame_format = "preflight"
            shared.link = host
            shared.requested_format = frame_format
            shared.message = "只读检查每个电机的 can_br…"
        # Classic configuration reads are also used by the official change_baud tool.
        # Accept both Classic and FD responses; do not guess a mode from the host alone.
        for motor_id in IDS:
            if stop.is_set():
                return
            code = reader.read_parameter(motor_id, 35)
            with shared.lock:
                shared.baud_codes[motor_id] = code
        selected = select_format(frame_format, host, shared.baud_codes)
        reader.frame_format = selected
        with shared.lock:
            shared.frame_format = selected
        for motor_id in IDS:
            if stop.is_set():
                return
            limits = reader.read_range(motor_id)
            with shared.lock:
                shared.ranges[motor_id] = limits
                shared.message = f"已确认解码量程：{motor_id}/8"
        silent_cycles = 0
        cycles = 0
        while not stop.is_set():
            start = time.monotonic()
            received = 0
            for motor_id in IDS:
                if stop.is_set():
                    return
                state = reader.refresh(motor_id, shared.ranges[motor_id])
                with shared.lock:
                    if state is None:
                        shared.missed += 1
                    else:
                        shared.states[motor_id] = state
                        shared.responses += 1
                        if state.frame_format == "fd":
                            shared.rx_fd += 1
                        else:
                            shared.rx_classic += 1
                        received += 1
            silent_cycles = silent_cycles + 1 if received < len(IDS) else 0
            if silent_cycles >= 3:
                raise RuntimeError("Incomplete motor replies for three cycles; polling stopped")
            with shared.lock:
                if received == len(IDS):
                    shared.error = ""
                else:
                    shared.error = "Incomplete motor feedback; waiting for all eight motors"
                mode = (
                    "兼容模式：Classic请求 / 混合回复，非全FD验收"
                    if frame_format == "compat"
                    else selected
                )
                shared.message = f"只读查询 · {mode} · 目标 {rate:g} Hz"
            cycles += 1
            if max_cycles is not None and cycles >= max_cycles:
                return
            stop.wait(max(0, 1 / rate - (time.monotonic() - start)))
    except Exception as exc:
        with shared.lock:
            shared.error = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            if reader is not None:
                reader.close()  # No enable/disable packet, even at shutdown.
        finally:
            ownership.close()


def poll_reconnecting(interface, shared, stop, *, frame_format="auto"):
    """Retry read-only sessions with interruptible, capped exponential backoff.

    Each poll closes its socket and releases ownership before the retry wait.
    Old samples retain their timestamps; an error clears only on a complete cycle.
    """
    delay = 1.0
    while not stop.is_set():
        with shared.lock:
            shared.connection_attempts += 1
            shared.retry_delay_s = 0.
        before = shared.responses
        poll(interface, shared, stop, frame_format=frame_format)
        if stop.is_set():
            return
        if shared.responses > before:
            delay = 1.0
        with shared.lock:
            shared.message = f"只读连接中断；{delay:g} 秒后重试（不重置总线）"
            shared.retry_delay_s = delay
        if stop.wait(delay):
            return
        delay = min(delay * 2, 30.0)


class ArmConnection:
    """One independent read worker per bus; restarting one never touches the other."""

    def __init__(self, interface: str, frame_format: str = "auto"):
        self.interface = interface
        self.frame_format = frame_format
        self.shared = Telemetry()
        self.stop = threading.Event()
        self.worker = None
        self.lock = threading.Lock()

    def restart(self):
        with self.lock:
            self.close()
            self.shared = Telemetry()
            self.stop = threading.Event()
            self.worker = threading.Thread(
                target=poll_reconnecting,
                args=(self.interface, self.shared, self.stop),
                kwargs={"frame_format": self.frame_format},
                daemon=True,
            )
            self.worker.start()

    def close(self):
        self.stop.set()
        if self.worker is not None:
            self.worker.join(timeout=3)
            if self.worker.is_alive():
                raise RuntimeError("Read worker did not stop; refusing a duplicate worker")
