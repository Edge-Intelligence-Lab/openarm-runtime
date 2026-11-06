"""Bounded Classic SocketCAN transport; official SDK supplies command encoding.

Unlike OpenArm.init_arm_motors this preflight never writes CTRL_MODE. Cached SDK
positions are not used: every state sample must arrive during a bounded request.
"""

import math
import socket
import struct
import sys
import time
from dataclasses import asdict

from .telemetry import IDS, MotorRange, decode_state, inspect_link, read_frame
from .config import require_classic_link

MODELS = ("DM8009", "DM8009", "DM4340", "DM4340", "DM4310", "DM4310", "DM4310", "DM4310")


class ClassicBus:
    def __init__(self, interface: str, timeout: float = 0.04):
        if sys.platform != "linux":
            raise RuntimeError("Linux SocketCAN required")
        self.interface, self.timeout = interface, timeout
        require_classic_link(inspect_link(interface))
        self.socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.socket.setsockopt(socket.SOL_CAN_RAW, 1, b"".join(
                struct.pack("=II", i + 16, 0xC00007FF) for i in IDS))
            self.socket.setsockopt(socket.SOL_CAN_RAW, 2, struct.pack("=I", 0x1FFFFFFF))
            self.socket.bind((interface,))
            self.socket.settimeout(timeout)
        except BaseException:
            self.socket.close()
            raise
        self.ranges = {}
        self.sdk = None
        self.motors = {}

    def close(self):
        self.socket.close()

    def send(self, can_id, data):
        if len(data) != 8 or can_id not in (*IDS, 0x7FF):
            raise ValueError("Invalid Classic motor packet")
        frame = struct.pack("=IB3x8s", can_id, 8, bytes(data))
        self.socket.settimeout(self.timeout)
        if self.socket.send(frame) != 16:
            raise OSError("Short CAN write")
        self.last_send = time.monotonic()

    def _decode_frame(self, frame):
        if len(frame) != 16:
            raise RuntimeError("Non-Classic frame")
        can_id, size = struct.unpack_from("=IB", frame)
        data = frame[8:8 + size]
        if can_id & 0x20000000:
            if can_id == 0x20000004 and len(data) >= 2 and data[1] == 0x40:
                return None
            raise RuntimeError(f"CAN fault {can_id:#x}: {data.hex()}")
        return can_id, data

    def drain(self):
        self.socket.setblocking(False)
        try:
            for _ in range(256):
                try:
                    self._decode_frame(self.socket.recv(16))
                except BlockingIOError:
                    return
            raise RuntimeError("Receive queue never drained; another sender may be active")
        finally:
            self.socket.settimeout(self.timeout)

    def receive(self, predicate, timeout=None):
        end = time.monotonic() + (self.timeout if timeout is None else timeout)
        for _ in range(256):
            remaining = end - time.monotonic()
            if remaining <= 0:
                break
            self.socket.settimeout(remaining)
            try:
                result = self._decode_frame(self.socket.recv(16))
            except socket.timeout:
                break
            if result is not None:
                value = predicate(*result)
                if value is not None:
                    return value
        raise TimeoutError("No matching fresh CAN reply within deadline")

    def parameter(self, mid, rid):
        for attempt in range(2):
            # Parameter processing needs spacing even when a reply arrives quickly.
            time.sleep(0.02)
            try:
                return self._parameter_once(mid, rid)
            except TimeoutError:
                if attempt:
                    raise TimeoutError(f"ID{mid}: no reply for RID{rid}") from None

    def _parameter_once(self, mid, rid):
        self.drain()
        self.send(0x7FF, bytes((mid, 0, 0x33, rid, 0, 0, 0, 0)))

        def match(cid, data):
            if cid == mid + 16 and len(data) == 8 and data[:4] == bytes((mid, 0, 0x33, rid)):
                return struct.unpack("<f" if rid in (21, 22, 23) else "<I", data[4:])[0]
        return self.receive(match, 0.35)

    def state(self, mid):
        # Control commands also produce state replies. Let their reply arrive and
        # discard it before a new refresh, rather than accepting a prior status as
        # the result of the enable/disable or position command just sent.
        time.sleep(max(0, 0.02 - (time.monotonic() - getattr(self, "last_send", 0))))
        self.drain()
        self.send(0x7FF, read_frame(mid)[8:])
        return self.receive(lambda cid, data: decode_state(cid, data, mid, self.ranges[mid], time.monotonic()))

    def preflight(self):
        report = {}
        for mid in IDS:
            code = self.parameter(mid, 35)
            if code != 4:
                raise RuntimeError(f"ID{mid} can_br={code}; expected 4")
            self.ranges[mid] = MotorRange(*(self.parameter(mid, rid) for rid in (21, 22, 23)))
            state = self.state(mid)
            if state.status != 0:
                raise RuntimeError(
                    f"ID{mid} must be disabled; status={state.status}"
                    + (" (OVERLOAD/过载；不是零点问题，请排查负载和碰撞，勿重复使能或标定)"
                       if state.status == 14 else ""))
            report[mid] = {"baud_code": code, "range": asdict(self.ranges[mid]), "state": asdict(state)}
        require_classic_link(inspect_link(self.interface))
        return report

    def exchange(self, targets, kp, kd, feedforward=None):
        """Batch MIT controls followed by their eight state replies, as in follower."""
        if set(targets) != set(IDS):
            raise ValueError("Whole-arm exchange requires IDs 1..8")
        if feedforward is not None:
            if set(feedforward) != set(IDS) or any(
                    not math.isfinite(v) or abs(v) > self.ranges[mid].torque
                    for mid, v in feedforward.items()):
                raise ValueError("Invalid motor-space feedforward")
        time.sleep(max(0, 0.02 - (time.monotonic() - getattr(self, "last_send", 0))))
        self.drain()
        for mid in IDS:
            if feedforward is None:
                self.command(mid, "position", targets[mid], kp[mid - 1], kd[mid - 1])
            else:
                self.command(mid, "position", targets[mid], kp[mid - 1], kd[mid - 1], feedforward[mid])
        states = {}
        def collect(cid, data):
            mid = cid - 16
            if mid not in IDS:
                return None
            state = decode_state(cid, data, mid, self.ranges[mid], time.monotonic())
            if state is not None:
                states[mid] = state
            return states if len(states) == len(IDS) else None
        try:
            return self.receive(collect)
        except TimeoutError as exc:
            failure = TimeoutError(f"Missing control feedback IDs {sorted(set(IDS) - set(states))}")
            failure.feedback = dict(states)
            raise failure from exc

    def prepare_encoder(self, ids):
        import openarm_can as oa

        if oa.__version__ != "1.4.1":
            raise RuntimeError("Validated encoder requires SDK 1.4.1")
        for mid in ids:
            if mid not in IDS:
                raise ValueError("Motor ID must be 1..8")
            kind = getattr(oa.MotorType, MODELS[mid - 1])
            expected = oa.Motor.get_limit_param(kind)
            actual = self.ranges[mid]
            if not all(math.isclose(a, b, rel_tol=1e-5) for a, b in zip(
                (actual.position, actual.velocity),
                (expected.pMax, expected.vMax))):
                raise RuntimeError(f"ID{mid} firmware ranges differ from SDK; refusing incorrectly scaled commands")
            self.motors[mid] = oa.Motor(kind, mid, mid + 16)
            # Verify the legacy zero midpoint. Nonzero feedforward is rescaled to
            # the firmware TMAX and decoded/checked in command().
            packet = oa.CanPacketEncoder.create_mit_control_command(
                self.motors[mid], oa.MITParam(0, 0, 0, 0, 0))
            self._require_zero_feedforward(packet.data)
        self.sdk = oa

    @staticmethod
    def _require_zero_feedforward(data):
        if len(data) != 8 or ((data[6] & 15) << 8 | data[7]) != 2047:
            raise RuntimeError("Expected SDK zero-feedforward midpoint; refusing command")

    def command(self, mid, action, position=0.0, kp=0.0, kd=0.0, feedforward=0.0):
        encoder = self.sdk.CanPacketEncoder
        motor = self.motors[mid]
        if action == "position":
            if not math.isfinite(position) or abs(position) >= self.ranges[mid].position:
                raise ValueError("Target outside firmware position range")
            actual = self.ranges[mid].torque
            if not math.isfinite(feedforward) or abs(feedforward) > actual:
                raise ValueError("Feedforward outside firmware torque range")
            # The official encoder uses model-default TMAX. Scale its input so
            # decoding the same bits with the actual firmware TMAX yields Nm.
            encoded_torque = 0.0
            if feedforward != 0:
                kind = getattr(self.sdk.MotorType, MODELS[mid - 1])
                sdk_max = self.sdk.Motor.get_limit_param(kind).tMax
                encoded_torque = feedforward * sdk_max / actual
            packet = encoder.create_mit_control_command(motor, self.sdk.MITParam(kp, kd, position, 0.0, encoded_torque))
            if feedforward == 0:
                self._require_zero_feedforward(packet.data)
            else:
                bits = ((packet.data[6] & 15) << 8) | packet.data[7]
                decoded = bits / 4095 * 2 * actual - actual
                if abs(decoded - feedforward) > 2 * actual / 4095 + 1e-9:
                    raise RuntimeError("SDK feedforward encoding disagrees with firmware range")
        elif action in ("enable", "disable"):
            packet = getattr(encoder, f"create_{action}_command")(motor)
        else:
            raise ValueError("Unknown motor command")
        self.send(packet.send_can_id, packet.data)

    def set_zero(self, mid, expected_position):
        """Explicit calibration-only write; disabled state checked again here."""
        state = self.state(mid)
        if (state.motor_id != mid or state.status != 0 or state.frame_format != "classic"
                or not state.fresh(time.monotonic())
                or not all(math.isfinite(x) for x in (expected_position, state.position, state.velocity))
                or abs(state.position-expected_position) > math.radians(0.5)
                or abs(state.velocity) > 0.08):
            raise RuntimeError(f"ID{mid}: zero write requires stationary disabled motor")
        packet = self.sdk.CanPacketEncoder.create_set_zero_command(self.motors[mid])
        self.send(packet.send_can_id, packet.data)


class ReadOnlyBus(ClassicBus):
    """Packet-level allowlist for feedback and parameter queries only.

    Even accidental calls to inherited control/calibration methods cannot send
    enable, disable, zero-write, parameter-write or MIT-position packets.
    """
    def send(self, can_id, data):
        packet = bytes(data)
        if (can_id != 0x7FF or len(packet) != 8 or packet[0] not in IDS
                or packet[1] != 0 or packet[2] not in (0x33, 0xCC)
                or any(packet[4:]) or (packet[2] == 0xCC and packet[3] != 0)):
            raise ValueError("Read-only CAN transport refuses a non-query packet")
        return super().send(can_id, packet)
