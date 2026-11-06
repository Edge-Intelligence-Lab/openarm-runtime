"""Snapshot access has no control packets and never takes over a busy bus."""
from contextlib import nullcontext
import time
import unittest
from unittest.mock import Mock, patch

from openarm_runtime.telemetry import MotorState
from openarm_runtime.transport import ClassicBus, ReadOnlyBus
from openarm_runtime.checks import snapshot


class ReadOnlySnapshotTests(unittest.TestCase):
    def test_packet_allowlist_refuses_motor_controls_and_register_writes(self):
        bus = object.__new__(ReadOnlyBus)
        good = (bytes((1, 0, 0xCC, 0, 0, 0, 0, 0)), bytes((1, 0, 0x33, 35, 0, 0, 0, 0)))
        with patch.object(ClassicBus, "send") as send:
            for packet in good:
                bus.send(0x7FF, packet)
            self.assertEqual(send.call_count, 2)
            for can_id, packet in ((1, b'\xff'*7+b'\xfc'), (1, bytes(8)),
                                   (0x7FF, bytes((1, 0, 0x55, 35, 0, 0, 0, 0))),
                                   (0x7FF, bytes((1, 0, 0x33, 35, 1, 0, 0, 0))),
                                   (0x7FF, bytes((9, 0, 0xCC, 0, 0, 0, 0, 0)))):
                with self.assertRaises(ValueError): bus.send(can_id, packet)
            self.assertEqual(send.call_count, 2)

    def test_disabled_snapshot_uses_fresh_states_and_closes(self):
        bus = Mock()
        states = {mid: MotorState(mid, 0., 0., 0., 20, 20, 0, time.monotonic()) for mid in range(1, 9)}
        bus.state.side_effect = lambda mid: states[mid]
        bus.preflight.return_value = {mid: {} for mid in range(1, 9)}
        profile = Mock(); profile.interface.return_value = "can0"
        with patch("openarm_runtime.checks.bus_lease", return_value=nullcontext()) as lease, \
                patch("openarm_runtime.transport.ReadOnlyBus", return_value=bus):
            result = snapshot(profile)
        lease.assert_called_once_with("can0", exclusive=True)
        self.assertFalse(result["motion_commands_sent"])
        self.assertFalse(result["synchronized"])
        self.assertEqual(len(result["states"]), 8)
        bus.close.assert_called_once()
        bus.command.assert_not_called()
        bus.exchange.assert_not_called()

    def test_stale_and_enabled_feedback_fail_without_cleanup_commands(self):
        for status, age in ((1, 0), (0, 1)):
            bus = Mock()
            bus.preflight.return_value = {}
            bus.state.side_effect = lambda mid: MotorState(mid, 0., 0., 0., 20, 20, status, time.monotonic()-age)
            profile = Mock(); profile.interface.return_value = "can0"
            with patch("openarm_runtime.checks.bus_lease", return_value=nullcontext()), \
                    patch("openarm_runtime.transport.ReadOnlyBus", return_value=bus):
                with self.assertRaises(RuntimeError): snapshot(profile)
            bus.close.assert_called_once()
            bus.command.assert_not_called()

    def test_busy_bus_never_creates_a_transport(self):
        profile = Mock(); profile.interface.return_value = "can0"
        with patch("openarm_runtime.checks.bus_lease", side_effect=RuntimeError("busy")), \
                patch("openarm_runtime.transport.ReadOnlyBus") as bus:
            with self.assertRaises(RuntimeError): snapshot(profile)
        bus.assert_not_called()


if __name__ == "__main__":
    unittest.main()
