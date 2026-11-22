"""Durable runtime contracts; no hardware access or native SDK required."""
from contextlib import nullcontext
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from openarm_runtime.config import Profile, bus_lease, require_classic_link
from openarm_runtime.report import Report
from openarm_runtime.transport import ClassicBus
from openarm_runtime.telemetry import MotorRange


class RuntimeTests(unittest.TestCase):
    def test_profile_rejects_duplicate_interfaces_and_nonclassic(self):
        self.assertEqual(Profile("can1", "can0").interface("right"), "can0")
        with self.assertRaises(ValueError):
            Profile("can0", "can0")
        with tempfile.TemporaryDirectory() as root:
            p = Path(root) / "profile.json"
            p.write_text(json.dumps({"format": "fd", "bitrate": 1000000}))
            with self.assertRaises(ValueError):
                Profile.load(p)

    def test_classic_link_rejects_errors(self):
        link = {"flags": ["UP"], "mtu": 16, "linkinfo": {"info_data": {
            "state": "ERROR-ACTIVE", "bittiming": {"bitrate": 1000000}}}}
        require_classic_link(link)
        link["linkinfo"]["info_data"]["berr_counter"] = {"tx": 1}
        with self.assertRaises(RuntimeError):
            require_classic_link(link)

    def test_shared_lease_and_exclusive_control_cannot_overlap(self):
        with tempfile.TemporaryDirectory() as root, patch("pathlib.Path.home", return_value=Path(root)):
            with bus_lease("can0", False), bus_lease("can0", False):
                with self.assertRaises(RuntimeError), bus_lease("can0", True):
                    pass
            with bus_lease("can0", True):
                with self.assertRaises(RuntimeError), bus_lease("can0", False):
                    pass

    def test_receive_timeout_never_returns_cached_state(self):
        bus = ClassicBus.__new__(ClassicBus)
        bus.timeout, bus.socket = 0.01, Mock()
        bus.socket.recv.side_effect = socket.timeout()
        with self.assertRaises(TimeoutError):
            bus.receive(lambda *args: object())

    def test_encoder_position_velocity_mismatch_refused(self):
        sdk = Mock()
        sdk.__version__ = "1.4.1"
        sdk.Motor.get_limit_param.return_value = SimpleNamespace(pMax=12.5, vMax=30, tMax=10)
        bus = ClassicBus.__new__(ClassicBus)
        bus.ranges, bus.motors = {1: MotorRange(12.5, 135, 200)}, {}
        with patch.dict(sys.modules, {"openarm_can": sdk}):
            with self.assertRaisesRegex(RuntimeError, "ranges differ"):
                bus.prepare_encoder([1])
        sdk.CanPacketEncoder.create_mit_control_command.assert_not_called()

    def test_torque_only_mismatch_accepts_only_zero_midpoint(self):
        sdk = Mock()
        sdk.__version__ = "1.4.1"
        sdk.Motor.get_limit_param.return_value = SimpleNamespace(pMax=12.5, vMax=30, tMax=10)
        sdk.CanPacketEncoder.create_mit_control_command.return_value = SimpleNamespace(
            data=bytes([127, 255, 127, 240, 0, 0, 7, 255]))
        bus = ClassicBus.__new__(ClassicBus)
        bus.ranges, bus.motors = {5: MotorRange(12.5, 30, 3)}, {}
        with patch.dict(sys.modules, {"openarm_can": sdk}):
            bus.prepare_encoder([5])
            sdk.CanPacketEncoder.create_mit_control_command.return_value.data = bytes(8)
            with self.assertRaisesRegex(RuntimeError, "midpoint"):
                bus.prepare_encoder([5])

    def test_check_reports_both_buses_and_closes_failed_bus_without_controls(self):
        from openarm_runtime.checks import check
        bus = Mock()
        bus.preflight.side_effect = [TimeoutError("missing ID"), {}]
        bus.sdk.__version__ = "1.4.1"
        with patch("openarm_runtime.checks.ClassicBus", return_value=bus), \
             patch("openarm_runtime.checks.bus_lease", side_effect=lambda *a, **k: nullcontext()):
            result = check(Profile("can1", "can0"))
        self.assertFalse(result["passed"])
        self.assertTrue(result["buses"]["right"]["passed"])
        self.assertEqual(bus.close.call_count, 2)
        bus.command.assert_not_called()
        bus.exchange.assert_not_called()

    def test_report_preserves_evidence_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "run.json"
            report = Report(path, {"interface": "can0"})
            report.record({"event": "arm_disable", "verified": True})
            self.assertTrue(json.loads(path.read_text())["events"][0]["verified"])
            with self.assertRaises(FileExistsError):
                Report(path, {})

    def test_single_cli_help_requires_no_linux_sdk_or_config(self):
        result = subprocess.run([sys.executable, "-m", "openarm_runtime", "--help"],
                                cwd=Path(__file__).resolve().parents[1],
                                capture_output=True, text=True, check=True)
        self.assertIn("prepare,simulate,leader,check,calibrate,view,panel", result.stdout)

    def test_per_joint_sliders_bind_to_correct_motor(self):
        from openarm_runtime.panel import attach
        handles = {}
        class Handle:
            def __init__(self, label, options=None, **kwargs):
                self.value = kwargs.get("initial_value", options[0] if options else None)
                handles[label] = self
            def on_click(self, callback):
                self.callback = callback
                return callback
            on_update = on_click
        gui = SimpleNamespace(add_folder=lambda *args, **kwargs: nullcontext(),
                              add_markdown=Handle, add_slider=Handle,
                              add_checkbox=Handle, add_button=Handle, add_dropdown=Handle)
        server = SimpleNamespace(gui=gui, on_client_disconnect=lambda fn: fn)
        with patch("openarm_runtime.panel.MotionSession") as factory:
            session = factory.return_value
            attach(server, Profile("can1", "can0"), {}, "/unused")
            session.start.assert_not_called()

            for side, label in (("left", "左臂"), ("right", "右臂")):
                handles["机械臂"].value = label
                handles["开始标定"].callback(
                    SimpleNamespace(client=SimpleNamespace(client_id=42)))
                self.assertEqual(session.start.call_args.args[:2], (42, side))
                self.assertIs(session.start.call_args.args[3], True)
                self.assertEqual(session.start.call_args.kwargs, {"calibration": True, "write_zero": True})
            handles["操作"].value = "自动标定（不写零位）"
            handles["开始标定"].callback(SimpleNamespace(client=SimpleNamespace(client_id=42)))
            self.assertEqual(session.start.call_args.kwargs, {"calibration": True, "write_zero": False})
            session.start.reset_mock()
            for side, label in (("right", "右臂"), ("left", "左臂")):
                handles["机械臂"].value = label
                for mid in range(1, 9):
                    name = f"J{mid}" if mid < 8 else "夹爪"
                    slider = handles[f"{name} 角度 (°)" if mid < 8 else f"{name} 偏移 (°)"]
                    slider.value = 1.2
                    slider.callback(SimpleNamespace(client=SimpleNamespace(client_id=42)))
                    self.assertEqual(session.set_target.call_args.args, (42, side, mid, 1.2))
            self.assertEqual(session.set_target.call_count, 16)
            session.start.assert_not_called()

    def test_panel_creation_is_passive_and_requires_readiness(self):
        from openarm_runtime.panel import MotionSession
        with patch("openarm_runtime.panel.ClassicBus") as bus:
            session = MotionSession(Profile("can1", "can0", mapping_verified=True), {}, "/unused")
            with self.assertRaises(ValueError):
                session.start(1, "right", False, False)
            bus.assert_not_called()
            self.assertEqual(session.speed, 5)
            session.owner = 1
            session.stop(2)
            self.assertFalse(session.cancel.is_set())
            session.stop(1)
            self.assertTrue(session.cancel.is_set())

    def test_panel_wrong_mode_does_not_enable_and_restores_reader(self):
        from openarm_runtime.panel import MotionSession
        connections = {"can0": Mock(), "can1": Mock()}
        with tempfile.TemporaryDirectory() as root:
            session = MotionSession(Profile("can1", "can0", mapping_verified=True), connections, root)
            session.lock.acquire()
            with patch("openarm_runtime.panel.bus_lease", return_value=nullcontext()), \
                 patch("openarm_runtime.panel.ClassicBus") as factory:
                bus = factory.return_value
                bus.preflight.return_value = {}
                bus.parameter.return_value = 2
                session._run("can0", True)
                bus.command.assert_not_called()
                bus.close.assert_called_once()
            connections["can0"].restart.assert_called_once()
            self.assertFalse(session.busy)
            self.assertIn("expected MIT", session.message)


if __name__ == "__main__":
    unittest.main()
