"""Installed-package and non-GUI lifecycle contracts."""

from contextlib import nullcontext
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, Mock, patch

from openarm_runtime import Profile, hold_arm
from openarm_leader import LeaderBus


class PublicApiTests(unittest.TestCase):
    def test_core_imports_are_passive_and_optional_dependencies_stay_unloaded(self):
        script = """
import sys
from unittest.mock import patch
with patch('socket.socket', side_effect=AssertionError('opened socket')), \
     patch('subprocess.Popen', side_effect=AssertionError('started process')), \
     patch('threading.Thread.start', side_effect=AssertionError('started thread')):
    import openarm_runtime, openarm_leader, openarm_video
    from openarm_tactile import TactileReader, Sample, SENSOR, POINT_COUNT
    reader = TactileReader('/dev/not-a-real-device')
    assert reader.snapshot()['state'] == 'waiting'
    reader.close()
    assert reader.snapshot()['state'] == 'closed'
    assert SENSOR == 'PXSR-STDIP03A' and POINT_COUNT == 68
    assert Sample((0, 0, 0), ((0, 0, 0),) * POINT_COUNT).resultant == (0, 0, 0)
assert not any(n in sys.modules for n in ('viser', 'numpy', 'serial', 'scservo_sdk', 'openarm_can', 'openarm_viewer', 'cv2', 'flask', 'paxini_sdk', 'openarm_tactile.app', 'openarm_tactile.dual'))
print(openarm_runtime.__version__)
"""
        with tempfile.TemporaryDirectory() as root:
            result = subprocess.run([sys.executable, "-I", "-c", script], cwd=root,
                                    capture_output=True, text=True, check=True)
        self.assertEqual(result.stdout.strip(), "0.8.2")

    def test_cli_works_outside_checkout(self):
        with tempfile.TemporaryDirectory() as root:
            result = subprocess.run([sys.executable, "-I", "-m", "openarm_runtime", "--help"],
                                    cwd=root, capture_output=True, text=True, check=True)
        for command in ("prepare", "leader", "check", "view", "panel", "calibrate", "video", "tactile", "platform"):
            self.assertIn(command, result.stdout)
        with tempfile.TemporaryDirectory() as root:
            help_result = subprocess.run([sys.executable, "-I", "-m", "openarm_runtime", "video", "--help"],
                                         cwd=root, capture_output=True, text=True, check=True)
        self.assertIn("--simulate", help_result.stdout)
        self.assertIn("--profile", help_result.stdout)
        with tempfile.TemporaryDirectory() as root:
            tactile_help = subprocess.run(
                [sys.executable, "-I", "-m", "openarm_runtime", "tactile", "--help"],
                cwd=root, capture_output=True, text=True, check=True,
            )
        for option in ("--device", "--right-device", "--simulate", "--device-id", "--baudrate", "--port"):
            self.assertIn(option, tactile_help.stdout)

    @unittest.skipUnless(os.environ.get("OPENARM_EXPECT_INSTALLED"), "Wheel-only validation")
    def test_validation_uses_installed_wheel_not_checkout(self):
        import openarm_runtime, openarm_tactile, openarm_platform
        from importlib.metadata import version
        self.assertIn("site-packages", Path(openarm_runtime.__file__).parts)
        self.assertIn("site-packages", Path(openarm_tactile.__file__).parts)
        self.assertIn("site-packages", Path(openarm_platform.__file__).parts)
        self.assertEqual(version("openarm-runtime"), "0.8.2")

    def test_video_resources_are_present_outside_source_tree(self):
        from importlib.resources import files
        root = files("openarm_video")
        for name in ("static/index.html", "static/viewer.js", "native/depth_capture.cpp"):
            self.assertTrue(root.joinpath(name).is_file(), name)
        self.assertIn("/frame/", root.joinpath("static/viewer.js").read_text())
        platform = files("openarm_platform")
        for name in ("dashboard.py", "static/platform.css"):
            self.assertTrue(platform.joinpath(name).is_file(), name)

    def test_video_cli_rejects_missing_profile_without_starting_devices(self):
        result = subprocess.run([sys.executable, "-I", "-m", "openarm_runtime", "video"],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn("--profile", result.stderr)

    def test_tactile_cli_rejects_invalid_args_before_loading_viewer_or_hardware(self):
        script = """
import builtins
import sys
from unittest.mock import patch
from openarm_runtime.__main__ import main
original_import = builtins.__import__
def checked_import(name, *args, **kwargs):
    if name.split('.')[0] in {'viser', 'serial', 'paxini_sdk', 'numpy'} or name in {'openarm_tactile.app', 'openarm_tactile.dual'}:
        raise AssertionError('loaded hardware/viewer dependency: ' + name)
    return original_import(name, *args, **kwargs)
sys.argv = ['openarm-runtime', 'tactile', *sys.argv[1:]]
with patch('builtins.__import__', side_effect=checked_import):
    main()
"""
        cases = (
            ([], "required"),
            (["--simulate", "--device", "not-a-device"], "not allowed"),
            (["--simulate", "--device-id", "0"], "Device ID"),
            (["--simulate", "--device-id", "256"], "Device ID"),
            (["--simulate", "--baudrate", "0"], "baudrate"),
            (["--simulate", "--port", "1023"], "Port"),
            (["--simulate", "--right-device", "not-a-device"], "--right-device"),
            (["--device", "first", "--right-device", "second", "--port", "65525"], "65524"),
            (["--device", "same", "--right-device", "./same"], "different"),
        )
        with tempfile.TemporaryDirectory() as root:
            for args, expected in cases:
                with self.subTest(args=args):
                    result = subprocess.run([sys.executable, "-I", "-c", script, *args],
                                            cwd=root, capture_output=True, text=True)
                    self.assertEqual(result.returncode, 2, result.stderr)
                    self.assertIn(expected, result.stderr)

    def test_unconfirmed_mapping_never_opens_bus(self):
        with patch("openarm_runtime.control.ClassicBus") as factory:
            with self.assertRaises(ValueError), hold_arm(Profile("can1", "can0"), "right"):
                pass
        factory.assert_not_called()

    def test_wrong_mode_closes_without_enabling(self):
        bus = Mock()
        bus.parameter.return_value = 2
        with patch("openarm_runtime.control.bus_lease", return_value=nullcontext()), \
             patch("openarm_runtime.control.ClassicBus", return_value=bus), \
             patch("openarm_runtime.control.ArmHold") as controller:
            with self.assertRaisesRegex(RuntimeError, "expected MIT"), \
                 hold_arm(Profile("can1", "can0", mapping_verified=True), "right"):
                pass
        controller.assert_not_called()
        bus.command.assert_not_called()
        bus.close.assert_called_once()

    def test_lifecycle_propagates_errors_and_exits_control_before_closing_bus(self):
        events = []
        bus = Mock()
        bus.parameter.return_value = 1
        bus.close.side_effect = lambda: events.append("close")
        control = MagicMock()
        control.__exit__.side_effect = lambda *args: events.append("disable") or False
        record = Mock()
        with patch("openarm_runtime.control.bus_lease", return_value=nullcontext()), \
             patch("openarm_runtime.control.ClassicBus", return_value=bus), \
             patch("openarm_runtime.control.ArmHold", return_value=control):
            with self.assertRaisesRegex(RuntimeError, "application failure"):
                with hold_arm(Profile("can1", "can0", mapping_verified=True), "left", record=record):
                    raise RuntimeError("application failure")
        self.assertEqual(events, ["disable", "close"])
        self.assertEqual(record.call_args_list[1].args[0]["readback"], dict.fromkeys(range(1, 9), 1))

    def test_serial_context_closes_on_caller_error(self):
        bus = LeaderBus.__new__(LeaderBus)
        bus.close = Mock()
        with self.assertRaises(RuntimeError):
            with bus:
                raise RuntimeError("application failure")
        bus.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
