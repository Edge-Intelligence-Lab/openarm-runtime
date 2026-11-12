"""Offline subprocess ownership tests; no hardware services are started."""
from pathlib import Path
import signal
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from openarm_platform.services import ServiceManager, tcp_reachable


def page(name="control", port=8893, command=("python", "service.py"), group=None):
    return SimpleNamespace(id=name, label=name, port=port, command=command, cwd=None,
                           exclusive_group=group)


def process(pid=10000):
    child = Mock(spec=subprocess.Popen)
    child.pid = pid
    child.poll.return_value = None
    child.wait.return_value = 0
    return child


class PlatformServiceTests(unittest.TestCase):
    def test_top_stop_only_closes_owned_motion_and_keeps_capture_alive(self):
        control, camera = process(10001), process(10002)
        with tempfile.TemporaryDirectory() as logs, \
             patch('openarm_platform.services.tcp_reachable', return_value=False), \
             patch('openarm_platform.services.subprocess.Popen', side_effect=[control, camera]), \
             patch.object(ServiceManager, '_alive', return_value=False), \
             patch.object(ServiceManager, '_signal') as signal_group:
            manager = ServiceManager([page(group='motion'), page('vision', 5003)], logs).start()
            state = manager.stop_motion()
            control.terminate.assert_called_once()
            camera.terminate.assert_not_called()
            signal_group.assert_not_called()
            self.assertEqual(state['control']['state'], 'standby')
            self.assertIs(manager._services['vision'].process, camera)
            manager.close()

    def test_top_stop_does_not_signal_external_services_or_release_model_lease(self):
        with patch('openarm_platform.services.tcp_reachable', return_value=True):
            manager = ServiceManager([page(group='motion')], '/unused').start()
            with self.assertRaisesRegex(ValueError, '外部控制'):
                manager.stop_motion()
            manager.close()
        with patch('openarm_platform.services.tcp_reachable', return_value=False):
            manager = ServiceManager([], '/unused').start()
            manager.claim_motion('model-session')
            manager.stop_motion()
            self.assertEqual(manager._motion_owner, 'model-session')
            manager.release_motion('model-session', clean=True)
            manager.close()
    def test_read_connections_pause_before_granting_motion_and_close_with_manager(self):
        with patch('openarm_platform.services.tcp_reachable', return_value=False):
            manager = ServiceManager([], '/unused').start()
            reader = Mock()
            manager.register_passive_reader(reader)
            manager.claim_motion('operator-start')
            reader.pause.assert_called_once()
            self.assertTrue(manager.motion_active())
            manager.release_motion('operator-start', clean=True)
            self.assertFalse(manager.motion_active())
            manager.close()
            reader.close.assert_called_once()

    def test_probe_cleanup_failure_blocks_motion_instead_of_launching_a_controller(self):
        with patch('openarm_platform.services.tcp_reachable', return_value=False):
            manager = ServiceManager([], '/unused').start()
            reader = Mock(); reader.pause.side_effect = RuntimeError('reader still stopping')
            manager.register_passive_reader(reader)
            with self.assertRaisesRegex(RuntimeError, 'reader still stopping'):
                manager.claim_motion('operator-start')
            self.assertIsNone(manager._motion_owner)
            manager.close()

    def test_construction_is_passive_and_invalid_ids_are_rejected(self):
        with patch("openarm_platform.services.tcp_reachable") as probe, \
             patch("openarm_platform.services.subprocess.Popen") as spawn:
            manager = ServiceManager([page()], "/unused")
            probe.assert_not_called()
            spawn.assert_not_called()
            manager.close()
            spawn.assert_not_called()
        for pages in ([page("../bad")], [page(), page()]):
            with self.assertRaises(ValueError):
                ServiceManager(pages, "/unused")

    def test_existing_services_are_reused_and_never_signalled(self):
        with tempfile.TemporaryDirectory() as logs, \
             patch("openarm_platform.services.tcp_reachable", return_value=True), \
             patch("openarm_platform.services.subprocess.Popen") as spawn, \
             patch.object(ServiceManager, "_signal") as kill:
            manager = ServiceManager([page()], logs).start()
            state = manager.status()["control"]
            self.assertEqual(state["state"], "external")
            self.assertTrue(state["reachable"])
            self.assertFalse(state["managed"])
            manager.close()
            spawn.assert_not_called()
            kill.assert_not_called()

    def test_missing_command_stays_unconfigured(self):
        with patch("openarm_platform.services.tcp_reachable", return_value=False), \
             patch("openarm_platform.services.subprocess.Popen") as spawn:
            manager = ServiceManager([page(command=())], "/unused").start()
            self.assertEqual(manager.status()["control"]["state"], "unconfigured")
            manager.close()
            spawn.assert_not_called()

    def test_spawn_failure_does_not_prevent_other_services_and_logs_close(self):
        child = process()
        with tempfile.TemporaryDirectory() as logs, \
             patch("openarm_platform.services.tcp_reachable", return_value=False), \
             patch("openarm_platform.services.subprocess.Popen", side_effect=[
                 FileNotFoundError("missing executable"), child]) as spawn, \
             patch.object(ServiceManager, "_signal"), \
             patch.object(ServiceManager, "_alive", return_value=False):
            manager = ServiceManager([page(), page("vision", 5003)], logs).start()
            states = manager.status()
            self.assertEqual(states["control"]["state"], "error")
            self.assertIn("missing executable", states["control"]["error"])
            self.assertEqual(states["vision"]["state"], "starting")
            self.assertTrue(states["vision"]["managed"])
            for call in spawn.call_args_list:
                self.assertTrue(call.kwargs["stdout"].closed)
                self.assertNotIn("shell", call.kwargs)
                self.assertEqual(call.kwargs["stdin"], subprocess.DEVNULL)
            manager.close()
            child.wait.assert_called_once()
            self.assertTrue(Path(logs, "vision.log").is_file())

    def test_early_exit_is_error_even_if_another_listener_appears(self):
        child = process()
        with tempfile.TemporaryDirectory() as logs, \
             patch("openarm_platform.services.tcp_reachable", side_effect=[False, True]), \
             patch("openarm_platform.services.subprocess.Popen", return_value=child), \
             patch.object(ServiceManager, "_signal"), \
             patch.object(ServiceManager, "_alive", return_value=False):
            manager = ServiceManager([page()], logs).start()
            child.poll.return_value = 4
            state = manager.status()["control"]
            self.assertEqual(state["state"], "error")
            self.assertIn("status 4", state["error"])
            self.assertTrue(state["reachable"])
            manager.close()

    def test_close_signals_only_owned_sessions_and_is_idempotent(self):
        child = process()
        with tempfile.TemporaryDirectory() as logs, \
             patch("openarm_platform.services.tcp_reachable", side_effect=[True, False]), \
             patch("openarm_platform.services.subprocess.Popen", return_value=child) as spawn, \
             patch.object(ServiceManager, "_signal") as kill, \
             patch.object(ServiceManager, "_alive", return_value=False):
            manager = ServiceManager([page(), page("tactile", 8891)], logs).start()
            manager.close()
            manager.close()
            kill.assert_called_once_with(child, signal.SIGTERM)
            self.assertEqual(spawn.call_args.args[0], ["python", "service.py"])
            child.wait.assert_called_once()
            with self.assertRaises(RuntimeError):
                manager.start()

    def test_unresponsive_process_group_is_killed_after_grace_period(self):
        child = process()
        with tempfile.TemporaryDirectory() as logs, \
             patch("openarm_platform.services.tcp_reachable", return_value=False), \
             patch("openarm_platform.services.subprocess.Popen", return_value=child), \
             patch.object(ServiceManager, "_signal") as kill, \
             patch.object(ServiceManager, "_alive", return_value=True), \
             patch("openarm_platform.services.SHUTDOWN_TIMEOUT", 0):
            manager = ServiceManager([page()], logs).start()
            manager.close()
            self.assertEqual([c.args[1] for c in kill.call_args_list],
                             [signal.SIGTERM, signal.SIGKILL])
            child.wait.assert_called_once()


    def test_start_runs_one_motion_service_and_all_independent_services(self):
        control, vision = process(10001), process(10002)
        with tempfile.TemporaryDirectory() as logs, \
             patch("openarm_platform.services.tcp_reachable", return_value=False), \
             patch("openarm_platform.services.subprocess.Popen", side_effect=[control, vision]) as spawn, \
             patch.object(ServiceManager, "_signal"), \
             patch.object(ServiceManager, "_alive", return_value=False):
            manager = ServiceManager([page(group="motion"), page("teleop", 8890, group="motion"),
                                      page("vision", 5003)], logs).start()
            states = manager.status()
            self.assertEqual(spawn.call_count, 2)
            self.assertEqual(states["teleop"]["state"], "standby")
            self.assertEqual(states["teleop"]["generation"], 0)
            self.assertEqual(states["control"]["generation"], 1)
            self.assertTrue(states["vision"]["managed"])
            manager.close()

    def test_external_motion_peer_blocks_switch_without_terminating_it(self):
        reachable = {8893: True, 8890: False}
        with tempfile.TemporaryDirectory() as logs, \
             patch("openarm_platform.services.tcp_reachable", side_effect=lambda port: reachable[port]), \
             patch("openarm_platform.services.subprocess.Popen") as spawn, \
             patch.object(ServiceManager, "_signal") as kill:
            manager = ServiceManager([page(group="motion"), page("teleop", 8890, group="motion")], logs).start()
            self.assertEqual(manager.status()["teleop"]["state"], "standby")
            with self.assertRaisesRegex(ValueError, "请先停止独立运行"):
                manager.activate("teleop")
            manager.close()
            spawn.assert_not_called()
            kill.assert_not_called()

    def test_motion_switch_waits_for_parent_cleanup_and_preserves_vision(self):
        control, vision, teleop, next_control = [process(pid) for pid in range(10001, 10005)]
        with tempfile.TemporaryDirectory() as logs, \
             patch("openarm_platform.services.tcp_reachable", return_value=False), \
             patch("openarm_platform.services.subprocess.Popen", side_effect=[control, vision, teleop, next_control]) as spawn, \
             patch.object(ServiceManager, "_signal") as group_signal, \
             patch.object(ServiceManager, "_alive", return_value=False):
            manager = ServiceManager([page(group="motion"), page("teleop", 8890, group="motion"),
                                      page("vision", 5003)], logs).start()
            states = manager.activate("teleop")
            control.terminate.assert_called_once()
            control.wait.assert_called_once()
            group_signal.assert_not_called()
            self.assertEqual(states["control"]["state"], "standby")
            self.assertEqual(states["teleop"]["generation"], 1)
            self.assertTrue(states["vision"]["managed"])
            vision.terminate.assert_not_called()
            states = manager.activate("control")
            self.assertEqual(states["control"]["generation"], 2)
            self.assertEqual(spawn.call_count, 4)
            manager.close()

    def test_motion_timeout_keeps_process_and_never_forces_or_starts_peer(self):
        control = process()
        with tempfile.TemporaryDirectory() as logs, \
             patch("openarm_platform.services.tcp_reachable", return_value=False), \
             patch("openarm_platform.services.subprocess.Popen", return_value=control) as spawn, \
             patch.object(ServiceManager, "_signal") as kill, \
             patch.object(ServiceManager, "_alive", return_value=True), \
             patch("openarm_platform.services.MOTION_SHUTDOWN_TIMEOUT", 0):
            manager = ServiceManager([page(group="motion"), page("teleop", 8890, group="motion")], logs).start()
            with self.assertRaisesRegex(ValueError, "未能正常退出"):
                manager.activate("teleop")
            self.assertEqual(spawn.call_count, 1)
            state = manager.status()["control"]
            self.assertTrue(state["managed"])
            self.assertEqual(state["state"], "error")
            with self.assertRaisesRegex(ValueError, "重启平台"):
                manager.activate("teleop")
            with self.assertRaisesRegex(RuntimeError, "关闭超时"):
                manager.close()
            kill.assert_not_called()
            control.kill.assert_not_called()
            control.wait.assert_not_called()
            # A later explicit cleanup can succeed after the process finishes.
            with patch("openarm_platform.services.MOTION_SHUTDOWN_TIMEOUT", 1), \
                 patch.object(ServiceManager, "_alive", return_value=False):
                manager.close()
            control.wait.assert_called_once()

    def test_failed_motion_exit_blocks_peer_and_requires_operator_restart(self):
        control = process()
        control.wait.return_value = 1
        with tempfile.TemporaryDirectory() as logs, \
             patch("openarm_platform.services.tcp_reachable", return_value=False), \
             patch("openarm_platform.services.subprocess.Popen", return_value=control) as spawn, \
             patch.object(ServiceManager, "_alive", return_value=False):
            manager = ServiceManager([page(group="motion"), page("teleop", 8890, group="motion")], logs).start()
            with self.assertRaisesRegex(ValueError, "退出状态为 1"):
                manager.activate("teleop")
            with self.assertRaises(ValueError):
                manager.activate("teleop")
            self.assertEqual(spawn.call_count, 1)
            manager.close()

    def test_unexpected_motion_exit_blocks_peer(self):
        control = process()
        with tempfile.TemporaryDirectory() as logs, \
             patch("openarm_platform.services.tcp_reachable", return_value=False), \
             patch("openarm_platform.services.subprocess.Popen", return_value=control) as spawn, \
             patch.object(ServiceManager, "_alive", return_value=False):
            manager = ServiceManager([page(group="motion"), page("teleop", 8890, group="motion")], logs).start()
            control.poll.return_value = 1
            with self.assertRaisesRegex(ValueError, "异常退出"):
                manager.activate("teleop")
            self.assertEqual(spawn.call_count, 1)
            manager.close()

    def test_selecting_running_capture_does_not_duplicate_or_restart_it(self):
        child = process()
        with tempfile.TemporaryDirectory() as logs, \
             patch("openarm_platform.services.tcp_reachable", return_value=False), \
             patch("openarm_platform.services.subprocess.Popen", return_value=child) as spawn, \
             patch.object(ServiceManager, "_signal") as kill, \
             patch.object(ServiceManager, "_alive", return_value=False):
            manager = ServiceManager([page("tactile", 8891)], logs).start()
            self.assertEqual(manager.activate("tactile")["tactile"]["generation"], 1)
            self.assertEqual(spawn.call_count, 1)
            kill.assert_not_called()
            manager.close()

    def test_capture_retry_reaps_failed_session_before_relaunch(self):
        first, second = process(10001), process(10002)
        events = []
        first.wait.side_effect = lambda **_: events.append("reaped") or 1
        def spawn(*_, **__):
            events.append("spawn")
            return first if events.count("spawn") == 1 else second
        with tempfile.TemporaryDirectory() as logs, \
             patch("openarm_platform.services.tcp_reachable", return_value=False), \
             patch("openarm_platform.services.subprocess.Popen", side_effect=spawn), \
             patch.object(ServiceManager, "_signal") as kill, \
             patch.object(ServiceManager, "_alive", return_value=False):
            manager = ServiceManager([page("tactile", 8891)], logs).start()
            first.poll.return_value = 1
            self.assertEqual(manager.status()["tactile"]["state"], "error")
            state = manager.activate("tactile")["tactile"]
            self.assertEqual(events, ["spawn", "reaped", "spawn"])
            kill.assert_called_once_with(first, signal.SIGTERM)
            self.assertEqual(state["generation"], 2)
            self.assertEqual(state["state"], "starting")
            self.assertIsNone(state["error"])
            manager.close()

    def test_capture_retry_does_not_launch_when_owned_cleanup_fails(self):
        child = process()
        child.wait.side_effect = subprocess.TimeoutExpired("capture", 2)
        with tempfile.TemporaryDirectory() as logs, \
             patch("openarm_platform.services.tcp_reachable", return_value=False), \
             patch("openarm_platform.services.subprocess.Popen", return_value=child) as spawn, \
             patch.object(ServiceManager, "_signal"), \
             patch.object(ServiceManager, "_alive", return_value=False):
            manager = ServiceManager([page("tactile", 8891)], logs).start()
            child.poll.return_value = 1
            with self.assertRaisesRegex(ValueError, "清理失败"):
                manager.activate("tactile")
            self.assertEqual(spawn.call_count, 1)
            # Status polling preserves the actionable failure message.
            self.assertIn("清理失败", manager.status()["tactile"]["error"])
            child.wait.side_effect = None
            manager.close()

    def test_capture_retry_after_spawn_failure_and_external_disappearance(self):
        child = process()
        with tempfile.TemporaryDirectory() as logs, \
             patch("openarm_platform.services.tcp_reachable", return_value=False) as reachable, \
             patch("openarm_platform.services.subprocess.Popen", side_effect=[
                 FileNotFoundError("missing executable"), child]) as spawn, \
             patch.object(ServiceManager, "_signal"), \
             patch.object(ServiceManager, "_alive", return_value=False):
            manager = ServiceManager([page("vision", 5003)], logs).start()
            self.assertEqual(manager.status()["vision"]["state"], "error")
            reachable.return_value = True
            state = manager.activate("vision")["vision"]
            self.assertEqual(state["state"], "external")
            self.assertFalse(state["managed"])
            self.assertEqual(spawn.call_count, 1)
            reachable.return_value = False
            self.assertTrue(manager.activate("vision")["vision"]["managed"])
            self.assertEqual(spawn.call_count, 2)
            manager.close()

    def test_posix_signal_targets_the_owned_group_not_just_parent(self):
        child = process()
        with patch("openarm_platform.services.os.name", "posix"), \
             patch("openarm_platform.services.os.killpg") as kill:
            ServiceManager._signal(child, signal.SIGTERM)
            kill.assert_called_once_with(child.pid, signal.SIGTERM)

    def test_probe_is_loopback_only_and_closes_socket(self):
        connection = Mock()
        connection.__enter__ = Mock(return_value=connection)
        connection.__exit__ = Mock(return_value=False)
        with patch("openarm_platform.services.socket.create_connection", return_value=connection) as create:
            self.assertTrue(tcp_reachable(8893))
            create.assert_called_once_with(("127.0.0.1", 8893), timeout=0.15)
            connection.__exit__.assert_called_once()
        with patch("openarm_platform.services.socket.create_connection", side_effect=ConnectionRefusedError):
            self.assertFalse(tcp_reachable(8893))


if __name__ == "__main__":
    unittest.main()
