"""Offline regressions: no physical CAN access."""
from contextlib import nullcontext
import copy
import threading
import time
import unittest
from unittest.mock import Mock, patch
from openarm_runtime.config import require_classic_link, require_classic_read_link
from openarm_runtime.telemetry import MotorRange, MotorState, Telemetry, poll, poll_reconnecting


def link():
    return dict(flags=['UP'], mtu=16, linkinfo=dict(info_data=dict(
        state='ERROR-WARNING', bittiming=dict(bitrate=1000000),
        ctrlmode=[], restart_ms=0, berr_counter=dict(tx=124, rx=0))))


class RecoveryTests(unittest.TestCase):
    def test_warning_allowed_only_for_reads_without_mutating_evidence(self):
        host = link()
        original = copy.deepcopy(host)
        require_classic_read_link(host)
        self.assertEqual(host, original)
        with self.assertRaises(RuntimeError):
            require_classic_link(host)

    def test_unsafe_configuration_and_states_still_rejected(self):
        for key, value in [('state', 'BUS-OFF'), ('state', 'ERROR-PASSIVE'),
                           ('state', 'STOPPED'), ('state', 'unknown'),
                           ('ctrlmode', ['FD']), ('restart_ms', 100),
                           ('bittiming', {'bitrate': 500000})]:
            host = link()
            host['linkinfo']['info_data'][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(RuntimeError):
                require_classic_read_link(host)
        for key, value in [('flags', []), ('mtu', 72)]:
            host = link()
            host[key] = value
            with self.assertRaises(RuntimeError):
                require_classic_read_link(host)

    def run_poll(self, shared, reader, host=None, cycles=1):
        with patch('openarm_runtime.config.bus_lease', return_value=nullcontext()), \
             patch('openarm_runtime.telemetry.inspect_link', return_value=host or link()), \
             patch('openarm_runtime.telemetry.ReadOnlyCAN', return_value=reader) as factory:
            poll('can0', shared, threading.Event(), frame_format='classic', max_cycles=cycles)
        return factory

    def reader(self):
        reader = Mock()
        reader.read_parameter.return_value = 4
        reader.read_range.return_value = MotorRange(12.5, 45, 54)
        reader.refresh.side_effect = lambda i, limits: MotorState(i, 0, 0, 0, 24, 24, 0, time.monotonic())
        return reader

    def test_warning_recovers_with_complete_new_feedback(self):
        shared = Telemetry(error='disconnected', baud_codes={99: 9})
        reader = self.reader()
        self.run_poll(shared, reader)
        self.assertEqual(shared.error, '')
        self.assertEqual(set(shared.states), set(range(1, 9)))
        self.assertEqual(shared.baud_codes, dict.fromkeys(range(1, 9), 4))
        self.assertEqual(reader.frame_format, 'classic')
        reader.close.assert_called_once()

    def test_wrong_mode_rejected_before_socket_or_send(self):
        host = link()
        host['mtu'] = 72
        reader = self.reader()
        shared = Telemetry()
        self.run_poll(shared, reader, host).assert_not_called()
        reader.read_parameter.assert_not_called()
        self.assertTrue(shared.error)

    def test_missing_one_motor_keeps_error_and_closes(self):
        shared = Telemetry(error='disconnected')
        reader = self.reader()
        refresh = reader.refresh.side_effect
        reader.refresh.side_effect = lambda i, limits: None if i == 8 else refresh(i, limits)
        self.run_poll(shared, reader, cycles=3)
        self.assertIn('three cycles', shared.error)
        self.assertNotIn(8, shared.states)
        reader.close.assert_called_once()

    def test_retry_backoff_capped_and_stop_interrupts_wait(self):
        shared = Telemetry()
        stop = Mock()
        stop.is_set.return_value = False
        stop.wait.side_effect = [False] * 7 + [True]
        with patch('openarm_runtime.telemetry.poll') as attempt:
            poll_reconnecting('can0', shared, stop, frame_format='classic')
        self.assertEqual(attempt.call_count, 8)
        self.assertEqual(shared.metadata()['connection_attempts'], 8)
        self.assertEqual(shared.metadata()['retry_delay_s'], 30)
        self.assertEqual([c.args[0] for c in stop.wait.call_args_list], [1, 2, 4, 8, 16, 30, 30, 30])
        self.assertTrue(all(c.kwargs['frame_format'] == 'classic' for c in attempt.call_args_list))

    def test_reconnect_after_failure_clears_error(self):
        shared = Telemetry()
        first, second = self.reader(), self.reader()
        first.read_parameter.side_effect = OSError('unplugged')
        stop = Mock()
        stop.is_set.return_value = False
        def wait(delay):
            first.close.assert_called_once()
            self.assertIn('unplugged', shared.error)
            return False
        stop.wait.side_effect = wait
        original_poll = poll
        readers = iter([first, second])
        def attempt(interface, shared, event, **kwargs):
            reader = next(readers)
            with patch('openarm_runtime.config.bus_lease', return_value=nullcontext()), \
                 patch('openarm_runtime.telemetry.inspect_link', return_value=link()), \
                 patch('openarm_runtime.telemetry.ReadOnlyCAN', return_value=reader):
                original_poll(interface, shared, threading.Event(), max_cycles=1, **kwargs)
            if reader is second:
                stop.is_set.return_value = True
        with patch('openarm_runtime.telemetry.poll', side_effect=attempt):
            poll_reconnecting('can0', shared, stop, frame_format='classic')
        self.assertEqual(shared.error, '')
        second.close.assert_called_once()
