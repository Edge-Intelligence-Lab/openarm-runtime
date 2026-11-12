"""Actual NiceGUI element lifetimes around mocked service boundaries, no hardware."""
import asyncio
import copy
import importlib.util
import unittest
from unittest.mock import AsyncMock

from openarm_platform.config import load_config
from openarm_platform.dashboard import Dashboard, ssh_command


HAS_PLATFORM = importlib.util.find_spec('nicegui') is not None


def running():
    return {name: dict(state='standby' if name == 'teleop' else 'running',
                      reachable=name != 'teleop', managed=name != 'teleop', generation=1)
            for name in ('control', 'teleop', 'tactile', 'vision')} | {
                'policy': dict(state='running', reachable=True, managed=False, generation=0),
                'collection': dict(state='running', reachable=True, managed=False, generation=0)}


@unittest.skipUnless(HAS_PLATFORM, 'NiceGUI platform extra required')
class DashboardTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        from nicegui import Client, ui
        self.client = Client(ui.page('/dashboard-tests'))
        self.coordinator = AsyncMock()
        self.states = running()
        self.coordinator.status.side_effect = lambda: copy.deepcopy(self.states)
        self.coordinator.activate.side_effect = lambda name: copy.deepcopy(self.states)
        with self.client:
            self.dashboard = Dashboard(load_config(), self.coordinator)
            self.dashboard._set_hash = AsyncMock()
            await self.dashboard.poll()

    async def asyncTearDown(self):
        self.client.delete()
        await asyncio.sleep(0)

    async def test_sensing_navigation_preserves_controller_element_and_origin(self):
        d = self.dashboard
        with self.client:
            await d.select('control')
            controller = d.frames['control'].element
            await d.select('tactile')
            await d.select('vision')
            self.assertIs(d.frames['control'].element, controller)
            self.assertIn(controller.id, self.client.elements)
            self.assertFalse(controller.visible)
            self.assertEqual(d.frames['vision'].element.props['src'], '/apps/vision/')
            self.assertEqual(d.external.props['href'], '/apps/vision/')
            await d.select('control')
            self.assertIs(d.frames['control'].element, controller)
            self.assertTrue(controller.visible)
            d._render()
            d._render()
            self.assertIn('active', d.nav['control'].classes)
            self.assertNotIn('active', d.nav['vision'].classes)

    async def test_policy_uses_same_origin_and_preserves_control_frame(self):
        d = self.dashboard
        with self.client:
            await d.select('control')
            controller = d.frames['control'].element
            await d.select('policy')
            self.assertIs(d.frames['control'].element, controller)
            self.assertEqual(d.frames['policy'].element.props['src'], '/policy')
            self.assertEqual(d.external.props['href'], '/policy')
            await d.select('collection')
            self.assertIs(d.frames['control'].element, controller)
            self.assertEqual(d.frames['collection'].element.props['src'], '/collection')
            self.assertEqual(d.external.props['href'], '/collection')
            self.assertFalse(d.confirm_dialog.value)

    async def test_cancel_and_failed_switch_preserve_current_controller(self):
        d = self.dashboard
        with self.client:
            await d.select('control')
            controller = d.frames['control'].element
            calls = self.coordinator.activate.await_count
            selecting = asyncio.create_task(d.select('teleop'))
            await asyncio.sleep(0)
            self.assertTrue(d.busy)
            self.assertTrue(d.views.props['inert'])
            self.assertFalse(d.nav['tactile'].enabled)
            d.confirm_dialog.submit(False)
            await selecting
            self.assertEqual(self.coordinator.activate.await_count, calls)
            self.assertIs(d.frames['control'].element, controller)
            self.assertFalse(d.busy)
            self.assertTrue(controller.visible)
            self.coordinator.activate.side_effect = RuntimeError('Controller still stopping')
            selecting = asyncio.create_task(d.select('teleop'))
            await asyncio.sleep(0)
            d.confirm_dialog.submit(True)
            await selecting
            self.assertEqual(d.active, 'control')
            self.assertIs(d.frames['control'].element, controller)
            self.assertEqual(d.notice.text, 'Controller still stopping')
            self.assertTrue(d.notice.visible)
            self.assertNotIn('inert', d.views.props)

    async def test_successful_mode_switch_removes_only_confirmed_stopped_peer(self):
        d = self.dashboard
        with self.client:
            await d.select('control')
            controller = d.frames['control'].element
            await d.select('tactile')
            sensor = d.frames['tactile'].element
            self.states['control'].update(state='standby', reachable=False, managed=False)
            self.states['teleop'].update(state='running', reachable=True, managed=True)
            selecting = asyncio.create_task(d.select('teleop'))
            await asyncio.sleep(0)
            d.confirm_dialog.submit(True)
            await selecting
            self.assertNotIn('control', d.frames)
            self.assertNotIn(controller.id, self.client.elements)
            self.assertIs(d.frames['tactile'].element, sensor)
            self.assertEqual(d.frames['teleop'].element.props['src'], '/apps/teleop/')

    async def test_status_failure_retains_frame_and_service_error_is_actionable(self):
        d = self.dashboard
        with self.client:
            await d.select('vision')
            camera = d.frames['vision'].element
            self.coordinator.status.side_effect = OSError('Connection closed')
            await d.poll()
            self.assertIs(d.frames['vision'].element, camera)
            self.assertIn('SSH', d.notice.text)
            self.assertFalse(camera.visible)
            self.states['vision'].update(state='error', reachable=False, error='Camera process exited')
            self.coordinator.status.side_effect = lambda: copy.deepcopy(self.states)
            await d.poll()
            self.assertEqual(d.notice.text, 'Camera process exited')
            self.states['vision'].update(state='running', reachable=True, error=None)
            await d.poll()
            self.assertIs(d.frames['vision'].element, camera)
            self.assertTrue(camera.visible)
            self.assertFalse(d.notice.visible)

    async def test_old_status_poll_cannot_overwrite_a_completed_selection(self):
        d = self.dashboard
        stale = asyncio.Future()
        async def delayed():
            return await stale
        self.coordinator.status.side_effect = delayed
        with self.client:
            polling = asyncio.create_task(d.poll())
            await asyncio.sleep(0)
            self.states['vision']['generation'] = 2
            await d.select('vision')
            camera = d.frames['vision'].element
            stale.set_result(running())
            await polling
            self.assertEqual(d.statuses['vision']['generation'], 2)
            self.assertIs(d.frames['vision'].element, camera)

    async def test_restart_generation_and_explicit_refresh_replace_frame(self):
        d = self.dashboard
        with self.client:
            await d.select('vision')
            original = d.frames['vision'].element
            self.states['vision']['generation'] = 2
            await d.poll()
            restarted = d.frames['vision'].element
            self.assertIsNot(original, restarted)
            self.assertNotIn(original.id, self.client.elements)
            refreshing = asyncio.create_task(d.refresh())
            await asyncio.sleep(0)
            d.confirm_dialog.submit(False)
            await refreshing
            self.assertIs(d.frames['vision'].element, restarted)
            refreshing = asyncio.create_task(d.refresh())
            await asyncio.sleep(0)
            d.confirm_dialog.submit(True)
            await refreshing
            self.assertIsNot(d.frames['vision'].element, restarted)

    async def test_connection_command_forwards_only_the_platform_port(self):
        self.assertEqual(ssh_command(18888, 8888), 'ssh -N -L 18888:127.0.0.1:8888 USER@WORKSTATION')
        self.assertEqual(self.dashboard.command.text.count('-L '), 1)
        self.coordinator.activate.assert_not_awaited()

    async def test_mode_menu_does_not_rebuild_or_activate_current_controller(self):
        d = self.dashboard
        with self.client:
            await d.select('control')
            controller = d.frames['control'].element
            calls = self.coordinator.activate.await_count
            d.mode_menu.open()
            d.mode_menu.close()
            self.assertEqual(self.coordinator.activate.await_count, calls)
            self.assertIs(d.frames['control'].element, controller)
            self.assertTrue(controller.visible)
            self.assertFalse(any('platform-sidebar' in e.classes for e in self.client.elements.values()))
            await d.choose_mode('vision')
            self.assertEqual(d.mode_button.text, '视觉')
            self.assertIs(d.frames['control'].element, controller)

    async def test_top_stop_stays_available_during_switching_and_poll_failure(self):
        d = self.dashboard
        self.coordinator.stop_motion.return_value = copy.deepcopy(self.states)
        with self.client:
            d.busy = True
            d.status_error = True
            d._render()
            self.assertTrue(d.stop_button.enabled)
            self.assertFalse(d.mode_button.enabled)
            await d.stop_motion()
            self.coordinator.stop_motion.assert_awaited_once()
            self.assertIn('请求停止', d.notice.text)
            self.assertTrue(d.stop_button.enabled)

    async def test_readonly_top_stop_cannot_request_control(self):
        d = self.dashboard
        d.attach_only = True
        with self.client:
            d._render()
            self.assertFalse(d.stop_button.enabled)
            await d.stop_motion()
        self.coordinator.stop_motion.assert_not_awaited()


if __name__ == '__main__':
    unittest.main()
