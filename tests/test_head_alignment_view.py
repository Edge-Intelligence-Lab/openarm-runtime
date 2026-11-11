import importlib.util
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock,Mock,PropertyMock,patch


@unittest.skipUnless(importlib.util.find_spec('nicegui'),'platform extra')
class HeadAlignmentViewTests(unittest.IsolatedAsyncioTestCase):
    async def test_workspace_does_not_launch_on_open_and_embeds_its_own_service(self):
        from nicegui import Client,ui
        from openarm_platform.head_alignment_view import HeadAlignmentView
        client=Client(ui.page('/head-alignment-test'))
        state={'head_alignment':dict(reachable=False,state='standby',generation=0)}
        coordinator=SimpleNamespace(status=AsyncMock(side_effect=lambda:state),activate=AsyncMock(),deactivate=AsyncMock())
        try:
            with client,ui.column():
                view=HeadAlignmentView(SimpleNamespace(alignment_coordinator=coordinator,attach_only=False))
            view.mode_lock=Mock()
            coordinator.activate.assert_not_called()
            with patch.object(Client,'has_socket_connection',new_callable=PropertyMock,return_value=True):
                await view.refresh()
                self.assertTrue(view.start.enabled)
                self.assertFalse(view.frame.visible)
                state['head_alignment']=dict(reachable=True,state='running',generation=1)
                await view.refresh()
                self.assertTrue(view.frame.visible)
                self.assertEqual(view.frame._props['src'],'/apps/head_alignment/')
                self.assertIn('已就绪',view.message.text)
                self.assertFalse(view.start.enabled)
                self.assertTrue(view.close.enabled)
                view.mode_lock.assert_called_with(True)
                await view.call('deactivate')
                coordinator.deactivate.assert_awaited_once_with('head_alignment')
            coordinator.activate.assert_not_called()
        finally:client.delete()
