"""Home UI wiring without constructing NiceGUI, robots or model servers."""
import unittest
import importlib.util
import tempfile
from unittest.mock import AsyncMock, Mock

from openarm_platform.model_view import ModelView


class HomeViewTests(unittest.IsolatedAsyncioTestCase):
    def view(self):
        view = ModelView.__new__(ModelView)
        view.workspace, view.ui = Mock(), Mock()
        view.owner = 'browser-one'
        view.select = Mock()
        view.select.value = 'registered'
        view.act = AsyncMock()
        return view

    async def test_home_can_be_requested_without_loading_or_testing_model(self):
        view = self.view()
        view.workspace.snapshot.return_value = dict(can_return_home=True, selected=None,
            models=[dict(id='registered', home_configured=True, home_requires_nominal=True)])
        await view.confirm_home()
        view.act.assert_awaited_once_with('return_home', 'registered', confirmed=True,
                                          owner='browser-one', allow_nominal=True)

    async def test_unconfigured_or_blocked_home_does_not_dispatch(self):
        view = self.view()
        for can, configured in ((False, True), (True, False)):
            view.workspace.snapshot.return_value = dict(can_return_home=can,
                models=[dict(id='registered', home_configured=configured)])
            await view.confirm_home()
        view.act.assert_not_awaited()

    async def test_start_uses_manual_stop_without_duration_or_auto_return(self):
        view = self.view()
        view.workspace.snapshot.return_value = dict(can_start=True, selected='registered',
            requires_nominal_opt_in=False, models=[dict(id='registered', home_configured=True)])
        await view.confirm_start()
        view.act.assert_awaited_once_with('start', confirmed=True, owner='browser-one',
                                          allow_nominal=False)

    async def test_transient_render_error_does_not_remove_stop_or_break_next_poll(self):
        view = self.view()
        view.workspace.readonly = False
        view.error, view.error_hint, view.start, view.home, view.stop = Mock(), Mock(), Mock(), Mock(), Mock()
        view._render = Mock(side_effect=[OSError('temporary profile replacement'), None])
        view.render()
        view.start.set_enabled.assert_called_with(False)
        view.home.set_enabled.assert_called_with(False)
        view.stop.set_enabled.assert_called_with(True)
        view.render()
        self.assertEqual(view._render.call_count, 2)


@unittest.skipUnless(importlib.util.find_spec('nicegui'), 'install platform extra')
class HomeViewConstructionTests(unittest.TestCase):
    def test_actual_widgets_are_present_and_passive_without_home_configuration(self):
        from nicegui import ui
        from openarm_platform.model_workspace import ModelWorkspace
        with tempfile.TemporaryDirectory() as root:
            manager = Mock()
            workspace = ModelWorkspace(None, root, manager, readonly=True)
            with ui.column() as container:
                view = ModelView(workspace)
            self.assertEqual(view.home.text, '归位')
            self.assertFalse(hasattr(view, 'test'))
            self.assertEqual(view.load.text, '加载')
            self.assertIn('自动', view.status_hint.text)
            self.assertFalse(view.home.enabled)
            self.assertFalse(hasattr(view, 'auto_home'))
            self.assertFalse(hasattr(view, 'duration'))
            for button in (view.load, view.start, view.stop, view.home):
                self.assertIn('model-action-button', button.classes)
            self.assertIn('model-load-button', view.load.classes)
            manager.claim_motion.assert_not_called()
            container.delete()
            workspace.close()
