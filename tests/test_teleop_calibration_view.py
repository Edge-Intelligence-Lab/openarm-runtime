"""The real native GUI, with passive state fixtures and no robot readers."""
import importlib.util
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,MagicMock


@unittest.skipUnless(importlib.util.find_spec('viser'),'viewer extra')
class TeleopWorkspaceTests(unittest.TestCase):
    def setUp(self):
        import viser
        from openarm_platform.teleop_calibration_view import TeleopCalibrationView
        self.server=viser.ViserServer(host='127.0.0.1',port=0,verbose=False)
        self.state=dict(count=0,error=None,group=None,ready=False,stationary=False,owner=None,
            phase='connect',busy=False,next_role='fit',result=None,solve_error=None,solve_attempted=False,
            rejected=0,directory=None,history=[])
        self.calibration=Mock(status=lambda:dict(self.state))
        self.session=SimpleNamespace(busy=False,stop=Mock())
        self.view=TeleopCalibrationView(self.server,self.session,self.calibration,start_preview=False)
        self.addCleanup(self.server.stop);self.addCleanup(self.view.close)

    def test_controls_explain_readiness_and_fixed_validation_phase(self):
        import time
        self.view.refresh()
        self.assertTrue(self.view.begin.disabled)

        self.assertIn('开始左臂跟随',self.view.instruction.content)
        self.state.update(phase='mount',ready=True)
        self.view.mount.value=True;self.view.refresh()
        self.assertFalse(self.view.begin.disabled)
        self.state.update(phase='capture',group='group',count=3)
        self.view.refresh()
        self.assertTrue(self.view.capture.disabled)
        self.view.camera_good=True;self.view.preview_at=time.monotonic()
        self.state['stationary']=True;self.view.refresh()
        self.assertFalse(self.view.capture.disabled)
        self.assertEqual(self.view.capture.label,'采集拟合姿态 4/12')
        self.assertIn('点一次采集保存一张',self.view.instruction.content)
        self.state['count']=12;self.view.refresh()
        self.assertEqual(self.view.capture.label,'采集独立检查 1/4')
        self.assertIn('不参与拟合',self.view.instruction.content)
        self.state.update(phase='reset',error='夹爪已改变')
        self.view.refresh()
        self.assertFalse(self.view.mount.value)
        self.assertTrue(self.view.begin.disabled)
        self.assertIn('不是跳页',self.view.instruction.content)

    def test_video_is_continuous_and_zoom_is_client_local(self):
        self.assertIn('src="/apps/vision/stream/head.mjpg"',self.view.camera_image.content)
        self.assertIn('<img ',self.view.camera_image.content)
        gui=MagicMock();modal=gui.add_modal.return_value.__enter__.return_value
        self.view._zoom(SimpleNamespace(client=SimpleNamespace(gui=gui)))
        gui.add_html.assert_called_once_with(self.view._stream_html())
        gui.add_image.assert_not_called()
        gui.add_button.return_value.on_click.call_args.args[0](None)
        modal.close.assert_called_once()

    def test_stream_url_is_escaped(self):
        self.view.stream_url='/stream?a=1&b="x"'
        self.assertIn('a=1&amp;b=&quot;x&quot;',self.view._stream_html())

    def test_stale_camera_blocks_capture_without_stopping_human_control(self):
        import time
        self.state.update(phase='capture',group='group',ready=True,stationary=True)
        self.view.camera_good=True;self.view.preview_at=time.monotonic()-3
        self.session.busy=True
        self.view.refresh()
        self.assertTrue(self.view.capture.disabled)
        self.assertIn('过期',self.view.camera_status.content)
        self.assertFalse(self.view.stop.disabled)
        self.session.stop.assert_not_called()

    def test_failed_solve_stays_visible_and_does_not_offer_repeated_writes(self):
        self.state.update(phase='review',count=16,group='group',ready=True,
            solve_attempted=True,solve_error='validation: inconsistent stationary calibration')
        self.view.refresh()
        self.assertTrue(self.view.result.visible)
        self.assertIn('检查未通过',self.view.result.content)
        self.assertTrue(self.view.solve.disabled)
        self.assertTrue(self.view.restart.visible)
        self.assertFalse(self.view.restart.disabled)
