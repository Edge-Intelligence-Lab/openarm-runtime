"""Synthetic camera conventions only; EGL rendering never opens real devices."""
import importlib.util
import os
import unittest


@unittest.skipUnless(all(importlib.util.find_spec(n) for n in ('numpy','scipy','mujoco')),'simulation extras')
class CalibratedCameraTests(unittest.TestCase):
    def test_axes_intrinsics_and_off_center_projection(self):
        import numpy as np
        import mujoco as mj
        from openarm_simulation.calibrated_replay import camera_parameters
        K=[[200.,0.,183.],[0.,180.,101.],[0.,0.,1.]]
        p=camera_parameters(dict(image_size=[320,240],matrix=K),np.eye(4))
        self.assertTrue(np.allclose(p['quat'],[0,1,0,0]))
        # Render an optical-axis sphere: unequal fx/fy and noncenter cx/cy
        # must not silently turn into a symmetric fovy camera.
        if os.environ.get('MUJOCO_GL')!='egl':self.skipTest('explicit EGL renderer test')
        text=lambda xs:' '.join(map(str,xs))
        for x,y,z in ((0.,0.,1.),(-.2,.25,1.),(.3,-.2,1.5)):
            m=mj.MjModel.from_xml_string(f'''<mujoco><visual><global offwidth="320" offheight="240"/></visual>
            <worldbody><camera name="head" pos="0 0 0" quat="{text(p['quat'])}"
            sensorsize="320 240" resolution="320 240" focal="{text(p['intrinsic'][:2])}"
            principal="{text(p['intrinsic'][2:])}"/>
            <geom type="sphere" size=".015" pos="{x} {y} {z}" rgba="1 0 0 1"/>
            <light pos="0 0 0" dir="0 0 1"/></worldbody></mujoco>''')
            d=mj.MjData(m);mj.mj_forward(m,d)
            with mj.Renderer(m,240,320) as renderer:
                renderer.update_scene(d,camera='head');im=renderer.render()
            yy,xx=np.where((im[:,:,0]>80)&(im[:,:,1]<50)&(im[:,:,2]<50))
            self.assertGreater(len(xx),5)
            np.testing.assert_allclose([xx.mean(),yy.mean()],[200*x/z+183,180*y/z+101],atol=1.)

    def test_rejects_skew_or_invalid_transform(self):
        import numpy as np
        from openarm_simulation.calibrated_replay import camera_parameters
        with self.assertRaises(ValueError):
            camera_parameters(dict(image_size=[320,240],matrix=[[200,1,160],[0,200,120],[0,0,1]]),np.eye(4))
        with self.assertRaises(ValueError):
            camera_parameters(dict(image_size=[320,240],matrix=np.eye(3)),np.zeros((4,4)))

    @unittest.skipUnless(os.environ.get('MUJOCO_GL')=='egl','explicit EGL renderer test')
    def test_replay_board_uv_is_not_mirrored(self):
        import tempfile
        from pathlib import Path
        import hashlib
        import cv2
        import numpy as np
        import mujoco as mj
        from openarm_simulation.world import build_model
        from openarm_runtime.camera_calibration import charuco_pose
        spec=dict(squares_x=6,squares_y=4,square_size_m=.0225,marker_size_m=.0165,dictionary='DICT_4X4_50')
        with tempfile.TemporaryDirectory() as root:
            source=Path(root)/'robot.xml'
            source.write_text('<mujoco><worldbody><body name="tool"><geom size=".001"/></body></worldbody></mujoco>')
            c=dict(position_m=[.0675,-.02,-.3],target_m=[.0675,.045,0],width=480,height=360,fovy_deg=45)
            config=dict(geometry=dict(model_xml=str(source),model_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                robot_roots=['tool'],tool_body='tool'),cameras={'head':c},tool_from_board=np.eye(4))
            model=build_model(dict(board_spec=spec),config,board_visual_mesh=True)
            model.geom_rgba[mj.mj_name2id(model,mj.mjtObj.mjOBJ_GEOM,'sim_board'),3]=1.
            data=mj.MjData(model);mj.mj_forward(model,data)
            with mj.Renderer(model,360,480) as renderer:
                renderer.update_scene(data,camera='sim_head');image=renderer.render()
            import math
            f=180/math.tan(math.radians(45)/2)
            obs=charuco_pose(image,[[f,0,240],[0,f,180],[0,0,1]],[0.]*5,spec)
            self.assertGreaterEqual(len(obs['corner_ids']),8)
