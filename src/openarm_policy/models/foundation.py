"""Published upright-foundation camera/state and gripper-only action adapters.

Model dependencies and devices remain outside the controller. Calibration files
are explicit external HF assets; no camera parameters or machine paths live here.
"""
import json
from pathlib import Path

from .tacpilot import UprightObservation, UprightActions, latest


class FoundationObservation(UprightObservation):
    def __init__(self, overview, wrist, image_contract, head_calibration,
                 wrist_calibration, side='left', state_contract=None):
        import numpy as np
        super().__init__(overview, wrist, side=side, state_contract=state_contract)
        contract = json.loads(Path(image_contract).read_text())
        expected = dict(schema='tacpilot.upright-foundation-images.v1',
                        views=['head_calibrated', 'camera_wrist_left'],
                        native_size_wh=[1280, 720], policy_size_wh=[256, 256],
                        preprocessing='Pillow bilinear full frame, RGB uint8, no crop/flip')
        if any(contract.get(k) != v for k, v in expected.items()):
            raise ValueError('Unsupported upright-foundation image contract')
        if side != 'left':
            raise ValueError('Published foundation model is left-arm only')
        head = json.loads(Path(head_calibration).read_text())
        wrist_spec = json.loads(Path(wrist_calibration).read_text())
        if (head.get('schema') != 'openarm.camera-baseline.v1'
                or head.get('version') != '20261110-v1'
                or wrist_spec.get('schema') != 'openarm.wrist-intrinsics.v1'):
            raise ValueError('Require the published head/wrist calibration formats')
        self.intrinsics = {}
        for name, camera in ((overview, head['camera']), (wrist, wrist_spec)):
            k, d = np.asarray(camera['matrix'], float), np.asarray(camera['distortion'], float)
            if (camera.get('image_size') != [1280, 720] or camera.get('distortion_model') != 'opencv_5'
                    or k.shape != (3, 3) or d.shape != (5,)
                    or not np.isfinite(k).all() or not np.isfinite(d).all()
                    or min(k[0, 0], k[1, 1]) <= 0 or not np.array_equal(k[2], [0., 0., 1.])):
                raise ValueError('Invalid native camera calibration')
            self.intrinsics[name] = (k, d)
        self.metadata.update(image_contract=contract,
            image_transform='native_BGR_to_RGB_both_undistort_same_K_Pillow_bilinear',
            calibration_status='installation_baseline_not_independent_hardware_validation')

    def image(self, observation, name):
        import cv2
        import numpy as np
        from PIL import Image
        image = np.asarray(latest(observation, name))
        if image.shape != (720, 1280, 3) or image.dtype != np.uint8:
            raise ValueError('Foundation cameras require native 1280x720 BGR uint8')
        k, d = self.intrinsics[name]
        rgb = cv2.undistort(np.ascontiguousarray(image[..., ::-1]), k, d, None, k)
        self.metadata.setdefault('source_shapes', {})[name] = list(image.shape)
        return np.asarray(Image.fromarray(rgb).resize((256, 256), Image.Resampling.BILINEAR)).copy()[None]


class FoundationActions(UprightActions):
    def __init__(self, action_contract, closed_opening_m, open_opening_m,
                 gripper_action_curve, side='left', execute_steps=25):
        contract = json.loads(Path(action_contract).read_text())
        expected = dict(schema='tacpilot.upright-normalized-delta.v1', dimension=7,
            translation_scale_m=.02, rotation_scale_rad=.1,
            translation_frame='world; relative to previous target',
            rotation_frame='world; left multiplication of previous target rotation',
            gripper='absolute normalized -1 open, +1 closed', control_hz=20,
            open_finger_target_rad=.7854, closed_finger_target_rad=-.1, executor='PaxiniTaskEnv.step')
        if contract != expected or side != 'left' or execute_steps != 25:
            raise ValueError('Require published foundation action semantics and execute25')
        super().__init__(closed_opening_m, open_opening_m, side, execute_steps,
                         gripper_action_curve=gripper_action_curve)
        self.metadata.update(action_contract=contract,
                             normalized_action_saturation={'gripper': [-1, 1], 'pose': 'reject'})

    def decode(self, output, observation):
        import numpy as np
        actions = np.array(output, dtype=float, copy=True)
        if (actions.ndim != 2 or actions.shape[1] != 7 or len(actions) < self.count
                or not np.isfinite(actions).all()):
            raise ValueError('Expected finite upright action chunk')
        actions = actions[:self.count]
        if (abs(actions[:, :6]) > 1).any():
            raise ValueError('Pose action outside [-1,1]; only gripper clipping is allowed')
        actions[:, 6] = np.clip(actions[:, 6], -1, 1)
        # The inherited Cartesian conversion now sees an already valid action7;
        # its historical saturation cannot alter any pose value.
        return super().decode(actions, observation)
