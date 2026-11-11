"""TacPilot upright state/camera and delta-action adapters, without TacPilot imports.

    RGB + measured task-frame TCP + physical finger gap -> raw state8.
    action7 -> absolute task-frame TCP goals, NOT absolute motor positions.
"""
from ..contracts import TCP_POSE


def latest(observation, name):
    samples = observation.streams.get(name, ())
    if not samples or not samples[-1].valid or samples[-1].error:
        raise ValueError(f"Missing valid model input: {name}")
    sample = samples[-1]
    if sample.received_at > observation.captured_at:
        raise ValueError(f"Future model input: {name}")
    return sample.data


class UprightObservation:
    def __init__(self, overview, wrist, side="left", size=256, crops=None, execution=False,
                 image_contract=None, head_calibration=None):
        if side not in ("left", "right") or type(size) is not int or size < 1:
            raise ValueError("Invalid side or image size")
        if overview == wrist:
            raise ValueError("Two distinct camera streams required")
        self.overview, self.wrist, self.side = overview, wrist, side
        self.size, self.crops = size, dict(crops or {})
        if type(execution) is not bool:raise ValueError('execution must be boolean')
        self.execution = execution
        if set(self.crops) - {overview, wrist}:
            raise ValueError("Crop names must match selected camera streams")
        self.metadata = dict(overview=overview, wrist=wrist, side=side, size=size,
                              state_width=10 if execution else 8,
                              crops=self.crops, image_transform="BGR_to_RGB_area_resize")
        self.camera_matrix = self.distortion = None
        self.wrist_native_shape = None
        self.wrist_matrix = self.wrist_distortion = None
        if image_contract is not None or head_calibration is not None:
            import json
            from pathlib import Path
            import numpy as np
            if image_contract is None or head_calibration is None or self.crops or size != 256:
                raise ValueError("Calibrated images require contract/calibration, size256 and no crop")
            raw = Path(image_contract).read_bytes()
            contract = json.loads(raw)
            expected = {"schema": "tacpilot.policy-images.v1", "channels": "RGB", "dtype": "uint8",
                        "head_geometry": "ideal_pinhole_same_K_undistorted",
                        "head_render_size_wh": [1280, 720], "output_size_wh": [256, 256],
                        "head_resize": "Pillow_bilinear_full_frame_no_crop",
                        "real_head_preprocessing": "undistort native RGB with frozen K/distortion and new_K=K, then same resize"}
            if any(contract.get(key) != value for key, value in expected.items()):
                raise ValueError("Unsupported calibrated ACT image contract")
            wrist_size = contract.get("wrist_render_size_wh")
            if wrist_size == [1280, 720]:
                wrist_spec = contract.get("wrist")
                expected_wrist = {"source": "pinned_upstream",
                    "asset_revision": "06b4c8ad6d547c3faaa3bf780629ee3e1ac2b0d0"}
                if (contract.get("wrist_resize") != "Pillow_bilinear_full_frame_no_crop"
                        or not isinstance(wrist_spec,dict)
                        or any(wrist_spec.get(k)!=v for k,v in expected_wrist.items())
                        or set(wrist_spec)-set(expected_wrist)-{'intrinsics','optical_center_forward_m','positions_m'}
                        or contract.get("wrist_fovy_jitter_deg") != 0.):
                    raise ValueError("Unsupported native wrist image contract")
                offset = wrist_spec.get('optical_center_forward_m',0.)
                if type(offset) not in (int,float) or not np.isfinite(offset) or not 0<=offset<=.03:
                    raise ValueError('Invalid wrist optical-centre offset')
                if 'positions_m' in wrist_spec:
                    positions = wrist_spec['positions_m']
                    if (not isinstance(positions,dict) or set(positions) != {'left','right'}
                            or any(np.asarray(p).shape != (3,) or not np.isfinite(p).all() for p in positions.values())):
                        raise ValueError('Invalid wrist camera positions')
                measured = wrist_spec.get('intrinsics')
                if measured is not None:
                    if (not isinstance(measured,dict)
                            or measured.get('schema') not in ('openarm.shared-wrist-intrinsics.v1','openarm.wrist-intrinsics.v1')
                            or measured.get('image_size')!=[1280,720]
                            or measured.get('distortion_model')!='opencv_5'
                            or (measured.get('schema')=='openarm.shared-wrist-intrinsics.v1' and
                                (measured.get('measured_side')!='left' or measured.get('applies_to')!=['left','right']
                                 or measured.get('right_reuse')!='operator_authorized_not_independently_calibrated'))
                            or contract.get('wrist_geometry')!='ideal_pinhole_same_K_undistorted'
                            or contract.get('real_wrist_preprocessing')!='undistort native RGB with shared wrist K/distortion and new_K=K, then same resize'):
                        raise ValueError('Unsupported measured wrist image contract')
                    k,d=np.asarray(measured.get('matrix'),float),np.asarray(measured.get('distortion'),float)
                    if (k.shape!=(3,3) or d.shape!=(5,) or not np.isfinite(k).all() or not np.isfinite(d).all()
                            or min(k[0,0],k[1,1])<=0 or not np.array_equal(k[2],[0.,0.,1.])
                            or k[0,1]!=0 or k[1,0]!=0):
                        raise ValueError('Invalid wrist intrinsic matrix/distortion')
                    self.wrist_matrix,self.wrist_distortion=k,d
                self.wrist_native_shape = (720, 1280, 3)
            elif wrist_size != [256, 256] or "wrist_resize" in contract:
                raise ValueError("Unsupported calibrated ACT wrist image geometry")
            calibration_raw = Path(head_calibration).read_bytes()
            head = contract.get("head", {})
            calibration = json.loads(calibration_raw)
            camera = calibration["camera"]
            if (calibration.get("schema") != "openarm.camera-baseline.v1"
                    or calibration.get("version") != head.get("version")
                    or camera.get("image_size") != [1280, 720]
                    or camera.get("image_geometry") != "native_no_transform"
                    or camera.get("distortion_model") != "opencv_5"):
                raise ValueError("Unsupported native head calibration")
            self.camera_matrix = np.asarray(camera["matrix"], dtype=float)
            self.distortion = np.asarray(camera["distortion"], dtype=float)
            if (self.camera_matrix.shape != (3, 3) or self.distortion.shape != (5,)
                    or not np.isfinite(self.camera_matrix).all() or not np.isfinite(self.distortion).all()
                    or min(self.camera_matrix[0, 0], self.camera_matrix[1, 1]) <= 0
                    or not np.array_equal(self.camera_matrix[2], [0., 0., 1.])):
                raise ValueError("Invalid head intrinsic matrix/distortion")
            self.metadata.update(image_contract=contract,
                image_transform="native_BGR_to_RGB_head_undistort_same_K_Pillow_bilinear",
                wrist_calibration=contract.get('wrist_calibration','nominal_not_hardware_calibrated'))

    def reset(self, context):
        specs = {s.name: s for s in context.streams}
        for name in (self.overview, self.wrist):
            if name not in specs or specs[name].encoding != "bgr8" or not specs[name].required:
                raise ValueError(f"Required BGR camera not declared: {name}")
        tcp = specs.get(f"tcp/{self.side}")
        if tcp is None or tcp.shape != (8,) or not tcp.required:
            raise ValueError("Required calibrated measured TCP/opening stream missing")
        if self.execution:
            status = specs.get(f'execution/{self.side}')
            if status is None or status.shape != (2,) or not status.required:
                raise ValueError('Execution-aware checkpoint requires measured execution feedback stream')

    def image(self, observation, name):
        import cv2
        import numpy as np
        image = np.asarray(latest(observation, name))
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("Expected HWC uint8 BGR camera image")
        if self.camera_matrix is not None:
            from PIL import Image
            rgb = np.ascontiguousarray(image[..., ::-1])
            if name == self.overview:
                if rgb.shape != (720, 1280, 3):
                    raise ValueError("Calibrated head requires native 1280x720, not resized previews")
                rgb = cv2.undistort(rgb, self.camera_matrix, self.distortion, None, self.camera_matrix)
            elif self.wrist_native_shape is not None and rgb.shape != self.wrist_native_shape:
                raise ValueError("Native wrist requires 1280x720, not resized previews")
            elif self.wrist_matrix is not None:
                rgb = cv2.undistort(rgb,self.wrist_matrix,self.wrist_distortion,None,self.wrist_matrix)
            self.metadata.setdefault("source_shapes", {})[name] = list(image.shape)
            return np.asarray(Image.fromarray(rgb).resize((256, 256), Image.Resampling.BILINEAR)).copy()[None]
        if name in self.crops:
            crop = self.crops[name]
            if len(crop) != 4 or any(type(v) is not int for v in crop):
                raise ValueError("Crop must be integer [x, y, width, height]")
            x, y, w, h = crop
            if min(x, y) < 0 or min(w, h) < 1 or x+w > image.shape[1] or y+h > image.shape[0]:
                raise ValueError("Crop outside image")
            image = image[y:y+h, x:x+w]
        image = cv2.resize(image, (self.size, self.size), interpolation=cv2.INTER_AREA)
        return np.ascontiguousarray(image[..., ::-1])[None]

    def encode(self, observation):
        import numpy as np
        from scipy.spatial.transform import Rotation
        tcp = np.asarray(latest(observation, f"tcp/{self.side}"), dtype=float)
        if (tcp.shape != (8,) or not np.isfinite(tcp).all() or tcp[7] < 0
                or not np.isclose(np.linalg.norm(tcp[3:7]), 1., atol=1e-6)):
            raise ValueError("Invalid measured task-frame TCP/opening")
        state = np.r_[tcp[:3], Rotation.from_quat(tcp[3:7]).as_rotvec(), [tcp[7]/2]*2]
        if self.execution:
            feedback = np.asarray(latest(observation, f'execution/{self.side}'), dtype=float)
            if (feedback.shape != (2,) or not np.isfinite(feedback).all()
                    or feedback[0] not in (0, 1) or not 0 <= feedback[1] <= .15):
                raise ValueError('Invalid measured execution feedback')
            state = np.r_[state, feedback[0], feedback[1]/.15]
        return {"state": state.astype(np.float32)[None],
                "image": self.image(observation, self.overview),
                "wrist": self.image(observation, self.wrist)}


class UprightActions:
    """Checkpoint action7: normalized task-frame increments and +1=closed gripper.

    Anchor at the observation's last DISPATCHED Cartesian goal, not an unexecuted
    previous predicted chunk. All returned steps are absolute and safe to hold or
    skip on latency. The hardware wrapper handles bounded feedback-driven IK.
    """
    def __init__(self, closed_opening_m, open_opening_m, side="left", execute_steps=25,
                 translation_scale_m=.02, rotation_scale_rad=.1, gripper_action_curve=None):
        import math
        values = (closed_opening_m, open_opening_m, translation_scale_m, rotation_scale_rad)
        if (any(isinstance(v, bool) or not math.isfinite(v) for v in values)
                or not 0 <= closed_opening_m < open_opening_m
                or min(translation_scale_m, rotation_scale_rad) <= 0
                or type(execute_steps) is not int or not 1 <= execute_steps <= 100
                or side not in ("left", "right")):
            raise ValueError("Invalid upright action calibration/scales")
        self.closed, self.open = closed_opening_m, open_opening_m
        self.side, self.count = side, execute_steps
        self.translation, self.rotation = translation_scale_m, rotation_scale_rad
        self.gripper_curve = gripper_action_curve
        if gripper_action_curve is not None:
            import numpy as np
            curve = np.asarray(gripper_action_curve, dtype=float)
            if (curve.ndim != 2 or curve.shape[1] != 2 or len(curve) < 2
                    or not np.isfinite(curve).all() or not (np.diff(curve[:, 0]) > 0).all()
                    or curve[0, 0] != -1 or curve[-1, 0] != 1
                    or not (np.diff(curve[:, 1]) <= 0).all()
                    or not np.isclose(curve[0, 1], self.open) or not np.isclose(curve[-1, 1], self.closed)):
                raise ValueError("Gripper curve must map increasing [-1,+1] actions to decreasing apertures")
            self.gripper_curve = curve.tolist()
        self.metadata = dict(side=side, execute_steps=execute_steps,
                             translation_scale_m=self.translation, rotation_scale_rad=self.rotation,
                             gripper_mapping="action_aperture_curve" if self.gripper_curve else "linear_aperture_endpoints",
                             gripper_action_curve=self.gripper_curve,
                             closed_opening_m=self.closed, open_opening_m=self.open,
                             normalized_action_saturation=[-1, 1])

    def reset(self, context):
        expected = tuple(f"{self.side}.{n}" for n in ("x", "y", "z", "qx", "qy", "qz", "qw", "opening"))
        if context.action.kind != TCP_POSE or context.action.names != expected:
            raise ValueError("UprightActions requires the selected CartesianRobot action contract")
        if not any(s.name == f"target/{self.side}" and s.required for s in context.streams):
            raise ValueError("Dispatched Cartesian reference stream is required")
        if self.closed < context.action.lower[7] or self.open > context.action.upper[7]:
            raise ValueError("Action opening range exceeds physical calibration")

    def decode(self, output, observation):
        import numpy as np
        from scipy.spatial.transform import Rotation
        actions = np.asarray(output)
        if (actions.ndim != 2 or actions.shape[1] != 7 or len(actions) < self.count
                or not np.isfinite(actions).all()):
            raise ValueError("Expected a finite H x 7 upright prediction")
        # Match the frozen simulator's explicit normalized-action saturation.
        actions = np.clip(actions[:self.count], -1, 1)
        anchor = np.asarray(latest(observation, f"target/{self.side}"), dtype=float)
        if anchor.shape != (8,) or not np.isfinite(anchor).all():
            raise ValueError("Invalid dispatched Cartesian anchor")
        position = anchor[:3].copy()
        rotation = Rotation.from_quat(anchor[3:7])
        result = []
        for row in actions:
            position += row[:3] * self.translation
            rotation = Rotation.from_rotvec(row[3:6] * self.rotation) * rotation
            opening = self.closed + (1-row[6])/2 * (self.open-self.closed)
            if self.gripper_curve is not None:
                curve = np.asarray(self.gripper_curve)
                opening = float(np.interp(row[6], curve[:, 0], curve[:, 1]))
            result.append(np.r_[position, rotation.as_quat(), opening])
        return result


class AbsoluteUprightActions(UprightActions):
    """Opt-in fixed-home absolute goal7, bounded from own dispatched reference.

    A predicted row is a destination, never an increment. Chunk-local references
    advance only toward that destination; subsequent calls re-anchor to target/side.
    The executor independently governs actual dispatched FK and hardware limits.
    """
    def __init__(self, action_contract, closed_opening_m, open_opening_m, side="left",
                 execute_steps=25, gripper_action_curve=None):
        import json
        import math
        from pathlib import Path
        import numpy as np

        raw = Path(action_contract).read_bytes()
        contract = json.loads(raw)
        if not isinstance(contract, dict) or contract.get("schema") != "tacpilot.absolute-cartesian-goals.v1":
            raise ValueError("Unsupported absolute Cartesian goal contract")

        def number(value):
            try:
                return type(value) in (int, float) and math.isfinite(value)
            except OverflowError:
                return False

        for key, width in (("position_anchor_m", 3), ("reference_xyzw", 4)):
            value = contract.get(key)
            if not isinstance(value, list) or len(value) != width or not all(number(v) for v in value):
                raise ValueError(f"Invalid absolute goal {key}")
        if not math.isclose(np.linalg.norm(contract["reference_xyzw"]), 1., abs_tol=1e-6, rel_tol=0):
            raise ValueError("Absolute goal reference must be a unit quaternion")
        for key in ("position_scale_m", "rotation_scale_rad",
                    "maximum_translation_step_m", "maximum_rotation_step_rad"):
            value = contract.get(key)
            if not number(value) or value <= 0:
                raise ValueError(f"Invalid absolute goal {key}")
        if not math.isclose(contract["rotation_scale_rad"], math.pi, abs_tol=1e-12, rel_tol=0):
            raise ValueError("Absolute goal rotation scale must be pi")
        # Never permit a larger physical step than the original normalized delta
        # interface. Tighter task-specific caps belong to the bound contract file.
        if contract["maximum_translation_step_m"] > .02 or contract["maximum_rotation_step_rad"] > .1:
            raise ValueError("Absolute goal step exceeds original physical action bounds")
        super().__init__(closed_opening_m, open_opening_m, side, execute_steps,
                         gripper_action_curve=gripper_action_curve)
        self.contract = contract
        self.metadata.update(action_semantics=contract["schema"], action_contract=contract)

    def reset(self, context):
        import math
        super().reset(context)
        execution = context.execution
        capability = execution.get("cartesian_progress") if isinstance(execution, dict) else None
        if (not isinstance(capability, dict) or capability.get("side") != self.side
                or capability.get("feedback_pacing") is not True):
            raise ValueError("Absolute goals require a same-side feedback-paced Cartesian execution governor")
        for key in ("maximum_translation_step_m", "maximum_rotation_step_rad"):
            value = capability.get(key)
            try:
                valid = (type(value) in (int, float) and math.isfinite(value)
                         and 0 < value <= self.contract[key])
            except OverflowError:
                valid = False
            if not valid:
                raise ValueError(f"Cartesian execution governor exceeds or lacks absolute goal bound: {key}")

    def decode(self, output, observation):
        import numpy as np
        from scipy.spatial.transform import Rotation

        actions = np.asarray(output)
        if (actions.ndim != 2 or actions.shape[1] != 7 or len(actions) < self.count
                or actions.dtype.kind not in "fiu" or not np.isfinite(actions).all()):
            raise ValueError("Expected a finite H x 7 absolute goal prediction")
        actions = np.clip(actions[:self.count], -1, 1)
        anchor = np.asarray(latest(observation, f"target/{self.side}"), dtype=float)
        if (anchor.shape != (8,) or not np.isfinite(anchor).all()
                or not np.isclose(np.linalg.norm(anchor[3:7]), 1., atol=1e-6, rtol=0)):
            raise ValueError("Invalid dispatched Cartesian anchor")
        position, rotation = anchor[:3].copy(), Rotation.from_quat(anchor[3:7])
        contract = self.contract
        reference = Rotation.from_quat(contract["reference_xyzw"])
        result = []
        for row in actions:
            goal_position = np.asarray(contract["position_anchor_m"]) + row[:3] * contract["position_scale_m"]
            goal_rotation = reference * Rotation.from_rotvec(row[3:6] * contract["rotation_scale_rad"])
            dp = goal_position - position
            dr = (goal_rotation * rotation.inv()).as_rotvec()
            dp *= min(1., contract["maximum_translation_step_m"] / max(np.linalg.norm(dp), 1e-12))
            dr *= min(1., contract["maximum_rotation_step_rad"] / max(np.linalg.norm(dr), 1e-12))
            position = position + dp
            rotation = Rotation.from_rotvec(dr) * rotation
            opening = self.closed + (1-row[6])/2 * (self.open-self.closed)
            if self.gripper_curve is not None:
                curve = np.asarray(self.gripper_curve)
                opening = float(np.interp(row[6], curve[:, 0], curve[:, 1]))
            result.append(np.r_[position, rotation.as_quat(), opening])
        return result
