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
    def __init__(self, overview, wrist, side="left", size=256, crops=None):
        if side not in ("left", "right") or type(size) is not int or size < 1:
            raise ValueError("Invalid side or image size")
        if overview == wrist:
            raise ValueError("Two distinct camera streams required")
        self.overview, self.wrist, self.side = overview, wrist, side
        self.size, self.crops = size, dict(crops or {})
        if set(self.crops) - {overview, wrist}:
            raise ValueError("Crop names must match selected camera streams")
        self.metadata = dict(overview=overview, wrist=wrist, side=side, size=size,
                             crops=self.crops, image_transform="BGR_to_RGB_area_resize")

    def reset(self, context):
        specs = {s.name: s for s in context.streams}
        for name in (self.overview, self.wrist):
            if name not in specs or specs[name].encoding != "bgr8" or not specs[name].required:
                raise ValueError(f"Required BGR camera not declared: {name}")
        tcp = specs.get(f"tcp/{self.side}")
        if tcp is None or tcp.shape != (8,) or not tcp.required:
            raise ValueError("Required calibrated measured TCP/opening stream missing")

    def image(self, observation, name):
        import cv2
        import numpy as np
        image = np.asarray(latest(observation, name))
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("Expected HWC uint8 BGR camera image")
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
