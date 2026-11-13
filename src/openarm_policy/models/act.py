"""Optional strict ACT state8/action7/H100 backend for released TacPilot assets.

Not a universal LeRobot loader: other checkpoint/normalizer formats are separate
predictor plugins. No downloads or optional imports at module import time.
Adapted TacPilot portions: Copyright (c) 2026 TacPilot contributors, MIT;
see THIRD_PARTY.md for the retained permission notice.
"""
import json
from pathlib import Path
import subprocess


class TacPilotACT:
    def __init__(self, checkpoint, device="cpu", state_contract=None):
        self.checkpoint, self.device = Path(checkpoint), device
        from .tacpilot import validate_state_contract
        self.state_contract = None if state_contract is None else validate_state_contract(state_contract)
        self.policy = None

    def reset(self, context):
        root = self.checkpoint
        if not root.is_dir():
            raise ValueError("Checkpoint must be a complete local model directory")
        adapter = root / "observation-adapter.json"
        self.adapter = json.loads(adapter.read_text()) if adapter.exists() else None
        execution = root / 'execution-adapter.json'
        if self.state_contract is not None:
            if self.adapter != self.state_contract['observation_adapter']:
                raise ValueError('Checkpoint observation adapter differs from the model state contract')
            if execution.exists():
                raise ValueError('TCP state8 contract cannot load execution state10')
        import torch
        from safetensors.torch import load_file
        from lerobot.policies.act.configuration_act import ACTConfig
        from lerobot.policies.act.modeling_act import ACTPolicy
        weights = root / "model.safetensors"
        cfg = ACTConfig.from_pretrained(root, local_files_only=True)
        self.execution_adapter = json.loads(execution.read_text()) if execution.exists() else None
        self.state_width = 8
        if self.execution_adapter is not None:
            expected = {'schema': 'tacpilot.execution-state.v1', 'base_state_width': 8, 'state_width': 10,
                        'features': ['waiting', 'joint_tracking_error_fraction'], 'tracking_scale_rad': .15}
            if self.execution_adapter != expected:raise ValueError('Unsupported execution-state adapter')
            self.state_width = 10
        # External state remains state8; rotation6d is checkpoint-bound internal
        # preprocessing, not a change to measured robot-state/action semantics.
        self.feature_width = self.relative_state([[0.] * self.state_width]).shape[-1]
        if (cfg.chunk_size != 100 or cfg.robot_state_feature.shape != (self.feature_width,)
                or cfg.action_feature.shape != (7,) or set(cfg.image_features) != {
                    "observation.images.image", "observation.images.image2"}):
            raise ValueError("Unsupported ACT checkpoint: expected audited state8/action7/H100/two-camera format")
        cfg.device, cfg.pretrained_backbone_weights = self.device, None
        self.policy = ACTPolicy(cfg).to(self.device)
        self.policy.load_state_dict(load_file(str(weights)), strict=True)
        self.policy.eval()
        self.stats = {k: v.to(self.device) for k, v in load_file(str(
            root / "policy_preprocessor_step_3_normalizer_processor.safetensors")).items()}
        post = load_file(str(root / "policy_postprocessor_step_0_unnormalizer_processor.safetensors"))
        for key, width in (("observation.state", self.feature_width), ("action", 7),
                           ("observation.images.image", None), ("observation.images.image2", None)):
            for field in ("mean", "std"):
                value = self.stats[key + "." + field]
                if not torch.isfinite(value).all() or (width and tuple(value.shape) != (width,)):
                    raise ValueError("Invalid ACT normalization shape/values")
            if (self.stats[key + ".std"] < 0).any():
                raise ValueError("Negative ACT normalization scale")
        for field in ("mean", "std"):
            torch.testing.assert_close(self.stats["action." + field].cpu(), post["action." + field])
        self.metadata = {"family": "tacpilot_act", "checkpoint": str(root), "device": self.device,
                          "state_width": self.state_width, "feature_width": self.feature_width,
                         "execution_adapter": self.execution_adapter}
        if self.state_contract is not None:
            self.metadata['state_contract'] = self.state_contract
        try:
            revision = subprocess.run(['git', '-C', str(root), 'rev-parse', 'HEAD'],
                                      capture_output=True, text=True)
            self.metadata['git_revision'] = revision.stdout.strip() if revision.returncode == 0 else None
        except OSError:
            self.metadata['git_revision'] = None
        for name in ('action', 'image'):
            path = root / (name + '-contract.json')
            if path.is_file():
                self.metadata[name + '_contract'] = json.loads(path.read_text())

    def relative_state(self, state):
        import numpy as np
        state = np.array(state, dtype=np.float32, copy=True)
        spec = self.adapter
        if spec is None:
            return state
        rotation6d = {"schema": "tacpilot.rotation6d-state.v1", "input_width": 8,
                      "feature_width": 11, "rotation_columns": [0, 1], "flatten_order": "column-major"}
        if spec.get("schema") == rotation6d["schema"]:
            if spec != rotation6d or state.shape[-1] != 8 or not np.isfinite(state).all():
                raise ValueError("Unsupported/invalid rotation6d observation adapter")
            from scipy.spatial.transform import Rotation
            matrix = Rotation.from_rotvec(state[..., 3:6].astype(float).reshape(-1, 3)).as_matrix()
            columns = matrix[..., :, :2].swapaxes(-1, -2).reshape(*state.shape[:-1], 6)
            return np.concatenate([state[..., :3], columns, state[..., 6:8]], axis=-1).astype(np.float32)
        reference = np.asarray(spec.get("reference_xyzw"), dtype=float)
        anchor = np.asarray(spec.get("rotation_anchor"), dtype=float)
        if (spec.get("schema") != "tacpilot.fixed-relative-rotation-state.v1"
                or reference.shape != (4,) or anchor.shape != (3,)
                or not np.isfinite(reference).all() or not np.isfinite(anchor).all()
                or not np.isclose(np.linalg.norm(reference), 1., atol=1e-6)):
            raise ValueError("Unsupported/invalid ACT observation adapter")
        # q_relative = inverse(q_reference) * q_state. Keep the released chart's
        # shortest-arc quaternion convention, including its near-zero handling.
        vector = state[..., 3:6].astype(float)
        angle = np.linalg.norm(vector, axis=-1, keepdims=True)
        scale = np.divide(np.sin(angle/2), angle, out=np.full_like(angle, .5), where=angle > 1e-12)
        xyz, w = vector*scale, np.cos(angle/2)
        rv, rw = reference[:3], reference[3]
        relative_xyz = rw*xyz - w*rv - np.cross(rv, xyz)
        relative_w = rw*w + np.sum(rv*xyz, axis=-1, keepdims=True)
        sign = np.where(relative_w < 0, -1., 1.)
        relative_xyz, relative_w = relative_xyz*sign, relative_w*sign
        norm = np.linalg.norm(relative_xyz, axis=-1, keepdims=True)
        turn = 2*np.arctan2(norm, relative_w)
        factor = np.divide(turn, norm, out=np.full_like(norm, 2.), where=norm > 1e-12)
        state[..., 3:6] = relative_xyz*factor + anchor
        return state

    def predict(self, inputs):
        import numpy as np
        import torch
        if self.policy is None:
            raise RuntimeError("ACT backend must be reset before prediction")
        state = np.asarray(inputs["state"])
        if state.shape != (1, self.state_width) or not np.isfinite(state).all():
            raise ValueError("Expected one finite state8 batch")
        if self.state_width == 10 and (((state[:, 8:] < 0) | (state[:, 8:] > 1)).any()
                                       or not np.isin(state[:, 8], [0, 1]).all()):
            raise ValueError('Invalid execution feedback features')
        batch = {"observation.state": torch.as_tensor(self.relative_state(state), device=self.device).float()}
        for source, key in (("image", "observation.images.image"), ("wrist", "observation.images.image2")):
            image = np.asarray(inputs[source])
            if image.shape != (1, 256, 256, 3) or image.dtype != np.uint8:
                raise ValueError("Expected RGB uint8 batch1 x 256 x 256 x 3")
            batch[key] = torch.as_tensor(image, device=self.device).float().permute(0, 3, 1, 2) / 255
        for key in batch:
            batch[key] = (batch[key] - self.stats[key + ".mean"]) / (self.stats[key + ".std"] + 1e-8)
        with torch.inference_mode():
            output = self.policy.predict_action_chunk(batch)
            result = (output * (self.stats["action.std"] + 1e-8) + self.stats["action.mean"]).cpu().numpy()[0]
        if result.shape != (100, 7) or not np.isfinite(result).all():
            raise ValueError("Invalid ACT output")
        return result

    def close(self):
        self.policy = None
