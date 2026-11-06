"""Optional strict ACT state8/action7/H100 backend for released TacPilot assets.

Not a universal LeRobot loader: other checkpoint/normalizer formats are separate
predictor plugins. No downloads or optional imports at module import time.
Adapted TacPilot portions: Copyright (c) 2026 TacPilot contributors, MIT;
see THIRD_PARTY.md for the retained permission notice.
"""
import hashlib
import json
from pathlib import Path


class TacPilotACT:
    def __init__(self, checkpoint, device="cpu", expected_model_sha256=None):
        self.checkpoint, self.device = Path(checkpoint), device
        self.expected_hash = expected_model_sha256
        self.policy = None

    def reset(self, context):
        import torch
        from safetensors.torch import load_file
        from lerobot.policies.act.configuration_act import ACTConfig
        from lerobot.policies.act.modeling_act import ACTPolicy
        root = self.checkpoint
        if not root.is_dir():
            raise ValueError("Checkpoint must be a complete local model directory")
        weights = root / "model.safetensors"
        with weights.open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        if self.expected_hash is not None and digest != self.expected_hash:
            raise ValueError("ACT model SHA256 mismatch")
        cfg = ACTConfig.from_pretrained(root, local_files_only=True)
        if (cfg.chunk_size != 100 or cfg.robot_state_feature.shape != (8,)
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
        for key, width in (("observation.state", 8), ("action", 7),
                           ("observation.images.image", None), ("observation.images.image2", None)):
            for field in ("mean", "std"):
                value = self.stats[key + "." + field]
                if not torch.isfinite(value).all() or (width and tuple(value.shape) != (width,)):
                    raise ValueError("Invalid ACT normalization shape/values")
            if (self.stats[key + ".std"] < 0).any():
                raise ValueError("Negative ACT normalization scale")
        for field in ("mean", "std"):
            torch.testing.assert_close(self.stats["action." + field].cpu(), post["action." + field])
        adapter = root / "observation-adapter.json"
        self.adapter = json.loads(adapter.read_text()) if adapter.exists() else None
        if self.adapter is not None:
            # Validate checkpoint-bound constants before any robot startup.
            self.relative_state([[0.] * 8])
        self.metadata = {"family": "tacpilot_act", "model_sha256": digest, "device": self.device,
                         "files_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                          for p in root.iterdir() if p.suffix in (".json", ".safetensors")
                                          and p.name != "model.safetensors"}}

    def relative_state(self, state):
        import numpy as np
        state = np.array(state, dtype=np.float32, copy=True)
        spec = self.adapter
        if spec is None:
            return state
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
        if state.shape != (1, 8) or not np.isfinite(state).all():
            raise ValueError("Expected one finite state8 batch")
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
