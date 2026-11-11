"""Current ACT feature/image contracts, synthetic pixels only; no physical devices."""
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from openarm_policy.contracts import ActionSpec, Context, Observation, SensorSample
from openarm_policy.models import ModelPolicy


NUMERIC = all(importlib.util.find_spec(n) for n in ("numpy", "scipy", "cv2", "PIL"))
ROTATION6D = {"schema": "tacpilot.rotation6d-state.v1", "input_width": 8,
              "feature_width": 11, "rotation_columns": [0, 1], "flatten_order": "column-major"}


class BindingTests(unittest.TestCase):
    def test_image_mismatch_or_missing_contract_rejects_before_hardware(self):
        context = Context(ActionSpec(("a",), (-1.,), (1.,)), (), 20)
        for model_spec, encoder_spec in (({"version": "a"}, {"version": "b"}), ({"version": "a"}, None), (None, {"version": "a"})):
            model = Mock(metadata={} if model_spec is None else {"image_contract": model_spec})
            encoder = Mock(metadata={} if encoder_spec is None else {"image_contract": encoder_spec})
            policy = ModelPolicy({}, {}, {})
            with patch("openarm_policy.models.policy.component", side_effect=[model, encoder, Mock()]):
                with self.assertRaisesRegex(ValueError, "image"):
                    policy.reset(context)
            model.close.assert_called_once()
            encoder.close.assert_called_once()

    def test_matching_image_binding_accepts(self):
        context = Context(ActionSpec(("a",), (-1.,), (1.,)), (), 20)
        policy = ModelPolicy({}, {}, {})
        with patch("openarm_policy.models.policy.component", side_effect=[
                Mock(metadata={"image_contract": {"version": "a"}}),
                Mock(metadata={"image_contract": {"version": "a"}}), Mock(metadata={})]):
            policy.reset(context)
        policy.close()


@unittest.skipUnless(NUMERIC, "optional numeric/image dependencies unavailable")
class CurrentUprightTests(unittest.TestCase):
    def test_rotation6d_columns_and_external_state_width(self):
        import numpy as np
        from scipy.spatial.transform import Rotation
        from openarm_policy.models.act import TacPilotACT
        model = TacPilotACT("unused")
        model.adapter = dict(ROTATION6D)
        state = np.random.default_rng(4).normal(size=(20, 8)).astype(np.float32)
        original = state.copy()
        features = model.relative_state(state)
        matrix = Rotation.from_rotvec(state[:, 3:6]).as_matrix()
        expected = np.concatenate([state[:, :3], matrix[:, :, 0], matrix[:, :, 1], state[:, 6:]], axis=1).astype(np.float32)
        np.testing.assert_array_equal(features, expected)
        np.testing.assert_array_equal(state, original)
        self.assertEqual(features.shape, (20, 11))
        for bad in (np.zeros((1, 10)), np.full((1, 8), np.nan)):
            with self.assertRaises(ValueError): model.relative_state(bad)
        model.adapter = {**ROTATION6D, "flatten_order": "row-major"}
        with self.assertRaises(ValueError): model.relative_state(state)

    def fixture(self, root):
        calibration = {"schema": "openarm.camera-baseline.v1", "version": "synthetic-test",
            "camera": {"image_size": [1280, 720], "image_geometry": "native_no_transform",
                "distortion_model": "opencv_5", "matrix": [[900., 0., 640.], [0., 900., 360.], [0., 0., 1.]],
                "distortion": [.08, -.02, .001, .002, 0.]}}
        path = root/"calibration.json"
        path.write_text(json.dumps(calibration))
        contract = {"schema": "tacpilot.policy-images.v1", "channels": "RGB", "dtype": "uint8",
            "head_geometry": "ideal_pinhole_same_K_undistorted", "head_render_size_wh": [1280, 720],
            "output_size_wh": [256, 256], "head_resize": "Pillow_bilinear_full_frame_no_crop",
            "real_head_preprocessing": "undistort native RGB with frozen K/distortion and new_K=K, then same resize",
            "wrist_render_size_wh": [256, 256],
            "head": {"version": "synthetic-test"}}
        spec = root/"image-contract.json"
        spec.write_text(json.dumps(contract))
        return spec, path, calibration

    def test_native_head_undistortion_and_bilinear_rgb_exact(self):
        import cv2
        import numpy as np
        from PIL import Image
        from openarm_policy.models.tacpilot import UprightObservation
        with tempfile.TemporaryDirectory() as temp:
            spec, path, calibration = self.fixture(Path(temp))
            encoder = UprightObservation("camera/head", "camera/wrist", image_contract=spec, head_calibration=path)
            bgr = np.random.default_rng(5).integers(0, 256, (720, 1280, 3), dtype=np.uint8)
            obs = Observation(1, 2., {"camera/head": (SensorSample(1, 1.9, bgr),)})
            camera = calibration["camera"]
            k, d = np.array(camera["matrix"]), np.array(camera["distortion"])
            rgb = cv2.undistort(bgr[..., ::-1].copy(), k, d, None, k)
            expected = np.asarray(Image.fromarray(rgb).resize((256, 256), Image.Resampling.BILINEAR))
            np.testing.assert_array_equal(encoder.image(obs, "camera/head")[0], expected)
            self.assertEqual(encoder.metadata["image_contract"], json.loads(spec.read_text()))
            bad = replace(obs, streams={"camera/head": (SensorSample(1, 1.9, bgr[:256, :256]),)})
            with self.assertRaisesRegex(ValueError, "native"):
                encoder.image(bad, "camera/head")
            # Wrong RGB/BGR order must not happen to pass on a gray fixture.
            self.assertFalse(np.array_equal(expected, expected[..., ::-1]))

    def test_calibration_formatting_is_ignored_but_crop_and_unknown_transform_rejected(self):
        from openarm_policy.models.tacpilot import UprightObservation
        with tempfile.TemporaryDirectory() as temp:
            spec, path, _ = self.fixture(Path(temp))
            kwargs = dict(image_contract=spec, head_calibration=path)
            with self.assertRaises(ValueError):
                UprightObservation("head", "wrist", crops={"head": [0, 0, 20, 20]}, **kwargs)
            with self.assertRaises(ValueError):
                UprightObservation("head", "wrist", image_contract=spec)
            raw = path.read_text()
            path.write_text(raw+"\n")
            with patch('hashlib.sha256', side_effect=AssertionError('Content hashing is disabled')):
                UprightObservation("head", "wrist", **kwargs)
            path.write_text(raw)
            contract = json.loads(spec.read_text());contract["head_resize"] = "area"
            spec.write_text(json.dumps(contract))
            with self.assertRaisesRegex(ValueError, "Unsupported"):
                UprightObservation("head", "wrist", **kwargs)

    def test_native_wrist_contract_exact_full_frame_and_rejects_previews(self):
        import numpy as np
        from PIL import Image
        from openarm_policy.models.tacpilot import UprightObservation
        with tempfile.TemporaryDirectory() as temp:
            spec, path, _ = self.fixture(Path(temp))
            contract = json.loads(spec.read_text())
            contract.update(wrist_render_size_wh=[1280, 720],
                wrist_resize="Pillow_bilinear_full_frame_no_crop",
                wrist={"source": "pinned_upstream", "asset_revision": "06b4c8ad6d547c3faaa3bf780629ee3e1ac2b0d0"},
                wrist_fovy_jitter_deg=0.)
            spec.write_text(json.dumps(contract))
            encoder = UprightObservation("head", "wrist", image_contract=spec, head_calibration=path)
            bgr = np.random.default_rng(7).integers(0, 256, (720, 1280, 3), dtype=np.uint8)
            obs = Observation(1, 2., {"wrist": (SensorSample(1, 1.9, bgr),)})
            expected = np.asarray(Image.fromarray(bgr[..., ::-1]).resize((256, 256), Image.Resampling.BILINEAR))
            np.testing.assert_array_equal(encoder.image(obs, "wrist")[0], expected)
            bad = replace(obs, streams={"wrist": (SensorSample(1, 1.9, bgr[:256, :256]),)})
            with self.assertRaisesRegex(ValueError, "Native wrist"):
                encoder.image(bad, "wrist")
            for key, value in [("wrist_resize", "crop"), ("wrist", {"source": "identity"}),
                               ("wrist_fovy_jitter_deg", 4.)]:
                wrong = dict(contract, **{key: value});spec.write_text(json.dumps(wrong))
                with self.assertRaisesRegex(ValueError, "Unsupported native wrist"):
                    UprightObservation("head", "wrist", image_contract=spec, head_calibration=path)

    def test_shared_measured_wrist_undistorts_both_sides_with_same_K(self):
        import cv2
        import numpy as np
        from PIL import Image
        from openarm_policy.models.tacpilot import UprightObservation
        with tempfile.TemporaryDirectory() as temp:
            spec,path,_=self.fixture(Path(temp))
            measured=dict(schema='openarm.shared-wrist-intrinsics.v1',image_size=[1280,720],
                matrix=[[800.,0.,650.],[0.,790.,380.],[0.,0.,1.]],distortion=[-.06,.18,0,0,-.14],
                distortion_model='opencv_5',measured_side='left',applies_to=['left','right'],
                right_reuse='operator_authorized_not_independently_calibrated',source_candidate_sha256='a'*64)
            contract=json.loads(spec.read_text());contract.update(wrist_render_size_wh=[1280,720],
                wrist_resize='Pillow_bilinear_full_frame_no_crop',wrist_fovy_jitter_deg=0.,
                wrist=dict(source='pinned_upstream',asset_revision='06b4c8ad6d547c3faaa3bf780629ee3e1ac2b0d0',
                    intrinsics=measured,optical_center_forward_m=.004),
                wrist_geometry='ideal_pinhole_same_K_undistorted',
                real_wrist_preprocessing='undistort native RGB with shared wrist K/distortion and new_K=K, then same resize',
                wrist_calibration='left_measured_right_operator_shared')
            spec.write_text(json.dumps(contract))
            bgr=np.random.default_rng(9).integers(0,256,(720,1280,3),dtype=np.uint8)
            obs=Observation(1,2.,{'wrist':(SensorSample(1,1.9,bgr),)})
            k=np.array(measured['matrix']);d=np.array(measured['distortion'])
            rgb=cv2.undistort(bgr[...,::-1].copy(),k,d,None,k)
            expected=np.asarray(Image.fromarray(rgb).resize((256,256),Image.Resampling.BILINEAR))
            for side in ('left','right'):
                encoder=UprightObservation('head','wrist',side=side,image_contract=spec,head_calibration=path)
                np.testing.assert_array_equal(encoder.image(obs,'wrist')[0],expected)
                self.assertEqual(encoder.metadata['wrist_calibration'],'left_measured_right_operator_shared')
            measured['right_reuse']='independently_calibrated'
            spec.write_text(json.dumps(contract))
            with self.assertRaisesRegex(ValueError,'Unsupported measured wrist'):
                UprightObservation('head','wrist',image_contract=spec,head_calibration=path)
            measured['schema']='openarm.wrist-intrinsics.v1'
            for key in ('right_reuse','measured_side','applies_to'):
                measured.pop(key)
            contract['wrist'].pop('optical_center_forward_m')
            contract['wrist']['positions_m']={'left':[.02,.016,-.106],'right':[.02,-.016,-.106]}
            contract['wrist_calibration']='standard_training_cameras'
            spec.write_text(json.dumps(contract))
            encoder=UprightObservation('head','wrist',image_contract=spec,head_calibration=path)
            np.testing.assert_array_equal(encoder.image(obs,'wrist')[0],expected)


if __name__ == "__main__":
    unittest.main()
