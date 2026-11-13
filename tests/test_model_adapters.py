"""Model composition, released ACT semantics and Cartesian dispatch, no hardware."""
from dataclasses import replace
import importlib.util
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from openarm_policy.contracts import ActionSpec, ActionChunk, Context, Observation, SensorSample, StreamSpec, TCP_POSE
from openarm_policy.models import ModelPolicy


NUMERIC = all(importlib.util.find_spec(name) for name in ("numpy", "scipy", "cv2"))


class Predictor:
    def reset(self, context):
        pass

    def predict(self, inputs):
        return [[inputs]]

    def close(self):
        pass


class Encoder:
    def reset(self, context):
        pass

    def encode(self, observation):
        return .2


class Decoder:
    def reset(self, context):
        pass

    def decode(self, output, observation):
        return output


def tcp_spec():
    names = tuple("left." + n for n in ("x", "y", "z", "qx", "qy", "qz", "qw", "opening"))
    return ActionSpec(names, (-2.,)*3 + (-1.,)*4 + (0.,), (2.,)*3 + (1.,)*4 + (.1,), kind=TCP_POSE)


class CompositionTests(unittest.TestCase):
    def test_state_contract_mismatch_or_missing_rejects_even_at_width8(self):
        from copy import deepcopy
        contract = {'schema': 'tacpilot.tcp-state8.v1',
                    'quaternion_to_rotvec': 'acos_preserve_sign', 'observation_adapter': None}
        wrong = dict(contract, quaternion_to_rotvec='scipy_shortest')
        for encoder_contract in (wrong, None):
            with self.subTest(encoder_contract=encoder_contract):
                model = Mock(metadata={'state_width': 8, 'state_contract': deepcopy(contract)})
                meta = {'state_width': 8}
                if encoder_contract is not None:
                    meta['state_contract'] = encoder_contract
                encoder, decoder = Mock(metadata=meta), Mock(metadata={})
                policy = ModelPolicy({}, {}, {})
                with patch('openarm_policy.models.policy.component', side_effect=[model, encoder, decoder]):
                    with self.assertRaisesRegex(ValueError, 'state contract'):
                        policy.reset(Context(tcp_spec(), (), 20))
                model.close.assert_called_once()

    def test_state_contract_matches_across_http_metadata_wrapper(self):
        contract = {'schema': 'tacpilot.tcp-state8.v1',
                    'quaternion_to_rotvec': 'acos_preserve_sign', 'observation_adapter': None}
        model = Mock(metadata={'model': {'state_width': 8, 'state_contract': contract}})
        encoder = Mock(metadata={'state_width': 8, 'state_contract': contract})
        decoder = Mock(metadata={})
        policy = ModelPolicy({}, {}, {})
        with patch('openarm_policy.models.policy.component', side_effect=[model, encoder, decoder]):
            policy.reset(Context(tcp_spec(), (), 20))
        policy.close()

    def test_state_contract_boundary_is_not_limited_to_act(self):
        contract = {'schema': 'example.joint-state.v1', 'units': 'rad', 'order': ['j1']}
        model = Mock(metadata={'state_width': 1, 'state_contract': contract})
        encoder = Mock(metadata={'state_width': 1, 'state_contract': contract})
        policy = ModelPolicy({}, {}, {})
        with patch('openarm_policy.models.policy.component', side_effect=[model, encoder, Mock(metadata={})]):
            policy.reset(Context(ActionSpec(('j1',), (-1.,), (1.,)), (), 20))
        policy.close()

    def test_declared_no_adapter_rejects_old_checkpoint_before_optional_imports(self):
        import json
        from openarm_policy.models.act import TacPilotACT
        contract = {'schema': 'tacpilot.tcp-state8.v1',
                    'quaternion_to_rotvec': 'acos_preserve_sign', 'observation_adapter': None}
        with tempfile.TemporaryDirectory() as d:
            (Path(d)/'observation-adapter.json').write_text(json.dumps({
                'schema': 'tacpilot.fixed-relative-rotation-state.v1'}))
            with self.assertRaisesRegex(ValueError, 'observation adapter'):
                TacPilotACT(d, state_contract=contract).reset(None)

    def test_passive_import_without_optional_backends(self):
        code = ('import sys; import openarm_policy.models, openarm_policy.cartesian; '
                'assert not set(("torch", "lerobot", "cv2", "pinocchio", "pink")) & set(sys.modules)')
        subprocess.run([sys.executable, "-c", code], check=True)

    def test_composable_factories_and_original_joint_contract(self):
        specs = [{"factory": f"{__name__}:{name}"} for name in ("Predictor", "Encoder", "Decoder")]
        policy = ModelPolicy(*specs, lead_s=.1)
        context = Context(ActionSpec(("left.joint1",), (-1.,), (1.,)), (), 20)
        policy.reset(context)
        obs = Observation(3, 10, {})
        result = policy.infer(obs)
        self.assertEqual(result.steps, ((.2,),))
        self.assertEqual(result.start_at, 10.1)
        result.validate(context.action, obs, 10.11, 1)
        self.assertEqual(result.diagnostics["raw_actions"], [[.2]])
        policy.close()

    def test_component_cleanup_on_reset_failure(self):
        first = Mock()
        second = Mock()
        second.reset.side_effect = ValueError("bad format")
        policy = ModelPolicy({}, {}, {})
        with patch("openarm_policy.models.policy.component", side_effect=[first, second]):
            with self.assertRaises(ValueError):
                policy.reset(Context(ActionSpec(("a",), (-1.,), (1.,)), (), 20))
        first.close.assert_called_once()
        second.close.assert_called_once()

    def test_period_mismatch_rejected_before_loading(self):
        with self.assertRaises(ValueError):
            ModelPolicy({}, {}, {}, period_s=.1).reset(Context(tcp_spec(), (), 20))

    def test_execution_state_width_mismatch_rejects_before_hardware_start(self):
        model = Mock(metadata={'state_width': 8})
        encoder = Mock(metadata={'state_width': 10})
        policy = ModelPolicy({}, {}, {})
        with patch('openarm_policy.models.policy.component', side_effect=[model, encoder, Mock()]):
            with self.assertRaises(ValueError):policy.reset(Context(tcp_spec(), (), 20))
        model.close.assert_called_once()
        encoder.close.assert_called_once()

    def test_tcp_schema_is_not_joint_schema(self):
        spec = tcp_spec()
        obs = Observation(1, 10, {})
        row = (0., 0., .5, 0., 0., 0., 1., .04)
        chunk = ActionChunk(1, 10, .05, (row,), spec.names, kind=TCP_POSE)
        chunk.validate(spec, obs, 10.01, 1)
        with self.assertRaises(ValueError):
            replace(chunk, kind="joint_position_rad").validate(spec, obs, 10.01, 1)
        for bad in (row[:6] + (0., .04), row[:7] + (.2,)):
            with self.assertRaises(ValueError):
                spec.validate(bad)


@unittest.skipUnless(NUMERIC, "install Cartesian/video numeric dependencies")
class UprightTests(unittest.TestCase):
    def setUp(self):
        import numpy as np
        self.np = np
        specs = (StreamSpec("camera/overview", "bgr8", (4, 6, 3), "uint8", "overview"),
                 StreamSpec("camera/wrist", "bgr8", (4, 6, 3), "uint8", "wrist"),
                 StreamSpec("tcp/left", TCP_POSE, (8,), "m,xyzw,m", "task"),
                 StreamSpec("target/left", TCP_POSE, (8,), "m,xyzw,m", "task"))
        self.context = Context(tcp_spec(), specs, 20)
        image = np.zeros((4, 6, 3), dtype=np.uint8)
        image[:] = [10, 20, 30]
        self.obs = Observation(2, 10, {name: (SensorSample(1, 9.9, value),) for name, value in (
            ("camera/overview", image), ("camera/wrist", image),
            ("tcp/left", [0., 0., .5, 0., 0., 0., 1., .06]),
            ("target/left", [.1, 0., .5, 0., 0., 0., 1., .06]))})

    def test_rgb_state_and_physical_finger_gap(self):
        from openarm_policy.models.tacpilot import UprightObservation
        encoder = UprightObservation("camera/overview", "camera/wrist")
        encoder.reset(self.context)
        inputs = encoder.encode(self.obs)
        self.assertEqual(inputs["image"].shape, (1, 256, 256, 3))
        self.np.testing.assert_array_equal(inputs["image"][0, 0, 0], [30, 20, 10])
        self.np.testing.assert_allclose(inputs["state"], [[0, 0, .5, 0, 0, 0, .03, .03]])

    def test_fulltask_quaternion_formula_matches_training_float32_and_sign(self):
        from scipy.spatial.transform import Rotation
        from openarm_policy.models.tacpilot import tcp_state8
        np = self.np
        q = Rotation.from_rotvec([[np.deg2rad(t), 0, 0]
                                 for t in [0, .001, 179.999, 180., 180.001, 359.999]]).as_quat()
        q = np.concatenate([q, -q])
        tcp = np.c_[np.tile([.25, .2, 1.04], (len(q), 1)), q, np.full(len(q), .06)]
        raw = np.c_[tcp[:, :7], np.repeat(tcp[:, 7:8]/2, 2, axis=1)].astype(np.float32)
        # Exact frozen geometry.pose_state formula on the whole training batch.
        w = np.clip(raw[..., 6], -1, 1)
        den = np.sqrt(1 - w*w)
        scale = np.divide(2*np.arccos(w), den, out=np.zeros_like(w), where=den > 1e-8)
        expected = np.concatenate([raw[:, :3], raw[:, 3:6]*scale[:, None], raw[:, 7:]], axis=-1)
        actual = np.stack([tcp_state8(row, 'acos_preserve_sign') for row in tcp])
        np.testing.assert_array_equal(actual, expected)
        self.assertGreater(np.linalg.norm(actual[4, 3:6] - tcp_state8(tcp[4])[3:6]), 6.)
        with self.assertRaises(ValueError):tcp_state8(tcp[0], 'guess')

    def test_fulltask_encoder_uses_selected_state_rule_and_keeps_legacy_default(self):
        from scipy.spatial.transform import Rotation
        from openarm_policy.models.tacpilot import UprightObservation, tcp_state8
        contract = {'schema': 'tacpilot.tcp-state8.v1',
                    'quaternion_to_rotvec': 'acos_preserve_sign', 'observation_adapter': None}
        streams = dict(self.obs.streams)
        tcp = [.25, .2, 1.04, *Rotation.from_rotvec([3.15, 0, 0]).as_quat(), .06]
        streams['tcp/left'] = (SensorSample(1, 9.9, tcp),)
        obs = replace(self.obs, streams=streams)
        encoder = UprightObservation('camera/overview', 'camera/wrist', state_contract=contract)
        encoder.reset(self.context)
        self.np.testing.assert_array_equal(encoder.encode(obs)['state'][0], tcp_state8(tcp, 'acos_preserve_sign'))
        self.assertEqual(encoder.metadata['state_contract'], contract)
        legacy = UprightObservation('camera/overview', 'camera/wrist')
        self.np.testing.assert_array_equal(legacy.encode(obs)['state'][0], tcp_state8(tcp))
        self.assertNotIn('state_contract', legacy.metadata)
        with self.assertRaises(ValueError):
            UprightObservation('camera/overview', 'camera/wrist', execution=True, state_contract=contract)

    def test_execution_encoder_requires_live_feedback_and_explicit_state10_opt_in(self):
        from openarm_policy.models.tacpilot import UprightObservation
        encoder = UprightObservation('camera/overview', 'camera/wrist', execution=True)
        with self.assertRaises(ValueError):encoder.reset(self.context)
        context = replace(self.context, streams=self.context.streams+(
            StreamSpec('execution/left', 'waiting_joint_error', (2,), 'bool,rad', 'motor'),))
        encoder.reset(context)
        streams = dict(self.obs.streams)
        streams['execution/left'] = (SensorSample(1, 9.9, [1., .075]),)
        result = encoder.encode(replace(self.obs, streams=streams))
        self.assertEqual(result['state'].shape, (1, 10))
        self.np.testing.assert_allclose(result['state'][0, 8:], [1., .5])
        streams['execution/left'] = (SensorSample(1, 9.9, [1., .3]),)
        with self.assertRaises(ValueError):encoder.encode(replace(self.obs, streams=streams))

    def test_missing_optional_camera_bad_dtype_crop_and_future_fail(self):
        from openarm_policy.models.tacpilot import UprightObservation
        encoder = UprightObservation("camera/overview", "camera/wrist", crops={"camera/overview": [0, 0, 20, 20]})
        encoder.reset(self.context)
        with self.assertRaises(ValueError): encoder.encode(self.obs)
        bad = replace(self.context, streams=(replace(self.context.streams[0], required=False), *self.context.streams[1:]))
        with self.assertRaises(ValueError): encoder.reset(bad)
        encoder = UprightObservation("camera/overview", "camera/wrist")
        streams = dict(self.obs.streams)
        streams["tcp/left"] = (replace(streams["tcp/left"][-1], received_at=11),)
        with self.assertRaises(ValueError): encoder.encode(replace(self.obs, streams=streams))

    def test_incremental_actions_use_dispatched_anchor_not_measured(self):
        from openarm_policy.models.tacpilot import UprightActions
        decoder = UprightActions(.01, .09, execute_steps=2)
        decoder.reset(self.context)
        actions = self.np.zeros((100, 7))
        actions[:, 0] = .5
        actions[0, 6], actions[1, 6] = 1, -1
        result = decoder.decode(actions, self.obs)
        self.np.testing.assert_allclose([r[0] for r in result], [.11, .12])
        self.np.testing.assert_allclose([r[7] for r in result], [.01, .09])
        # The same observation is deterministic; an unexecuted prior prediction
        # cannot advance the anchor in a hidden recurrent decoder.
        self.np.testing.assert_allclose(result, decoder.decode(actions, self.obs))

    def test_task_frame_left_rotation_and_normalized_saturation(self):
        from scipy.spatial.transform import Rotation
        from openarm_policy.models.tacpilot import UprightActions
        streams = dict(self.obs.streams)
        q = Rotation.from_rotvec([.4, .2, -.1]).as_quat()
        streams["target/left"] = (SensorSample(1, 9.9, [0, 0, .5, *q, .06]),)
        decoder = UprightActions(.01, .09, execute_steps=1)
        decoder.reset(self.context)
        actions = self.np.zeros((100, 7)); actions[0, 3:6] = [2, .2, .3]
        row = decoder.decode(actions, replace(self.obs, streams=streams))[0]
        expected = Rotation.from_rotvec([.1, .02, .03]) * Rotation.from_quat(q)
        self.np.testing.assert_allclose(Rotation.from_quat(row[3:7]).as_matrix(), expected.as_matrix())

    def test_explicit_nonlinear_gripper_action_curve_is_not_motor_angle(self):
        from openarm_policy.models.tacpilot import UprightActions
        decoder = UprightActions(.01, .09, execute_steps=1,
                                  gripper_action_curve=[[-1, .09], [0, .07], [1, .01]])
        decoder.reset(self.context)
        result = decoder.decode(self.np.zeros((100, 7)), self.obs)
        self.assertAlmostEqual(result[0][7], .07)
        self.assertEqual(decoder.metadata["gripper_mapping"], "action_aperture_curve")
        with self.assertRaises(ValueError):
            UprightActions(.01, .09, gripper_action_curve=[[-1, .09], [0, .001], [1, .01]])

    def test_action_shapes_calibration_and_wrong_contract_rejected(self):
        from openarm_policy.models.tacpilot import UprightActions
        decoder = UprightActions(.01, .09)
        decoder.reset(self.context)
        for value in (self.np.zeros((24, 7)), self.np.zeros((100, 8)), self.np.full((100, 7), self.np.nan)):
            with self.assertRaises(ValueError): decoder.decode(value, self.obs)
        with self.assertRaises(ValueError): UprightActions(.09, .01)
        with self.assertRaises(ValueError): UprightActions(0, .2).reset(self.context)

    def test_checkpoint_orientation_chart_matches_original_quaternion_formula(self):
        from scipy.spatial.transform import Rotation
        from openarm_policy.models.act import TacPilotACT
        rng = self.np.random.default_rng(12)
        model = TacPilotACT("unused")
        reference = Rotation.from_rotvec([.3, -.4, .8]).as_quat()
        anchor = self.np.array([2.9, -.2, -.1])
        model.adapter = {"schema": "tacpilot.fixed-relative-rotation-state.v1",
                         "reference_xyzw": reference.tolist(), "rotation_anchor": anchor.tolist()}
        states = rng.normal(size=(100, 8)).astype("float32")
        q = Rotation.from_rotvec(states[:, 3:6].astype(float)).as_quat()
        xyz, w = q[:, :3], q[:, 3:]
        rv, rw = reference[:3], reference[3]
        relative_xyz = rw*xyz - w*rv - self.np.cross(rv, xyz)
        relative_w = rw*w + self.np.sum(rv*xyz, axis=-1, keepdims=True)
        sign = self.np.where(relative_w < 0, -1., 1.)
        expected = Rotation.from_quat(self.np.c_[relative_xyz*sign, relative_w*sign]).as_rotvec() + anchor
        self.np.testing.assert_allclose(model.relative_state(states)[:, 3:6], expected, atol=3e-7)

    def test_gripper_both_signs_and_out_of_calibration(self):
        from openarm_policy.cartesian import GripperCalibration
        for angles in ([0, .5, 1], [0, -.5, -1]):
            grip = GripperCalibration(angles, [0, .03, .08])
            self.assertAlmostEqual(grip.aperture(angles[1]), .03)
            self.assertAlmostEqual(grip.angle(.03), angles[1])
            with self.assertRaises(ValueError): grip.angle(.09)
            with self.assertRaises(ValueError): grip.aperture(2.)
        for angles, gaps in (([0, 0], [0, .1]), ([0, 1, 2], [0, .1, .05])):
            with self.assertRaises(ValueError): GripperCalibration(angles, gaps)

    def test_closed_feedback_margin_is_directional_and_never_expands_commands(self):
        from openarm_policy.cartesian import GripperCalibration
        import math
        for sign in (1, -1):
            grip = GripperCalibration([0., sign], [0., .08])
            for measured in (-.0074387732, -.010490578, -math.radians(1)):
                self.assertEqual(grip.feedback_motor(sign*measured), 0.)
                self.assertEqual(grip.aperture(sign*measured), 0.)
            for invalid in (-math.radians(1)-1e-6, float('nan'), float('inf')):
                with self.assertRaises(ValueError): grip.aperture(sign*invalid)
            # No tolerance at the open end, including a very short table.
            with self.assertRaises(ValueError): grip.aperture(sign*1.000001)
            short = GripperCalibration([0., sign*.01], [0., .08])
            with self.assertRaises(ValueError): short.aperture(sign*.010001)
            with self.assertRaises(ValueError): grip.angle(-.000001)
            with self.assertRaises(ValueError): grip.angle(.080001)

    def test_loopback_transport_does_not_follow_redirects_or_use_proxies(self):
        from openarm_policy.models.http import NumpyHTTP
        for url in ("http://example.org", "https://localhost", "http://user:pass@localhost"):
            with self.assertRaises(ValueError): NumpyHTTP(url)
        client = NumpyHTTP("http://127.0.0.1:8765")
        response = Mock()
        opener = Mock()
        opener.open.return_value.__enter__ = Mock(return_value=response)
        opener.open.return_value.__exit__ = Mock(return_value=False)
        response.read.return_value = b'{"ready": true, "model": {"git_revision": "fixture"}}'
        with patch("openarm_policy.models.http.build_opener", return_value=opener):
            client.reset(None)
        response.read.return_value = b'{"actions": [[0, 1]]}'
        self.np.testing.assert_array_equal(client.predict({"state": self.np.zeros((1, 8))}), [[0, 1]])

    def test_model_service_git_revision_mismatch_rejects_before_start(self):
        from openarm_policy.models.http import NumpyHTTP
        response = Mock()
        response.read.return_value = b'{"ready": true, "model": {"git_revision": "wrong"}}'
        opener = Mock()
        opener.open.return_value.__enter__ = Mock(return_value=response)
        opener.open.return_value.__exit__ = Mock(return_value=False)
        client = NumpyHTTP("http://127.0.0.1:8765", expected_identity={"git_revision": "expected"})
        with patch("openarm_policy.models.http.build_opener", return_value=opener):
            with self.assertRaises(ValueError): client.reset(None)

    def camera_response(self):
        import cv2
        image = self.np.full((8, 12, 3), 80, dtype=self.np.uint8)
        ok, encoded = cv2.imencode('.jpg', image)
        self.assertTrue(ok)
        response = Mock()
        response.headers = {"X-Frame-Sequence": "7", "X-Frame-Age-Ms": "10"}
        response.read.return_value = encoded.tobytes()
        opener = Mock()
        opener.open.return_value.__enter__ = Mock(return_value=response)
        opener.open.return_value.__exit__ = Mock(return_value=False)
        return opener, response

    def test_borrowed_camera_preserves_source_age_and_preview_provenance(self):
        from openarm_policy.observations import HTTPVideoObservation
        opener, response = self.camera_response()
        with patch("urllib.request.build_opener", return_value=opener), patch("time.monotonic", return_value=100.):
            source = HTTPVideoObservation("http://127.0.0.1:15003", ["head", "left"])
            samples = source.snapshot()
        self.assertEqual(source.streams[0].shape, (8, 12, 3))
        self.assertEqual(samples["camera/head"].sequence, 7)
        self.assertAlmostEqual(samples["camera/head"].received_at, 99.99)
        self.assertFalse(source.metadata["lossless"])

    def test_borrowed_camera_rejects_stale_invalid_frames_and_urls(self):
        from openarm_policy.observations import HTTPVideoObservation
        for url, names in (("http://example.org", ["head"]), ("http://localhost", ["../head"]),
                           ("http://localhost", ["head", "head"])):
            with self.assertRaises(ValueError): HTTPVideoObservation(url, names)
        opener, response = self.camera_response()
        with patch("urllib.request.build_opener", return_value=opener):
            response.headers["X-Frame-Age-Ms"] = "900"
            with self.assertRaises(RuntimeError): HTTPVideoObservation("http://localhost", ["head"])
            response.headers["X-Frame-Age-Ms"] = "-1"
            with self.assertRaises(RuntimeError): HTTPVideoObservation("http://localhost", ["head"])
            response.headers["X-Frame-Age-Ms"] = "0"
            response.read.return_value = b"not a JPEG"
            with self.assertRaises(ValueError): HTTPVideoObservation("http://localhost", ["head"])


@unittest.skipUnless(NUMERIC, "install Cartesian/video numeric dependencies")
class CartesianDispatchTests(unittest.TestCase):
    def test_preparation_speed_is_separate_from_learned_policy_speed(self):
        from openarm_policy.cartesian import CartesianRobot
        from openarm_policy.preparation import FixedJointPreparation
        from openarm_policy.contracts import Context
        robot = CartesianRobot(self.plant, dict(self.config, preparation_velocity_rad_s=.5))
        prep = FixedJointPreparation('left', [{'name':'ready', 'joints_rad':[0.]*7}], velocity_rad_s=.5)
        prep.bind(robot, Context(robot.spec, robot.streams, 20))
        self.assertEqual(robot.speed, .25)
        self.assertEqual(robot.preparation_speed, .5)
        robot.start()
        robot.begin_preparation()
        robot.submit_preparation((.02, 0, 0, 0, 0, 0, 0, 0), time.monotonic()+1)
        with self.assertRaisesRegex(ValueError, 'velocity'):
            robot.submit_preparation((.05, 0, 0, 0, 0, 0, 0, 0), time.monotonic()+1)
        robot.close()

    def setUp(self):
        import numpy as np
        from openarm_policy.cartesian import CartesianRobot
        from openarm_policy.robots import SimulatedRobot
        self.np = np
        self.plant = SimulatedRobot(("left",))
        self.chain = Mock()
        self.chain.indices = list(range(7))
        self.chain.metadata = {"fixture": True}
        self.chain.forward.return_value = np.eye(4)
        self.config = dict(side="left", calibration_verified=True, chain={}, task_from_base=np.eye(4).tolist(),
                           workspace_m=[[-1, -1, -1], [1, 1, 1]],
                           gripper={"motor_rad": [0, 1.], "opening_m": [0, .08]})
        self.patch_chain = patch("openarm_runtime.cartesian.KinematicChain", return_value=self.chain)
        self.patch_chain.start(); self.addCleanup(self.patch_chain.stop)
        self.robot = CartesianRobot(self.plant, self.config)
        self.solver = Mock()
        self.solver.step.return_value = dict(joints=[0.]*7, position_error_m=0., orientation_error_rad=0., solve_s=0.)
        self.patch_solver = patch("openarm_runtime.cartesian.DifferentialIK", return_value=self.solver)
        self.patch_solver.start(); self.addCleanup(self.patch_solver.stop)

    def test_construction_passive_and_local_ik_at_each_dispatch(self):
        self.assertFalse(self.plant.started)
        self.robot.start()
        snapshot = self.robot.snapshot()
        import math
        self.assertAlmostEqual(self.robot.joint_margin, math.radians(1))
        self.assertIn("tcp/left", snapshot)
        target = (0., 0., 0., 0., 0., 0., 1., .04)
        self.robot.submit(target, time.monotonic()+1)
        self.robot.submit(target, time.monotonic()+1)
        self.assertEqual(self.solver.step.call_count, 2)
        self.assertEqual(len(self.plant.commands), 2)
        self.assertEqual(self.robot.spec.kind, TCP_POSE)
        self.robot.close()
        self.assertFalse(self.plant.started)

    def test_closed_negative_feedback_starts_and_prepares_without_negative_commands(self):
        measured = -.010490578
        self.plant.values = (0.,)*7 + (measured,)
        self.robot.start()
        self.assertEqual(self.robot.reference[7], 0.)
        samples = self.robot.snapshot()
        self.assertEqual(samples['robot/left'].data[7][0], measured)
        self.assertEqual(samples['tcp/left'].data[7], 0.)
        _, actual = self.robot._feedback(samples)
        self.assertEqual(actual[7], measured)
        self.robot.begin_preparation()
        self.robot.submit_preparation((0.,)*8, time.monotonic()+1)
        self.robot.finish_preparation()
        self.plant.values = (0.,)*7 + (measured,)
        self.robot.submit((0., 0., 0., 0., 0., 0., 1., 0.), time.monotonic()+1)
        self.assertTrue(all(command[1][7] >= 0. for command in self.plant.commands))
        self.robot.close()

    def test_closed_margin_does_not_hide_real_gripper_tracking_error(self):
        self.plant.values = (0.,)*7 + (-.010490578,)
        self.robot.start()
        self.robot.reference[7] = .195
        # A clamped feedback position would wrongly pass (.195 < .20).
        # The actual error is .20549 rad and must still reject.
        with self.assertRaisesRegex(RuntimeError, 'commanded lead exceeded'):
            self.robot.submit((0., 0., 0., 0., 0., 0., 1., .195*.08), time.monotonic()+1)
        self.assertFalse(self.plant.commands)
        self.robot.close()

    def test_preparation_preserves_closed_feedback_but_never_plans_negative_goal(self):
        from openarm_policy.preparation import FixedJointPreparation
        self.plant.values = (0.,)*7 + (-.010490578,)
        self.robot.start()
        stage = FixedJointPreparation('left', [{'name': 'ready', 'joints_rad': [0.]*7}])
        stage.bind(self.robot, Context(self.robot.spec, self.robot.streams, 20))
        samples = self.robot.snapshot()
        observation = Observation(1, time.monotonic(), {k: (v,) for k, v in samples.items()})
        plan = stage.begin(observation)
        self.assertEqual(plan['initial_joints_rad'][7], -.010490578)
        self.assertEqual(plan['initial_command_rad'][7], 0.)
        self.assertEqual(stage.measured(observation)[7], -.010490578)
        self.assertGreaterEqual(stage.step(observation).values[7], 0.)
        # Only measured endpoint interpretation has tolerance; an explicitly
        # supplied negative command remains invalid, including handoff goals.
        other = FixedJointPreparation('left', [{'name': 'ready', 'joints_rad': [0.]*7}])
        other.bind(self.robot, Context(self.robot.spec, self.robot.streams, 20))
        with self.assertRaises(ValueError):
            other.begin(observation, commanded_joints=(0.,)*7+(-.01,))
        self.robot.close()

    def test_large_negative_gripper_feedback_still_rejects_start(self):
        import math
        self.plant.values = (0.,)*7 + (-math.radians(1)-1e-6,)
        with self.assertRaisesRegex(ValueError, 'Gripper feedback outside calibrated motor range'):
            self.robot.start()
        self.assertFalse(self.plant.started)
        self.assertFalse(self.plant.commands)

    def test_closed_upper_endpoint_feedback_with_reversed_calibration(self):
        from openarm_policy.cartesian import CartesianRobot
        robot = CartesianRobot(self.plant, dict(self.config,
            gripper={'motor_rad': [0., 1.], 'opening_m': [.08, 0.]}))
        self.plant.values = (0.,)*7 + (1.010490578,)
        robot.start()
        self.assertEqual(robot.reference[7], 1.)
        self.assertAlmostEqual(robot.snapshot()['robot/left'].data[7][0], 1.010490578)
        self.assertEqual(robot.snapshot()['tcp/left'].data[7], 0.)
        robot.submit((0., 0., 0., 0., 0., 0., 1., .08), time.monotonic()+1)
        self.assertLess(robot.reference[7], 1.)
        self.assertLessEqual(self.plant.commands[-1][1][7], 1.)
        robot.close()

    def test_expired_stale_workspace_and_failed_solver_send_nothing(self):
        self.robot.start()
        target = (0., 0., 0., 0., 0., 0., 1., .04)
        with self.assertRaises(TimeoutError): self.robot.submit(target, time.monotonic()-1)
        with self.assertRaises(ValueError): self.robot.submit((2., *target[1:]), time.monotonic()+1)
        self.solver.step.side_effect = RuntimeError("QP failed")
        with self.assertRaises(RuntimeError): self.robot.submit(target, time.monotonic()+1)
        self.assertFalse(self.plant.commands)
        self.robot.close()

    def test_rejected_gripper_command_records_exact_candidate_without_dispatch(self):
        events = []
        self.robot.record = events.append
        self.robot.start()
        self.plant.values = (0.,)*7 + (.3,)
        target = (0., 0., 0., 0., 0., 0., 1., 0.)
        with self.assertRaisesRegex(RuntimeError, 'Cartesian commanded lead exceeded: .*gripper error=0.300000'):
            self.robot.submit(target, time.monotonic()+1)
        event = events[-1]
        self.assertEqual(event['event'], 'cartesian_command_rejected')
        self.assertEqual(event['measured_joints_rad'][7], .3)
        self.assertEqual(event['candidate_joints_rad'][7], 0.)
        self.assertEqual(event['gripper_limit_rad'], .2)
        self.assertFalse(self.plant.commands)
        self.robot.close()

    def test_blocked_closing_holds_safe_reference_and_opening_still_responds(self):
        events = []
        self.robot.record = events.append
        self.robot.start()
        # Frozen last feedback from the reviewed closure failure, not a contact
        # simulation. No motor, force estimate or successful-grasp assumption.
        measured = .22144655527580603
        reference = .040306570424160186
        self.robot.reference[7] = reference
        target = (0., 0., 0., 0., 0., 0., 1., .017479968756121296 * .08)
        for _ in range(100):
            self.plant.values = (0.,)*7 + (measured,)
            self.robot.submit(target, time.monotonic()+1)
            self.assertEqual(self.plant.commands[-1][1][7], reference)
            self.assertTrue(events[-1]['gripper_closure_limited'])
            self.assertLessEqual(abs(self.robot.reference[7]-measured), .2)
        self.assertEqual(self.solver.step.call_count, 100)
        self.assertEqual(len(self.plant.commands), 100)
        # No latch: a new opening goal is handled with the original slew limit.
        self.plant.values = (0.,)*7 + (measured,)
        self.robot.submit((*target[:7], .08), time.monotonic()+1)
        self.assertAlmostEqual(self.robot.reference[7], reference+.025)
        self.assertFalse(events[-1]['gripper_closure_limited'])
        # Once measured feedback catches up, closing can advance again.
        self.plant.values = tuple(self.robot.reference)
        before = self.robot.reference[7]
        self.robot.submit(target, time.monotonic()+1)
        self.assertAlmostEqual(self.robot.reference[7], before-.025)
        self.assertFalse(events[-1]['gripper_closure_limited'])
        self.robot.close()

    def test_closing_limit_handles_reversed_motor_direction(self):
        from openarm_policy.cartesian import CartesianRobot
        robot = CartesianRobot(self.plant, dict(self.config,
            gripper={'motor_rad': [0., 1.], 'opening_m': [.08, 0.]}))
        events = []; robot.record = events.append
        self.plant.values = (0.,)*7 + (.5,)
        robot.start()
        robot.reference[7] = .69
        self.solver.step.return_value['joints'] = [.001] + [0.]*6
        for _ in range(20):
            self.plant.values = (0.,)*7 + (.5,)
            robot.submit((0., 0., 0., 0., 0., 0., 1., 0.), time.monotonic()+1)
            self.assertAlmostEqual(robot.reference[7], .69)
            self.assertEqual(self.plant.commands[-1][1][0], .001)
            self.assertTrue(events[-1]['gripper_closure_limited'])
        self.plant.values = (0.,)*7 + (.5,)
        robot.submit((0., 0., 0., 0., 0., 0., 1., .08), time.monotonic()+1)
        self.assertAlmostEqual(robot.reference[7], .665)
        self.assertFalse(events[-1]['gripper_closure_limited'])
        robot.close()

    def test_closing_limiter_does_not_mask_opening_or_existing_feedback_fault(self):
        self.robot.start()
        target = (0., 0., 0., 0., 0., 0., 1., .08)
        self.robot.reference[7] = .19
        # Opening away from measured feedback is still a hard fault, not contact.
        with self.assertRaisesRegex(RuntimeError, 'commanded lead exceeded'):
            self.robot.submit(target, time.monotonic()+1)
        self.assertFalse(self.plant.commands)
        # Even a closing request cannot disguise an already-unsafe reference.
        self.plant.values = (0.,)*7 + (.5,)
        with self.assertRaisesRegex(RuntimeError, 'commanded lead exceeded'):
            self.robot.submit((*target[:7], 0.), time.monotonic()+1)
        self.assertFalse(self.plant.commands)
        self.robot.close()

    def test_unblocked_closing_keeps_original_slew_and_arm_guard_still_wins(self):
        self.plant.values = (0.,)*7 + (.5,)
        self.robot.start()
        target = (0., 0., 0., 0., 0., 0., 1., 0.)
        self.robot.submit(target, time.monotonic()+1)
        self.assertAlmostEqual(self.robot.reference[7], .475)
        self.robot.reference[7] = .31
        self.plant.values = (0.,)*7 + (.5,)
        self.solver.step.return_value['joints'] = [.2] + [0.]*6
        count = len(self.plant.commands)
        with self.assertRaisesRegex(RuntimeError, 'commanded lead exceeded'):
            self.robot.submit(target, time.monotonic()+1)
        self.assertEqual(len(self.plant.commands), count)
        self.assertEqual(self.robot.reference[7], .31)
        self.robot.close()

    def test_joint_command_margin_reaches_both_solvers_and_preserves_hard_feedback_guard(self):
        from openarm_policy.cartesian import CartesianRobot
        robot = CartesianRobot(self.plant, dict(self.config, joint_command_margin_rad=.02))
        events = []
        robot.record = events.append
        with patch('openarm_runtime.cartesian.DifferentialIK', return_value=self.solver) as constructor:
            robot.start()
            self.assertEqual(constructor.call_args.kwargs['command_margin_rad'], .02)
            robot.begin_preparation()
            robot.finish_preparation()
            self.assertEqual(constructor.call_args.kwargs['command_margin_rad'], .02)
        samples = self.plant.snapshot()
        sample = samples['robot/left']
        state = self.np.asarray(sample.data).copy()
        state[2, 0] = self.plant.spec.upper[2] + .001
        samples['robot/left'] = replace(sample, data=state)
        with self.assertRaisesRegex(ValueError, 'Measured .* outside joint safety range'):
            robot._feedback(samples)
        self.assertEqual(events[-1]['event'], 'joint_feedback_out_of_range')
        self.assertEqual(events[-1]['joint'], self.plant.spec.names[2])
        self.assertEqual(events[-1]['measured_rad'], state[2, 0])
        self.assertFalse(self.plant.commands)
        robot.close()

    def test_progress_holds_reference_and_anchors_inference_to_realized_fk(self):
        from openarm_policy.cartesian import CartesianRobot
        with self.assertRaises(ValueError):
            CartesianRobot(self.plant, dict(self.config, progress={}, tracking_limit_rad=.06))
        robot = CartesianRobot(self.plant, dict(self.config, progress={}))
        robot.start()
        robot.reference[0] = .08  # plant has not caught up
        robot.target = self.np.array([.5, 0, 0, 0, 0, 0, 1, 0])
        robot.submit((.5, 0, 0, 0, 0, 0, 1, 0), time.monotonic()+1)
        self.solver.step.assert_not_called()
        self.assertAlmostEqual(self.plant.commands[-1][1][0], .08)
        self.assertAlmostEqual(robot.snapshot()["target/left"].data[0], 0.)
        self.assertIn("execution/left", robot.snapshot())
        # SimulatedRobot now reports the held reference, permitting resume.
        robot.submit((.5, 0, 0, 0, 0, 0, 1, 0), time.monotonic()+1)
        self.assertAlmostEqual(self.solver.step.call_args.args[1][0, 3], .002)
        robot.close()

    def test_requires_verified_calibration_and_one_arm(self):
        from openarm_policy.cartesian import CartesianRobot
        from openarm_policy.robots import SimulatedRobot
        with self.assertRaises(ValueError): CartesianRobot(self.plant, dict(self.config, calibration_verified=False))
        with self.assertRaises(ValueError): CartesianRobot(SimulatedRobot(), self.config)

    def test_nominal_geometry_needs_separate_opt_in_and_is_never_verified(self):
        from openarm_policy.cartesian import CartesianRobot
        nominal = dict(self.config, calibration_verified=False, calibration_status="nominal_model_not_hardware_calibrated")
        with self.assertRaises(ValueError): CartesianRobot(self.plant, nominal)
        robot = CartesianRobot(self.plant, nominal, allow_nominal=True)
        self.assertFalse(robot.config["calibration_verified"])
        with self.assertRaises(ValueError):
            CartesianRobot(self.plant, dict(nominal, calibration_verified=True), allow_nominal=True)

    def test_joint_preparation_and_tcp_handoff_use_one_robot_and_reset_ik_posture(self):
        self.robot.start()
        self.robot.begin_preparation()
        joints = (.005, 0, 0, 0, 0, 0, 0, .01)
        self.robot.submit_preparation(joints, time.monotonic()+1)
        self.assertEqual(self.solver.step.call_count, 0)
        self.assertEqual(len(self.plant.commands), 1)
        tcp = tuple(self.robot.target)
        with self.assertRaises(RuntimeError): self.robot.submit(tcp, time.monotonic()+1)
        with patch("openarm_runtime.cartesian.DifferentialIK", return_value=self.solver) as reset_ik:
            handoff = self.robot.finish_preparation()
        self.np.testing.assert_allclose(reset_ik.call_args.kwargs["posture"], joints[:7])
        self.np.testing.assert_allclose(handoff, self.robot.target)
        self.np.testing.assert_allclose(self.robot.reference, joints)
        self.robot.submit(handoff, time.monotonic()+1)
        self.assertEqual(self.solver.step.call_count, 1)
        self.robot.close()

    def test_preparation_direct_joint_limits_reject_before_dispatch(self):
        self.robot.start()
        with self.assertRaises(RuntimeError): self.robot.submit_preparation((0.,)*8, time.monotonic()+1)
        self.robot.begin_preparation()
        with self.assertRaises(ValueError): self.robot.submit_preparation((.1, 0, 0, 0, 0, 0, 0, 0), time.monotonic()+1)
        self.assertFalse(self.plant.commands)
        self.robot.close()

    def test_stale_feedback_and_large_joint_lead_refuse_dispatch(self):
        self.robot.start()
        target = (0., 0., 0., 0., 0., 0., 1., .04)
        snapshot = self.plant.snapshot()
        sample = snapshot["robot/left"]
        stale = {"robot/left": replace(sample, received_at=time.monotonic()-1)}
        with patch.object(self.plant, "snapshot", return_value=stale):
            with self.assertRaises(RuntimeError): self.robot.submit(target, time.monotonic()+1)
        self.solver.step.return_value["joints"] = [.2] * 7
        with self.assertRaises(RuntimeError): self.robot.submit(target, time.monotonic()+1)
        self.assertFalse(self.plant.commands)
        self.robot.close()


@unittest.skipUnless(NUMERIC and all(importlib.util.find_spec(n) for n in ("pinocchio", "pink", "daqp")),
                     "install Cartesian solver dependencies")
class RealIKDispatchTests(unittest.TestCase):
    def test_real_fk_ik_with_nonidentity_task_frame_and_simulated_robot(self):
        import numpy as np
        from openarm_policy.cartesian import CartesianRobot
        from openarm_policy.robots import SimulatedRobot
        links = ['<link name="base"/>'] + [f'<link name="l{i}"/>' for i in range(7)]
        joints = []
        for i in range(7):
            parent = f"l{i-1}" if i else "base"
            axis = ("0 0 1", "0 1 0", "1 0 0")[i % 3]
            joints.append(f'<joint name="j{i}" type="revolute"><parent link="{parent}"/>'
                          f'<child link="l{i}"/><origin xyz="0 0 .1"/><axis xyz="{axis}"/>'
                          '<limit lower="-2" upper="2" velocity="1" effort="1"/></joint>')
        source = ('<robot name="fixture">' + ''.join(links+joints) + '</robot>').encode()
        with tempfile.TemporaryDirectory() as directory:
            urdf = Path(directory)/"fixture.urdf"
            urdf.write_bytes(source)
            transform = np.eye(4); transform[:3, 3] = [.2, .3, .1]
            chain = dict(urdf=str(urdf), sha256=hashlib.sha256(source).hexdigest(),
                         joints=[f"j{i}" for i in range(7)], signs=[1]*7, offsets=[0]*7,
                         locked={}, frame="l6", tcp=np.eye(4).tolist())
            config = dict(side="left", calibration_verified=True, chain=chain,
                          task_from_base=transform.tolist(), workspace_m=[[-2]*3, [2]*3],
                          gripper={"motor_rad": [0, 1.], "opening_m": [0, .08]})
            plant = SimulatedRobot(("left",))
            robot = CartesianRobot(plant, config)
            robot.start()
            try:
                measured = robot.snapshot()["tcp/left"].data
                np.testing.assert_allclose(measured[:3], [.2, .3, .8], atol=1e-8)
                target = list(measured); target[0] += .002
                previous = np.asarray(plant.values)
                for _ in range(5):
                    robot.submit(tuple(target), time.monotonic()+1)
                    current = np.asarray(plant.values)
                    self.assertLessEqual(float(abs(current[:7]-previous[:7]).max()), .01250001)
                    previous = current
                    time.sleep(.05)
                self.assertTrue(plant.commands)
                self.assertLess(abs(robot.snapshot()["tcp/left"].data[0]-target[0]), .002)
            finally:
                robot.close()
            self.assertFalse(plant.started)


if __name__ == "__main__":
    unittest.main()
