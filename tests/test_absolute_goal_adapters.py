"""Optional absolute goal decoding and pre-start contract binding; no hardware."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from openarm_policy.contracts import ActionChunk, ActionSpec, Context, Observation, SensorSample, StreamSpec, TCP_POSE
from openarm_policy.models.policy import ModelPolicy


NUMERIC = all(importlib.util.find_spec(name) for name in ("numpy", "scipy"))


def context(side="left", execution=None):
    names = tuple(f"{side}.{n}" for n in ("x", "y", "z", "qx", "qy", "qz", "qw", "opening"))
    action = ActionSpec(names, (-2.,)*3 + (-1.,)*4 + (0.,),
                        (2.,)*3 + (1.,)*4 + (.1,), kind=TCP_POSE)
    return Context(action, (StreamSpec(f"target/{side}", TCP_POSE, (8,), "m,xyzw,m", "task"),), 20,
                   execution={} if execution is None else execution)


def progress_receipt(side="left", translation=.002, rotation=.015):
    return {"cartesian_progress": {"side": side, "maximum_translation_step_m": translation,
                                    "maximum_rotation_step_rad": rotation, "feedback_pacing": True}}


class ResetAbsolutePolicy:
    """Picklable reset probe for the actual spawned inference lifecycle."""
    def __init__(self, path):
        self.path = path
        # A policy's claim must never substitute for the runtime's receipt.
        self.metadata = {"execution": progress_receipt()}

    def reset(self, context):
        from openarm_policy.models.tacpilot import AbsoluteUprightActions
        AbsoluteUprightActions(self.path, .01, .07).reset(context)

    def infer(self, observation):
        raise AssertionError("Reset rejection must precede inference")

    def close(self):
        pass


class BindingTests(unittest.TestCase):
    def test_context_execution_defaults_preserve_positional_version(self):
        old = Context(context().action, (), 20, "task", 1)
        self.assertEqual(old.version, 1)
        self.assertEqual(old.execution, {})
        old.execution["test"] = True
        self.assertEqual(Context(context().action, (), 20).execution, {})

    def test_session_uses_only_robot_execution_and_copies_receipt(self):
        from openarm_policy.session import PolicySession
        policy = SimpleNamespace(metadata={"execution": progress_receipt()})
        robot = SimpleNamespace(spec=context().action, streams=())
        self.assertEqual(PolicySession(robot, policy).context.execution, {})
        robot.execution = progress_receipt()
        session = PolicySession(robot, policy)
        self.assertEqual(session.context.execution, robot.execution)
        robot.execution["cartesian_progress"]["maximum_translation_step_m"] = .01
        self.assertEqual(session.context.execution, progress_receipt())

    def bind(self, model_meta, decoder_meta, encoder_meta=None):
        objects = [Mock(metadata=model_meta), Mock(metadata=encoder_meta or {}), Mock(metadata=decoder_meta)]
        policy = ModelPolicy({}, {}, {})
        with patch("openarm_policy.models.policy.component", side_effect=objects):
            try:
                policy.reset(context())
            except ValueError:
                for obj in objects:
                    obj.close.assert_called_once()
                self.assertEqual(policy.components, [])
                raise
        policy.close()

    def test_legacy_and_matching_direct_and_http_metadata(self):
        self.bind({}, {})
        digest = {"action_contract_sha256": "a"*64}
        self.bind(digest, digest)
        self.bind({"model": digest}, digest)

    def test_mismatch_missing_and_invalid_metadata_cleanup(self):
        good = {"action_contract_sha256": "a"*64}
        for model, decoder in ((good, {}), ({}, good),
                               (good, {"action_contract_sha256": "b"*64}),
                               ({"model": good}, {}), ([], {}), ({"model": None}, {}), ({}, [])):
            with self.subTest(model=model, decoder=decoder), self.assertRaises(ValueError):
                self.bind(model, decoder)
        for bad in (None, True, 1, [], {}, "", "a"*63, "g"*64):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.bind({"action_contract_sha256": bad}, good)

    def test_state_width_checks_are_preserved(self):
        for model in ({"state_width": 8}, {}, {"model": {"state_width": 8}}):
            with self.assertRaises(ValueError):
                self.bind(model, {}, {"state_width": 10})

    def test_imports_do_not_load_numeric_or_hardware_libraries(self):
        subprocess.run([sys.executable, "-c",
                        "import sys; import openarm_policy.models.tacpilot, openarm_policy.models.act; "
                        "assert not set(('numpy','scipy','torch','lerobot','cv2','pinocchio','openarm_can')) & set(sys.modules)"],
                       check=True)


@unittest.skipUnless(NUMERIC, "install optional numpy/scipy dependencies")
class AbsoluteGoalTests(unittest.TestCase):
    def setUp(self):
        import numpy as np
        from scipy.spatial.transform import Rotation
        self.np, self.R = np, Rotation
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "action-contract.json"
        self.contract = dict(schema="tacpilot.absolute-cartesian-goals.v1",
                             position_anchor_m=[.1, -.2, .5], position_scale_m=.25,
                             reference_xyzw=Rotation.from_rotvec([.4, -.3, .6]).as_quat().tolist(),
                             rotation_scale_rad=math.pi, maximum_translation_step_m=.002,
                             maximum_rotation_step_rad=.015)
        self.write(self.contract)

    def write(self, contract):
        self.path.write_text(json.dumps(contract, indent=2) + "\n")

    def decoder(self, **kwargs):
        from openarm_policy.models.tacpilot import AbsoluteUprightActions
        decoder = AbsoluteUprightActions(str(self.path), .01, .09, **kwargs)
        side = kwargs.get("side", "left")
        decoder.reset(context(side, progress_receipt(side)))
        return decoder

    def test_required_governor_finite_caps_side_and_feedback_pacing(self):
        decoder = self.decoder()
        for translation, rotation in ((.002, .015), (.001, .01)):
            decoder.reset(context(execution=progress_receipt(translation=translation, rotation=rotation)))
        invalid = [{}, None, [], {"cartesian_progress": None}, progress_receipt("right"),
                   progress_receipt(translation=.002001), progress_receipt(rotation=.015001)]
        for key in ("maximum_translation_step_m", "maximum_rotation_step_rad"):
            for value in (None, True, "0.001", 0, -1, math.nan, math.inf, 10**500):
                receipt = progress_receipt()
                receipt["cartesian_progress"][key] = value
                invalid.append(receipt)
            receipt = progress_receipt()
            del receipt["cartesian_progress"][key]
            invalid.append(receipt)
        for value in (False, 1, "true", None):
            receipt = progress_receipt()
            receipt["cartesian_progress"]["feedback_pacing"] = value
            invalid.append(receipt)
        for execution in invalid:
            with self.subTest(execution=execution), self.assertRaises(ValueError):
                decoder.reset(context(execution=execution))

    def test_session_rejects_missing_or_looser_governor_before_start(self):
        from openarm_policy.session import PolicySession
        for receipt in ({}, progress_receipt(translation=.003), progress_receipt(rotation=.02),
                        progress_receipt("right")):
            with self.subTest(receipt=receipt):
                robot = SimpleNamespace(spec=context().action, streams=context().streams,
                                        execution=receipt, start=Mock(), close=Mock())
                session = PolicySession(robot, ResetAbsolutePolicy(str(self.path)))
                with self.assertRaisesRegex(RuntimeError, "Policy reset failed"):
                    session.run(duration_s=.01)
                robot.start.assert_not_called()
                robot.close.assert_called_once()
                self.assertEqual(session.state, "failed")

    def cartesian_robot(self, progress):
        from openarm_policy.cartesian import CartesianRobot
        from openarm_policy.robots import SimulatedRobot
        transform = self.np.eye(4)
        transform[:3, 3] = self.contract["position_anchor_m"]
        transform[:3, :3] = self.R.from_quat(self.contract["reference_xyzw"]).as_matrix()
        chain = Mock(indices=list(range(7)), metadata={"fixture": True})
        chain.forward.return_value = transform
        plant = SimulatedRobot(("left",))
        config = dict(side="left", calibration_verified=True, chain={},
                      task_from_base=self.np.eye(4).tolist(), workspace_m=[[-1]*3, [1]*3],
                      gripper={"motor_rad": [0, 1.], "opening_m": [0, .1]}, progress=progress)
        with patch("openarm_runtime.cartesian.KinematicChain", return_value=chain):
            robot = CartesianRobot(plant, config)
        self.addCleanup(robot.close)
        return robot

    def test_cartesian_receipt_is_read_only_and_derived_from_live_governor(self):
        robot = self.cartesian_robot(None)
        self.assertEqual(robot.execution, {})
        robot = self.cartesian_robot({"translation_step_m": .001, "rotation_step_rad": .01})
        self.assertEqual(robot.execution, progress_receipt(translation=.001, rotation=.01))
        robot.config["progress"]["translation_step_m"] = .009
        receipt = robot.execution
        receipt["cartesian_progress"]["maximum_translation_step_m"] = .009
        self.assertEqual(robot.execution, progress_receipt(translation=.001, rotation=.01))
        with self.assertRaises(AttributeError): robot.execution = {}
        from openarm_policy.session import PolicySession
        session = PolicySession(robot, ResetAbsolutePolicy(str(self.path)))
        self.decoder().reset(session.context)

    def test_skipped_prefix_is_governed_from_actual_dispatched_fk_at_first_submit(self):
        from openarm_policy.session import PolicySession
        robot = self.cartesian_robot({})
        decoder = self.decoder(execute_steps=100)
        decoder.reset(PolicySession(robot, ResetAbsolutePolicy(str(self.path))).context)
        solver = Mock()
        solver.step.return_value = dict(joints=[0.]*7, position_error_m=0., orientation_error_rad=0., solve_s=0.)
        with patch("openarm_runtime.cartesian.DifferentialIK", return_value=solver):
            robot.start()
        dispatched = self.np.asarray(robot.snapshot()["target/left"].data)
        steps = decoder.decode(self.np.tile([1., 0, 0, 0, 0, 1., 0], (100, 1)), self.observation())
        active = ActionChunk(1, 10., .05, tuple(tuple(float(v) for v in row) for row in steps),
                             robot.spec.names, kind=TCP_POSE)
        requested = active.at(11.251)  # First dispatch skips rows 0..24.
        self.assertEqual(requested, active.steps[25])
        self.assertGreater(self.np.linalg.norm(self.np.asarray(requested[:3])-dispatched[:3]), .05)
        self.assertGreater((self.R.from_quat(requested[3:7]) * self.R.from_quat(dispatched[3:7]).inv()).magnitude(), .3)
        robot.submit(requested, time.monotonic()+1)
        governed = solver.step.call_args.args[1]
        # This is the actual submit() governor output reaching IK, not decoder rows.
        self.assertAlmostEqual(self.np.linalg.norm(governed[:3, 3]-dispatched[:3]), .002)
        turn = self.R.from_matrix(governed[:3, :3]) * self.R.from_quat(dispatched[3:7]).inv()
        self.assertAlmostEqual(turn.magnitude(), .015)
        self.assertEqual(len(robot.robot.commands), 1)
        # Feedback pacing still holds instead of advancing IK when joints lag.
        robot.reference[0] = .08
        robot.submit(requested, time.monotonic()+1)
        self.assertEqual(solver.step.call_count, 1)

    def observation(self, position=None, rotation=None, side="left"):
        if position is None:
            position = self.contract["position_anchor_m"]
        if rotation is None:
            rotation = self.R.from_quat(self.contract["reference_xyzw"])
        return Observation(1, 10, {f"target/{side}": (SensorSample(1, 9.9, [*position, *rotation.as_quat(), .05]),)})

    def test_exact_byte_fingerprint_including_whitespace(self):
        first = self.decoder().metadata["action_contract_sha256"]
        self.assertEqual(first, hashlib.sha256(self.path.read_bytes()).hexdigest())
        self.path.write_text(json.dumps(self.contract))
        self.assertNotEqual(first, self.decoder().metadata["action_contract_sha256"])

    def test_repeated_fixed_goal_converges_without_drift_and_reanchors(self):
        row = self.np.array([.12, -.08, .04, .1, -.05, .08, 0])
        decoder = self.decoder(execute_steps=100)
        rows = self.np.tile(row, (100, 1))
        observation = self.observation()
        result = self.np.asarray(decoder.decode(rows, observation))
        goal = self.np.asarray(self.contract["position_anchor_m"]) + row[:3]*.25
        rotation = self.R.from_quat(self.contract["reference_xyzw"]) * self.R.from_rotvec(row[3:6]*math.pi)
        self.np.testing.assert_allclose(result[-20:, :3], self.np.tile(goal, (20, 1)), atol=1e-12)
        self.assertLess((self.R.from_quat(result[-1, 3:7]) * rotation.inv()).magnitude(), 1e-12)
        self.np.testing.assert_allclose(result, decoder.decode(rows, observation))
        for _ in range(10):
            observation = self.observation(result[-1, :3], self.R.from_quat(result[-1, 3:7]))
            result = self.np.asarray(decoder.decode(rows, observation))
            self.np.testing.assert_allclose(result[:, :3], self.np.tile(goal, (100, 1)), atol=1e-12)

    def test_near_pi_fixed_home_world_rotation_and_step_caps(self):
        relative = self.R.from_rotvec(self.np.array([.3, -.4, .5]) / self.np.linalg.norm([.3, -.4, .5]) * (math.pi-1e-8))
        reference = self.R.from_quat(self.contract["reference_xyzw"])
        goal = reference * relative
        initial = self.R.from_rotvec([.02, -.03, .01]) * goal
        row = self.np.r_[[1, 1, 1], relative.as_rotvec()/math.pi, 0]
        observation = self.observation(rotation=initial)
        result = self.decoder(execute_steps=100).decode(self.np.tile(row, (100, 1)), observation)
        previous_p = self.np.asarray(self.contract["position_anchor_m"])
        previous_r = initial
        self.assertAlmostEqual(self.np.linalg.norm(result[0][:3]-previous_p), .002)
        self.assertAlmostEqual((self.R.from_quat(result[0][3:7]) * initial.inv()).magnitude(), .015)
        for value in result:
            self.assertLessEqual(self.np.linalg.norm(value[:3]-previous_p), .002+1e-12)
            rotation = self.R.from_quat(value[3:7])
            self.assertLessEqual((rotation * previous_r.inv()).magnitude(), .015+1e-12)
            previous_p, previous_r = value[:3], rotation
        self.assertLess((previous_r * goal.inv()).magnitude(), 1e-12)
        # Noncommuting reference makes swapped/home-vs-world rotation detectable.
        self.assertGreater((previous_r * (relative * reference).inv()).magnitude(), .1)

    def test_static_reference_matches_independent_source_applied_delta_math(self):
        rng = self.np.random.default_rng(42)
        decoder = self.decoder(execute_steps=1)
        previous_p = self.np.array([.2, -.1, .6])
        previous_r = self.R.from_rotvec([.5, -.7, 1.])
        for goal in rng.uniform(-1, 1, (40, 7)):
            target_p = self.np.asarray(self.contract["position_anchor_m"]) + goal[:3]*.25
            target_r = (self.R.from_quat(self.contract["reference_xyzw"]).as_matrix()
                        @ self.R.from_rotvec(goal[3:6]*math.pi).as_matrix())
            dp = target_p-previous_p
            dr = self.R.from_matrix(target_r @ previous_r.as_matrix().T).as_rotvec()
            dp *= min(1., .002/max(self.np.linalg.norm(dp), 1e-12))
            dr *= min(1., .015/max(self.np.linalg.norm(dr), 1e-12))
            # Source applied_delta returns normalized float32 legacy deltas.
            delta = self.np.r_[dp/.02, dr/.1, goal[6]].astype(self.np.float32)
            actual = decoder.decode(goal[None], self.observation(previous_p, previous_r))[0]
            self.np.testing.assert_allclose(actual[:3], previous_p + delta[:3]*.02, atol=1e-9)
            expected_r = self.R.from_rotvec(delta[3:6]*.1) * previous_r
            self.assertLess((self.R.from_quat(actual[3:7]) * expected_r.inv()).magnitude(), 2e-9)

    def test_gripper_curve_right_side_and_legacy_defaults(self):
        from openarm_policy.models.tacpilot import UprightActions, UprightObservation
        decoder = self.decoder(side="right", execute_steps=3,
                               gripper_action_curve=[[-1, .09], [0, .07], [1, .01]])
        actions = self.np.zeros((3, 7)); actions[:, 6] = [-1, 0, 1]
        result = decoder.decode(actions, self.observation(side="right"))
        self.np.testing.assert_allclose(self.np.asarray(result)[:, 7], [.09, .07, .01])
        self.assertEqual(decoder.metadata["side"], "right")
        linear = self.decoder(execute_steps=3).decode(actions, self.observation())
        self.np.testing.assert_allclose(self.np.asarray(linear)[:, 7], [.09, .05, .01])
        legacy = UprightActions(.01, .09)
        self.assertEqual((legacy.count, legacy.translation, legacy.rotation), (25, .02, .1))
        self.assertNotIn("action_contract_sha256", legacy.metadata)
        self.assertEqual(UprightObservation("a", "b").metadata["state_width"], 8)
        actions = self.np.zeros((25, 7)); actions[:, 0] = 1
        result = legacy.decode(actions, self.observation())
        self.assertAlmostEqual(result[-1][0], .1 + 25*.02)

    def test_contract_strict_validation(self):
        invalid = {"schema": [None, "wrong"], "position_anchor_m": [[0, True, 0], [0, 0], [0, 0, math.nan]],
                   "reference_xyzw": [[0, 0, 0, 2], [0, 0, 0, True]],
                   "position_scale_m": [True, "0.25", 0, -1, math.inf, 10**500],
                   "rotation_scale_rad": [True, 0, .1],
                   "maximum_translation_step_m": [True, 0, .020001, math.nan],
                   "maximum_rotation_step_rad": [True, -1, .100001, math.inf]}
        for key, values in invalid.items():
            for value in values:
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    self.write(dict(self.contract, **{key: value}))
                    self.decoder()
            with self.subTest(missing=key), self.assertRaises(ValueError):
                self.write({k: v for k, v in self.contract.items() if k != key})
                self.decoder()
        self.write([])
        with self.assertRaises(ValueError): self.decoder()

    def test_invalid_prediction_anchor_and_calibration(self):
        decoder = self.decoder()
        for value in (self.np.zeros((24, 7)), self.np.zeros((25, 8)),
                      self.np.full((25, 7), self.np.nan), self.np.zeros((25, 7), dtype=bool)):
            with self.assertRaises(ValueError): decoder.decode(value, self.observation())
        observation = Observation(1, 10, {"target/left": (SensorSample(1, 9.9, [0]*8),)})
        with self.assertRaises(ValueError): decoder.decode(self.np.zeros((25, 7)), observation)
        from openarm_policy.models.tacpilot import AbsoluteUprightActions
        with self.assertRaises(ValueError): AbsoluteUprightActions(str(self.path), .09, .01)
        with self.assertRaises(ValueError): AbsoluteUprightActions(str(self.path), 0, .2).reset(context())

    def test_saturation_and_required_dispatched_reference(self):
        from dataclasses import replace
        decoder = self.decoder(execute_steps=1)
        actions = self.np.array([[2., -3., 4., 2., -.2, .3, 2.]])
        actual = decoder.decode(actions, self.observation())
        expected = decoder.decode(self.np.clip(actions, -1, 1), self.observation())
        self.np.testing.assert_allclose(actual, expected)
        self.assertAlmostEqual(actual[0][7], .01)
        with self.assertRaises(ValueError): decoder.reset(replace(context(), streams=()))
        with self.assertRaises(ValueError): decoder.reset(context("right"))

    def test_act_publishes_optional_contract_identity_without_semantic_parse(self):
        from openarm_policy.models.act import TacPilotACT
        # Backend stubs exercise the real reset metadata path, not ML inference.
        np = self.np
        class Tensor:
            def __init__(self, width): self.shape = (width,)
            def to(self, device): return self
            def cpu(self): return self
            def __lt__(self, other): return np.array([False])
        stats = {f"{key}.{field}": Tensor(width) for key, width in (
            ("observation.state", 8), ("action", 7),
            ("observation.images.image", 3), ("observation.images.image2", 3)) for field in ("mean", "std")}
        cfg = SimpleNamespace(chunk_size=100, robot_state_feature=SimpleNamespace(shape=(8,)),
                              action_feature=SimpleNamespace(shape=(7,)),
                              image_features={"observation.images.image", "observation.images.image2"})
        modules = {"torch": SimpleNamespace(isfinite=lambda v: np.array([True]),
                                            testing=SimpleNamespace(assert_close=lambda a, b: None)),
                   "safetensors.torch": SimpleNamespace(load_file=lambda path: stats),
                   "lerobot.policies.act.configuration_act": SimpleNamespace(ACTConfig=Mock(from_pretrained=Mock(return_value=cfg))),
                   "lerobot.policies.act.modeling_act": SimpleNamespace(ACTPolicy=Mock())}
        root = Path(self.directory.name)
        (root/"model.safetensors").write_bytes(b"fixture")
        self.path.write_bytes(b'{"schema":"not parsed by predictor"}\n')
        with patch.dict(sys.modules, modules):
            model = TacPilotACT(root)
            model.reset(None)
            expected = hashlib.sha256(self.path.read_bytes()).hexdigest()
            self.assertEqual(model.metadata["action_contract_sha256"], expected)
            self.assertEqual(model.metadata["files_sha256"]["action-contract.json"], expected)
            self.assertEqual(model.metadata["model_sha256"], hashlib.sha256(b"fixture").hexdigest())
            self.path.unlink()
            model.reset(None)
            self.assertNotIn("action_contract_sha256", model.metadata)


if __name__ == "__main__":
    unittest.main()
