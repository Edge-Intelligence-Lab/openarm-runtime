"""Causal temporal aggregation and decoder composition, entirely offline."""
from dataclasses import replace
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

from openarm_policy.contracts import ActionSpec, Context, Observation, SensorSample, StreamSpec, TCP_POSE
from openarm_policy.models.policy import ModelPolicy
from openarm_policy.models.temporal import TemporalActionEnsembler, TemporalEnsembleActions


NUMPY = importlib.util.find_spec("numpy") is not None
SCIPY = importlib.util.find_spec("scipy") is not None


class ConfigurationTests(unittest.TestCase):
    def test_import_and_construction_without_optional_libraries(self):
        code = """
import sys
from openarm_policy.models.temporal import TemporalActionEnsembler, TemporalEnsembleActions
TemporalActionEnsembler()
TemporalEnsembleActions({'factory': 'not_imported:Decoder'})
assert not set(('numpy', 'scipy', 'torch', 'openarm_can')) & set(sys.modules)
assert 'openarm_policy.contracts' not in sys.modules
assert 'openarm_policy.models.policy' not in sys.modules
"""
        subprocess.run([sys.executable, "-S", "-c", code], check=True)

    def test_lazy_public_exports_preserve_class_identity(self):
        import openarm_policy
        import openarm_policy.models
        self.assertIs(openarm_policy.ActionSpec, ActionSpec)
        self.assertIs(openarm_policy.Context, Context)
        self.assertIs(openarm_policy.models.ModelPolicy, ModelPolicy)
        self.assertIn('PolicySession', dir(openarm_policy))
        with self.assertRaises(AttributeError):getattr(openarm_policy,'not_an_export')

    def test_coefficient_and_mode_validation(self):
        for bad in (True, False, "0.01", None, float("nan"), float("inf"), -.01, 1.01, 1j, 10**1000):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                TemporalActionEnsembler(bad)
        for good in (0, .01, 1):
            self.assertEqual(TemporalActionEnsembler(good).metadata["coefficient"], good)
        with self.assertRaises(ValueError):
            TemporalActionEnsembler(rotation_average="quaternion")


@unittest.skipUnless(NUMPY, "install optional NumPy")
class EnsembleTests(unittest.TestCase):
    def setUp(self):
        import numpy as np
        self.np = np

    def test_four_row_reference_and_future_slot_renormalization(self):
        ensemble = TemporalActionEnsembler()
        first = ensemble.add([[0], [1], [2], [3]], 0)
        self.np.testing.assert_array_equal(first[:, 0], [0, 1, 2, 3])
        actual = ensemble.add([[10], [11], [12], [13]], 2)
        weight = math.exp(.02)
        expected = [(weight*2+10)/(weight+1), (weight*3+11)/(weight+1), 12, 13]
        self.np.testing.assert_allclose(actual[:, 0], expected)
        self.assertEqual(actual.dtype, self.np.float32)
        self.assertEqual(actual.shape, (4, 1))
        self.assertLess(actual[0, 0], 6)  # Older, not newer, predictions are preferred.

    def test_valid_zero_rows_and_uniform_weights(self):
        ensemble = TemporalActionEnsembler(0)
        ensemble.add(self.np.zeros((4, 7)), 0)
        result = ensemble.add(self.np.full((4, 7), 2), 1)
        self.np.testing.assert_array_equal(result[:3], 1)
        self.np.testing.assert_array_equal(result[3], 2)

    def test_caller_and_result_aliases_do_not_change_history(self):
        ensemble = TemporalActionEnsembler(0)
        original = self.np.arange(4, dtype=float)[:, None]
        original.flags.writeable = False
        result = ensemble.add(original, 0)
        result[:] = -100
        original.flags.writeable = True
        original[:] = 100
        blended = ensemble.add(self.np.zeros((4, 1)), 1)
        self.np.testing.assert_array_equal(blended[:, 0], [.5, 1, 1.5, 0])

    def test_pruning_is_bounded_and_sparse_source_steps_are_supported(self):
        ensemble = TemporalActionEnsembler()
        for step in range(12):
            ensemble.add([[step]] * 4, step)
            self.assertLessEqual(len(ensemble._chunks), 4)
            self.assertTrue(all(origin+4 > step for origin, _ in ensemble._chunks))
        self.np.testing.assert_array_equal(ensemble.add([[9]] * 4, 25), 9)
        self.assertEqual(len(ensemble._chunks), 1)
        ensemble.reset()
        ensemble.add(self.np.zeros((100, 7)), 0)
        result = ensemble.add(self.np.ones((100, 7)), 25)
        self.assertLess(result[0, 0], .5)
        self.np.testing.assert_array_equal(result[75:], 1)

    def test_reset_clears_shape_and_clock(self):
        ensemble = TemporalActionEnsembler()
        ensemble.add([[1]] * 4, 25)
        ensemble.reset()
        self.assertEqual(ensemble._chunks, [])
        self.np.testing.assert_array_equal(ensemble.add([[0, 0]], 0), [[0, 0]])

    def test_invalid_shape_finiteness_dtype_and_clock_do_not_consume_step(self):
        ensemble = TemporalActionEnsembler()
        ensemble.add([[0], [1], [2], [3]], 0)
        invalid = ([], [1, 2], [[1]], [[1, 2]] * 4,
                   [[[1]]] * 4, [[float("nan")]] * 4, [[float("inf")]] * 4,
                   [[1e300]] * 4, [[1j]] * 4, [["1"]] * 4, [[True]] * 4)
        for chunk in invalid:
            with self.subTest(chunk=chunk), self.assertRaises(ValueError):
                ensemble.add(chunk, 1)
        for step in (-1, 0, 1.0, True, float("nan"), "1"):
            with self.subTest(step=step), self.assertRaises(ValueError):
                ensemble.add([[0]] * 4, step)
        ensemble.add([[0]] * 4, self.np.int64(1))
        with self.assertRaises(ValueError):
            ensemble.add([[0]] * 4, 0)

    def test_large_horizon_log_weights_are_stable_per_future_slot(self):
        ensemble = TemporalActionEnsembler(1)
        ensemble.add(self.np.zeros((1100, 1)), 0)
        result = ensemble.add(self.np.ones((1100, 1)), 1000)
        self.assertTrue(self.np.isfinite(result).all())
        self.np.testing.assert_allclose(result[:100], 0, atol=1e-30)
        self.np.testing.assert_array_equal(result[100:], 1)

    @unittest.skipUnless(SCIPY, "install optional SciPy")
    def test_rotvec_pi_branch_mean_and_repeated_goals(self):
        ensemble = TemporalActionEnsembler(0, "rotvec_pi")
        old = self.np.tile([1, 2, 3, .99, 0, 0, -1], (4, 1))
        new = self.np.tile([3, 4, 5, -.99, 0, 0, 1], (4, 1))
        ensemble.add(old, 0)
        result = ensemble.add(new, 1)
        self.np.testing.assert_allclose(result[:3, :3], [[2, 3, 4]] * 3)
        self.np.testing.assert_allclose(result[:3, 6], 0)
        self.np.testing.assert_allclose(abs(result[:3, 3]), 1, atol=1e-6)
        linear = TemporalActionEnsembler(0)
        linear.add(old, 0)
        self.np.testing.assert_allclose(linear.add(new, 1)[:3, 3], 0)
        stable = TemporalActionEnsembler(.01, "rotvec_pi")
        for step in range(7):
            actual = stable.add(old, step)
            self.np.testing.assert_allclose(actual, old, atol=1e-6)
        with self.assertRaises(ValueError):
            TemporalActionEnsembler(rotation_average="rotvec_pi").add([[0] * 5], 0)


@unittest.skipUnless(NUMPY, "install optional NumPy")
class WrapperTests(unittest.TestCase):
    def setUp(self):
        import numpy as np
        self.np = np
        self.context = Context(ActionSpec(("a",), (-100.,), (100.,)), (), 20)
        self.inner = Mock(metadata={"state_width": 10, "action_contract_sha256": "a" * 64})
        self.inner.decode.side_effect = lambda output, observation: output
        self.wrapper = TemporalEnsembleActions({"factory": "unused:Decoder"})

    def reset_wrapper(self):
        with patch("openarm_policy.models.policy.component", return_value=self.inner) as factory:
            self.wrapper.reset(self.context)
        factory.assert_called_once_with(self.wrapper.action)
        self.inner.reset.assert_called_once_with(self.context)

    def test_actual_clock_alignment_and_preserved_observation(self):
        self.reset_wrapper()
        self.assertEqual(self.wrapper.period, .05)
        self.assertIsNone(self.wrapper.epoch)
        first = Observation(1, 10., {})
        self.wrapper.decode([[0], [1], [2], [3]], first)
        current = Observation(2, 10.1, {})
        result = self.wrapper.decode([[10], [11], [12], [13]], current)
        weight = math.exp(.02)
        self.assertAlmostEqual(result[0, 0], (weight*2+10)/(weight+1), places=6)
        self.assertIs(self.inner.decode.call_args.args[1], current)
        self.assertEqual(self.wrapper.ensemble._last_step, 2)

    def test_same_step_stale_backwards_nonfinite_reject_before_inner_decode(self):
        self.reset_wrapper()
        self.wrapper.decode([[0]] * 4, Observation(1, 10., {}))
        for capture in (10., 9., 10.001, float("nan"), float("inf"), True):
            with self.subTest(capture=capture), self.assertRaises(ValueError):
                self.wrapper.decode([[1]] * 4, Observation(2, capture, {}))
        self.assertEqual(self.inner.decode.call_count, 1)
        self.wrapper.decode([[1]] * 4, Observation(2, 10.05, {}))
        self.assertEqual(self.wrapper.ensemble._last_step, 1)

    def test_metadata_copies_inner_contract_and_state_fields(self):
        self.reset_wrapper()
        meta = self.wrapper.metadata
        self.assertEqual(meta["action_contract_sha256"], "a" * 64)
        self.assertEqual(meta["state_width"], 10)
        self.assertEqual(meta["temporal_ensemble"]["rotation_average"], "linear")
        meta["state_width"] = 8
        self.assertEqual(self.inner.metadata["state_width"], 10)
        self.assertNotIn("temporal_ensemble", self.inner.metadata)

    @unittest.skipUnless(SCIPY, "install optional SciPy")
    def test_absolute_goal_clipping_precedes_rotation_mean_and_preserves_invalid_rejection(self):
        self.wrapper = TemporalEnsembleActions({}, rotation_average="rotvec_pi")
        self.reset_wrapper()
        goal = self.np.zeros((4, 7), self.np.float32)
        goal[:, 0] = 2.; goal[:, 3] = 1.5
        result = self.wrapper.decode(goal, Observation(1, 10., {}))
        self.np.testing.assert_allclose(result[:, 0], 1.)
        self.np.testing.assert_allclose(abs(result[:, 3]), 1., atol=1e-6)
        self.np.testing.assert_allclose(goal[:, 3], 1.5)
        for invalid in (self.np.full((4, 7), self.np.inf), self.np.ones((4, 7), dtype=bool)):
            with self.assertRaises(ValueError):
                self.wrapper.decode(invalid, Observation(2, 10.05, {}))
        self.assertEqual(self.inner.decode.call_count, 1)

    def test_modelpolicy_binding_and_reset_failure_cleanup(self):
        for digest, fail in (("a" * 64, False), ("b" * 64, True)):
            model = Mock(metadata={"state_width": 10, "action_contract_sha256": digest})
            encoder = Mock(metadata={"state_width": 10})
            wrapper = TemporalEnsembleActions({})
            inner = Mock(metadata=self.inner.metadata)
            policy = ModelPolicy({}, {}, {})
            with patch("openarm_policy.models.policy.component", side_effect=[model, encoder, wrapper, inner]):
                if fail:
                    with self.assertRaisesRegex(ValueError, "contract SHA256 differ"):
                        policy.reset(self.context)
                else:
                    policy.reset(self.context)
                    policy.close()
            inner.reset.assert_called_once_with(self.context)
            inner.close.assert_called_once()
            model.close.assert_called_once()
            self.assertIsNone(wrapper.inner)
        self.inner.reset.side_effect = ValueError("governor missing")
        policy = ModelPolicy({}, {}, {})
        with patch("openarm_policy.models.policy.component",
                   side_effect=[Mock(metadata={}), Mock(metadata={}), self.wrapper, self.inner]):
            with self.assertRaisesRegex(ValueError, "governor missing"):
                policy.reset(self.context)
        self.inner.close.assert_called_once()

    def test_inner_constructor_failure_is_cleaned_by_modelpolicy(self):
        model, encoder = Mock(), Mock()
        policy = ModelPolicy({}, {}, {})
        with patch("openarm_policy.models.policy.component",
                   side_effect=[model, encoder, self.wrapper, ValueError("constructor failed")]):
            with self.assertRaisesRegex(ValueError, "constructor failed"):
                policy.reset(self.context)
        model.close.assert_called_once()
        encoder.close.assert_called_once()
        self.assertIsNone(self.wrapper.inner)
        self.assertIsNone(self.wrapper.period)

    def test_close_clears_state_even_if_inner_close_raises(self):
        self.reset_wrapper()
        self.wrapper.decode([[1]] * 4, Observation(1, 10., {}))
        self.inner.close.side_effect = ValueError("close failed")
        with self.assertRaisesRegex(ValueError, "close failed"):
            self.wrapper.close()
        self.assertEqual(self.wrapper.ensemble._chunks, [])
        self.assertIsNone(self.wrapper.epoch)
        self.assertIsNone(self.wrapper.inner)
        self.wrapper.close()
        with self.assertRaises(RuntimeError):
            self.wrapper.decode([[1]], Observation(2, 11., {}))

    @unittest.skipUnless(SCIPY, "install optional SciPy")
    def test_absolute_governor_reset_checks_and_actual_target_reanchoring(self):
        names = tuple("left." + n for n in ("x", "y", "z", "qx", "qy", "qz", "qw", "opening"))
        action = ActionSpec(names, (-2.,)*3 + (-1.,)*4 + (0.,),
                            (2.,)*3 + (1.,)*4 + (.1,), kind=TCP_POSE)
        context = Context(action, (StreamSpec("target/left", TCP_POSE, (8,), "m,xyzw,m", "task"),), 20)
        contract = dict(schema="tacpilot.absolute-cartesian-goals.v1", position_anchor_m=[0, 0, .5],
                        position_scale_m=.25, reference_xyzw=[0, 0, 0, 1], rotation_scale_rad=math.pi,
                        maximum_translation_step_m=.002, maximum_rotation_step_rad=.015)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "contract.json"
            path.write_text(json.dumps(contract))
            spec = {"factory": "openarm_policy.models.tacpilot:AbsoluteUprightActions", "kwargs": {
                "action_contract": str(path), "closed_opening_m": .01, "open_opening_m": .09,
                "execute_steps": 2}}
            wrapper = TemporalEnsembleActions(spec, rotation_average="rotvec_pi")
            for execution in ({}, {"cartesian_progress": {"side": "left", "feedback_pacing": True,
                                                         "maximum_translation_step_m": .02,
                                                         "maximum_rotation_step_rad": .015}}):
                with self.assertRaises(ValueError):
                    wrapper.reset(replace(context, execution=execution))
                wrapper.close()
            good = {"cartesian_progress": {"side": "left", "feedback_pacing": True,
                                           "maximum_translation_step_m": .002,
                                           "maximum_rotation_step_rad": .015}}
            wrapper.reset(replace(context, execution=good))
            goals = self.np.tile([1, 0, 0, 0, 0, 0, 0], (4, 1))
            for sequence, x in ((0, 0), (1, .1)):
                observation = Observation(sequence, 10+sequence*.05, {"target/left": (
                    SensorSample(sequence, 10+sequence*.05, [x, 0, .5, 0, 0, 0, 1, .06]),)})
                decoded = wrapper.decode(goals, observation)
                self.np.testing.assert_allclose([row[0] for row in decoded], [x+.002, x+.004])
            self.assertEqual(len(wrapper.metadata["action_contract_sha256"]), 64)
            wrapper.close()


if __name__ == "__main__":
    unittest.main()
