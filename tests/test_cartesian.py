"""Optional offline kinematics tests; never import hardware transports."""
import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest


AVAILABLE = all(importlib.util.find_spec(n) for n in ("numpy", "pinocchio", "pink", "daqp"))


@unittest.skipUnless(AVAILABLE, "install cartesian extra")
class CartesianTests(unittest.TestCase):
    def setUp(self):
        import numpy as np
        from openarm_runtime.cartesian import KinematicChain
        self.np = np
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        # 7-DOF alternating-axis fixture, intentionally not a real leader model.
        links = ['<link name="base"/>'] + [f'<link name="l{i}"/>' for i in range(7)]
        joints = []
        for i in range(7):
            parent = f"l{i-1}" if i else "base"
            axis = ("0 0 1", "0 1 0", "1 0 0")[i % 3]
            joints.append(f'<joint name="j{i}" type="revolute"><parent link="{parent}"/>'
                          f'<child link="l{i}"/><origin xyz="0 0 .1"/><axis xyz="{axis}"/>'
                          '<limit lower="-2" upper="2" velocity="1" effort="1"/></joint>')
        raw = ('<robot name="fixture">' + ''.join(links + joints) + '</robot>').encode()
        self.path = Path(self.temp.name) / 'fixture.urdf'
        self.path.write_bytes(raw)
        self.kwargs = dict(sha256=hashlib.sha256(raw).hexdigest(), joints=[f'j{i}' for i in range(7)],
                           signs=[1]*7, offsets=[0]*7, locked={}, frame='l6', tcp=np.eye(4))
        self.chain = KinematicChain(self.path, **self.kwargs)

    def solver(self):
        from openarm_runtime.cartesian import DifferentialIK
        return DifferentialIK(self.chain, lower=[-1.8]*7, upper=[1.8]*7,
                              velocity=[.5]*7, posture=[.1]*7)

    def test_no_jump_takeover_and_reclutch(self):
        from openarm_runtime.cartesian import RelativePose
        leader = self.chain.forward([.3]*7)
        follower = self.chain.forward([-.2]*7)
        mapping = RelativePose(leader, follower, self.np.eye(3), scale=2.)
        self.np.testing.assert_allclose(mapping.target(leader), follower, atol=1e-12)
        changed = leader.copy(); changed[0, 3] += .01
        self.assertAlmostEqual(mapping.target(changed)[0, 3] - follower[0, 3], .02)
        reset = RelativePose(changed, follower, self.np.eye(3))
        self.np.testing.assert_allclose(reset.target(changed), follower, atol=1e-12)

    def test_axis_rotation_and_orientation_mapping(self):
        import pinocchio as pin
        from openarm_runtime.cartesian import RelativePose
        a = pin.rpy.rpyToMatrix(0., 0., self.np.pi / 2)
        origin = self.np.eye(4)
        mapping = RelativePose(origin, origin, a)
        current = origin.copy(); current[0, 3] = .1
        current[:3, :3] = pin.rpy.rpyToMatrix(.2, 0., 0.)
        result = mapping.target(current)
        self.np.testing.assert_allclose(result[:3, 3], [0, .1, 0], atol=1e-12)
        self.np.testing.assert_allclose(result[:3, :3], a @ current[:3, :3] @ a.T)

    def test_reject_invalid_pose_hash_and_missing_joint_mapping(self):
        from openarm_runtime.cartesian import pose, KinematicChain
        for invalid in (self.np.zeros((4, 4)), self.np.eye(4)*float('nan')):
            with self.assertRaises(ValueError): pose(invalid)
        bad = dict(self.kwargs, sha256='wrong')
        with self.assertRaises(ValueError): KinematicChain(self.path, **bad)
        bad = dict(self.kwargs, joints=['j0'], signs=[1], offsets=[0])
        with self.assertRaises(ValueError): KinematicChain(self.path, **bad)

    def test_reachable_target_converges_and_each_step_is_bounded(self):
        solver = self.solver()
        q = self.np.array([.1]*7)
        target = self.chain.forward([.15, .2, .1, .15, .1, .2, .1])
        for _ in range(300):
            result = solver.step(q, target, .01)
            next_q = self.np.array(result['joints'])
            self.assertLessEqual(abs(next_q-q).max(), .00500001)
            q = next_q
        self.assertLess(result['position_error_m'], .002)
        self.assertLess(result['orientation_error_rad'], .01)

    def test_unreachable_and_singular_targets_are_finite_bounded_steps(self):
        solver = self.solver()
        q = self.np.zeros(7)
        target = self.np.eye(4); target[0, 3] = 10
        for _ in range(50):
            result = solver.step(q, target, .01)
            next_q = self.np.array(result['joints'])
            self.assertTrue(self.np.isfinite(next_q).all())
            self.assertLessEqual(abs(next_q-q).max(), .00500001)
            q = next_q
        self.assertGreater(result['position_error_m'], 1.)

    def test_faults_reject_instead_of_returning_a_command(self):
        solver = self.solver()
        for q, dt in (([0]*7, .1), ([0]*7, 0), ([float('nan')]*7, .01), ([2]*7, .01)):
            with self.assertRaises(ValueError): solver.step(q, self.np.eye(4), dt)

    def test_soft_boundary_holds_repeated_outward_goals_without_disabling(self):
        from openarm_runtime.cartesian import DifferentialIK
        margin = .02
        solver = DifferentialIK(self.chain, lower=[-.2]*7, upper=[.2]*7,
                                velocity=[.5]*7, posture=[.18]*7, command_margin_rad=margin)
        q = self.np.full(7, .18)
        target = self.chain.forward([.7]*7)
        for _ in range(100):
            result = solver.step(q, target, .05)
            next_q = self.np.asarray(result['joints'])
            self.assertTrue((next_q <= .18 + 1e-9).all())
            self.assertTrue((next_q >= -.18 - 1e-9).all())
            self.assertLessEqual(abs(next_q-q).max(), .02500001)
            q = next_q
        self.assertTrue(self.np.isfinite(q).all())
        self.assertTrue(result['boundary_joints'])
        with self.assertRaisesRegex(ValueError, 'outside physical limits'):
            solver.step([.201]*7, target, .05)

    def test_start_in_soft_margin_allows_only_hold_or_inward_without_jump(self):
        from openarm_runtime.cartesian import DifferentialIK
        solver = DifferentialIK(self.chain, lower=[-.2]*7, upper=[.2]*7,
                                velocity=[.5]*7, posture=[.195]*7, command_margin_rad=.02)
        q = self.np.full(7, .195)
        target = self.chain.forward([.7]*7)
        for _ in range(15):
            next_q = self.np.asarray(solver.step(q, target, .05)['joints'])
            self.assertTrue((next_q <= self.np.maximum(q, .18) + 1e-9).all())
            self.assertLessEqual(abs(next_q-q).max(), .02500001)
            q = next_q

    def test_command_margin_validation(self):
        from openarm_runtime.cartesian import DifferentialIK
        for margin in (-.01, .2, float('nan'), True):
            with self.assertRaises(ValueError):
                DifferentialIK(self.chain, lower=[-.2]*7, upper=[.2]*7,
                               velocity=[.5]*7, posture=[0]*7, command_margin_rad=margin)

    def test_soft_boundary_respects_negative_encoder_signs_and_offsets(self):
        from openarm_runtime.cartesian import DifferentialIK, KinematicChain
        chain = KinematicChain(self.path, **dict(self.kwargs, signs=[-1]*7, offsets=[.2]*7))
        solver = DifferentialIK(chain, lower=[-.2]*7, upper=[.2]*7,
                                velocity=[.5]*7, posture=[.18]*7, command_margin_rad=.02)
        q = self.np.full(7, .18)
        for _ in range(20):
            next_q = self.np.asarray(solver.step(q, chain.forward([.7]*7), .05)['joints'])
            self.assertTrue((abs(next_q) <= .18 + 1e-9).all())
            self.assertLessEqual(abs(next_q-q).max(), .02500001)
            q = next_q

    def test_encoder_order_sign_offsets_and_tcp(self):
        from openarm_runtime.cartesian import KinematicChain
        tcp = self.np.eye(4); tcp[2, 3] = .07
        names = list(reversed(self.kwargs['joints']))
        chain = KinematicChain(self.path, **dict(self.kwargs, joints=names,
                                signs=[-1]*7, offsets=[.2]*7, tcp=tcp))
        angles = self.np.arange(7)*.01
        expected = self.chain.forward(list(reversed(.2-angles))) @ tcp
        self.np.testing.assert_allclose(chain.forward(angles), expected, atol=1e-12)

    def test_limit_intersection_and_solver_failure(self):
        from unittest.mock import patch
        from openarm_runtime.cartesian import DifferentialIK
        with self.assertRaises(ValueError):
            DifferentialIK(self.chain, lower=[3]*7, upper=[4]*7,
                           velocity=[1]*7, posture=[.1]*7)
        solver = self.solver()
        with patch('pink.solve_ik', side_effect=RuntimeError('infeasible')):
            with self.assertRaisesRegex(RuntimeError, 'infeasible'):
                solver.step([.1]*7, self.np.eye(4), .01)
        with patch('pink.solve_ik', return_value=self.np.ones(7)*100):
            with self.assertRaisesRegex(RuntimeError, 'bounds'):
                solver.step([.1]*7, self.np.eye(4), .01)

    def test_virtual_leader_live_adapter_and_stale_feedback(self):
        from types import SimpleNamespace
        from openarm_runtime.cartesian_teleop import VirtualLeaderIK
        controller = VirtualLeaderIK.__new__(VirtualLeaderIK)
        controller.side = 'left'; controller.chain = self.chain; controller.speed = .25
        base = dict.fromkeys(range(1, 9), .1)
        controller.reset(base)
        states = {i: SimpleNamespace(position=.1, fresh=lambda *args: True) for i in base}
        targets, event = controller.step(base, states, .05)
        self.np.testing.assert_allclose(list(targets.values()), [.1]*8, atol=1e-8)
        self.assertEqual(event['target_semantics'], 'virtual_follower_fk_not_real_leader_tcp')
        states[1].fresh = lambda *args: False
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            controller.step(base, states, .05)

    def test_static_friction_does_not_reset_reference_and_blocked_arm_stops(self):
        from types import SimpleNamespace
        from openarm_runtime.cartesian_teleop import VirtualLeaderIK
        controller = VirtualLeaderIK.__new__(VirtualLeaderIK)
        controller.side = 'left'; controller.chain = self.chain; controller.speed = .25
        base = dict.fromkeys(range(1, 9), .1)
        controller.reset(base)
        states = {i: SimpleNamespace(position=.1, fresh=lambda *args: True) for i in base}
        requested = dict(enumerate([.8, -.5, .7, .8, .5, .3, .7, .1], 1))
        first, _ = controller.step(requested, states, .05)
        previous = dict(first)
        grew = False
        for _ in range(40):
            try:
                targets, event = controller.step(requested, states, .05)
            except RuntimeError as exc:
                self.assertIn('tracking limit', str(exc))
                self.assertTrue(grew, 'reference never progressed beyond first step')
                break
            self.assertLessEqual(max(abs(targets[i]-previous[i]) for i in range(1,8)), .01250001)
            self.assertLessEqual(max(abs(targets[i]-.1) for i in range(1,8)), .15000001)
            grew |= max(abs(targets[i]-.1) for i in range(1,8)) > .02
            previous = dict(targets)
        else:
            self.fail('blocked arm did not stop at bounded following error')

    def test_qp_roundoff_at_zero_is_normalized_not_real_out_of_bounds_feedback(self):
        from unittest.mock import patch
        from openarm_runtime.cartesian import DifferentialIK
        solver = DifferentialIK(self.chain, lower=[0]*7, upper=[1]*7,
                                velocity=[.25]*7, posture=[0]*7)
        target = self.chain.forward([0]*7)
        with patch('pink.solve_ik', return_value=self.np.array([0,0,0,-1e-16,0,0,0])):
            result = solver.step([0]*7, target, .05)
        self.assertEqual(result['joints'][3], 0.)
        solver.step(result['joints'], target, .05)
        with self.assertRaises(ValueError):
            solver.step([0,0,0,-.001,0,0,0], target, .05)
        with patch('pink.solve_ik', return_value=self.np.array([0,0,0,-.01,0,0,0])):
            with self.assertRaisesRegex(RuntimeError, 'bounds'):
                solver.step([0]*7, target, .05)


if __name__ == '__main__':
    unittest.main()
