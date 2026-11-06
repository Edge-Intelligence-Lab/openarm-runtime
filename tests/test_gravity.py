"""Analytic statics, firmware torque scaling and compensation lifecycle tests."""
import hashlib
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from openarm_runtime.gravity import GravityCompensation, GravityModel, load_gravity
from openarm_runtime.config import Profile
from openarm_runtime.arm import ArmHold
from openarm_runtime.telemetry import MotorState, MotorRange
from openarm_runtime.transport import ClassicBus


def fixture(root):
    text = '<robot name="analytic"><link name="base"/>'
    for i in range(7):
        text += f'''<link name="l{i}"><inertial><origin xyz="0.5 0 0"/>
        <mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
        </inertial></link><joint name="j{i}" type="revolute">
        <parent link="{('l'+str(i-1)) if i else 'base'}"/><child link="l{i}"/>
        <origin xyz="{1 if i else 0} 0 0"/><axis xyz="0 1 0"/>
        <limit lower="-3.14" upper="3.14" effort="500" velocity="10"/></joint>'''
    text += '</robot>'
    (root / "robot.urdf").write_text(text)
    return dict(urdf="robot.urdf", urdf_sha256=hashlib.sha256(text.encode()).hexdigest(),
                joints=[f"j{i}" for i in range(7)], signs=[1]*7, offsets_rad=[0]*7,
                gravity_m_s2=[0, 0, -9.81], locked_joints={}, mode="shadow",
                limits_nm=[10]*7, slew_nm_s=[5]*7)


def states(now, q=0):
    return {i: MotorState(i, q, 0, 0, 25, 25, 1, now) for i in range(1, 9)}


class ModelTests(unittest.TestCase):
    def setUp(self):
        try:
            import pinocchio
        except ImportError:
            self.skipTest("optional gravity extra")
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = fixture(self.root)

    def test_planar_chain_matches_independent_potential_gradient(self):
        model = GravityModel(self.config, self.root)
        def potential(q):
            angle = z = energy = 0.
            for i in range(7):
                if i:
                    z -= math.sin(angle)
                angle += q[i]
                energy += 9.81 * (z - .5 * math.sin(angle))
            return energy
        for q in ([0.]*8, [.12, -.3, .2, .1, -.4, .6, -.2, 0]):
            tau = model.torques(q)
            for i in range(7):
                a, b = list(q), list(q)
                a[i] += 1e-6; b[i] -= 1e-6
                self.assertAlmostEqual(tau[i], (potential(a)-potential(b))/2e-6, places=5)
            self.assertEqual(tau[7], 0)

    def test_motor_sign_offset_and_mounting_gravity(self):
        normal = GravityModel(self.config, self.root)
        config = self.config | {"signs": [-1]*7, "offsets_rad": [.2]*7}
        mapped = GravityModel(config, self.root)
        expected = normal.torques([.1]*7 + [0])
        actual = mapped.torques([.1]*8)
        for a, b in zip(actual[:7], expected[:7]):
            self.assertAlmostEqual(a, -b)
        inverted = GravityModel(self.config | {"gravity_m_s2": [0, 0, 9.81]}, self.root)
        self.assertAlmostEqual(inverted.torques([0]*8)[0], -normal.torques([0]*8)[0])

    def test_payload_increases_expected_support_torque(self):
        normal = GravityModel(self.config, self.root).torques([0]*8)
        loaded = GravityModel(self.config | {"payload": {"link": "l6", "mass_kg": 2, "com_m": [.5,0,0]}}, self.root)
        tau = loaded.torques([0]*8)
        self.assertAlmostEqual(tau[0] - normal[0], -2*9.81*6.5)
        self.assertAlmostEqual(tau[6] - normal[6], -2*9.81*.5)

    def test_invalid_model_configuration_fails_before_execution(self):
        for update in ({"urdf_sha256": "wrong"}, {"signs": [0]*7},
                       {"locked_joints": {"unknown": 0}}, {"gravity_m_s2": [0,0,0]},
                       {"joints": ["j0"]*7}, {"offsets_rad": [float("nan")]*7}):
            with self.subTest(update=update), self.assertRaises(ValueError):
                GravityModel(self.config | update, self.root)
        model = GravityModel(self.config, self.root)
        with self.assertRaises(ValueError): model.torques([4]*8)
        with self.assertRaises(ValueError): model.torques([float("nan")]*8)

    def test_profile_resolution_and_unknown_fields(self):
        (self.root/"gravity.json").write_text(json.dumps(self.config))
        profile = self.root/"machine.json"
        profile.write_text(json.dumps(dict(format="classic",bitrate=1000000,left="can1",right="can0",
                                          gravity={"right":"gravity.json"})))
        loaded = Profile.load(profile)
        self.assertIsNone(loaded.make_gravity("left"))
        self.assertEqual(loaded.make_gravity("right").mode, "shadow")
        (self.root/"gravity.json").write_text(json.dumps(self.config | {"typo":1}))
        with self.assertRaises(ValueError):load_gravity(self.root/"gravity.json")

    def test_episode_audit_uses_measured_pose_and_deduplicates(self):
        from openarm_runtime.gravity import diagnose_episode
        from openarm_policy import EpisodeRecorder, Observation, SensorSample
        model=GravityModel(self.config,self.root)
        c=GravityCompensation(model,mode="shadow",limits_nm=[10]*7,slew_nm_s=[5]*7)
        q=(.1,0.,0.,0.,0.,0.,0.,0.)
        g=model.torques(q)
        sample=SensorSample(7,10.,tuple((v,0.,t) for v,t in zip(q,g)))
        directory=self.root/"episode"
        with EpisodeRecorder(directory) as recorder:
            for sequence in (1,2):
                recorder.record({"event":"observation","observation":Observation(sequence,10.1,{"robot/right":(sample,)})})
        result=diagnose_episode(c,directory,"right")
        self.assertEqual(result["motors"][0]["samples"],1)
        self.assertAlmostEqual(result["motors"][0]["residual_rms_nm"],0)
        with self.assertRaises(ValueError):diagnose_episode(c,directory,"left")


class BilateralTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        single = fixture(self.root)
        text = (self.root/"robot.urdf").read_text()
        body = text.split('<link name="base"/>', 1)[1].removesuffix('</robot>')
        left = body
        right = body
        for i in range(7):
            left = left.replace(f'"j{i}"', f'"left_j{i}"').replace(f'"l{i}"', f'"left_l{i}"')
            right = right.replace(f'"j{i}"', f'"right_j{i}"').replace(f'"l{i}"', f'"right_l{i}"')
        text = '<robot name="dual"><link name="base"/>' + left + right + '</robot>'
        (self.root/"robot.urdf").write_text(text)
        arms = {}
        for side, other in (("left", "right"), ("right", "left")):
            arms[side] = {k: v for k, v in single.items() if k not in ("urdf", "urdf_sha256", "gravity_m_s2")}
            arms[side].update(joints=[f"{side}_j{i}" for i in range(7)],
                              locked_joints={f"{other}_j{i}": 0 for i in range(7)})
        self.bundle = dict(schema="openarm.gravity.bimanual.v1", urdf="robot.urdf",
                           urdf_sha256=hashlib.sha256(text.encode()).hexdigest(),
                           gravity_m_s2=[0, 0, -9.81], arms=arms)
        self.path = self.root/"gravity.json"

    def save(self):
        self.path.write_text(json.dumps(self.bundle))

    def test_schema_side_and_overlap_fail_without_optional_dependencies(self):
        from openarm_runtime.gravity import select_gravity_config
        for side in (None, "both", "typo"):
            with self.assertRaises(ValueError): select_gravity_config(self.bundle, side)
        self.bundle["arms"]["right"]["joints"] = self.bundle["arms"]["left"]["joints"]
        with self.assertRaises(ValueError): select_gravity_config(self.bundle, "left")

    def test_branch_order_and_foreign_payload_rejected(self):
        from openarm_runtime.gravity import select_gravity_config, validate_arm_branch
        self.bundle["arms"]["left"]["joints"].reverse()
        with self.assertRaises(ValueError):
            validate_arm_branch(select_gravity_config(self.bundle, "left"), self.bundle, "left", self.root)
        self.bundle["arms"]["left"]["joints"].reverse()
        self.bundle["arms"]["left"]["payload"] = dict(link="right_l6", mass_kg=1, com_m=[0, 0, 0])
        with self.assertRaises(ValueError):
            validate_arm_branch(select_gravity_config(self.bundle, "right"), self.bundle, "right", self.root)

    def test_both_sides_identical_statics_independent_state_and_profile_routing(self):
        try:
            import pinocchio
        except ImportError:
            self.skipTest("optional gravity extra")
        self.save()
        profile = Profile("can1", "can0", gravity={s: str(self.path) for s in ("left", "right")})
        left, right = profile.make_gravity("left"), profile.make_gravity("right")
        q = [.1]*7+[0]
        self.assertEqual(left.model.torques(q), right.model.torques(q))
        before = right.model.torques(q)
        left.model.torques([.2]*7+[0])
        self.assertEqual(before, right.model.torques(q))
        self.assertIsNot(left.model.data, right.model.data)
        self.assertEqual(left.metadata()["side"], "left")
        self.assertEqual(right.metadata()["side"], "right")
        self.assertEqual(left.metadata()["profile_sha256"], right.metadata()["profile_sha256"])
        from openarm_runtime.gravity import diagnose_episode
        with self.assertRaisesRegex(ValueError, "side does not match"):
            diagnose_episode(left, self.root/"nonexistent", "right")


class CompensationTests(unittest.TestCase):
    def compensation(self, mode="active"):
        model = Mock(metadata={})
        model.torques.return_value = (20.,)*7 + (0.,)
        return GravityCompensation(model, mode=mode, limits_nm=[3]*7,
                                   slew_nm_s=[1]*7, ramp_s=1)

    def test_ramp_slew_caps_and_gripper_zero(self):
        c = self.compensation()
        self.assertEqual(c.update(states(10),10)[1],0)
        for i in range(1,21):
            now=10+i*.1
            self.assertAlmostEqual(c.update(states(now),now)[1],i*.1)
        self.assertEqual(c.last["saturated"], list(range(1,8)))
        self.assertEqual(c.applied[7],0)
        self.assertLessEqual(max(c.applied),3)

    def test_shadow_predicts_but_sends_zero(self):
        c=self.compensation("shadow")
        c.update(states(10),10)
        result=c.update(states(10.1),10.1)
        self.assertFalse(any(result.values()))
        self.assertEqual(c.last["raw_nm"][0],20)

    def test_stale_feedback_and_update_gap_reject(self):
        c=self.compensation()
        with self.assertRaises(RuntimeError):c.update(states(10),11)
        c.update(states(10),10)
        with self.assertRaises(RuntimeError):c.update(states(11),11)

    def test_firmware_cap_checked(self):
        c=self.compensation()
        ranges={i:MotorRange(12.5,30,3) for i in range(1,9)}
        with self.assertRaises(ValueError):c.preflight(states(10),ranges)

    def test_model_failure_in_hold_disables_all_motors(self):
        from test_arm import FakeBus
        bus=FakeBus()
        c=self.compensation()
        exchange=bus.exchange
        bus.exchange=lambda targets,kp,kd,ff: exchange(targets,kp,kd)
        with patch("openarm_runtime.arm.time.sleep"):
            with self.assertRaisesRegex(RuntimeError,"model failed"):
                with ArmHold(bus,lambda:False,Mock(),Mock(),gravity=c,tracking=True) as arm:
                    c.model.torques.side_effect=RuntimeError("model failed")
                    arm.track(dict.fromkeys(range(1,9),0))
        self.assertFalse(bus.enabled)

    def test_model_preflight_failure_never_enables(self):
        from test_arm import FakeBus
        bus=FakeBus();c=self.compensation()
        c.model.torques.side_effect=ValueError("bad mapping")
        with self.assertRaises(ValueError):
            ArmHold(bus,lambda:False,Mock(),Mock(),gravity=c).__enter__()
        self.assertFalse(any(action=="enable" for _,action in bus.commands))


class TorqueEncodingTests(unittest.TestCase):
    def test_firmware_range_scaling_and_zero_legacy_packet(self):
        # Independent encoder stub follows the documented 12-bit MIT torque field.
        def encode(motor, params):
            kp,kd,q,v,t=params
            bits=int((t+10)/20*4095)
            return SimpleNamespace(send_can_id=5,data=bytes([0]*6+[(bits>>8)&15,bits&255]))
        bus=ClassicBus.__new__(ClassicBus)
        bus.ranges={5:MotorRange(12.5,30,3)};bus.motors={5:object()};bus.send=Mock()
        bus.sdk=SimpleNamespace(MotorType=SimpleNamespace(DM4310=1),
             Motor=SimpleNamespace(get_limit_param=lambda _:SimpleNamespace(tMax=10)),
             MITParam=lambda *args:args,CanPacketEncoder=SimpleNamespace(create_mit_control_command=encode))
        for torque in (-3,-1.2,0,1.2,3):
            bus.command(5,"position",0,24,.3,torque)
            d=bus.send.call_args.args[1];bits=((d[6]&15)<<8)|d[7]
            self.assertLessEqual(abs(bits/4095*6-3-torque),6/4095+1e-9)
            if torque==0:self.assertEqual(bits,2047)
        for torque in (3.1,float("nan")):
            with self.assertRaises(ValueError):bus.command(5,"position",feedforward=torque)


if __name__ == "__main__":
    unittest.main()
