"""Explicit virtual-leader FK -> constrained follower IK, no hardware ownership."""
import json
import math
from pathlib import Path
import time

from .joints import JOINT_LIMITS


class VirtualLeaderIK:
    def __init__(self, config, side):
        from .cartesian import KinematicChain
        self.side = side
        options = dict(config[side])
        self.speed = options.pop("velocity_rad_s", .25)
        if not .01 <= self.speed <= .5:
            raise ValueError("Cartesian velocity must be in [.01, .5] rad/s")
        self.chain = KinematicChain(**options)
        if len(self.chain.indices) != 7:
            raise ValueError("Cartesian follower needs seven joints")
        self.solver = None
        self.metadata = dict(mode="virtual_leader_fk_constrained_ik", real_leader_fk=False,
                             model=self.chain.metadata, velocity_rad_s=self.speed)

    def reset(self, base):
        from .cartesian import DifferentialIK
        bounds = JOINT_LIMITS[self.side][:7]
        self.solver = DifferentialIK(self.chain,
            lower=[math.radians(v[0]) for v in bounds],
            upper=[math.radians(v[1]) for v in bounds],
            velocity=[self.speed]*7, posture=[base[i] for i in range(1, 8)])
        self.reference = dict(base)

    def step(self, virtual_joints, states, period):
        if self.solver is None:
            raise RuntimeError("Cartesian takeover not initialized")
        now = time.monotonic()
        for state in states.values():
            if not state.fresh(now, .15):
                raise RuntimeError("Cartesian feedback stale")
        measured = [states[i].position for i in range(1, 8)]
        if any(not lo <= q <= hi for q, lo, hi in zip(measured, self.solver.lower, self.solver.upper)):
            raise RuntimeError("Cartesian measured angles outside physical limits")
        requested = self.chain.forward([virtual_joints[i] for i in range(1, 8)])
        measured_tcp = self.chain.forward(measured)
        # Integrate the commanded trajectory, not a fresh one-step offset from
        # measured feedback. Otherwise static friction can permanently consume
        # the entire velocity-limited step. Bound following error independently
        # so a blocked arm cannot accumulate an unbounded pending trajectory.
        tracking_limit = .15  # radians, not a physical workspace restriction
        if max(abs(self.reference[i] - states[i].position) for i in range(1, 8)) > tracking_limit:
            raise RuntimeError("Cartesian tracking error >0.15rad; stop and re-clutch")
        result = self.solver.step([self.reference[i] for i in range(1, 8)], requested, period)
        if time.monotonic() - now > period:
            raise RuntimeError("Cartesian solve deadline missed")
        targets = {i: result['joints'][i-1] for i in range(1, 8)}
        # The gripper is independent of six-DoF TCP IK, with its own slew bound.
        grip = self.reference[8]
        targets[8] = grip + max(-.5*period, min(.5*period, virtual_joints[8]-grip))
        if (max(abs(targets[i] - states[i].position) for i in range(1, 8)) > tracking_limit
                or abs(targets[8] - states[8].position) > .20):
            raise RuntimeError("Cartesian commanded lead exceeded tracking limit; stop and re-clutch")
        # The solver residual is for the commanded configuration. Actual error
        # remains separately observable and must not be confused with that value.
        import numpy as np
        import pinocchio as pin
        actual_error = pin.log6(pin.SE3(measured_tcp).inverse() * pin.SE3(requested)).vector
        event = dict(requested_tcp_to_base=requested.tolist(),
                     measured_tcp_to_base=measured_tcp.tolist(), ik=result,
                     measured_position_error_m=float(np.linalg.norm(measured_tcp[:3, 3]-requested[:3, 3])),
                     measured_orientation_error_rad=float(np.linalg.norm(actual_error[3:])),
                     reference_lead_rad={i: targets[i]-states[i].position for i in targets},
                     target_semantics="virtual_follower_fk_not_real_leader_tcp")
        self.reference = dict(targets)
        return targets, event


def load_factory(path):
    config = json.loads(Path(path).read_text())
    for side, options in config.items():
        if side not in ("left", "right"):
            raise ValueError("Cartesian profile accepts left/right only")
        options['urdf'] = str((Path(path).resolve().parent / options['urdf']).resolve())
    return lambda side: VirtualLeaderIK(config, side)
