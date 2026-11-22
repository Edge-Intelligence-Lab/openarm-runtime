"""Read-only MuJoCo telemetry for the OpenArm grasp practice scene."""
import math
import time

from .grasp import SIDES, TOOL_CENTER
from .practice_scenes import GRASPABLE_OBJECTS


class GraspTelemetry:
    def __init__(self, world):
        self.world = world
        model = world.model
        self.tools = {side: model.body(f'openarm_{side}_ee_base_link').id for side in SIDES}
        self.pads = {model.geom(f'practice_{side}_{jaw}_pad').id: (side, jaw)
                     for side in SIDES for jaw in ('inner', 'outer')}
        self.objects = {model.body(name).id for name in GRASPABLE_OBJECTS}

    def snapshot(self):
        world = self.world
        with world.lock:
            if world.error:
                raise RuntimeError(world.error)
            press = self._pad_forces()
            arms = {}
            for side in SIDES:
                positions, velocities = world.positions(side)
                tool = world.data.body(self.tools[side])
                tip = tool.xpos + tool.xmat.reshape(3, 3) @ TOOL_CENTER
                jaw_forces = [values for (hand, _), values in press.items() if hand == side]
                arms[side] = {
                    'joint_deg': [math.degrees(value) for value in positions[:7]],
                    'joint_velocity_deg_s': [math.degrees(value) for value in velocities[:7]],
                    # Positive angle of one jaw relative to closed, averaged over both jaws.
                    'jaw_opening_deg': max(0., math.degrees(positions[7])),
                    # Require both jaws on the same object; do not double-count the opposing load.
                    'grip_force_n': max((min(pair) for pair in jaw_forces), default=0.),
                    'tcp_m': tip.tolist(),
                }
            return {'timestamp_ms': time.time_ns() // 1_000_000,
                    'scene': world.practice_scenes.current,
                    'sim_time_s': float(world.data.time), 'arms': arms}

    def _pad_forces(self):
        world = self.world
        forces = {}
        contact_force = world.np.zeros(6)
        for index in range(world.data.ncon):
            contact = world.data.contact[index]
            pad, other = contact.geom1, contact.geom2
            if pad not in self.pads:
                pad, other = other, pad
            if pad not in self.pads or other < 0:
                continue
            object_id = int(world.model.geom_bodyid[other])
            if object_id not in self.objects:
                continue
            side, jaw = self.pads[pad]
            world.mj.mj_contactForce(world.model, world.data, index, contact_force)
            pair = forces.setdefault((side, object_id), [0., 0.])
            pair[jaw == 'outer'] += max(0., float(contact_force[0]))
        return forces
