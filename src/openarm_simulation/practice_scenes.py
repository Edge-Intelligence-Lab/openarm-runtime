"""Task geometry and selection for OpenArm's MuJoCo practice table."""
import math
import xml.etree.ElementTree as ET

SCENES = {
    'wood': ('shared_wood_beam', 'wood_block_left', 'wood_block_right'),
    'cylinder': ('fallen_cylinder', 'cylinder_stand'),
    'tube': ('insertion_tube', 'tube_socket'),
    'key': ('practice_key', 'key_rest', 'key_lock'),
}
GRASPABLE_OBJECTS = (
    *SCENES['wood'], 'fallen_cylinder', 'insertion_tube', 'practice_key',
)
CLOSEUP_WIDTH, CLOSEUP_HEIGHT, CLOSEUP_FPS = 640, 480, 16
# Target, horizontal distance, height above target, and vertical field of view.
CLOSEUP_VIEWS = {
    'wood': ((.23, 0, -.54), .47, .77, 41),
    'cylinder': ((.245, -.265, -.47), .37, .42, 34),
    'tube': ((.225, -.260, -.50), .29, .37, 33),
    'key': ((.216, -.270, -.508), .215, .30, 37),
}


def closeup_cameras(overview):
    """Use the overview heading so screen-right agrees with teleoperation."""
    dx, dy = (overview['target_m'][i] - overview['position_m'][i] for i in range(2))
    length = math.hypot(dx, dy)
    if not length:
        raise ValueError('Overview camera needs a horizontal viewing direction')
    forward_x, forward_y = dx / length, dy / length
    for scene, (target, distance, height, fovy) in CLOSEUP_VIEWS.items():
        x, y, z = target
        position = (x - distance * forward_x, y - distance * forward_y, z + height)
        yield f'closeup_{scene}', position, target, fovy


def box(body, name, pos, size, rgba, *, mass=None):
    attrs = dict(name=name, type='box', pos=pos, size=size, rgba=rgba,
                 contype='4', conaffinity='14', condim='4', friction='1.2 .012 .002')
    if mass is not None:
        attrs['mass'] = str(mass)
    return ET.SubElement(body, 'geom', **attrs)


def ring(body, name, inner, thickness, bottom, height, color, *, mass=None):
    count = 12
    middle = inner + thickness / 2
    tangent = middle * math.tan(math.pi / count) * 1.02
    for index in range(count):
        angle = index * 2 * math.pi / count
        piece = box(body, f'{name}_{index}',
                    f'{middle * math.cos(angle)} {middle * math.sin(angle)} {bottom + height / 2}',
                    f'{thickness / 2} {tangent} {height / 2}', color,
                    mass=None if mass is None else mass / count)
        piece.set('euler', f'0 0 {angle}')


def tube_lead_in(socket, assets):
    count = 12
    outside = .059
    tangent = math.tan(math.pi / count) * 1.025
    vertices = []
    for z, opening in ((.010, .033), (.027, .052)):
        for radius in (opening, outside):
            vertices.extend((radius, side * radius * tangent, z) for side in (-1, 1))
    ET.SubElement(assets, 'mesh', name='tube_lead_in_segment',
                  vertex=' '.join(str(component) for vertex in vertices for component in vertex))
    for index in range(count):
        ET.SubElement(socket, 'geom', name=f'tube_lead_in_{index}', type='mesh',
                      mesh='tube_lead_in_segment', euler=f'0 0 {2 * math.pi * index / count}',
                      rgba='.42 .63 .77 1', contype='4', conaffinity='14', condim='4',
                      friction='.35 .003 .0005')


def slot(body, name, bottom, height, color, *, mass=None):
    """A rectangular key channel, built from four walls instead of a solid mesh."""
    for suffix, center, size in (
        ('east', '.030 0', '.009 .042'),
        ('west', '-.030 0', '.009 .042'),
        ('north', '0 .02425', '.021 .01775'),
        ('south', '0 -.02425', '.021 .01775'),
    ):
        box(body, f'{name}_{suffix}', f'{center} {bottom + height / 2}',
            f'{size} {height / 2}', color, mass=None if mass is None else mass / 4)


def free_body(world, name, pos, **attrs):
    body = ET.SubElement(world, 'body', name=name, pos=pos, **attrs)
    ET.SubElement(body, 'freejoint', name=f'{name}_joint')
    return body


def add_task_props(world, assets):
    cylinder = free_body(world, 'fallen_cylinder', '.185 -.226 -.543',
                         quat='.70710678 0 .70710678 0')
    ET.SubElement(cylinder, 'geom', name='fallen_cylinder_main', type='cylinder',
                  size='.023 .046', rgba='.95 .64 .22 1', mass='.075',
                  contype='4', conaffinity='14', condim='4',
                  friction='1.3 .018 .008')
    ET.SubElement(cylinder, 'geom', name='fallen_cylinder_end', type='cylinder',
                  pos='0 0 .040', size='.0231 .006', rgba='.97 .83 .40 1',
                  contype='0', conaffinity='0', mass='0')
    stand = ET.SubElement(world, 'body', name='cylinder_stand', pos='.280 -.309 -.566')
    box(stand, 'cylinder_stand_base', '0 0 .089', '.040 .040 .089', '.30 .57 .69 1')

    tube = free_body(world, 'insertion_tube', '.157 -.260 -.566')
    ring(tube, 'moving_tube', .010, .010, 0., .075, '.27 .78 .94 1', mass=.08)
    socket = ET.SubElement(world, 'body', name='tube_socket', pos='.285 -.260 -.566')
    ring(socket, 'tube_socket_wall', .033, .016, .004, .006, '.33 .48 .64 1')
    tube_lead_in(socket, assets)
    ET.SubElement(socket, 'geom', name='tube_socket_floor', type='cylinder',
                  pos='0 0 .002', size='.038 .002', rgba='.23 .34 .44 1',
                  contype='4', conaffinity='14', condim='4')

    key = free_body(world, 'practice_key', '.153 -.272 -.546')
    box(key, 'key_blade', '0 0 .025', '.010 .0036 .025', '.84 .70 .34 1', mass=.021)
    box(key, 'key_tooth', '.013 0 .011', '.004 .0036 .008', '.91 .77 .42 1', mass=.004)
    box(key, 'key_bow', '0 0 .062', '.022 .022 .012', '.98 .63 .16 1', mass=.033)
    rest = ET.SubElement(world, 'body', name='key_rest', pos='.153 -.272 -.566')
    slot(rest, 'key_rest_wall', .018, .019, '.32 .47 .53 1')
    box(rest, 'key_rest_floor', '0 0 .010', '.039 .042 .010', '.32 .47 .53 1')
    lock = ET.SubElement(world, 'body', name='key_lock', pos='.256 -.269 -.566')
    box(lock, 'lock_housing', '0 0 .010', '.052 .056 .010', '.26 .37 .46 1')
    plug = ET.SubElement(lock, 'body', name='key_lock_plug', pos='0 0 .022')
    ET.SubElement(plug, 'joint', name='key_lock_turn', type='hinge', axis='0 0 1',
                  limited='true', range='-1.75 1.75', damping='.08', frictionloss='.012',
                  armature='.001')
    slot(plug, 'lock_slot', 0., .024, '.52 .68 .73 1', mass=.08)
    box(plug, 'lock_position_mark', '.030 0 .025', '.006 .010 .001', '.95 .66 .22 1', mass=0)


class PracticeScenes:
    def __init__(self, world):
        self.world = world
        self.current = 'wood'
        self.groups = {name: self._build_group(bodies) for name, bodies in SCENES.items()}
        self.inactive = ()
        self.reset('wood')

    def _build_group(self, bodies):
        model, np = self.world.model, self.world.np
        roots = {int(model.body(name).id) for name in bodies}
        members = set(roots)
        for body_id in range(1, model.nbody):
            if int(model.body_parentid[body_id]) in members:
                members.add(body_id)
        geoms = np.array([i for i in range(model.ngeom) if model.geom_bodyid[i] in members], dtype=int)
        joints = []
        for i in range(model.njnt):
            if model.jnt_bodyid[i] not in members:
                continue
            free = model.jnt_type[i] == self.world.mj.mjtJoint.mjJNT_FREE
            q, v = int(model.jnt_qposadr[i]), int(model.jnt_dofadr[i])
            joints.append((slice(q, q + (7 if free else 1)), slice(v, v + (6 if free else 1))))
        return geoms, model.geom_rgba[geoms, 3].copy(), model.geom_contype[geoms].copy(), \
            model.geom_conaffinity[geoms].copy(), joints

    def reset(self, scene):
        if scene not in self.groups:
            raise ValueError('Unknown PICO practice scene')
        model, data = self.world.model, self.world.data
        inactive = []
        for name, (geoms, alpha, contype, conaffinity, joints) in self.groups.items():
            active = name == scene
            model.geom_rgba[geoms, 3] = alpha if active else 0.
            model.geom_contype[geoms] = contype if active else 0
            model.geom_conaffinity[geoms] = conaffinity if active else 0
            for position, velocity in joints:
                data.qpos[position] = model.qpos0[position]
                data.qvel[velocity] = 0.
                if not active:
                    inactive.append((position, velocity))
        self.inactive = tuple(inactive)
        self.current = scene

    def hold_inactive(self):
        model, data = self.world.model, self.world.data
        for position, velocity in self.inactive:
            data.qpos[position] = model.qpos0[position]
            data.qvel[velocity] = 0.
