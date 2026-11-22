"""Optional OpenArm grasp practice scene and matching simulated kinematics."""
import hashlib
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

SIDES = ('left', 'right')
OPEN_FINGERS = .35
TOOL_CENTER = (.005, 0., -.154)
SHARED_TRAVEL_M = .28
BEAM_X = .217
BEAM_HANDLES_Y = .130
BEAM_HANDLE_Z = -.525


def initial_angles(side):
    sign = 1 if side == 'left' else -1
    return [0., -.18 * sign, 0., .25, 0., -.25 * sign, -.18 * sign, OPEN_FINGERS]


def add_practice_scene(root, world):
    from .practice_scenes import add_task_props

    for body in world.iter('body'):
        if body.get('name', '').startswith('openarm_'):
            for geom in body.findall('geom'):
                geom.set('contype', '0')
                geom.set('conaffinity', '0')
    asset = root.find('asset')
    if asset is None:
        asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'texture', name='practice_wood', type='2d', builtin='checker',
                  width='64', height='64', rgb1='.54 .27 .09', rgb2='.69 .40 .17')
    ET.SubElement(asset, 'material', name='practice_wood', texture='practice_wood',
                  texuniform='false', texrepeat='2 2', specular='.13')
    ET.SubElement(world, 'geom', name='practice_table', type='box', pos='.28 0 -.588',
                  size='.18 .42 .022', rgba='.21 .29 .33 1', contype='8', conaffinity='4')
    beam = ET.SubElement(world, 'body', name='shared_wood_beam', pos=f'{BEAM_X} 0 -.557')
    ET.SubElement(beam, 'freejoint', name='shared_wood_beam_joint')
    ET.SubElement(beam, 'geom', name='shared_wood_beam_base', type='box', size='.027 .165 .009',
                  material='practice_wood', mass='.065', contype='4', conaffinity='14',
                  condim='4', friction='1.4 .012 .002')
    for side, y in zip(SIDES, (.26756, -.26756), strict=True):
        handle_y = BEAM_HANDLES_Y if side == 'left' else -BEAM_HANDLES_Y
        ET.SubElement(beam, 'geom', name=f'shared_wood_beam_{side}_handle', type='box',
                      pos=f'0 {handle_y} {BEAM_HANDLE_Z + .557:.3f}', size='.022 .023 .023',
                      rgba='.89 .55 .20 1', mass='.014', contype='4', conaffinity='14',
                      condim='4', friction='1.8 .018 .002')
        block = ET.SubElement(world, 'body', name=f'wood_block_{side}', pos=f'.147 {y} -.543')
        ET.SubElement(block, 'freejoint', name=f'wood_block_{side}_joint')
        ET.SubElement(block, 'geom', name=f'wood_block_{side}_geom', type='box',
                      size='.023 .023 .023', material='practice_wood', mass='.055',
                      contype='4', conaffinity='14', condim='4', friction='1.4 .012 .002')
        for jaw in ('inner', 'outer'):
            finger_name = f'openarm_{side}_ee_{jaw}_finger'
            finger = next(body for body in world.iter('body') if body.get('name') == finger_name)
            ET.SubElement(finger, 'geom', name=f'practice_{side}_{jaw}_pad', type='box',
                          pos='.006 0 -.085', size='.015 .006 .016',
                          contype='2', conaffinity='6', condim='4', mass='.005',
                          friction='1.7 .02 .002', rgba='.20 .21 .22 1')
    add_task_props(world, asset)


def shared_beam_grasped(world):
    """Require both pads on both sides to touch the same practice object."""
    model = world.model
    beam = int(model.body('shared_wood_beam').id)
    pads = {int(model.geom(f'practice_{side}_{jaw}_pad').id)
            for side in SIDES for jaw in ('inner', 'outer')}
    with world.lock:
        touching = set()
        for contact in world.data.contact:
            a, b = contact.geom1, contact.geom2
            if model.geom_bodyid[a] == beam:
                touching.add(b)
            if model.geom_bodyid[b] == beam:
                touching.add(a)
        return pads <= touching


def practice_chain(model, side):
    from scipy.spatial.transform import Rotation
    import numpy as np
    from openarm_runtime.cartesian import KinematicChain

    def vector(values):
        return ' '.join(map(str, values))

    names = [f'openarm_{side}_joint{i}' for i in range(1, 8)]
    robot = ET.Element('robot', name=f'openarm_{side}_sim')
    ET.SubElement(robot, 'link', name='world')
    mount = model.body(f'openarm_{side}_base_link')
    for body, joint in [(mount, None), *[(model.body(int(model.joint(n).bodyid[0])), model.joint(n)) for n in names]]:
        ET.SubElement(robot, 'link', name=body.name)
        node = ET.SubElement(robot, 'joint', name=joint.name if joint else f'{side}_mount',
                             type='revolute' if joint else 'fixed')
        ET.SubElement(node, 'parent', link=model.body(int(body.parentid[0])).name)
        ET.SubElement(node, 'child', link=body.name)
        ET.SubElement(node, 'origin', xyz=vector(body.pos),
                      rpy=vector(Rotation.from_quat(body.quat, scalar_first=True).as_euler('xyz')))
        if joint:
            if np.linalg.norm(joint.pos) > 1e-9:
                raise ValueError(f'Unsupported offset pivot: {joint.name}')
            ET.SubElement(node, 'axis', xyz=vector(joint.axis))
            ET.SubElement(node, 'limit', lower=str(joint.range[0]), upper=str(joint.range[1]),
                          effort='40', velocity='1')
    source = ET.tostring(robot)
    tcp = np.eye(4)
    tcp[:3, 3] = TOOL_CENTER
    with TemporaryDirectory() as tmp:
        path = Path(tmp) / 'arm.urdf'
        path.write_bytes(source)
        return KinematicChain(path, sha256=hashlib.sha256(source).hexdigest(), joints=names,
                              signs=[1] * 7, offsets=[0] * 7, locked={},
                              frame=f'openarm_{side}_ee_base_link', tcp=tcp)
