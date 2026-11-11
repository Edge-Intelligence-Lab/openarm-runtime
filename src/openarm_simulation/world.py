"""Single-owner MuJoCo physics and rendered cameras, never hardware transports.

The board is an explicitly ideal rigid mounting fixture, not a validated friction
grasp. Disabled/uncontrolled joints are supported at their last pose. These
assumptions are recorded and shown in the UI; do not call this a calibrated twin.
"""
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import uuid
import xml.etree.ElementTree as ET


def load_profile(path):
    path=Path(path).expanduser().resolve()
    data=json.loads(path.read_text())
    required={'schema','calibration_profile','geometry','output_root','initial_rad','cameras','tool_from_board','video_port'}
    if not isinstance(data,dict) or required-set(data) or set(data)-required-{'board_contact_geoms'} or data['schema']!='openarm.simulation.v1':
        raise ValueError('Invalid explicit simulation profile')
    for key in ('calibration_profile','output_root'):
        data[key]=str((path.parent/Path(data[key]).expanduser()).resolve())
    data['geometry']['model_xml']=str((path.parent/Path(data['geometry']['model_xml']).expanduser()).resolve())
    from openarm_runtime.stationary_calibration import joint_vector
    from openarm_runtime.cartesian import pose
    from openarm_platform.config import valid_port
    data['initial_rad']=joint_vector(data['initial_rad'])
    pose(data['tool_from_board']);valid_port(data['video_port'])
    contacts=data.get('board_contact_geoms',[])
    if (not isinstance(contacts,list) or len(contacts)>128
            or any(not isinstance(n,str) or not n for n in contacts)
            or len(set(contacts))!=len(contacts)):
        raise ValueError('Board contact geometry must be a bounded unique name list')
    if not isinstance(data['cameras'],dict) or 'head' not in data['cameras'] or 'overview' not in data['cameras']:
        raise ValueError('Simulation requires head and overview cameras')
    from openarm_video.camera import CameraConfig
    for name,c in data['cameras'].items():
        if set(c)!={'position_m','target_m','fovy_deg','width','height','fps'}:
            raise ValueError('Invalid simulator camera fields')
        CameraConfig(name,width=c['width'],height=c['height'],fps=c['fps'])
        if max(c['width'],c['height'])>1920 or not 1<=c['fps']<=30 or not 20<=c['fovy_deg']<=100:
            raise ValueError('Simulator camera exceeds bounded render mode')
    return data


def build_model(setup,config,*,board_visual_mesh=False):
    import cv2
    import numpy as np
    import mujoco as mj
    from scipy.spatial.transform import Rotation
    collision=config['geometry']
    source=Path(collision['model_xml'])
    if hashlib.sha256(source.read_bytes()).hexdigest()!=collision['model_sha256']:
        raise ValueError('Simulation geometry source hash mismatch')
    root=ET.fromstring(source.read_bytes())
    if root.findall('include') or root.findall('.//attach'):
        raise ValueError('Simulation requires an expanded pinned scene')
    compiler=root.find('compiler')
    if compiler is None:compiler=ET.SubElement(root,'compiler')
    compiler.set('meshdir',str((source.parent/compiler.get('meshdir','.')).resolve()))
    compiler.set('texturedir',str((source.parent/compiler.get('texturedir','.')).resolve()))
    compiler.set('angle','radian')
    world=root.find('worldbody')
    roots=set(collision['robot_roots'])
    kept={b.get('name') for b in world.findall('body') if b.get('name') in roots}
    if kept!=roots:raise ValueError('Simulation robot roots must be direct world bodies')
    for element in list(world):
        if element.tag=='body' and element.get('name') in roots:continue
        world.remove(element)  # Only this private parsed scene; no table or task object.
    for actuator in root.findall('actuator'):root.remove(actuator)
    for keyframe in root.findall('keyframe'):root.remove(keyframe)
    for camera in root.findall('.//camera'):
        for parent in root.iter():
            if camera in list(parent):parent.remove(camera);break
    option=root.find('option')
    if option is None:option=ET.SubElement(root,'option')
    option.set('timestep','.001');option.set('integrator','implicitfast')
    option.set('gravity','0 0 -9.81')
    visual=root.find('visual')
    if visual is None:visual=ET.SubElement(root,'visual')
    global_options=visual.find('global')
    if global_options is None:global_options=ET.SubElement(visual,'global')
    global_options.set('offwidth',str(max(c['width'] for c in config['cameras'].values())))
    global_options.set('offheight',str(max(c['height'] for c in config['cameras'].values())))
    ET.SubElement(world,'light',pos='0 0 3',dir='0 0 -1',diffuse='.8 .8 .8',ambient='.4 .4 .4')
    for name,c in config['cameras'].items():
        position=np.array(c['position_m'],dtype=float);target=np.array(c['target_m'],dtype=float)
        if position.shape!=(3,) or target.shape!=(3,) or not np.isfinite(position).all() or not np.isfinite(target).all():
            raise ValueError('Finite simulator camera placement required')
        forward=target-position;forward/=np.linalg.norm(forward)
        right=np.cross(forward,[0.,0.,1.]);right/=np.linalg.norm(right)
        up=np.cross(right,forward)
        if not np.isfinite(right).all():raise ValueError('Camera look direction cannot be parallel to up')
        ET.SubElement(world,'camera',name='sim_'+name,pos=' '.join(map(str,position)),
            xyaxes=' '.join(map(str,list(right)+list(up))),fovy=str(c['fovy_deg']))
    spec=setup['board_spec'];nx,ny=spec['squares_x'],spec['squares_y']
    w,h=nx*spec['square_size_m'],ny*spec['square_size_m']
    charuco=cv2.aruco.CharucoBoard((nx,ny),spec['square_size_m'],spec['marker_size_m'],
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,spec['dictionary'])))
    texture=charuco.generateImage((1200,round(1200*ny/nx)))
    ok,png=cv2.imencode('.png',texture)
    if not ok:raise RuntimeError('Cannot encode simulator board texture')
    asset=root.find('asset')
    if asset is None:asset=ET.SubElement(root,'asset')
    ET.SubElement(asset,'texture',name='sim_board_texture',type='2d',file='sim-charuco.png')
    ET.SubElement(asset,'material',name='sim_board_material',texture='sim_board_texture',
                  texuniform='false',reflectance='0',specular='0',shininess='0',emission='.3')
    tool=next((b for b in world.iter('body') if b.get('name')==collision['tool_body']),None)
    if tool is None:raise ValueError('Missing simulation tool body')
    T=np.array(config['tool_from_board'])
    quat=Rotation.from_matrix(T[:3,:3]).as_quat(scalar_first=True)
    board=ET.SubElement(tool,'body',name='sim_board_fixture',pos=' '.join(map(str,T[:3,3])),
                        quat=' '.join(map(str,quat)))
    if board_visual_mesh:
        # Explicit board-XY to texture-UV mapping. MuJoCo's automatic box UVs
        # mirror indexed marker faces; unsuitable for calibrated image replay.
        vertices=[[0,0,0],[w,0,0],[w,h,0],[0,h,0],
                  [0,0,.003],[w,0,.003],[w,h,.003],[0,h,.003]]
        uv=[[0,0],[1,0],[1,1],[0,1]]*2
        faces=[[0,2,1],[0,3,2],[4,5,6],[4,6,7],
               [0,1,5],[0,5,4],[1,2,6],[1,6,5],
               [2,3,7],[2,7,6],[3,0,4],[3,4,7]]
        flat=lambda values:' '.join(str(v) for row in values for v in row)
        ET.SubElement(asset,'mesh',name='sim_board_mesh',vertex=flat(vertices),
            texcoord=flat(uv),face=flat(faces))
        ET.SubElement(board,'geom',name='sim_board',type='mesh',mesh='sim_board_mesh',
            material='sim_board_material',rgba='1 1 1 0',contype='0',conaffinity='0',density='0',group='2')
    else:
        ET.SubElement(board,'geom',name='sim_board',type='box',size=f'{w/2} {h/2} .0015',
            pos=f'{w/2} {h/2} 0',material='sim_board_material',rgba='1 1 1 0',
            contype='0',conaffinity='0',density='0',group='2')
    # Nominal 12-mm mounting spacer: the source pads do not meet at encoder
    # zero. This explicit fixture is not a measured board thickness/transmission.
    ET.SubElement(tool,'geom',name='sim_board_spacer',type='box',
        size='.012 .006 .014',pos='.00531736 0 -.17',rgba='.3 .3 .3 0',
        contype='0',conaffinity='0',density='0',group='2')
    contacts=root.find('contact')
    if contacts is None:contacts=ET.SubElement(root,'contact')
    available={g.get('name') for g in tool.iter('geom')}
    for name in config.get('board_contact_geoms',[]):
        if name not in available:raise ValueError('Board contacts must name tool/finger geometry')
        ET.SubElement(contacts,'pair',geom1='sim_board_spacer',geom2=name,condim='4',
                      friction='1 .01 .01 .001 .001',solref='.01 1')
    model=mj.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'),{'sim-charuco.png':png.tobytes()})
    return model


class SimulationWorld:
    def __init__(self,config,setup):
        import mujoco as mj
        import numpy as np
        self.mj,self.np=mj,np
        self.config,self.setup=config,setup
        self.model=build_model(setup,config);self.data=mj.MjData(self.model)
        collision=config['geometry']
        self.mapping={}
        kin=setup['kinematics']
        for i,name in enumerate(kin['joints']):
            self.mapping[name]=(i,float(kin['signs'][i]),float(kin['offsets'][i]))
        for name,(scale,offset) in collision['gripper_joints'].items():self.mapping[name]=(7,scale,offset)
        self.joints={name:mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_JOINT,name) for name in self.mapping}
        if any(j<0 for j in self.joints.values()):raise ValueError('Simulator joint mapping mismatch')
        self.fixed={mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_JOINT,name):value
                    for name,value in collision['fixed_joints'].items()}
        if any(j<0 for j in self.fixed):raise ValueError('Unknown fixed simulator joint')
        self.q=list(config['initial_rad']);self.enabled=False
        self.targets=list(self.q);self.kp=[0.]*8;self.kd=[0.]*8;self.ff=[0.]*8;self.torques=[0.]*8
        self.limits=[20.,20.,9.,9.,3.,3.,3.,3.]
        self.lock=threading.RLock();self.lease=threading.Lock();self.stop=threading.Event()
        self.physics=None;self.error=None;self.steps=0;self.board_inserted=False
        self.board_geom=mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_GEOM,'sim_board')
        self.spacer_geom=mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_GEOM,'sim_board_spacer')
        self.spacer_position=self.model.geom_pos[self.spacer_geom].copy()
        self.model.geom_pos[self.spacer_geom]=[0.,0.,10.]
        self.board_position=self.model.geom_pos[self.board_geom].copy()
        self.model.geom_pos[self.board_geom]=[0.,0.,10.]
        self.source_id='mujoco-'+uuid.uuid4().hex
        self._set_pose(self.q)
        self.metadata=dict(simulation=True,engine='MuJoCo',version=mj.__version__,source_id=self.source_id,
            geometry_sha256=collision['model_sha256'],active_sides=['left'],
            scenario_sha256=hashlib.sha256(json.dumps(config,sort_keys=True,separators=(',',':')).encode()).hexdigest(),
            cameras=config['cameras'],tool_from_board=config['tool_from_board'],
            gripper_transmission=collision['gripper_joints'],fixed_joints=collision['fixed_joints'],
            board_contact_geoms=config.get('board_contact_geoms',[]),
            assumptions=['ideal rigid board fixture with nominal 12 mm mounting spacer; no board slip or calibrated gripping force',
                         'disabled and other-arm joints mechanically supported',
                         'nominal motor/finger transmission, friction, masses and camera placements'],
            physical_hardware_opened=False)

    def _set_pose(self,q):
        for name,(i,scale,offset) in self.mapping.items():
            self.data.qpos[self.model.jnt_qposadr[self.joints[name]]]=scale*q[i]+offset
        for j,value in self.fixed.items():self.data.qpos[self.model.jnt_qposadr[j]]=value
        self.mj.mj_forward(self.model,self.data)

    def positions(self):
        q=[[] for _ in range(8)];v=[[] for _ in range(8)]
        for name,(i,scale,offset) in self.mapping.items():
            j=self.joints[name]
            q[i].append((float(self.data.qpos[self.model.jnt_qposadr[j]])-offset)/scale)
            v[i].append(float(self.data.qvel[self.model.jnt_dofadr[j]])/scale)
        return [sum(a)/len(a) for a in q],[sum(a)/len(a) for a in v]

    def start(self):
        if self.physics is not None:raise RuntimeError('Simulator already started')
        self.physics=threading.Thread(target=self._run,name='mujoco-physics',daemon=True)
        self.physics.start()

    def _run(self):
        try:
            while not self.stop.is_set():
                started=time.monotonic()
                with self.lock:
                    if self.enabled:
                        for _ in range(4):
                            q,v=self.positions()
                            tau=[max(-cap,min(cap,k*(goal-pos)-d*vel+ff)) for cap,k,d,goal,pos,vel,ff in
                                 zip(self.limits,self.kp,self.kd,self.targets,q,v,self.ff)]
                            self.torques=tau
                            self.data.qfrc_applied[:]=0.
                            counts=[sum(value[0]==i for value in self.mapping.values()) for i in range(8)]
                            for name,(i,scale,offset) in self.mapping.items():
                                j=self.joints[name]
                                self.data.qfrc_applied[self.model.jnt_dofadr[j]]=tau[i]/(scale*counts[i])
                            self.mj.mj_step(self.model,self.data)
                            for j,value in self.fixed.items():
                                self.data.qpos[self.model.jnt_qposadr[j]]=value
                                self.data.qvel[self.model.jnt_dofadr[j]]=0.
                            self.steps+=1
                        self.mj.mj_forward(self.model,self.data)
                        if not self.np.isfinite(self.data.qpos).all() or not self.np.isfinite(self.data.qvel).all():
                            raise RuntimeError('Nonfinite simulator state')
                    else:
                        self.data.qvel[:]=0.;self.torques=[0.]*8
                self.stop.wait(max(0,.004-(time.monotonic()-started)))
        except BaseException as exc:
            self.error=f'{type(exc).__name__}: {exc}';self.enabled=False;self.stop.set()

    def feedback(self):
        from openarm_runtime.telemetry import MotorState
        with self.lock:
            if self.error:raise RuntimeError(self.error)
            q,v=self.positions();now=time.monotonic()
            return {i:MotorState(i,q[i-1],v[i-1],self.torques[i-1],25,25,int(self.enabled),now)
                    for i in range(1,9)}

    def close(self):
        self.stop.set()
        if self.physics is not None:
            self.physics.join(5)
            if self.physics.is_alive():raise RuntimeError('Physics thread did not stop')
        self.enabled=False

    def insert_board(self):
        with self.lock:
            q,_=self.positions()
            if not self.enabled or q[7]<math.radians(15):
                raise ValueError('请先抬臂并打开模拟夹爪')
            self.board_inserted=True
            self.model.geom_rgba[self.board_geom,3]=1.
            self.model.geom_pos[self.board_geom]=self.board_position
            self.model.geom_pos[self.spacer_geom]=self.spacer_position
            self.model.geom_rgba[self.spacer_geom,3]=1.
            self.mj.mj_forward(self.model,self.data)

    def state(self):
        with self.lock:
            q,v=self.positions()
            contacts=[]
            for i in range(self.data.ncon):
                contact=self.data.contact[i]
                if self.spacer_geom not in (contact.geom1,contact.geom2):continue
                force=self.np.zeros(6)
                self.mj.mj_contactForce(self.model,self.data,i,force)
                contacts.append(dict(geom1=self.mj.mj_id2name(self.model,self.mj.mjtObj.mjOBJ_GEOM,contact.geom1),
                    geom2=self.mj.mj_id2name(self.model,self.mj.mjtObj.mjOBJ_GEOM,contact.geom2),
                    distance_m=float(contact.dist),normal_force_n=float(force[0])))
            return dict(self.metadata,enabled=self.enabled,positions_rad=q,velocities_rad_s=v,
                        torques_nm=list(self.torques),physics_steps=self.steps,
                        board_inserted=self.board_inserted,board_contacts=contacts,error=self.error)
