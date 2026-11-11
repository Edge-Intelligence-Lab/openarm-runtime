"""Offline measured-pose rendering with a hash-bound physical camera estimate.

This does not turn physical observations into synthetic calibration inputs or
open any robot/camera transport. It only sets private MuJoCo model/data arrays.
"""
import hashlib
import json
from pathlib import Path


def camera_parameters(intrinsics,world_from_camera):
    """OpenCV (+X right,+Y down,+Z forward) to MuJoCo camera axes/intrinsics."""
    import numpy as np
    from scipy.spatial.transform import Rotation
    from openarm_runtime.camera_calibration import transform
    T=transform(world_from_camera)
    width,height=intrinsics['image_size'];K=np.asarray(intrinsics['matrix'],float)
    if (K.shape!=(3,3) or not np.isfinite(K).all() or min(width,height)<=0
            or K[0,0]<=0 or K[1,1]<=0 or not np.allclose(K[2],[0,0,1])
            or abs(K[0,1])+abs(K[1,0])>1e-10):
        raise ValueError('Finite zero-skew pinhole intrinsics required')
    R=T[:3,:3]@np.diag([1.,-1.,-1.])
    # MuJoCo frusta use center-minus-principal offsets in both image axes.
    # Sensor dimensions equal resolution here, so lengths equal pixels.
    return dict(position=T[:3,3],quat=Rotation.from_matrix(R).as_quat(scalar_first=True),
        sensor=[width,height],resolution=[width,height],
        intrinsic=[K[0,0],K[1,1],width/2-K[0,2],height/2-K[1,2]])


class CalibratedReplay:
    """One pinned binding, one private scene; forward kinematics only, no steps."""
    def __init__(self,binding_path):
        import numpy as np
        import mujoco as mj
        from openarm_runtime.stationary_calibration import digest
        from openarm_runtime.cartesian import KinematicChain
        from .world import build_model
        self.binding_path=Path(binding_path).resolve()
        self.binding=json.loads(self.binding_path.read_text())
        b=self.binding
        if b['schema']!='openarm.calibrated-replay.v1':raise ValueError('Unknown replay binding')
        documents={}
        for name,item in b['sources'].items():
            path=(self.binding_path.parent/item['path']).resolve();raw=path.read_bytes()
            if hashlib.sha256(raw).hexdigest()!=item['sha256']:raise ValueError('Source hash mismatch: '+name)
            documents[name]=json.loads(raw)
        self.estimate=documents['estimate'];self.observations=documents['observations']
        self.intrinsics=documents['intrinsics'];self.session=documents['session']
        if (digest(self.observations)!=self.estimate['observations_sha256']
                or b['sources']['intrinsics']['sha256']!=self.estimate['intrinsics_sha256']
                or self.estimate['transform_name']!='base_from_camera'
                or self.estimate['base_frame']!=b['base_frame']):
            raise ValueError('Calibration source/coordinate binding mismatch')
        kin=dict(b['kinematics'])
        self.chain=KinematicChain(kin['urdf'],sha256=kin['sha256'],joints=kin['joints'],
            signs=kin['signs'],offsets=kin['offsets'],locked=kin['locked'],frame=kin['frame'],
            tcp=kin['tcp'],encoder_tolerance_rad=kin.get('encoder_limit_tolerance_rad',0.))
        if digest(self.chain.metadata)!=self.estimate['kinematics_sha256']:
            raise ValueError('Replay FK differs from calibration FK')
        self.base_from_world=np.asarray(b['geometry']['base_from_collision_world'],float)
        from openarm_runtime.camera_calibration import transform
        transform(self.base_from_world)
        world_from_camera=np.linalg.inv(self.base_from_world)@np.asarray(self.estimate['transform'])
        self.camera=camera_parameters(self.intrinsics,world_from_camera)
        width,height=self.intrinsics['image_size']
        position=world_from_camera[:3,3];forward=world_from_camera[:3,2]
        world_from_base=np.linalg.inv(self.base_from_world)
        overview_position=(world_from_base@np.array([.75,.85,.7,1.]))[:3]
        overview_target=(world_from_base@np.array([0.,0.,.25,1.]))[:3]
        cameras=dict(head=dict(position_m=position.tolist(),target_m=(position+forward).tolist(),
            fovy_deg=45,width=width,height=height,fps=1),
            overview=dict(position_m=overview_position.tolist(),target_m=overview_target.tolist(),
                fovy_deg=55,width=width,height=height,fps=1))
        body_from_tool=transform(b['geometry']['body_from_tool'])
        config=dict(geometry=b['geometry'],cameras=cameras,
            tool_from_board=(body_from_tool@np.asarray(self.estimate['tool_from_board'])).tolist())
        self.model=build_model(dict(kinematics=kin,board_spec=self.observations['samples'][0]['board_spec']),config,
            board_visual_mesh=True)
        self.data=mj.MjData(self.model)
        cam=mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_CAMERA,'sim_head')
        self.model.cam_pos[cam]=self.camera['position'];self.model.cam_quat[cam]=self.camera['quat']
        self.model.cam_sensorsize[cam]=self.camera['sensor']
        self.model.cam_resolution[cam]=self.camera['resolution']
        self.model.cam_intrinsic[cam]=self.camera['intrinsic']
        geom=mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_GEOM,'sim_board')
        self.model.geom_rgba[geom,3]=1.
        self.joints={name:mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_JOINT,name)
            for name in set(kin['joints'])|set(kin['locked'])|set(b['geometry']['gripper_joints'])}
        if min(self.joints.values())<0:raise ValueError('Missing replay joints')
        self.renderer=None

    def set_sample(self,index):
        import numpy as np
        import mujoco as mj
        kin=self.binding['kinematics'];sample=self.observations['samples'][index]
        q=sample['measured_joints_rad']
        values=dict(kin['locked'])
        values.update({n:s*q[i]+o for i,(n,s,o) in enumerate(zip(kin['joints'],kin['signs'],kin['offsets']))})
        values.update({n:scale*q[7]+offset for n,(scale,offset) in self.binding['geometry']['gripper_joints'].items()})
        for name,value in values.items():self.data.qpos[self.model.jnt_qposadr[self.joints[name]]]=value
        self.data.qvel[:]=0.;mj.mj_forward(self.model,self.data)
        body=mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_BODY,self.binding['geometry']['tool_body'])
        T=np.eye(4);T[:3,:3]=self.data.xmat[body].reshape(3,3);T[:3,3]=self.data.xpos[body]
        predicted=self.base_from_world@T@np.asarray(self.binding['geometry']['body_from_tool'])
        expected=self.chain.forward(q[:7])
        if not np.allclose(predicted,expected,atol=1e-6):
            raise ValueError('MuJoCo/URDF FK mismatch; do not hand-adjust camera to compensate')
        return dict(waypoint_id=sample['waypoint_id'],frame_sequence=sample['frame_sequence'],
            role=sample['role'],fk_max_abs_error=float(np.max(np.abs(predicted-expected))))

    def render(self,index,*,overview=False):
        import mujoco as mj
        identity=self.set_sample(index)
        if self.renderer is None:
            width,height=self.intrinsics['image_size'];self.renderer=mj.Renderer(self.model,height,width)
        option=mj.MjvOption();option.geomgroup[3]=0
        if overview:option.flags[mj.mjtVisFlag.mjVIS_CAMERA]=True
        self.renderer.update_scene(self.data,camera='sim_overview' if overview else 'sim_head',scene_option=option)
        return self.renderer.render().copy(),identity

    def close(self):
        if self.renderer is not None:self.renderer.close();self.renderer=None
