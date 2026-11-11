"""Passive calibration configuration and qualified camera reads. No motion API."""
import hashlib
import json
import math
from pathlib import Path
import time

from .camera_calibration import borrow_frame, decode_frame


def load_setup(path):
    from openarm_runtime.cartesian import KinematicChain
    from openarm_runtime.config import Profile
    from openarm_runtime.camera_intrinsics import board
    path=Path(path).expanduser().resolve()
    data=json.loads(path.read_text())
    required={'schema','robot_profile','kinematics','intrinsics','intrinsics_sha256',
              'board_spec','base_frame','tool_frame','camera_name'}
    if not isinstance(data,dict) or set(data)!=required or data['schema']!='openarm.teleop-head-profile.v1':
        raise ValueError('需要遥操标定配置；自动运动/搜索配置已移除')
    def local(value):return (path.parent/Path(value).expanduser()).resolve()
    profile=Profile.load(local(data['robot_profile']))
    options=dict(data['kinematics']);options['urdf']=str(local(options['urdf']))
    chain=KinematicChain(**options,encoder_tolerance_rad=math.radians(.5))
    if len(chain.indices)!=7:raise ValueError('需要明确映射的七关节左臂FK')
    import xml.etree.ElementTree as ET
    xml=ET.fromstring(Path(options['urdf']).read_bytes())
    roots={e.get('name') for e in xml.findall('link')}-{e.find('child').get('link') for e in xml.findall('joint')}
    if roots!={data['base_frame']} or data['tool_frame']!=options['frame']:
        raise ValueError('基座或工具坐标与FK配置不一致')
    raw=local(data['intrinsics']).read_bytes()
    if hashlib.sha256(raw).hexdigest()!=data['intrinsics_sha256']:
        raise ValueError('冻结内参文件哈希不匹配')
    intrinsics=json.loads(raw)
    if (intrinsics.get('schema')!='openarm.camera-intrinsics.frozen.v1'
            or intrinsics.get('image_geometry')!='native_no_transform'
            or intrinsics.get('camera_model')!='pinhole'
            or intrinsics.get('distortion_model')!='opencv_5'
            or data['camera_name']!='head' or intrinsics.get('camera_name')!='head'
            or data['board_spec'].get('type')!='charuco'):
        raise ValueError('需要冻结的头部针孔内参和带索引ChArUco板')
    board(data['board_spec'])
    return data,profile,chain,intrinsics


def board_fits_image(observation,intrinsics,*,margin_px=5.):
    import cv2
    import numpy as np
    spec=observation['board_spec'];w=spec['squares_x']*spec['square_size_m'];h=spec['squares_y']*spec['square_size_m']
    pose=np.array(observation['camera_from_board'])
    pixels=cv2.projectPoints(np.array([[0.,0.,0.],[w,0.,0.],[w,h,0.],[0.,h,0.]]),
        cv2.Rodrigues(pose[:3,:3])[0],pose[:3,3],np.array(intrinsics['matrix']),
        np.array(intrinsics['distortion']))[0].reshape(-1,2)
    width,height=intrinsics['image_size']
    return bool(np.all(pixels>=margin_px) and np.all(pixels[:,0]<=width-margin_px)
                and np.all(pixels[:,1]<=height-margin_px))


class CalibrationCamera:
    def __init__(self,port,data,intrinsics,*,simulation=False,clock=time.monotonic,fetch=borrow_frame):
        self.port,self.data,self.intrinsics=port,data,intrinsics
        self.simulation,self.clock,self.fetch=simulation,clock,fetch

    def read(self):
        raw,meta=self.fetch(self.port,self.data['camera_name'])
        try:return self.parse(raw,meta)
        except ValueError as exc:
            exc.frame=(raw,meta)
            raise

    def parse(self,raw,meta):
        from openarm_runtime.camera_calibration import charuco_pose
        from openarm_runtime.stationary_calibration import digest
        image=decode_frame(raw,meta);acquisition=meta.get('acquisition',{})
        evidence=acquisition.get('qualification_evidence')
        if (acquisition.get('semantics')!='qualified_acquisition_interval'
                or not isinstance(evidence,dict) or digest(evidence)!=acquisition.get('qualification_evidence_sha256')
                or acquisition.get('png_sha256')!=meta['png_sha256']
                or not acquisition['earliest_s']<=acquisition['latest_s']<=meta['received_at_s']
                or not 0<=self.clock()-acquisition['latest_s']<=.5):
            raise ValueError('相机采集时序未合格，不能将画面与关节反馈配对')
        if meta['simulation']!=self.simulation:raise ValueError('仿真/实物来源不匹配，禁止混用')
        expected='synthetic_test' if self.simulation else 'v4l2_monotonic_capture'
        if acquisition.get('qualification')!=expected:raise ValueError('相机来源时序不符合当前模式')
        if ([meta['width'],meta['height']]!=self.intrinsics['image_size']
                or meta['camera_name']!=self.data['camera_name']
                or meta['camera_identity']!=self.intrinsics['camera_identity']):
            raise ValueError('相机身份或原尺寸与冻结内参不一致')
        pose=charuco_pose(image,self.intrinsics['matrix'],self.intrinsics['distortion'],
            {k:v for k,v in self.data['board_spec'].items() if k!='type'})
        identity={k:meta[k] for k in ('camera_name','camera_identity','source_id','width',
            'height','mode','image_geometry','pixel_format','simulation')}
        return raw,dict(pose,png_sha256=meta['png_sha256'],frame_sequence=meta['sequence'],
            camera_identity=identity,acquisition=acquisition),meta
