"""Calibration sink; only the existing teleoperator can issue motor commands.

Explicit mount confirmation retains its last gripper reference until following
ends. Arm poses and all hardware lifecycle/supervision stay with teleoperation.
"""
from collections import deque
import copy
from pathlib import Path
import threading
import time
import uuid

from .calibration_source import CalibrationCamera, load_setup, board_fits_image
from .camera_calibration import atomic_bytes, atomic_json
from openarm_runtime.stationary_calibration import digest, validate_stationary, fit_stationary


class TeleopHeadCalibration:
    def __init__(self, profile, root, video_port, *, clock=time.monotonic):
        self.data,self.profile,self.chain,self.intrinsics=load_setup(profile)
        self.profile.make_gravity('left',required=True)
        if self.intrinsics.get('simulation'):
            raise ValueError('Physical leader/follower calibration requires hardware intrinsics')
        self.root=Path(root);self.clock=clock
        self.camera=CalibrationCamera(video_port,self.data,self.intrinsics,clock=clock)
        self.lock=threading.Condition();self.operation=threading.Lock()
        # Serialize mount confirmation with a complete existing control exchange,
        # so a pre-confirmation gripper target cannot arrive after the snapshot.
        self.reference_lock=threading.RLock()
        self.held_grip_command=None
        self.trace=deque(maxlen=2000);self.sequence=0
        self.latest=None;self.context=None;self.group=None
        self.samples=[];self.points=[];self.error=None;self.result=None

    def observe(self, side, event, *, owner, generation, following):
        if side!='left':return
        states=event['feedback']
        row=dict(sequence=0,source='measured_feedback',status=[states[i]['status'] for i in range(1,9)],
            positions_rad=[states[i]['position'] for i in range(1,9)],
            velocities_rad_s=[states[i]['velocity'] for i in range(1,9)],
            received_at_s=[states[i]['received_at'] for i in range(1,9)],
            observed_at_s=event['exchange_completed_at'])
        command=event['targets_rad'][8]
        with self.lock:
            self.sequence+=1;row['sequence']=self.sequence
            self.latest=row;self.context=(owner,generation,following)
            self.last_grip_command=command
            self.gravity_mode=(event.get('gravity') or {}).get('mode')
            self.trace.append(row)
            if self.group and len(self.samples)<16 and not self.error:
                if self.context!=self.binding or self.gravity_mode!='active':
                    self.error='遥操接管或跟随状态改变，请重新确认装板并开始一组采样'
                elif abs(command-self.grip_command)>.003:
                    self.error=(f'夹爪保持目标发生变化（{command-self.grip_command:+.6f} rad），'
                        '这组不能继续混用；请停止跟随后重新装板')
                elif abs(row['positions_rad'][7]-self.grip)>.003:
                    self.error=(f'夹爪实测开度发生变化（{row["positions_rad"][7]-self.grip:+.6f} rad），'
                        '这组不能继续混用；请停止跟随后检查装夹')
            self.lock.notify_all()

    def ended(self):
        with self.lock:
            self.context=None
            self.held_grip_command=None
            if self.group and len(self.samples)<16 and not self.error:
                self.error='遥操已结束，请重新开始一组采样'
            self.lock.notify_all()

    def hold_targets(self,side,targets):
        """Called under reference_lock by the existing controller, never a sender.

        Keep holding even after group rejection/completion: releasing while still
        following would jump to the leader's meanwhile-changed gripper reference.
        Only the existing stop/disable lifecycle releases this reference.
        """
        with self.lock:
            if side=='left' and self.held_grip_command is not None:
                return dict(targets)|{8:self.held_grip_command}
            return targets

    def _owned(self, owner):
        if (self.context is None or self.context[0]!=owner or not self.context[2]
                or self.latest is None or self.latest['status']!=[1]*8
                or self.gravity_mode!='active'
                or not 0<=self.clock()-min(self.latest['received_at_s'])<=.15):
            raise ValueError('需要当前左臂遥操操作者及新鲜跟随反馈；本页不会启动电机')

    def begin(self, owner, *, mounted=False):
        if mounted is not True:raise ValueError('请先用主臂装板夹紧，并明确确认装夹固定')
        if not self.operation.acquire(blocking=False):raise ValueError('正在采样或求解')
        created=None
        try:
            with self.reference_lock,self.lock:
                self._owned(owner)
                self.binding=self.context
                self.grip=self.latest['positions_rad'][7]
                # The command is distinct from contact-deflected measured grip.
                self.grip_command=self.last_grip_command
                self.held_grip_command=self.grip_command
                self.group=uuid.uuid4().hex;self.directory=self.root/self.group
                created=self.group
                self.samples=[];self.points=[];self.error=None;self.result=None
                self.solve_error=None;self.solve_attempted=False;self.rejected=0
                self.bytes=0
                self.protocol=dict(schema='openarm.manual-head-protocol.v1',slots=[
                    dict(id=f'pose-{i+1:02d}',role='fit' if i<12 else 'validation') for i in range(16)])
                document=dict(protocol=self.protocol,protocol_sha256=digest(self.protocol),
                    simulation=False,control='existing_leader_follower',gripper_rad=self.grip,
                    gripper_command_rad=self.grip_command,gripper_control='hold_until_following_ends',
                    generation=self.binding[1],kinematics=self.chain.metadata,
                    intrinsics_sha256=self.data['intrinsics_sha256'],installed=False)
            self.directory.mkdir(parents=True,mode=0o700)
            atomic_json(self.directory/'session.json',document)
            return str(self.directory)
        except Exception:
            with self.lock:
                if created is not None and self.group==created:
                    self.error='新采样组未成功建立，请重新确认装板后再开始'
            raise
        finally:self.operation.release()

    def capture(self, owner):
        if not self.operation.acquire(blocking=False):raise ValueError('正在采样或求解')
        raw=None;metadata=None;retained=False
        try:
            with self.lock:
                self._owned(owner)
                if not self.group or self.error or len(self.samples)>=16:
                    raise ValueError(self.error or '先确认装板开始新的一组，或已采满16个姿态')
                group=self.group;slot=dict(self.protocol['slots'][len(self.samples)])
            raw,observation,metadata=self.camera.read()
            if not board_fits_image(observation,self.intrinsics):
                raise ValueError('标定板未完整入镜，请由主臂调整姿态')
            if self.bytes+len(raw)>256*1024**2:
                raise ValueError('本组原图达到256 MiB上限')
            acquisition=observation['acquisition'];start=acquisition['earliest_s'];end=acquisition['latest_s']
            deadline=self.clock()+2.
            with self.lock:
                while not self.error and min(self.latest['received_at_s'])<end+.2:
                    remaining=deadline-self.clock()
                    if remaining<=0:raise TimeoutError('等待曝光后实测反馈超时，未接受此帧')
                    self.lock.wait(min(.05,remaining))
                self._owned(owner)
                if self.error or group!=self.group:raise ValueError(self.error or '采样批次变化')
                rows=copy.deepcopy([r for r in self.trace
                    if min(r['received_at_s'])>=start-.8 and max(r['received_at_s'])<=end+.4])
            if not rows:raise ValueError('缺少曝光前后的连续实测反馈')
            import numpy as np
            index=int(np.argmin(np.abs(np.mean([r['received_at_s'] for r in rows],axis=1)-(start+end)/2)))
            measured=list(rows[index]['positions_rad'])
            sample=dict(observation,waypoint_id=slot['id'],role=slot['role'],measured_joints_rad=measured,
                stationary_evidence=dict(schema='openarm.stationary-bracket.v2',acquisition=acquisition,feedback=rows))
            validate_stationary(sample)
            from scipy.spatial.transform import Rotation
            pose=self.chain.forward(measured[:7])
            for i,previous in enumerate(self.samples):
                old=self.chain.forward(previous['measured_joints_rad'][:7])
                if (np.linalg.norm(pose[:3,3]-old[:3,3])<.01
                        and Rotation.from_matrix(old[:3,:3].T@pose[:3,:3]).magnitude()<.13962634):
                    raise ValueError(f'与第{i+1}张姿态过近，请换个位置并改变板的倾角')
            if self.samples and (sample['camera_identity']!=self.samples[0]['camera_identity']
                    or sample['frame_sequence']<=self.samples[-1]['frame_sequence']):
                raise ValueError('相机来源变化或重复帧，请重新开始一组')
            path=self.directory/(slot['id']+'-'+uuid.uuid4().hex[:8]);path.mkdir(mode=0o700)
            sample['artifact_path']=path.name
            atomic_bytes(path/'image.png',raw);atomic_json(path/'frame.json',metadata)
            retained=True
            atomic_json(path/'sample.json',sample)
            with self.lock:
                self._owned(owner)
                if self.error:raise ValueError(self.error)
                self.samples.append(sample)
                self.bytes+=len(raw)
                self.points.append(dict(slot,joints_rad=measured[:7]+[self.grip]))
            return len(self.samples)
        except Exception as exc:
            frame=getattr(exc,'frame',None)
            if raw is None and frame is not None:raw,metadata=frame
            if raw is not None and self.group and not retained and self.bytes+len(raw)<=256*1024**2:
                path=self.directory/'rejected'/uuid.uuid4().hex
                path.mkdir(parents=True,mode=0o700)
                atomic_bytes(path/'image.png',raw)
                atomic_json(path/'frame.json',metadata)
                atomic_json(path/'rejection.json',dict(reason=str(exc),accepted=False))
                self.bytes+=len(raw);self.rejected+=1
            raise
        finally:self.operation.release()

    def solve(self, owner):
        if not self.operation.acquire(blocking=False):raise ValueError('正在采样或求解')
        try:
            with self.lock:
                if len(self.samples)!=16 or owner!=self.binding[0] or self.error:
                    raise ValueError(self.error or '需要同一操作者完成12个拟合姿态和4个独立检查姿态')
                if self.result is not None:return self.result
                if self.solve_attempted:raise ValueError(self.solve_error or '本组已经求解，请检查保留的结果')
                self.solve_attempted=True
                route=dict(schema='openarm.calibration-manual-route.v1',side='left',
                    protocol=self.protocol,start_rad=self.points[0]['joints_rad'],waypoints=self.points)
                observations=dict(schema='openarm.stationary-head-observations.v1',units='m',simulation=False,
                    joint_state_source='measured_feedback',kinematics_sha256=digest(self.chain.metadata),
                    manual_protocol_sha256=digest(self.protocol),route=route,route_sha256=digest(route),
                    samples=list(self.samples),**{k:self.data[k] for k in
                        ('camera_name','base_frame','tool_frame','intrinsics_sha256')})
            atomic_json(self.directory/'observations.json',observations)
            result=fit_stationary(observations,self.chain)
            result.update(simulation=False,installed=False)
            atomic_json(self.directory/'estimate.json',result)
            self.result=result
            return result
        except Exception as exc:
            if getattr(self,'solve_attempted',False) and self.result is None:
                self.solve_error=str(exc)
                if not (self.directory/'solve-error.json').exists():
                    atomic_json(self.directory/'solve-error.json',dict(error=str(exc),installed=False))
            raise
        finally:self.operation.release()

    def status(self):
        with self.lock:
            now=self.clock()
            fresh=self.latest is not None and 0<=now-min(self.latest['received_at_s'])<=.15
            ready=bool(fresh and self.context is not None and self.context[2]
                and self.latest['status']==[1]*8 and self.gravity_mode=='active')
            rows=[r for r in self.trace if min(r['received_at_s'])>=now-.75]
            stationary=bool(ready and len(rows)>=12 and
                min(rows[-1]['received_at_s'])-max(rows[0]['received_at_s'])>=.5 and
                all(max(r['positions_rad'][i] for r in rows)-min(r['positions_rad'][i] for r in rows)<=.003 for i in range(8))
                and all(0<b-a<=.15 for before,after in zip(rows,rows[1:])
                        for a,b in zip(before['received_at_s'],after['received_at_s'])))
            count=len(self.samples)
            phase=('result' if self.result else 'review' if count==16 else 'reset' if self.error else
                'capture' if self.group else 'mount' if ready else 'connect')
            return dict(count=count,error=self.error,group=self.group,ready=ready,stationary=stationary,
                gripper_held=self.held_grip_command is not None,
                owner=self.context[0] if self.context else None,phase=phase,busy=self.operation.locked(),
                next_role='fit' if count<12 else 'validation',result=self.result,
                solve_error=getattr(self,'solve_error',None),solve_attempted=getattr(self,'solve_attempted',False),
                rejected=getattr(self,'rejected',0),directory=str(self.directory) if self.group else None,
                history=[dict(index=i+1,role=s['role'],rms=s['reprojection_rms_px'],
                    corners=len(s.get('corner_ids',[])),artifact=s['artifact_path']) for i,s in enumerate(self.samples)])


def attach(server, session, profile, root, video_port):
    calibration=TeleopHeadCalibration(profile,root,video_port)
    if calibration.profile!=session.profile:
        raise ValueError('标定FK配置引用的机器人配置必须与当前遥操一致')
    session.head_calibration=calibration
    from .teleop_calibration_view import TeleopCalibrationView
    session.head_calibration_view=TeleopCalibrationView(server,session,calibration)
    return calibration
