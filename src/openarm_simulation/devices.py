"""Simulation implements the same ArmHold bus and CalibrationArm lifecycle."""
from dataclasses import asdict
from types import SimpleNamespace


class SimulationBus:
    def __init__(self,world):
        from openarm_runtime.telemetry import MotorRange
        self.world=world
        self.sdk=SimpleNamespace(__version__='simulation-'+world.metadata['version'])
        self.ranges={i:MotorRange(12.5,45. if i<3 else 30.,cap)
                     for i,cap in enumerate((54.,54.,28.,28.,10.,10.,10.,10.),1)}

    def prepare_encoder(self,ids):
        if set(ids)!=set(range(1,9)):raise ValueError('Eight simulated motors required')

    def state(self,mid):return self.world.feedback()[mid]

    def preflight(self):
        states=self.world.feedback()
        if any(s.status for s in states.values()):raise ValueError('Simulator already enabled; stop the current controller first')
        return {i:dict(state=asdict(s),range=asdict(self.ranges[i]),baud_code=4,simulation=True)
                for i,s in states.items()}

    def command(self,mid,action,position=0.,kp=0.,kd=0.):
        with self.world.lock:
            if action=='enable':self.world.enabled=True
            elif action=='disable':self.world.enabled=False
            elif action=='position':
                self.world.targets[mid-1]=position;self.world.kp[mid-1]=kp;self.world.kd[mid-1]=kd
            else:raise ValueError('Unsupported simulation bus action')

    def exchange(self,targets,kp,kd,feedforward=None):
        with self.world.lock:
            if not self.world.enabled:raise RuntimeError('Simulated motors are disabled')
            self.world.targets=[float(targets[i]) for i in range(1,9)]
            self.world.kp=list(kp);self.world.kd=list(kd)
            self.world.ff=[float((feedforward or {}).get(i,0.)) for i in range(1,9)]
        return self.world.feedback()

    def close(self):pass


class SimulationCapture:
    simulate=True
    stale_after=1.

    def __init__(self,world):
        from openarm_video.camera import CameraConfig
        self.world=world
        self.configs=tuple(CameraConfig(name,width=c['width'],height=c['height'],fps=c['fps'],
            calibration_identity='mujoco:'+world.metadata['geometry_sha256'][:16]+':'+name) for name,c in world.config['cameras'].items())
        self.frames={};self.error='';self.thread=None
        import threading
        self.lock=threading.Lock();self.stop=threading.Event()

    def start(self):
        import threading
        self.thread=threading.Thread(target=self._run,name='mujoco-render',daemon=True);self.thread.start()

    def _run(self):
        import copy
        import time
        import mujoco as mj
        from openarm_video.camera import Frame
        from openarm_runtime.stationary_calibration import digest
        renderers={}
        try:
            with self.world.lock:model=copy.copy(self.world.model)
            data=mj.MjData(model)
            for c in self.configs:renderers[c.name]=mj.Renderer(model,c.height,c.width)
            options=mj.MjvOption();options.geomgroup[3]=0
            count=0
            while not self.stop.is_set():
                started=time.monotonic();count+=1
                for c in self.configs:
                    with self.world.lock:
                        data.qpos[:]=self.world.data.qpos;data.qvel[:]=self.world.data.qvel
                        model.geom_pos[:]=self.world.model.geom_pos
                        model.geom_rgba[:]=self.world.model.geom_rgba
                        sim_time=float(self.world.data.time)
                        earliest=time.monotonic()
                    mj.mj_forward(model,data)
                    renderer=renderers[c.name]
                    renderer.update_scene(data,camera='sim_'+c.name,scene_option=options)
                    rgb=renderer.render();latest=time.monotonic()
                    evidence=dict(simulation=True,source='MuJoCo offscreen render',simulation_time_s=sim_time,
                                  scenario_sha256=self.world.metadata['scenario_sha256'],
                                  geometry_sha256=self.world.metadata['geometry_sha256'])
                    acquisition=dict(clock='host_monotonic',semantics='qualified_acquisition_interval',
                        qualification='synthetic_test',qualification_evidence=evidence,
                        qualification_evidence_sha256=digest(evidence),earliest_s=earliest,latest_s=latest,
                        host_received_at_s=latest)
                    frame=Frame(rgb[:,:,::-1].copy(),latest,count,'bgr8',self.world.source_id,acquisition)
                    with self.lock:self.frames[c.name]=frame
                self.stop.wait(max(0,1/min(c.fps for c in self.configs)-(time.monotonic()-started)))
        except BaseException as exc:
            self.error=f'{type(exc).__name__}: {exc}'
        finally:
            for renderer in renderers.values():renderer.close()

    def snapshot(self,name):
        with self.lock:
            frame=self.frames.get(name)
            return frame,self.error,frame.sequence if frame else 0

    def status(self):
        import time
        return {c.name:dict(status='Online' if (f:=self.snapshot(c.name)[0]) and f.fresh(time.monotonic(),1) and not self.error else 'Starting' if not self.error else 'Disconnected',
                simulation=True,error=self.error,width=c.width,height=c.height) for c in self.configs}

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(10)
            if self.thread.is_alive():raise RuntimeError('Simulation renderer did not stop')
