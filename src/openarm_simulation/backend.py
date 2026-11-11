"""Platform-owned simulated devices and loopback video; no subprocess/device fallback."""
import threading

from .world import load_profile,SimulationWorld
from .devices import SimulationCapture


class SimulationBackend:
    def __init__(self,path):
        self.config=load_profile(path)
        from openarm_platform.calibration_source import load_setup
        self.setup,self.profile,self.chain,self.intrinsics=load_setup(self.config['calibration_profile'])
        self.world=SimulationWorld(self.config,self.setup)
        self.capture=SimulationCapture(self.world)
        import math
        import numpy as np
        head=self.config['cameras']['head']
        f=head['height']/2/math.tan(math.radians(head['fovy_deg'])/2)
        matrix=[[f,0,head['width']/2],[0,f,head['height']/2],[0,0,1]]
        identity=next(c.calibration_identity for c in self.capture.configs if c.name=='head')
        if (self.intrinsics.get('simulation') is not True or self.intrinsics['camera_identity']!=identity
                or self.intrinsics['image_size']!=[head['width'],head['height']]
                or not np.allclose(matrix,self.intrinsics['matrix'],atol=1e-8)
                or not np.allclose(self.intrinsics['distortion'],0)):
            raise ValueError('Simulation requires its own matching synthetic camera intrinsics')
        self.server=None;self.server_thread=None

    @property
    def video_port(self):return self.config['video_port']

    def start(self):
        from openarm_video.stream import create_app
        from werkzeug.serving import make_server,WSGIRequestHandler
        class Quiet(WSGIRequestHandler):
            def log_request(self,*args,**kwargs):pass
        self.server=make_server('127.0.0.1',self.video_port,create_app(self.capture),threaded=True,request_handler=Quiet)
        try:
            self.world.start();self.capture.start()
            self.server_thread=threading.Thread(target=self.server.serve_forever,name='sim-video-http',daemon=True)
            self.server_thread.start()
        except BaseException:
            self.close();raise

    def connection_status(self):
        return dict(label='MuJoCo 仿真 · 不连接 CAN/串口',ready=self.world.physics is not None and not self.world.error,
                    simulation=True,reason=self.world.error)

    def close(self):
        try:
            if self.server is not None and self.server_thread is not None:
                self.server.shutdown();self.server_thread.join(5)
        finally:
            try:self.capture.close()
            finally:
                self.world.close()
                if self.server is not None:self.server.server_close()
