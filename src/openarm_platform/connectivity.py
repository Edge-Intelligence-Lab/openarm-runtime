"""Background CAN readiness using the existing query-only reconnect workers.

No enable, disable, control, firmware write, bus reset or sudo operation. The
service manager pauses these readers before a motion lease can be granted.
"""
import threading
import time


class CANConnectivity:
    def __init__(self, manager):
        self.manager = manager
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.worker = None
        self.target = None
        self.connections = {}
        self.epoch = 0
        self.enabled = False
        self.result = dict(state='unmonitored', message='连接在执行前检查', sides=[], error='')

    def configure(self, entry):
        target = ((str(entry.robot_config), tuple(entry.sides))
                  if entry is not None and entry.robot == 'classic' and entry.robot_config is not None else None)
        with self.lock:
            if target != self.target:
                self.target = target
                self.epoch += 1
                self.result = dict(state='waiting' if target else 'unconfigured',
                    message='正在等待 CAN 只读连接' if target else '当前模型不需要实物 CAN', sides=[], error='')

    def start(self):
        with self.lock:
            if self.enabled or self.stop.is_set():
                return
            self.enabled = True
            self.worker = threading.Thread(target=self._run, daemon=True, name='openarm-can-connectivity')
            self.worker.start()

    def _close_readers(self):
        # Failed close retains the worker reference: never create a duplicate.
        for side, connection in list(self.connections.items()):
            connection.close()
            del self.connections[side]

    def pause(self):
        with self.lock:
            self.epoch += 1
            self._close_readers()
            self.result = dict(state='busy', message='控制交接中：已释放只读 CAN 连接', sides=[], error='')

    def tick(self):
        from openarm_runtime.config import Profile, require_classic_link
        from openarm_runtime.telemetry import ArmConnection, IDS, inspect_link
        with self.lock:
            epoch, target = self.epoch, self.target
        # Never call the manager while holding the connectivity lock: motion
        # handover holds the manager lock and calls pause().
        busy = self.manager.motion_active()
        with self.lock:
            if epoch != self.epoch or self.stop.is_set():
                return
            if target is None or busy:
                self._close_readers()
                self.result = dict(state='busy' if busy else 'unconfigured',
                    message='控制服务占用或尚待确认退出；不额外查询或重连' if busy else '当前模型不需要实物 CAN', sides=[], error='')
                return
            try:
                profile = Profile.load(target[0])
                if not profile.mapping_verified:
                    raise ValueError('CAN 侧别映射尚未确认')
            except (OSError, ValueError, TypeError) as exc:
                self._close_readers()
                self.result = dict(state='blocked', message='请修正机器人配置；不会自动使能', sides=[], error=str(exc))
                return
            expected = {side: profile.interface(side) for side in target[1]}
            for side, connection in list(self.connections.items()):
                if expected.get(side) != connection.interface or connection.stop.is_set():
                    connection.close()
                    del self.connections[side]
            for side, interface in expected.items():
                if side not in self.connections:
                    connection = ArmConnection(interface, frame_format='classic')
                    self.connections[side] = connection
                    connection.restart()
            statuses = []
            for side, connection in self.connections.items():
                states, error, message, _, _ = connection.shared.snapshot()
                fresh = set(states) == set(IDS) and all(s.fresh(time.monotonic(), .5) for s in states.values())
                ready = fresh and not error and all(s.status == 0 for s in states.values())
                if fresh and any(s.status != 0 for s in states.values()):
                    message = '电机已使能或报告故障；不自动使能、清错或接管'
                    error = message
                try:
                    # Reinspect host health; a historical ERROR-WARNING may have
                    # cleared during query-only recovery. Never rewrite the link.
                    require_classic_link(inspect_link(connection.interface))
                except Exception as exc:
                    ready, error = False, str(exc)
                metadata = connection.shared.metadata()
                statuses.append(dict(side=side, interface=connection.interface,
                    ready=ready, fresh=fresh, received_motors=len(states), message=message,
                    error=error, retry_delay_s=metadata.get('retry_delay_s', 0),
                    attempts=metadata.get('connection_attempts', 0)))
            ready = bool(statuses) and all(s['ready'] for s in statuses)
            self.result = dict(state='ready' if ready else 'waiting', sides=statuses,
                message='CAN 只读连接就绪，等待操作员启动' if ready else '等待 CAN 连接，后台自动重试；不自动使能',
                error='；'.join(s['error'] for s in statuses if s['error']))

    def _run(self):
        while not self.stop.is_set():
            try:
                self.tick()
            except Exception as exc:
                with self.lock:
                    self.result = dict(state='waiting', message='连接检查暂不可用，将自动重试；不自动使能',
                                       sides=[], error=str(exc))
            self.stop.wait(1.)

    def snapshot(self):
        from copy import deepcopy
        # Results are immutable after publication (whole-dict replacement).
        # Never make a UI poll wait for SocketCAN inspection or worker shutdown.
        result = self.result
        return dict(enabled=self.enabled, **deepcopy(result))

    def require_ready(self, entry):
        if not self.enabled or entry.robot != 'classic':
            return
        self.configure(entry)
        status = self.snapshot()
        if status['state'] not in ('ready', 'busy'):
            raise ValueError(status['message'] + ('：'+status['error'] if status['error'] else ''))

    def close(self):
        self.stop.set()
        if self.worker is not None:
            self.worker.join(timeout=10)
            if self.worker.is_alive():
                raise RuntimeError('只读连接检查尚未退出；未创建重复连接')
        with self.lock:
            self._close_readers()
