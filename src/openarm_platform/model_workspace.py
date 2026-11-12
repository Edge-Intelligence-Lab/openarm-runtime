"""Trusted model catalogue and operator-started deployment lifecycle.

No hardware/model imports at construction. Only configured IDs reach subprocesses;
the browser cannot supply paths, factories, commands, ports or environments.
Loading and sample prediction never construct a robot. Physical execution reuses
the policy CLI, session watchdogs, bus leases and verified cleanup.
"""
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time
import uuid
import zipfile

from .services import tcp_reachable, MOTION_SHUTDOWN_TIMEOUT


def read_json(path):
    if path.stat().st_size > 1_000_000:
        raise ValueError('模型配置超过大小限制')
    return json.loads(path.read_text())


def file_hash(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def child_environment(overrides):
    """Keep explicit checkout code first, while retaining per-model dependencies."""
    env = dict(os.environ, **overrides)
    source = os.environ.get('OPENARM_RUNTIME_SOURCE')
    if source:
        env['PYTHONPATH'] = source + (os.pathsep+env['PYTHONPATH'] if env.get('PYTHONPATH') else '')
    return env


@dataclass(frozen=True)
class ModelEntry:
    id: str
    label: str
    description: str
    model_profile: Path
    identity: dict
    output_shape: tuple
    inference_python: str
    runtime_python: str
    probe_inputs: Path | None
    policy_profile: Path | None
    robot_config: Path | None
    robot: str
    sides: tuple
    max_duration_s: float
    inference_env: dict
    runtime_env: dict
    load_timeout_s: float
    test_timeout_s: float

    def fingerprint(self):
        paths = [self.model_profile, self.probe_inputs, self.policy_profile, self.robot_config]
        if self.policy_profile is not None:
            preparation = read_json(self.policy_profile).get('preparation_profile')
            if preparation is not None:
                paths.append((self.policy_profile.parent / preparation).resolve(strict=True))
        return {str(p): file_hash(p) for p in paths if p is not None}

    def public(self):
        try:
            profile = read_json(self.policy_profile) if self.policy_profile is not None else {}
        except (OSError, ValueError):
            profile = {}
        return dict(id=self.id, label=self.label, description=self.description,
                    identity=self.identity, output_shape=list(self.output_shape),
                    robot=self.robot, max_duration_s=self.max_duration_s,
                    sample_available=self.probe_inputs is not None,
                     execution_configured=self.policy_profile is not None and
                     (self.robot != 'classic' or self.robot_config is not None),
                     home_configured=bool(profile.get('return_home')) and
                         (self.robot != 'classic' or self.robot_config is not None),
                     home_requires_nominal=self.robot == 'classic' and bool(profile.get('cartesian')) and
                         profile['cartesian'].get('calibration_verified') is not True)


def catalogue(path):
    data = read_json(path)
    if (not isinstance(data, dict) or set(data) != {'schema', 'port', 'models'}
            or data['schema'] != 'openarm.model-catalog.v1'
            or type(data['port']) is not int or not 1024 <= data['port'] <= 65535
            or not isinstance(data['models'], list) or len(data['models']) > 100):
        raise ValueError('无效的模型目录配置')
    root = path.parent

    def filename(value):
        if not isinstance(value, str) or not value or '\0' in value:
            raise ValueError('配置路径必须为非空字符串')
        p = Path(value).expanduser()
        p = (p if p.is_absolute() else root / p).resolve(strict=True)
        if not p.is_file():
            raise ValueError('模型配置路径必须是文件')
        return p

    def environment(value):
        if (not isinstance(value, dict) or set(value) - {'PYTHONPATH', 'OMP_NUM_THREADS',
                'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS'} or
                any(not isinstance(v, str) or '\0' in v for v in value.values())):
            raise ValueError('只允许显式的模块路径和线程环境设置')
        return dict(value)

    def interpreter(value):
        if not isinstance(value, str) or not value or '\0' in value:
            raise ValueError('解释器路径无效')
        p = Path(value).expanduser()
        p = p if p.is_absolute() else root / p
        if not p.is_file():
            raise ValueError('解释器不存在')
        # Preserve venv/bin/python symlinks: resolving them to the base binary
        # silently discards that environment's installed model dependencies.
        return str(p.absolute())

    entries = {}
    allowed = {'id', 'label', 'description', 'model_profile', 'identity', 'output_shape',
               'inference_python', 'runtime_python', 'probe_inputs', 'policy_profile',
               'robot_config', 'robot', 'sides', 'max_duration_s', 'inference_env', 'runtime_env'}
    allowed |= {'load_timeout_s', 'test_timeout_s'}
    for item in data['models']:
        if not isinstance(item, dict) or set(item) - allowed:
            raise ValueError('未知模型目录字段')
        identity = item.get('identity')
        shape = item.get('output_shape')
        name = item.get('id', '')
        sides = item.get('sides', ['left'])
        duration = item.get('max_duration_s', 30)
        timeouts = [item.get('load_timeout_s', 60), item.get('test_timeout_s', 2)]
        if (not isinstance(name, str) or not re.fullmatch('[a-z][a-z0-9-]{0,63}', name)
                or name in entries or not isinstance(item.get('label'), str) or not item['label'].strip()
                or not isinstance(identity, dict) or not identity
                or any(not isinstance(k, str) or not isinstance(v, (str, int, bool))
                       for k, v in identity.items())
                or not isinstance(shape, list) or len(shape) != 2
                or any(type(n) is not int or not 1 <= n <= 10000 for n in shape)
                or shape[0] * shape[1] > 100000
                or not isinstance(sides, list) or not sides or len(set(sides)) != len(sides)
                or any(s not in ('left', 'right') for s in sides)
                or isinstance(duration, bool) or not isinstance(duration, (int, float))
                or not math.isfinite(duration) or not 0 < duration <= 3600
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                       not math.isfinite(v) or not 0 < v <= 3600 for v in timeouts)
                or item.get('robot', 'classic') not in ('classic', 'simulate')):
            raise ValueError('模型ID、身份、输出协议或执行配置无效')
        paths = {k: filename(item[k]) if item.get(k) is not None else None
                 for k in ('model_profile', 'probe_inputs', 'policy_profile', 'robot_config')}
        if paths['model_profile'] is None:
            raise ValueError('必须配置模型预测器')
        entries[name] = ModelEntry(name, item['label'], str(item.get('description', '')),
            paths['model_profile'], dict(identity), tuple(shape),
            interpreter(item.get('inference_python', sys.executable)),
            interpreter(item.get('runtime_python', sys.executable)),
            paths['probe_inputs'], paths['policy_profile'], paths['robot_config'],
            item.get('robot', 'classic'), tuple(sides), float(duration),
            environment(item.get('inference_env', {})), environment(item.get('runtime_env', {})),
            *map(float, timeouts))
    return data['port'], entries


class ModelWorkspace:
    def __init__(self, path, record_root, manager, *, readonly=False, simulation=False):
        self.path = Path(path).resolve() if path is not None else None
        self.record_root = Path(record_root)
        self.manager = manager
        self.readonly, self.simulation = readonly, simulation
        self.lock = threading.RLock()
        self.operation = threading.Lock()
        self.cancel = threading.Event()
        self.closed = False
        self.port, self.entries = catalogue(self.path) if self.path is not None else (None, {})
        self.entry = self.server = self.process = self.run_root = self.owner = None
        self.state, self.execution = 'unloaded', 'idle'
        self.operation_kind = 'policy'
        self.phase = ''
        self.home_completed = False
        self.error = ''
        self.stop_reason = ''
        self.video_preview_port = None
        self.test_result = self.fingerprint = None
        self.generation = 0
        from .connectivity import CANConnectivity
        self.connectivity = CANConnectivity(manager)
        self.connectivity.configure(next((e for e in self.entries.values() if e.robot == 'classic'), None))

    def start_connectivity(self):
        if not self.readonly and not self.simulation:
            self.manager.register_passive_reader(self.connectivity)
            self.connectivity.start()

    def select_connection(self, name):
        # Metadata only; actual queries stay in the background, never a UI event.
        with self.lock:
            entry = self._entry(name)
        self.connectivity.configure(entry)
        return self.connectivity.snapshot()

    def _connection_allows(self, entry):
        connection = self.connectivity.snapshot()
        return (entry is None or entry.robot != 'classic' or not connection['enabled']
                or connection['state'] in ('ready', 'busy'))

    def _writable(self):
        if self.readonly or self.closed:
            raise ValueError('只读模式或平台已关闭，不能加载或执行模型')

    def _idle(self):
        if self.process is not None or self.execution in ('starting', 'stopping'):
            raise ValueError('请先停止执行并等待退出清理')

    def _entry(self, name):
        if name not in self.entries:
            raise ValueError('请选择已注册模型；不接受浏览器提供的路径或命令')
        return self.entries[name]

    def _client(self, entry, timeout=2):
        from openarm_policy.models.http import NumpyHTTP
        client = NumpyHTTP(f'http://127.0.0.1:{self.port}/', timeout_s=timeout,
                           expected_identity=entry.identity)
        client.reset(None)
        return client

    def _unchanged(self):
        if self.entry is None or self.entry.fingerprint() != self.fingerprint:
            self.test_result = None
            raise ValueError('模型配置已变化，请重新加载')
        if self.server is None or self.server.poll() is not None:
            self.state = 'error'
            self.test_result = None
            raise ValueError('模型进程已退出，请查看日志并重新加载')

    def refresh(self):
        with self.operation, self.lock:
            self._writable()
            self._idle()
            if self.server is not None:
                raise ValueError('刷新模型目录前请卸载当前模型')
            if self.path is not None:
                self.port, self.entries = catalogue(self.path)
            return self.snapshot()

    def load(self, name):
        with self.operation:
            with self.lock:
                self._writable()
                self._idle()
                entry = self._entry(name)
                fingerprint = entry.fingerprint()  # Validate new inputs before unloading an idle model.
                if self.server is not None and self.entry is entry and self.state != 'error':
                    raise ValueError('当前模型已加载')
                self.state, self.error, self.test_result = 'loading', '', None
                self.cancel.clear()
                self.generation += 1
            try:
                # Inference-only replacement, serialized with load/test/unload;
                # active motion was rejected by _idle() before touching a child.
                self._close_server()
                with self.lock:
                    if self.cancel.is_set() or self.closed:
                        raise RuntimeError('模型加载已取消')
                    if tcp_reachable(self.port):
                        raise ValueError('模型端口已被占用，不能接管外部推理服务')
                    self.entry, self.fingerprint = entry, fingerprint
                    self.connectivity.configure(entry)
                    self.record_root.mkdir(parents=True, exist_ok=True)
                    log_path = self.record_root / f'model-{entry.id}.log'
                with log_path.open('ab', buffering=0) as log:
                    process = subprocess.Popen([entry.inference_python, '-m', 'openarm_runtime',
                        'model-serve', '--profile', str(entry.model_profile), '--port', str(self.port)],
                        cwd=self.path.parent, env=child_environment(entry.inference_env),
                        stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                        start_new_session=os.name == 'posix')
                with self.lock:
                    self.server = process
                deadline = time.monotonic() + entry.load_timeout_s
                while True:
                    if self.cancel.is_set() or self.closed:
                        raise RuntimeError('模型加载已取消')
                    if process.poll() is not None:
                        raise RuntimeError(f'模型进程退出，请查看 {log_path.name}')
                    try:
                        self._client(entry)
                        break
                    except (OSError, TimeoutError):
                        if time.monotonic() >= deadline:
                            raise TimeoutError('模型加载超时')
                        self.cancel.wait(.1)
                with self.lock:
                    if self.cancel.is_set() or self.closed:
                        raise RuntimeError('模型加载已取消')
                    self._unchanged()
                    self.state = 'loaded'
                if entry.probe_inputs is not None:
                    self._check_sample(entry)
                return self.snapshot()
            except BaseException as exc:
                with self.lock:
                    self.error, self.state = str(exc), 'error'
                self._close_server()
                raise

    def test(self):
        """Optional programmatic recheck; the page runs this as part of load()."""
        with self.operation:
            with self.lock:
                self._writable()
                self._idle()
                self._unchanged()
                entry = self.entry
                self.cancel.clear()
            self._check_sample(entry)
            return self.snapshot()

    def _check_sample(self, entry):
        """One bounded, registered-input prediction. Never touch devices."""
        with self.lock:
            self.test_result = None
            self.state = 'checking'
        try:
            if entry.probe_inputs is None:
                raise ValueError('缺少已配置的加载校验样例；不会读取设备或虚构输入')
            import numpy as np
            if entry.probe_inputs.stat().st_size > 2_000_000:
                raise ValueError('加载校验样例过大')
            with zipfile.ZipFile(entry.probe_inputs) as archive:
                if sum(x.file_size for x in archive.infolist()) > 2_000_000:
                    raise ValueError('加载校验样例解压后过大')
            with np.load(entry.probe_inputs, allow_pickle=False) as fixture:
                inputs = {k: fixture[k] for k in fixture.files}
            client = self._client(entry, timeout=entry.test_timeout_s)
            started = time.monotonic()
            output = client.predict(inputs)
            latency = time.monotonic() - started
            if tuple(output.shape) != entry.output_shape or not np.isfinite(output).all():
                raise ValueError('预测形状或数值与注册协议不一致')
            with self.lock:
                if self.cancel.is_set() or self.closed:
                    raise RuntimeError('模型加载已取消')
                self._unchanged()
                self.test_result = dict(shape=list(output.shape), latency_s=latency,
                    scope='加载时自动校验 · 无设备读取、无动作执行，不代表实物验收')
                self.state, self.error = 'tested', ''
        except Exception as exc:
            with self.lock:
                self.state, self.error = 'loaded', str(exc)
            raise

    def _execution_options(self, *, allow_nominal=False, entry=None, home_only=False):
        entry = entry or self.entry
        if entry.policy_profile is None or entry.robot == 'classic' and entry.robot_config is None:
            raise ValueError('该模型仅完成推理接入；尚未注册机器人执行profile')
        if self.simulation and entry.robot != 'simulate':
            raise ValueError('仿真平台禁止启动实物模型控制')
        data = read_json(entry.policy_profile)
        kwargs = data.get('policy_kwargs', {})
        model = kwargs.get('model', {})
        options = model.get('kwargs', {})
        if not home_only and (model.get('factory') != 'openarm_policy.models.http:NumpyHTTP'
                or options.get('url') != f'http://127.0.0.1:{self.port}/'
                or options.get('expected_identity') != entry.identity):
            raise ValueError('执行profile必须绑定当前目录的推理地址和完整expected_identity')
        if entry.robot == 'classic' and data.get('cartesian'):
            if data['cartesian'].get('calibration_verified') is not True:
                nominal = (data['cartesian'].get('calibration_verified') is False and
                    data['cartesian'].get('calibration_status') == 'nominal_model_not_hardware_calibrated')
                if not nominal or not allow_nominal:
                    raise ValueError('当前Cartesian几何未独立实测；需显式接受已声明的名义几何研究试运行')
        return data

    def _validate_home(self, entry, data):
        from openarm_policy.preparation import FixedJointPreparation, reverse_preparation
        from openarm_policy.robots import SimulatedRobot
        from openarm_policy.contracts import Context
        from openarm_policy.session import SessionConfig
        prep = data.get('preparation')
        if data.get('preparation_profile') is not None:
            if prep is not None:
                raise ValueError('只允许一种准备路径配置')
            prep = read_json(entry.policy_profile.parent / data['preparation_profile'])
        if not isinstance(prep, dict) or prep.get('kind') != 'fixed_joint_waypoints_v1':
            raise ValueError('归位需要已配置的固定关节准备路径')
        prep = FixedJointPreparation(**{k: v for k, v in prep.items() if k != 'kind'})
        route = reverse_preparation(prep, data.get('return_home'))
        robot = SimulatedRobot(entry.sides)  # Passive contract validation only.
        route.bind(robot, Context(robot.spec, robot.streams, SessionConfig(**data.get('session', {})).control_hz))

    def start(self, duration=None, *, confirmed=False, owner=None, allow_nominal=False, auto_return_home=False):
        return self._start(duration, confirmed=confirmed, owner=owner, allow_nominal=allow_nominal,
                           auto_return_home=auto_return_home)

    def return_home(self, name, *, confirmed=False, owner=None, allow_nominal=False):
        return self._start(None, confirmed=confirmed, owner=owner, allow_nominal=allow_nominal,
                           home_only=True, name=name)

    def _start(self, duration, *, confirmed=False, owner=None, allow_nominal=False,
               auto_return_home=False, home_only=False, name=None):
        with self.operation:
            with self.lock:
                self._writable()
                self._idle()
                if self.execution == 'failed':
                    raise ValueError('上次退出清理未确认，请先现场检查，不能重新使能归位')
                if not home_only:
                    self._unchanged()
                entry = self._entry(name) if home_only else self.entry
                self.connectivity.require_ready(entry)
                if confirmed is not True or not isinstance(owner, str) or not owner:
                    raise ValueError('必须由当前页面的操作员明确确认启动')
                if not home_only and duration is not None and (isinstance(duration, bool) or not isinstance(duration, (int, float))
                        or not math.isfinite(duration) or not 0 < duration <= self.entry.max_duration_s):
                    raise ValueError('执行时长超出已注册范围')
                if not home_only and self.test_result is None:
                    raise ValueError('模型尚未就绪，请重新加载或检查加载输入配置')
                if type(allow_nominal) is not bool or type(auto_return_home) is not bool:
                    raise ValueError('名义几何研究试运行必须显式选择')
                if not home_only and duration is None and auto_return_home:
                    raise ValueError('手动停止模式不自动归位，请使用独立归位按钮')
                data = self._execution_options(allow_nominal=allow_nominal, entry=entry, home_only=home_only)
                if home_only or auto_return_home:
                    self._validate_home(entry, data)
                fingerprint = entry.fingerprint()
                if not home_only:
                    self._client(entry)
                token = ('home-' if home_only else 'policy-') + uuid.uuid4().hex
                self.execution, self.owner = 'starting', owner
                self.cancel.clear()
            # Normal peer shutdown may take 40 seconds. Keep status/cancel usable
            # and never block the ASGI event loop on the workspace lock.
            try:
                self.manager.claim_motion(token)
            except BaseException as exc:
                with self.lock:
                    self.execution, self.owner, self.error = 'idle', None, str(exc)
                raise
            try:
                with self.lock:
                    if self.cancel.is_set() or self.closed:
                        self.manager.release_motion(token, clean=True)
                        self.execution, self.owner = 'completed', None
                        return self.snapshot()
                    if not home_only:
                        self._unchanged()
                    if entry.fingerprint() != fingerprint:
                        raise ValueError('归位/执行配置在交接时发生变化，请重新检查')
                    root = self.record_root / token
                    root.mkdir(parents=True, exist_ok=False)
                    args = [entry.runtime_python, '-m', 'openarm_runtime']
                    if entry.robot_config is not None:
                        args += ['--config', str(entry.robot_config)]
                    args += ['policy', '--robot', entry.robot, '--sides', *entry.sides,
                        '--policy', 'openarm_policy.models.policy:ModelPolicy',
                        '--profile', str(entry.policy_profile)]
                    if home_only:
                        args += ['--home-only']
                    elif duration is not None:
                        args += ['--duration', str(duration)]
                    else:
                        args += ['--until-stopped']
                    if auto_return_home:
                        args += ['--auto-return-home']
                    args += ['--record', str(root / 'episode'), '--record-mode', 'summary']
                    if allow_nominal:
                        args += ['--allow-nominal-geometry']
                    (root / 'deployment.json').write_text(json.dumps(dict(id=entry.id,
                        identity=entry.identity, configuration_hashes=fingerprint,
                        duration_s=duration, sample_test=self.test_result,
                        allow_nominal_geometry=allow_nominal, operation='home' if home_only else 'policy',
                        auto_return_home=auto_return_home), indent=2) + '\n')
                    with (root / 'console.log').open('ab', buffering=0) as log:
                        process = subprocess.Popen(args, cwd=entry.policy_profile.parent,
                            env=child_environment(entry.runtime_env), stdin=subprocess.DEVNULL,
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=os.name == 'posix')
                    self.process, self.run_root = process, root
                    self.token, self.execution, self.error = token, 'running', ''
                    self.operation_kind = 'home' if home_only else 'policy'
                    self.phase = 'return_home' if home_only else 'preparation'
                    self.home_completed = False
                    self.stop_reason = ''
                    self.watcher = threading.Thread(target=self._watch, daemon=True,
                                                    name='openarm-model-session-monitor')
                    self.watcher.start()
            except BaseException:
                if self.process is None:
                    # No child owns hardware if Popen failed.
                    self.manager.release_motion(token, clean=True)
                    with self.lock:
                        self.execution, self.owner = 'idle', None
                else:
                    # If monitor-thread creation failed, retain the motion lease
                    # until the normal CLI cleanup has actually finished.
                    self.stop()
                    try:
                        self.process.wait(timeout=MOTION_SHUTDOWN_TIMEOUT)
                    except subprocess.TimeoutExpired:
                        self.error = '监控启动失败且控制会话尚未退出，请现场停止；未强制终止'
                    else:
                        with self.lock:
                            self._finished()
                raise
            return self.snapshot()

    def _watch(self):
        while True:
            with self.lock:
                if self.process is None:
                    return
                if self.operation_kind != 'home' and (self.server is None or self.server.poll() is not None):
                    self.error = '模型服务异常退出，正在请求控制会话安全停止'
                    self.stop()
                if self.process.poll() is not None:
                    self._finished()
                    return
                try:
                    report = read_json(self.run_root / 'episode/summary.json')
                    for event in reversed(report.get('lifecycle', [])):
                        if event.get('phase'):
                            self.phase = event['phase']
                            break
                except (OSError, ValueError, TypeError):
                    pass  # Summary creation/atomic replacement is asynchronous.
            time.sleep(.1)

    def _finished(self):
        code = self.process.poll()
        termination = None
        try:
            report = read_json(self.run_root / 'episode/summary.json')
            termination = report.get('termination')
            self.home_completed = any(e.get('event') == 'return_home_completed' for e in report.get('lifecycle', []))
            self.stop_reason = termination.get('stop_reason', '') if isinstance(termination, dict) else ''
            clean = (report.get('complete') is True
                     and isinstance(report.get('termination'), dict)
                     and report['termination'].get('cleanup_errors') == [])
        except (OSError, ValueError, TypeError):
            clean = False
        self.manager.release_motion(self.token, clean=clean)
        self.execution = 'completed' if clean and code == 0 else 'stopped_error' if clean else 'failed'
        if clean and code != 0:
            reason = report['termination'].get('error') or '控制进程异常退出'
            self.error = f'{reason}（退出清理已确认）'
        elif not clean:
            self.error = '会话结束但未确认正常清理，请现场检查；不会自动恢复其他控制服务'
            reason = termination.get('error') if isinstance(termination, dict) else None
            if isinstance(reason, str) and reason.strip():
                self.error += f'。原始错误：{reason}'
        self.process, self.owner = None, None

    def stop(self, *, owner=None):
        if owner is None:
            self.cancel.set()
        with self.lock:
            if self.execution == 'starting' and (owner is None or self.owner == owner):
                self.cancel.set()
                self.execution = 'stopping'
            if self.process is not None and (owner is None or self.owner == owner):
                if self.process.poll() is None:
                    self.process.terminate()  # Only the CLI parent; never force-kill motor control.
                self.execution = 'stopping'

    def _close_server(self):
        with self.lock:
            process = self.server
        if process is not None:
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()  # This child is inference-only, with no hardware backend.
                process.wait(timeout=5)
            with self.lock:
                self.server = None

    def unload(self):
        self.cancel.set()
        with self.operation:
            with self.lock:
                self._writable()
                self._idle()
            self._close_server()
            with self.lock:
                self.entry, self.test_result, self.fingerprint = None, None, None
                self.state, self.error = 'unloaded', ''
            return self.snapshot()

    def snapshot(self):
        with self.lock:
            if self.server is not None and self.server.poll() is not None:
                self.state, self.test_result = 'error', None
                self.error = self.error or '模型进程已退出，请重新加载'
            blockers = []
            requires_nominal = False
            if self.entry is not None:
                if self.entry.probe_inputs is None:
                    blockers.append('缺少加载校验样例；仅加载推理进程，不能执行')
                try:
                    options = self._execution_options(allow_nominal=True)
                    requires_nominal = (self.entry.robot == 'classic' and
                        bool(options.get('cartesian')) and
                        options['cartesian'].get('calibration_verified') is not True)
                except (ValueError, OSError, KeyError, TypeError) as exc:
                    blockers.append(str(exc))
            return dict(models=[e.public() for e in self.entries.values()], state=self.state,
                execution=self.execution, selected=self.entry.id if self.entry else None,
                error=self.error, sample_test=self.test_result, blockers=blockers,
                stop_reason=self.stop_reason,
                phase=self.phase, operation=self.operation_kind, home_completed=self.home_completed,
                connection=self.connectivity.snapshot(),
                can_return_home=not self.readonly and not self.closed and self.process is None
                    and self.state not in ('loading', 'checking') and self.execution not in ('starting', 'stopping', 'failed'),
                requires_nominal_opt_in=requires_nominal,
                can_start=not self.readonly and not self.closed and self.state == 'tested'
                    and self._connection_allows(self.entry)
                    and not blockers and self.process is None
                    and self.execution not in ('starting', 'stopping', 'failed'),
                running=self.process is not None or self.execution in ('starting', 'stopping'),
                readonly=self.readonly, generation=self.generation,
                record=str(self.run_root) if self.run_root else None)

    def input_status(self, name=None):
        """Configured source names and actual last inference inputs, no devices."""
        with self.lock:
            entry = self.entries.get(name) if name else self.entry
            result = {'cameras': [], 'images': [], 'state': {}, 'recorded_at': None,
                      'observation_sequence': None, 'rerun_available': False, 'run_id': None}
            if entry is None or entry.policy_profile is None:
                return result
            data = read_json(entry.policy_profile)
            options = data.get('policy_kwargs', {}).get('observation', {}).get('kwargs', {})
            selected = {v.removeprefix('camera/') for v in options.values()
                        if isinstance(v, str) and v.startswith('camera/')}
            from urllib.parse import urlsplit
            video = data.get('video_http', {})
            url = urlsplit(video.get('url', ''))
            if url.hostname in ('127.0.0.1', 'localhost', '::1') and url.port == self.video_preview_port and url.port is not None:
                result['cameras'] = [n for n in video.get('names', []) if not selected or n in selected]
            if (self.run_root is not None and entry is self.entry and
                    read_json(self.run_root/'deployment.json').get('id') == entry.id):
                result['run_id'] = self.run_root.name
                manifest = self.run_root / 'episode/model-inputs/inputs.json'
                if manifest.is_file():
                    result.update(read_json(manifest))
                result['rerun_available'] = self.process is None and (self.run_root/'episode/recording.rrd').is_file()
            return result

    def close(self):
        self.stop()
        self.connectivity.close()
        with self.lock:
            self.closed = True
            process = self.process
        if process is not None:
            try:
                process.wait(timeout=MOTION_SHUTDOWN_TIMEOUT)
            except subprocess.TimeoutExpired as exc:
                raise RuntimeError('控制会话尚未安全退出；没有强制终止或卸载模型') from exc
            self.watcher.join(timeout=2)
            if self.watcher.is_alive():
                raise RuntimeError('控制会话清理尚未完成')
        # A load operation observes closed/cancel and performs its own cleanup.
        with self.operation:
            self._close_server()
