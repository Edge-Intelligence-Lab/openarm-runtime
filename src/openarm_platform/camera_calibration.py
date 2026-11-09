"""Bounded, owner-controlled calibration evidence. Never owns devices or CAN."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import struct
import threading
import time
import uuid

MAX_PNG = 24_000_000
MAX_IMPORT = 2_000_000


def document(raw, limit=MAX_IMPORT):
    if not isinstance(raw, bytes) or len(raw) > limit:
        raise ValueError('文件超过大小限制')
    def reject(value):
        raise ValueError('JSON 不允许非有限数值')
    def pairs(values):
        result = {}
        for key, value in values:
            if key in result:
                raise ValueError('JSON 字段重复')
            result[key] = value
        return result
    value = json.loads(raw, parse_constant=reject, object_pairs_hook=pairs)
    if not isinstance(value, dict):
        raise ValueError('需要 JSON 对象')
    return value


def atomic_bytes(path, value):
    """Atomic create, never replace an existing result (including symlinks)."""
    temporary = path.with_name('.'+uuid.uuid4().hex+'.tmp')
    try:
        with temporary.open('xb') as stream:
            os.fchmod(stream.fileno(), 0o600)
            stream.write(value)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_json(path, value):
    atomic_bytes(path, json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False).encode())


def decode_frame(raw, metadata):
    """Check PNG dimensions before native decode; bind header to exact bytes."""
    if (not isinstance(raw, bytes) or not 24 <= len(raw) <= MAX_PNG
            or raw[:8] != b'\x89PNG\r\n\x1a\n' or raw[12:16] != b'IHDR'):
        raise ValueError('需要有界原尺寸 PNG')
    if not isinstance(metadata, dict) or metadata.get('schema') != 'openarm.calibration-frame.v1':
        raise ValueError('视频服务尚未提供标定帧契约')
    width, height = struct.unpack('>II', raw[16:24])
    if not 1 <= width <= 4096 or not 1 <= height <= 4096 or width*height > 8_000_000:
        raise ValueError('图像尺寸超过限制')
    for key in ('camera_name', 'camera_identity', 'source_id'):
        v = metadata.get(key)
        if not isinstance(v, str) or not v or len(v) > 256 or not v.isprintable():
            raise ValueError('缺少相机身份或采集代次')
    if (metadata.get('width') != width or metadata.get('height') != height
            or metadata.get('png_sha256') != hashlib.sha256(raw).hexdigest()
            or metadata.get('pixel_format') != 'bgr8'
            or metadata.get('image_geometry') != 'native_no_transform'
            or metadata.get('timestamp_semantics') != 'host_received_not_exposure'
            or type(metadata.get('sequence')) is not int or metadata['sequence'] < 1
            or type(metadata.get('simulation')) is not bool
            or not isinstance(metadata.get('mode'), dict)
            or set(metadata['mode']) != {'width', 'height', 'fps'}):
        raise ValueError('原图和元数据不匹配或图像几何不受支持')
    for key in ('received_at_s', 'age_s'):
        v = metadata.get(key)
        if type(v) not in (int, float) or not math.isfinite(v) or v < 0:
            raise ValueError('图像接收时间无效')
    if metadata['age_s'] > 1:
        raise ValueError('标定帧已过期')
    elapsed = metadata.get('transport_elapsed_s', 0.)
    if (type(elapsed) not in (int, float) or not math.isfinite(elapsed)
            or elapsed < 0 or elapsed+metadata['age_s'] > 1):
        raise ValueError('标定帧传输超时或已过期')
    import cv2
    import numpy as np
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape != (height, width, 3):
        raise ValueError('PNG 解码失败')
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def borrow_frame(port, name):
    """Only a configured loopback service; no redirects, proxies or device opens."""
    from urllib.request import build_opener, ProxyHandler, HTTPRedirectHandler
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_-]*', name):
        raise ValueError('无效相机名称')
    class NoRedirect(HTTPRedirectHandler):
        def redirect_request(self, *args, **kwargs):
            raise ValueError('标定帧禁止重定向')
    opener = build_opener(ProxyHandler({}), NoRedirect())
    started = time.monotonic()
    with opener.open(f'http://127.0.0.1:{int(port)}/calibration/frame/{name}.png', timeout=3) as response:
        header = response.headers.get('X-Calibration-Metadata', '')
        if response.headers.get_content_type() != 'image/png' or len(header) > 8192:
            raise ValueError('视频服务不支持原尺寸标定帧')
        metadata = document(header.encode(), 8192)
        raw = response.read(MAX_PNG+1)
    metadata['transport_elapsed_s'] = time.monotonic()-started
    # This is transport freshness, not exposure time or robot synchronization.
    return raw, metadata


class CalibrationWorkspace:
    def __init__(self, root, video_port, *, clock=time.monotonic, fetch=borrow_frame):
        self.root = Path(root).expanduser().resolve()
        self.video_port, self.clock, self.fetch = video_port, clock, fetch
        self.lock = threading.RLock()
        self.owner = None
        self.epoch = 0
        self.busy = False
        self.closed = False
        self.state = 'idle'
        self.samples = []
        self.result = self.head = self.identity = None
        self.session_id = None
        self.bytes = 0
        self.last_touch = 0

    def _owner(self, owner):
        if self.closed or self.owner != owner or self.state not in ('collecting', 'estimated'):
            raise ValueError('当前浏览器没有有效标定会话；请重新开始')
        if self.clock()-self.last_touch > 120:
            self.epoch += 1
            self.state = 'cancelled'
            raise ValueError('标定会话已超时暂停；取消不是急停')
        self.last_touch = self.clock()

    def _directory(self):
        path = self.root/self.session_id
        if path.is_symlink() or path.resolve() != path or not path.is_dir():
            raise ValueError('标定会话目录被替换或不可用')
        return path

    def status(self, owner):
        with self.lock:
            if owner == self.owner and self.state in ('collecting', 'estimated'):
                try:
                    self._owner(owner)
                except ValueError:
                    pass
            return dict(state=self.state, mine=self.owner == owner, busy=self.busy,
                        session_id=self.session_id, samples=copy.deepcopy(self.samples),
                        result=copy.deepcopy(self.result), head=copy.deepcopy(self.head),
                        realtime_extrinsics=False,
                        realtime_reason='缺少可信曝光时间与实测 FK 配对；实时外参采集禁用')

    def start(self, owner, camera_name, spec, camera_model='pinhole'):
        from openarm_runtime.camera_intrinsics import board
        if not isinstance(owner, str) or not owner or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_-]*', camera_name):
            raise ValueError('无效的会话或相机名称')
        if camera_model != 'pinhole':
            raise ValueError('首版仅支持普通针孔镜头，不支持鱼眼')
        board(spec)
        with self.lock:
            if self.closed or self.busy:
                raise ValueError('标定任务忙碌或平台正在关闭')
            if self.state in ('collecting', 'estimated') and self.clock()-self.last_touch <= 120:
                raise ValueError('已有标定会话；请由当前操作者先取消或保存')
            self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
            session_id = uuid.uuid4().hex
            path = self.root/session_id
            path.mkdir(mode=0o700)
            atomic_json(path/'session.json', dict(schema='openarm.camera-calibration.session.v1',
                session_id=session_id, camera_name=camera_name, board_spec=spec,
                camera_model=camera_model, realtime_extrinsics=False))
            self.epoch += 1
            self.owner, self.session_id = owner, session_id
            self.spec, self.camera_name = copy.deepcopy(spec), camera_name
            self.state, self.samples, self.bytes = 'collecting', [], 0
            self.identity = self.result = self.head = None
            self.last_touch = self.clock()
            return session_id

    def _begin(self, owner):
        with self.lock:
            self._owner(owner)
            if self.busy:
                raise ValueError('标定任务正在处理，请稍候')
            self.busy = True
            return self.epoch

    def _check(self, owner, epoch):
        self._owner(owner)
        if epoch != self.epoch:
            raise ValueError('任务已取消；迟到结果不会应用')

    def capture(self, owner, role):
        if role not in ('fit', 'validation'):
            raise ValueError('采样前必须指定拟合或留出验证')
        epoch = self._begin(owner)
        try:
            raw, metadata = self.fetch(self.video_port, self.camera_name)
            image = decode_frame(raw, metadata)
            if metadata['camera_name'] != self.camera_name:
                raise ValueError('相机名称不匹配')
            from openarm_runtime.camera_intrinsics import detect
            detection = detect(image, self.spec)
            identity = {k: metadata[k] for k in ('camera_name', 'camera_identity', 'source_id',
                'width', 'height', 'pixel_format', 'image_geometry', 'mode', 'simulation')}
            with self.lock:
                self._check(owner, epoch)
                if self.identity is not None and self.identity != identity:
                    raise ValueError('相机身份、模式、尺寸或采集代次变化；请新建会话')
                if len(self.samples) >= 64 or self.bytes+len(raw) > 256*1024**2:
                    raise ValueError('会话达到 64 帧或 256 MiB 限制')
                if any(s['metadata']['sequence'] == metadata['sequence'] for s in self.samples):
                    raise ValueError('拒绝重复采集同一帧')
                sample = dict(id=uuid.uuid4().hex, role=role, metadata=metadata, detection=detection)
                directory = self._directory()/sample['id']
                directory.mkdir(mode=0o700)
                atomic_bytes(directory/'image.png', raw)
                atomic_json(directory/'sample.json', sample)
                self.samples.append(sample)
                self.bytes += len(raw)
                self.identity = identity
                self.result = None
                self.state = 'collecting'
                return copy.deepcopy(sample)
        finally:
            with self.lock:
                self.busy = False

    def solve(self, owner):
        epoch = self._begin(owner)
        try:
            with self.lock:
                samples, identity = copy.deepcopy(self.samples), copy.deepcopy(self.identity)
            from openarm_runtime.camera_intrinsics import solve
            if identity is None:
                raise ValueError('尚未采集图像')
            result = solve(samples, [identity['width'], identity['height']])
            result.update(camera=identity, board_spec=copy.deepcopy(self.spec),
                          implementation=self.implementation())
            with self.lock:
                self._check(owner, epoch)
                self.result, self.state = result, 'estimated'
                return copy.deepcopy(result)
        finally:
            with self.lock:
                self.busy = False

    def import_head(self, owner, raw):
        epoch = self._begin(owner)
        try:
            if self.spec.get('type') == 'checkerboard':
                raise ValueError('普通棋盘格尚未建立跨图像固定板坐标方向；本会话仅支持内参，不接受外参导入')
            observations = document(raw)
            if observations.get('mode') != 'head' or len(observations.get('samples', [])) > 256:
                raise ValueError('仅支持最多 256 组 head 外参观测，不支持腕部')
            if observations.get('camera_name') != self.camera_name:
                raise ValueError('导入相机名称必须与会话相同')
            from openarm_runtime.camera_calibration import fit
            result = fit(observations)
            result['observations_sha256'] = hashlib.sha256(raw).hexdigest()
            result['provenance_boundary'] = 'operator_supplied_not_runtime_verified'
            result['web_intrinsics_binding'] = 'not_bound_to_web_intrinsics'
            result['implementation'] = self.implementation()
            with self.lock:
                self._check(owner, epoch)
                path = self._directory()/('head-'+uuid.uuid4().hex)
                path.mkdir(mode=0o700)
                atomic_bytes(path/'observations.json', raw)
                atomic_json(path/'estimate.json', result)
                self.head = dict(result, evidence_id=path.name)
                return copy.deepcopy(self.head)
        finally:
            with self.lock:
                self.busy = False

    @staticmethod
    def implementation():
        from importlib.metadata import version
        from openarm_runtime import __version__
        import openarm_runtime.camera_calibration as extrinsics
        import openarm_runtime.camera_intrinsics as intrinsics
        return dict(runtime_version=__version__,
            packages={name:version(name) for name in ('numpy','scipy','opencv-python-headless')},
            source_sha256={name:hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()
                for name,module in [('intrinsics',intrinsics),('extrinsics',extrinsics)]})

    def save(self, owner, *, confirmed=False):
        if confirmed is not True:
            raise ValueError('必须明确确认保存；保存不等于激活')
        with self.lock:
            self._owner(owner)
            if self.busy or (self.result is None and self.head is None):
                raise ValueError('尚无可保存估计，或任务正在处理')
            path = self._directory()/'candidate.json'
            files = ['session.json']
            for sample in self.samples:
                files.extend([sample['id']+'/image.png', sample['id']+'/sample.json'])
            if self.head:
                files.extend([self.head['evidence_id']+'/observations.json',
                              self.head['evidence_id']+'/estimate.json'])
            evidence = {}
            for name in files:
                source = path.parent/name
                if source.is_symlink() or source.resolve() != source:
                    raise ValueError('证据路径被替换')
                with source.open('rb') as stream:
                    evidence[name] = hashlib.file_digest(stream, 'sha256').hexdigest()
            for sample in self.samples:
                if (evidence[sample['id']+'/image.png'] != sample['metadata']['png_sha256']
                        or document((path.parent/sample['id']/'sample.json').read_bytes()) != sample):
                    raise ValueError('原始标定证据已改变，拒绝保存')
            if self.head and evidence[self.head['evidence_id']+'/observations.json'] != self.head['observations_sha256']:
                raise ValueError('外参观测证据已改变')
            atomic_json(path, dict(schema='openarm.camera-calibration.candidate.v1',
                session_id=self.session_id, intrinsics=self.result, head_extrinsics=self.head,
                camera_name=self.camera_name, board_spec=self.spec,
                calibration_status='estimated_not_independently_verified', installed=False,
                evidence_sha256=evidence,
                samples=[dict(id=s['id'], role=s['role'], png_sha256=s['metadata']['png_sha256']) for s in self.samples]))
            self.state = 'saved'
            return self.session_id+'/candidate.json'

    def cancel(self, owner):
        with self.lock:
            if owner != self.owner:
                raise ValueError('只有当前标定操作者可以取消')
            self.epoch += 1
            if self.state not in ('saved', 'idle'):
                self.state = 'cancelled'

    def close(self):
        with self.lock:
            self.closed = True
            self.epoch += 1
            if self.state in ('collecting', 'estimated'):
                self.state = 'cancelled'
