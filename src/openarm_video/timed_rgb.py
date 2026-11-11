"""Optional Linux V4L2 MJPEG source with auditable kernel capture metadata.

No camera/compiler/subprocess is touched on import. The single video owner runs
the helper; calibration only borrows its frames. Qualification is conservative
capture timing, not a claim of hardware exposure synchronization.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import select
import subprocess
import threading
import time

from .camera import Frame, _DeviceLease


def build_rgb_helper():
    import tempfile
    source = Path(__file__).with_name('native')/'rgb_capture.cpp'
    sha = hashlib.sha256(source.read_bytes()).hexdigest()
    root = Path.home()/'.cache/openarm-runtime/native'/sha
    root.mkdir(parents=True, exist_ok=True, mode=0o700)
    target = root/'rgb_capture'
    if not target.exists():
        fd, name = tempfile.mkstemp(prefix='rgb-', dir=root)
        os.close(fd)
        try:
            subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', str(source), '-o', name],
                           check=True, capture_output=True, timeout=60)
            os.chmod(name, 0o700)
            os.replace(name, target)
        finally:
            Path(name).unlink(missing_ok=True)
    return target, sha


def acquisition_metadata(header, received, source_sha):
    """Fail closed on unknown/copied/realtime timestamps or unbounded exposure.

    V4L2 timestamp flags explicitly distinguish SOE and EOF. SOE is bounded above
    by complete host reception: exposure duration is not guessed. EOF requires a
    queried integration bound plus one frame period. Kernel time precedes dequeue,
    so backlog cannot become falsely fresh. Copy/unknown clocks are rejected.
    """
    flags = header['flags']
    timestamp, period = header['timestamp_s'], header['period_s']
    exposure = header['exposure_100us']
    if (type(flags) is not int or flags & 0xe000 != 0x2000 or flags & 0x70000 not in (0, 0x10000)
            or type(timestamp) not in (int, float) or not math.isfinite(timestamp)
            or type(period) not in (int, float) or not math.isfinite(period)
            or not 1/120 <= period <= .1 or not 0 < timestamp <= received
            or received-timestamp > .5):
        raise ValueError('驱动时间戳、曝光上界或帧新鲜度未通过取样检查')
    soe = flags & 0x70000 == 0x10000
    if not soe and (header.get('exposure_readable') is not True
                    or type(exposure) is not int or not 0 < exposure <= 2000):
        raise ValueError('EOF 时间戳缺少可读曝光上界，无法建立采集区间')
    earliest = timestamp if soe else timestamp-period-exposure*.0001
    latest = received if soe else timestamp
    qualification = dict(schema='openarm.v4l2-timing-qualification.v1', helper_sha256=source_sha,
        interpretation_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        driver_version=header['driver_version'], timestamp_clock='monotonic', timestamp_source='SOE' if soe else 'EOF',
        timestamp_flags=flags, frame_period_s=period, integration_100us=exposure,
        integration_readable=header.get('exposure_readable') is True,
        interval_rule='SOE_to_complete_host_receive' if soe else 'EOF_minus_integration_minus_one_frame_period',
        exposure_timestamp_known=False)
    return dict(clock='host_monotonic', semantics='qualified_acquisition_interval',
        qualification='v4l2_monotonic_capture', earliest_s=earliest,
        latest_s=latest, host_received_at_s=received,
        qualification_evidence_sha256=hashlib.sha256(json.dumps(qualification, sort_keys=True,
            separators=(',', ':'), allow_nan=False).encode()).hexdigest(), qualification_evidence=qualification,
        kernel_sequence=header['kernel_sequence'])


class TimedRGBCamera:
    def __init__(self, config):
        import cv2
        if config.kind != 'rgb' or not config.device or int(config.fps) != config.fps:
            raise ValueError('V4L2 timed RGB needs explicit device and integer fps')
        self.config, self.cv2 = config, cv2
        self.lease = _DeviceLease(str(Path(config.device).expanduser().resolve()))
        self.process = None
        self.lock = threading.Lock()
        self.pending = bytearray()
        try:
            helper, self.source_sha = build_rgb_helper()
            self.process = subprocess.Popen([str(helper), config.device, str(config.width),
                str(config.height), str(int(config.fps))], stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL)
        except BaseException:
            self.close()
            raise

    def _read(self, count=None):
        deadline = time.monotonic()+3
        while True:
            if count is None:
                end = self.pending.find(b'\n')
                if end >= 0:
                    value = bytes(self.pending[:end]);del self.pending[:end+1]
                    return value
                if len(self.pending) > 4096:
                    raise ValueError('RGB helper header exceeds bound')
            elif len(self.pending) >= count:
                value = bytes(self.pending[:count]);del self.pending[:count]
                return value
            left = deadline-time.monotonic()
            if left <= 0 or not select.select([self.process.stdout], [], [], max(0, left))[0]:
                raise TimeoutError('RGB helper timed out')
            data = os.read(self.process.stdout.fileno(), 65536)
            if not data:
                raise OSError('RGB helper exited; inspect device mode and permissions')
            self.pending.extend(data)

    def read(self):
        import numpy as np
        with self.lock:
            if self.process is None:
                raise OSError('RGB source closed')
            header = json.loads(self._read())
            count = header['bytes']
            if type(count) is not int or not 1 <= count <= 24_000_000:
                raise ValueError('Invalid RGB payload length')
            raw = self._read(count)
            received = time.monotonic()
            image = self.cv2.imdecode(np.frombuffer(raw, np.uint8), self.cv2.IMREAD_COLOR)
            if image is None or image.shape != (self.config.height, self.config.width, 3):
                raise ValueError('Driver RGB geometry changed')
            try:
                timing = acquisition_metadata(header, received, self.source_sha)
            except ValueError as exc:
                timing = dict(qualified=False, reason=str(exc), kernel_metadata=header)
            return Frame(image, received, header['kernel_sequence'], acquisition=timing)

    def close(self):
        # Termination closes the helper's only device fd, including blocked pipe
        # writes; then read() exits before releasing the cooperating device lease.
        process = self.process
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill();process.wait(timeout=3)
            with self.lock:
                process.stdout.close()
                self.process = None
        self.lease.close()
