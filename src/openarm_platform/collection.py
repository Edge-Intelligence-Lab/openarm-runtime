"""Read-only native collection status and bounded, causal archive inspection."""
import base64
from bisect import bisect_right
from collections import defaultdict, deque
import hashlib
import json
import math
from pathlib import Path
import time


STATE_LABELS = {'idle': '待机', 'ready': '可录制', 'recording': '录制中', 'draining': '写入中',
                'review': '待验收', 'incomplete': '数据不完整', 'closed': '已关闭'}


def read_json(path):
    if not path.is_file() or path.stat().st_size > 1024 * 1024:
        raise ValueError('元数据路径无效或超过 1 MiB')
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError('元数据必须是 JSON 对象')
    return value


def inside(root, path):
    resolved = path.resolve()
    if not resolved.is_relative_to(root):
        raise ValueError('记录路径越过采集目录')
    return resolved


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError('采样时间或数值无效')
    return float(value)


def archive_state(manifest, review):
    state = '未结束' if manifest['state'] == 'recording' else STATE_LABELS.get(manifest['state'], manifest['state'])
    if type(review.get('accepted')) is bool:
        decision = '已验收' if review['accepted'] else '已弃用'
        state = decision if manifest['state'] == 'review' else f'{state} · {decision}'
    return state


class CollectionBrowser:
    def __init__(self, root):
        self.root = Path(root).expanduser().resolve()

    def live(self, now_ns=None):
        directory = inside(self.root, self.root/'.runtime')
        result = []
        for path in directory.glob('*.json') if directory.is_dir() else ():
            try:
                value = read_json(inside(directory, path))
                if value.get('schema') != 'openarm.collection-status/1':
                    continue
                # The publisher may atomically replace the file during I/O.
                # Observe now after reading, not before the newer snapshot exists.
                observed_ns = time.time_ns() if now_ns is None else now_ns
                age = (observed_ns-finite(value['updated_wall_time_ns'])) / 1e9
                value['fresh'] = 0 <= age <= 3
                value['status_age_s'] = max(0, age)
                result.append(value)
            except (OSError, ValueError, KeyError, TypeError):
                continue
        return sorted(result, key=lambda v:(v['fresh'] and v.get('state') != 'closed', v['updated_wall_time_ns']), reverse=True)

    def episodes(self):
        result = {}
        if not self.root.is_dir():
            return result
        for path in self.root.iterdir():
            if path.name.startswith('.') or path.is_symlink() or not path.is_dir() or not (path/'manifest.json').is_file():
                continue
            try:
                manifest = read_json(inside(self.root, path/'manifest.json'))
                if manifest.get('schema') != 'openarm.collection/1':
                    continue
                task = str(manifest.get('metadata', {}).get('task', '未命名任务'))
                review = read_json(inside(self.root, path/'review.json')) if (path/'review.json').exists() else {}
                state = archive_state(manifest, review)
                result[path.name] = f'{task} · {state} · {path.name[:8]}'
            except (OSError, ValueError, KeyError, TypeError, AttributeError):
                result[path.name] = f'元数据异常 · {path.name[:8]}'
        return dict(sorted(result.items(), reverse=True))

    def load(self, name):
        if not isinstance(name, str) or not name or Path(name).name != name:
            raise ValueError('请选择一个采集会话')
        path = inside(self.root, self.root/name)
        if path.parent != self.root:
            raise ValueError('请选择采集目录中的会话')
        try:
            return EpisodeTimeline(path)
        except (KeyError, TypeError, AttributeError, IndexError) as exc:
            raise ValueError(f'数采记录结构无效：{exc}') from exc


class EpisodeTimeline:
    def __init__(self, root):
        self.root = root.resolve()
        self.manifest = read_json(inside(self.root, self.root/'manifest.json'))
        if self.manifest.get('schema') != 'openarm.collection/1':
            raise ValueError('不支持的数采记录格式')
        self.review = read_json(inside(self.root, self.root/'review.json')) if (self.root/'review.json').exists() else {}
        if 'accepted' in self.review and type(self.review['accepted']) is not bool:
            raise ValueError('验收标记必须是布尔值')
        self.origin = finite(self.manifest['started_at'])
        self.streams = dict(self.manifest['streams'])
        if len(self.streams) > 64:
            raise ValueError('最多预览 64 路数据流')
        samples = defaultdict(lambda: deque(maxlen=2000))
        self.counts = defaultdict(int)
        self.problems = list(self.manifest.get('quality', {}).get('problems', ()))
        path = inside(self.root, self.root/'samples.jsonl')
        if not path.is_file() or path.stat().st_size > 128 * 1024**2:
            raise ValueError('采样记录无效或超过 128 MiB 预览上限')
        digest, total, previous = hashlib.sha256(), 0, {}
        with path.open('rb') as handle:
            while line := handle.readline(2 * 1024**2 + 1):
                total += len(line)
                if len(line) > 2 * 1024**2 or total > 128 * 1024**2:
                    raise ValueError('采样记录超过预览上限')
                digest.update(line)
                try:
                    sample = json.loads(line)
                except ValueError:
                    if not line.endswith(b'\n') and self.manifest['state'] == 'recording':
                        self.problems.append('末条采样尚未写完，已显示可读部分')
                        break
                    raise ValueError('采样记录损坏') from None
                stream = sample['stream']
                timestamp = finite(sample['timestamp'])
                if stream not in self.streams or timestamp < previous.get(stream, -math.inf):
                    raise ValueError('数据流未知或时间戳倒退')
                previous[stream] = timestamp
                samples[stream].append(sample)
                self.counts[stream] += 1
        expected = self.manifest.get('journal_sha256')
        if expected and digest.hexdigest() != expected:
            self.problems.append('采样记录 SHA256 校验不匹配')
        self.samples = {key: list(values) for key, values in samples.items()}
        self.times = {key: [s['timestamp']-self.origin for s in values] for key, values in self.samples.items()}
        self.start = max((values[0] for values in self.times.values() if values), default=0.)
        self.end = max((values[-1] for values in self.times.values() if values), default=0.)
        self.start = min(self.start, self.end)
        self.joints = {}
        for stream, values in self.samples.items():
            if not stream.startswith('robot/'):
                continue
            for mid in range(1, 9):
                target, feedback = [], []
                for sample in values:
                    data = sample['data']
                    goal = data.get('targets_rad', {}).get(str(mid))
                    state = data.get('feedback', {}).get(str(mid))
                    if goal is not None:
                        target.append([finite(data.get('dispatch_started_at', sample['timestamp']))-self.origin, math.degrees(finite(goal))])
                    if state is not None:
                        feedback.append([finite(state.get('received_at', data.get('exchange_completed_at', sample['timestamp'])))-self.origin,
                                         math.degrees(finite(state['position']))])
                self.joints[f'{stream}/{mid}'] = (sorted(target), sorted(feedback))
                self.end = max([self.end] + [v[-1][0] for v in (target, feedback) if v])

    def at(self, stream, elapsed):
        times = self.times.get(stream, ())
        index = bisect_right(times, elapsed) - 1
        return self.samples[stream][index] if index >= 0 else None

    def array(self, record):
        import numpy as np
        data = record['data']
        path = inside(self.root, self.root/data['array'])
        if path.suffix != '.npy' or not path.is_file() or path.stat().st_size > 64 * 1024**2:
            raise ValueError('采样数组路径或大小无效')
        if data.get('sha256') and hashlib.sha256(path.read_bytes()).hexdigest() != data['sha256']:
            raise ValueError('采样数组 SHA256 校验不匹配')
        values = np.load(path, mmap_mode='r', allow_pickle=False)
        if 'shape' in data and tuple(data['shape']) != values.shape or 'dtype' in data and str(values.dtype) != data['dtype']:
            raise ValueError('采样数组与记录元数据不符')
        return values

    def camera(self, stream, elapsed):
        record = self.at(stream, elapsed)
        if record is None:
            return None
        import cv2
        import numpy as np
        frame = self.array(record)
        encoding = self.streams[stream].get('encoding')
        if encoding == 'bgr8' and frame.dtype == np.uint8 and frame.ndim == 3 and frame.shape[2] == 3:
            image = frame
        elif encoding == 'depth_mm' and frame.dtype == np.uint16 and frame.ndim == 2:
            image = cv2.applyColorMap(np.clip(frame.astype(float)/4000*255, 0, 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
        else:
            raise ValueError('不支持的相机图像格式')
        height, width = image.shape[:2]
        if not height or not width:
            raise ValueError('图像尺寸无效')
        image = cv2.resize(image, (min(640, width), max(1, round(height*min(640,width)/width))))
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            raise ValueError('预览图编码失败')
        return dict(source='data:image/jpeg;base64,'+base64.b64encode(encoded).decode(),
                    age_s=elapsed-(record['timestamp']-self.origin), sequence=record['sequence'],
                    encoding=encoding)

    def tactile(self, stream, elapsed):
        record = self.at(stream, elapsed)
        if record is None:
            return None
        values = self.array(record).tolist() if isinstance(record['data'], dict) else record['data']
        if len(values) != 69 or any(len(row) != 3 for row in values):
            raise ValueError('触觉采样必须是 69 × 3 XYZ 原始值')
        rows = [[finite(v) for v in row] for row in values]
        return dict(resultant=rows[0], taxels=rows[1:], age_s=elapsed-(record['timestamp']-self.origin))

    def joint_rows(self, elapsed):
        rows = []
        for name, (targets, feedback) in self.joints.items():
            def latest(series):
                index = bisect_right([p[0] for p in series], elapsed)-1
                return series[index][1] if index >= 0 else None
            target, measured = latest(targets), latest(feedback)
            rows.append(dict(joint=name.replace('robot/', '').replace('/', ' · J'),
                             target='—' if target is None else f'{target:+.2f}',
                             feedback='—' if measured is None else f'{measured:+.2f}'))
        return rows
