"""Operator lifecycle borrowing native capture owners; no hardware control."""
import copy
import threading
import time

from .recording import Episode, review_episode


class Collection:
    def __init__(self, root, *, capacity=512, max_bytes=256 * 1024**2):
        self.root, self.capacity, self.max_bytes = root, capacity, max_bytes
        self.lock = threading.RLock()
        self.operation = threading.Lock()
        self.episode = None
        self.last = None
        self.streams = {}
        self.latest = {}
        self.sequence = {}
        self.context = {}
        self.previews = {}
        self.tactile_values = {}
        self.previous_episode = None
        self.last_state = None
        self.last_metadata = {}
        self.last_duration = 0.0
        self.ready = False
        self.message = "待机"
        self.closed = threading.Event()
        self.monitor = threading.Thread(target=self._monitor, name="collection-health", daemon=True)
        self.monitor.start()
        from .status import StatusPublisher
        self.status_publisher = StatusPublisher(self.root, self.status_snapshot)

    def register(self, name, **spec):
        with self.lock:
            if self.episode:
                raise RuntimeError("Cannot change streams during recording")
            self.streams[name] = {"required": True, "max_age_s": .5, **spec}

    def emit(self, name, timestamp, data, sequence=None):
        with self.lock:
            if name not in self.streams:
                return
            if sequence is None:
                sequence = self.sequence.get(name, 0) + 1
            self.sequence[name] = sequence
            self.latest[name] = timestamp
            if self.episode:
                self.episode.submit(name, sequence, timestamp, data)

    def start(self, task, operator):
        with self.operation:
            return self._start(task, operator)

    def _start(self, task, operator):
        with self.lock:
            if self.closed.is_set() or not self.ready:
                raise RuntimeError("Start following and wait for takeover before recording")
            if self.episode or self.last:
                raise RuntimeError("Finish and review the previous episode first")
            if not task.strip() or not operator.strip():
                raise ValueError("Task and operator are required")
            now = time.monotonic()
            for name, spec in self.streams.items():
                age = now - self.latest.get(name, float("-inf"))
                if spec["required"] and not 0 <= age <= spec["max_age_s"]:
                    raise RuntimeError(f"Required stream unavailable: {name}")
            metadata = dict(copy.deepcopy(self.context), task=task, operator=operator,
                            previous_episode=self.previous_episode)
            streams = copy.deepcopy(self.streams)
        episode = Episode(self.root, metadata, streams,
                          capacity=self.capacity, max_bytes=self.max_bytes)
        with self.lock:
            self.episode = episode
            self.message = f"录制中 · {self.episode.path.name}"
            return self.episode.path

    def finish(self, reason="operator", error=""):
        with self.operation:
            return self._finish(reason, error)

    def _finish(self, reason, error):
        with self.lock:
            episode = self.episode
            if episode is None:
                return
            # Keep episode attached until drain completes, preventing another start.
            if error:
                episode.fail(error)
            else:
                episode.stop.set()
        path = episode.finish(reason)
        with self.lock:
            self.last = path
            self.last_state = episode.manifest['state']
            self.last_metadata = {key: episode.manifest['metadata'].get(key, '') for key in ('task', 'operator')}
            self.last_duration = episode.manifest['ended_at'] - episode.manifest['started_at']
            self.episode = None
            self.message = f"待验收 · {episode.manifest['state']} · {path.name}"
            if episode.manifest["quality"]["problems"]:
                self.message += "\n\n" + "；".join(episode.manifest["quality"]["problems"])
        return path

    def review(self, outcome, accepted=True, note=""):
        with self.operation:
            return self._review(outcome, accepted, note)

    def _review(self, outcome, accepted, note):
        with self.lock:
            if self.last is None:
                raise RuntimeError("No finalized episode to review")
            path = self.last
        review_episode(path, outcome, accepted=accepted, note=note)
        with self.lock:
            self.previous_episode = self.last.name
            self.last = None
            self.message = "已保存，准备下一段" if accepted else "已标记弃用，准备重采"

    def _monitor(self):
        while not self.closed.wait(.05):
            with self.lock:
                episode = self.episode
                if episode is None:
                    continue
                error = episode.error
                now = time.monotonic()
                for name, spec in self.streams.items():
                    if spec["required"] and now - self.latest.get(name, 0) > spec["max_age_s"]:
                        error = error or f"Required stream stale: {name}"
            if error:
                try:
                    self.finish("acquisition_error", error)
                except Exception as exc:
                    self.message = str(exc)

    def status_snapshot(self):
        """Small diagnostic copy; never serialize or touch storage under this lock."""
        with self.lock:
            now = time.monotonic()
            episode = self.episode
            state = ('closed' if self.closed.is_set() else
                     ('draining' if episode.stop.is_set() else 'recording') if episode else
                     self.last_state if self.last else 'ready' if self.ready else 'idle')
            metadata = episode.manifest['metadata'] if episode else self.last_metadata
            return dict(state=state, ready=self.ready, message=self.message,
                        task=metadata.get('task', ''), operator=metadata.get('operator', ''),
                        episode_id=episode.path.name if episode else self.last.name if self.last else None,
                        duration_s=now-episode.manifest['started_at'] if episode else self.last_duration,
                        queue_items=episode.queue.qsize() if episode else 0,
                        pending_bytes=episode.bytes if episode else 0,
                        error=episode.error if episode else '',
                        streams={name: dict(encoding=spec.get('encoding', ''),
                                           sequence=self.sequence.get(name, 0),
                                           age_s=now-self.latest[name] if name in self.latest else None,
                                           max_age_s=spec.get('max_age_s', .5), required=spec.get('required', True))
                                 for name, spec in self.streams.items()},
                        tactile={name: list(values) for name, values in tuple(self.tactile_values.items())})

    def close(self):
        self.closed.set()
        self.monitor.join(timeout=2)
        try:
            self.finish("shutdown", "Collection closed while recording")
        finally:
            self.status_publisher.close()
