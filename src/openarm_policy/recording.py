"""Explicit episode storage with array sidecars; no pickle or hidden disk writes."""
from dataclasses import is_dataclass
from contextlib import contextmanager
import json
from pathlib import Path
import threading

from .contracts import Observation, SensorSample


@contextmanager
def record_tactile(directory, readers, options, recorder=None):
    """Native-rate sidecar using the existing collection journal, not the model.

    Reader callbacks enqueue small immutable frames off the control thread.
    Capture faults mark the sidecar incomplete; they never become fake zeros or
    commands. Readers must be closed before this context is exited.
    """
    from openarm_collection.recording import Episode, atomic_json
    from openarm_tactile import SENSOR
    root = Path(directory)
    streams = {}
    for name, reader in readers.items():
        identity = options[name].get('identity')
        if (not isinstance(identity, dict) or not identity.get('serial') or not identity.get('mount')):
            raise ValueError('Tactile recording needs serial and explicit mount identity')
        if reader.on_sample is not None:
            raise ValueError('Tactile reader already has a recording sink')
        streams['tactile/'+name] = dict(encoding='xyz_counts', shape=[69, 3], units='counts',
            required=True, max_age_s=.5, sensor=SENSOR, identity=identity,
            device=reader.device, device_id=reader.device_id, baudrate=reader.baudrate,
            simulated=reader.simulate,
            timestamp_semantics='host_received_after_two_separate_transactions',
            layout='resultant_xyz_then_68_taxel_xyz', baseline='raw_device_output_no_runtime_zero',
            calibration='absolute_force_and_physical_pad_axes_unverified')
    episode = Episode(root/'tactile', dict(policy_episode=root.parent.name,
        purpose='policy_sidecar_not_model_input', hardware_synchronized=False), streams)
    pointer = root/'tactile.json'
    receipt = dict(schema='openarm.policy-tactile.v1', path=str(episode.path.relative_to(root)),
                   complete=False, model_input=False, clock='host_monotonic_seconds')
    try:
        atomic_json(pointer, receipt)
        for name, reader in readers.items():
            def receive(sample, sequence, timestamp, key='tactile/'+name):
                episode.submit(key, sequence, timestamp, (sample.resultant, *sample.forces))
            reader.on_sample = receive
        if recorder is not None:
            recorder.record(dict(event='tactile_recording_started', **receipt))
        yield episode
    finally:
        status = {}
        for name, reader in readers.items():
            reader.on_sample = None
            snapshot = reader.snapshot()
            status[name] = {k: snapshot[k] for k in ('state', 'error', 'sequence', 'received_at', 'age_s', 'rate_hz')}
            if snapshot['error']:
                episode.fail(name+': '+snapshot['error'])
        episode.finish('policy_session_end')
        receipt.update(complete=episode.manifest['state'] == 'review',
                       quality=episode.manifest['quality'], reader_status=status)
        atomic_json(pointer, receipt)
        if recorder is not None:
            recorder.record(dict(event='tactile_recording_finished', **receipt))


class CompactEpisodeRecorder:
    """Bounded result receipt, not replay: no RGB, action/chunk or state archives.

    Recording never changes control. Lifecycle/error/cleanup evidence is retained;
    absence of an operator success judgment is explicitly unknown, not success.
    """
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.lock = threading.Lock()
        self.summary = {"schema": "openarm.compact-policy-episode.v1", "retention": "summary",
                        "task_success": None, "success_judgment": "operator_required",
                        "complete": False, "event_counts": {}, "lifecycle": []}
        self._write()

    def _plain(self, value):
        if hasattr(value, "tolist"):
            return value.tolist()
        if is_dataclass(value):
            from dataclasses import fields
            return {f.name: self._plain(getattr(value, f.name)) for f in fields(value)}
        if isinstance(value, dict):
            return {k: self._plain(v) for k, v in value.items()}
        if isinstance(value, (tuple, list)):
            return [self._plain(v) for v in value]
        return value

    def _write(self):
        path = self.directory / "summary.json"
        temporary = self.directory / "summary.json.tmp"
        temporary.write_text(json.dumps(self.summary, allow_nan=False, indent=2)+"\n")
        temporary.replace(path)

    def record(self, event):
        with self.lock:
            name = event["event"]
            counts = self.summary["event_counts"]
            counts[name] = counts.get(name, 0) + 1
            if name == "observation":
                self.summary["last_observation_sequence"] = event["observation"].sequence
                return
            if name in ('gravity_sample', 'integral_sample', 'hold_sample'):
                self.summary.setdefault('last_control_samples', {})[name] = self._plain(event)
                return
            if name == "chunk":
                chunk = event["chunk"]
                diagnostics = chunk.get("diagnostics", {})
                if "model" not in self.summary:
                    self.summary["model"] = self._plain({k: diagnostics[k] for k in (
                        "model", "observation_adapter", "action_adapter") if k in diagnostics})
                latency = event.get("latency_s")
                if latency is not None:
                    values = self.summary.setdefault("inference_latency_s", {"count": 0, "sum": 0., "max": 0.})
                    values["count"] += 1; values["sum"] += latency; values["max"] = max(values["max"], latency)
                return
            if name in ("command", "handoff_hold", "preparation_command", "return_home_command", "cartesian_dispatch",
                        "cartesian_progress_wait", "cartesian_preparation_dispatch"):
                # One terminal numeric snapshot replaces the previous one; no trajectory.
                if name.startswith("cartesian_"):
                    self.summary["last_cartesian_state"] = self._plain(event)
                return
            if name in ("context", "cartesian_context", "video_source", "retention_context"):
                self.summary[name] = self._plain(event)
            else:
                self.summary["lifecycle"].append(self._plain(event))
                self.summary["lifecycle"] = self.summary["lifecycle"][-32:]
            if name == "ended":
                self.summary["termination"] = self._plain(event)
                self.summary["complete"] = True
            self._write()

    def close(self):
        with self.lock:
            self._write()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


class EpisodeRecorder:
    """Lossless observations + submitted actions, not a training dataset format.

    Caller owns lifetime. Writes synchronously; use fast local storage. Device
    watchdogs remain independent of I/O. Failure is propagated, never silently
    dropping training samples. Repeated sample sequences reuse their sidecars.
    """
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.file = (self.directory / "events.jsonl").open("x")
        self.lock = threading.Lock()
        self.counter = 0
        self.arrays = {}

    def _encode(self, value):
        if hasattr(value, "shape") and hasattr(value, "dtype"):
            import numpy as np
            if value.dtype.hasobject:
                raise TypeError("Object arrays are not recordable")
            self.counter += 1
            name = f"array-{self.counter:08d}.npy"
            np.save(self.directory / name, value, allow_pickle=False)
            return {"array": name}
        if is_dataclass(value):
            # asdict deep-copies images; walk fields instead.
            from dataclasses import fields
            return {f.name: self._encode(getattr(value, f.name)) for f in fields(value)}
        if isinstance(value, dict):
            return {k: self._encode(v) for k, v in value.items()}
        if isinstance(value, (tuple, list)):
            return [self._encode(v) for v in value]
        return value

    def record(self, event):
        with self.lock:
            if event.get("event") == "observation":
                observation = event["observation"]
                streams = {}
                for name, history in observation.streams.items():
                    streams[name] = []
                    for sample in history:
                        key = (name, sample.sequence)
                        if key not in self.arrays:
                            self.arrays[key] = self._encode(sample)
                        streams[name].append(self.arrays[key])
                payload = dict(event, observation=dict(sequence=observation.sequence,
                               captured_at=observation.captured_at, version=observation.version, streams=streams))
                # Keep only this observation's bounded history, not an episode-sized cache.
                keys = {(name, s.sequence) for name, history in observation.streams.items() for s in history}
                self.arrays = {k: v for k, v in self.arrays.items() if k in keys}
            else:
                payload = self._encode(event)
            self.file.write(json.dumps(payload, allow_nan=False) + "\n")
            self.file.flush()

    def close(self):
        self.file.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def replay_observations(directory, *, phase=None):
    """Yield saved observations in their original monotonic time domain, offline."""
    root = Path(directory).resolve()
    if phase not in (None, "task", "preparation"):
        raise ValueError("Replay phase must be task, preparation or None")

    def decode(value):
        if isinstance(value, dict) and set(value) == {"array"}:
            import numpy as np
            path = (root / value["array"]).resolve()
            if path.parent != root:
                raise ValueError("Array path escapes episode")
            return np.load(path, allow_pickle=False)
        if isinstance(value, list):
            return tuple(decode(v) for v in value)
        return value

    with (root / "events.jsonl").open() as handle:
        for line in handle:
            event = json.loads(line)
            if event["event"] == "observation":
                if phase is not None and event.get("phase", "task") != phase:
                    continue
                obs = event["observation"]
                streams = {name: tuple(SensorSample(**dict(s, data=decode(s["data"]))) for s in history)
                           for name, history in obs["streams"].items()}
                yield Observation(obs["sequence"], obs["captured_at"], streams, obs["version"])
