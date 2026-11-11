"""Bounded asynchronous physical-episode recording; optional Rerun SDK.

Images are encoded off the control thread. Overflow/failure is explicit, never
silently reported as a complete recording. This does not open hardware/viewers.
"""
import json
from pathlib import Path
import queue
import re
import threading
import time
from dataclasses import replace

from .recording import CompactEpisodeRecorder


def safe_key(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', value):
        raise ValueError('Invalid diagnostic input name')
    return value


class RerunEpisodeRecorder:
    def __init__(self, directory):
        import rerun as rr
        self.rr = rr
        self.summary = CompactEpisodeRecorder(directory)
        self.directory = Path(directory)
        self.latest = self.directory / 'model-inputs'
        self.latest.mkdir()
        self.stream = rr.RecordingStream('openarm-physical-policy', recording_id=self.directory.parent.name)
        self.stream.save(self.directory / 'recording.rrd')
        self.queue = queue.Queue(maxsize=64)
        self.error = None
        self.seen = {}
        self.origin = time.monotonic()
        self.cameras = set()
        self.blueprint_written = False
        self.closed = False
        self.worker = threading.Thread(target=self._run, name='policy-rerun-writer', daemon=True)
        self.worker.start()

    def record(self, event):
        if self.error is not None:
            raise RuntimeError('Rerun recording failed') from self.error
        if event['event'] == 'observation':
            observation = event['observation']
            # Histories share camera buffers. Queue only latest samples instead of
            # retaining eight histories per pending observation.
            event = dict(event, observation=replace(observation, streams={
                key: samples[-1:] for key, samples in observation.streams.items()}))
        try:
            self.queue.put_nowait(dict(event, time=event.get('time', time.monotonic())))
        except queue.Full as exc:
            self.error = RuntimeError('Rerun recording queue overflow; episode recording incomplete')
            raise self.error from exc

    def _json(self, value):
        return json.dumps(self.summary._plain(value), ensure_ascii=False, allow_nan=False)

    def _image(self, path, image):
        self.stream.log(path, self.rr.Image(image, color_model='RGB').compress(jpeg_quality=90))

    def _write(self, event):
        import numpy as np
        rr, stream = self.rr, self.stream
        now = event.get('time', time.monotonic())
        stream.set_time('episode_time', duration=now-self.origin)
        stream.set_time('host_monotonic', duration=now)
        name = event['event']
        if name == 'observation':
            obs = event['observation']
            stream.set_time('observation', sequence=obs.sequence)
            for key, samples in obs.streams.items():
                if not samples:
                    continue
                sample = samples[-1]
                if self.seen.get(key) == sample.sequence:
                    continue
                self.seen[key] = sample.sequence
                stream.log('sources/'+key+'/received_at_s', rr.Scalars(sample.received_at))
                stream.log('sources/'+key+'/valid', rr.Scalars(int(sample.valid)))
                if key.startswith('camera/'):
                    self.cameras.add(key)
                    image = np.asarray(sample.data)
                    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
                        raise ValueError('Expected source BGR uint8 camera image')
                    self._image('sources/'+key+'/image', image[..., ::-1])
                else:
                    array = np.asarray(sample.data)
                    stream.log('sources/'+key+'/values', rr.Tensor(array))
                    if key.startswith('robot/') and array.shape == (8, 3):
                        for j, values in enumerate(array, 1):
                            for index, unit in enumerate(('position_rad', 'velocity_rad_s', 'torque_nm')):
                                stream.log(f'feedback/{key}/joint{j}/{unit}', rr.Scalars(values[index]))
        elif name == 'chunk':
            chunk = event['chunk']
            diagnostics = chunk.get('diagnostics', {})
            inputs = diagnostics.get('encoded_inputs', {})
            manifest = {'observation_sequence': chunk['observation_sequence'],
                        'recorded_at': now, 'images': [], 'state': {}}
            from PIL import Image
            for key, value in inputs.items():
                key = safe_key(key)
                array = np.asarray(value)
                stream.log('model/inputs/'+key+'/tensor', rr.Tensor(array))
                if array.dtype == np.uint8 and array.ndim == 4 and array.shape[0] == 1 and array.shape[-1] == 3:
                    self._image('model/inputs/'+key+'/image', array[0])
                    temp = self.latest / (key+'.tmp')
                    Image.fromarray(array[0]).save(temp, format='PNG')
                    temp.replace(self.latest / (key+'.png'))
                    manifest['images'].append(key)
                elif array.size <= 64:
                    manifest['state'][key] = array.tolist()
            temp = self.latest / 'inputs.tmp'
            temp.write_text(self._json(manifest))
            temp.replace(self.latest / 'inputs.json')
            if inputs and not self.blueprint_written and (self.cameras or manifest['images']):
                import rerun.blueprint as rrb
                source_views = [rrb.Spatial2DView(name=key, origin='sources/'+key+'/image')
                                for key in sorted(self.cameras)]
                model_views = [rrb.Spatial2DView(name=key+' · model input', origin='model/inputs/'+key+'/image')
                               for key in manifest['images']]
                stream.send_blueprint(rrb.Blueprint(rrb.Vertical(
                    rrb.Horizontal(*source_views, *model_views),
                    rrb.TimeSeriesView(name='Measured joint positions', origin='feedback', contents=['/**/position_rad']),
                    rrb.TextDocumentView(name='Phase and termination', origin='events'),
                    row_shares=[3, 1, 1]), auto_views=False, auto_layout=False))
                self.blueprint_written = True
            stream.log('model/actions/decoded', rr.Tensor(np.asarray(chunk['steps'])))
            stream.log('model/input_sources', rr.TextDocument(self._json({
                'captured_at': diagnostics.get('input_captured_at'),
                'sources': diagnostics.get('input_sources', {})}), media_type='application/json'))
            stream.log('model/actions/raw', rr.Tensor(np.asarray(diagnostics.get('raw_actions', []))))
            stream.log('model/observation_sequence', rr.Scalars(chunk['observation_sequence']))
            stream.log('model/latency_s', rr.Scalars(event.get('latency_s', 0.)))
        else:
            stream.log('events/'+name, rr.TextDocument(self._json(event), media_type='application/json'))
            if name in ('gravity_sample', 'integral_sample'):
                values = event.get('torque_nm', {})
                for key, value in values.items():
                    stream.log('control/integral/joint'+str(key), rr.Scalars(value))
            if name in ('command', 'preparation_command'):
                stream.log('control/'+name, rr.Tensor(np.asarray(event['values'])))
        # Keep the numeric summary small; images/tensors live only in RRD.
        self.summary.record(event)

    def _run(self):
        try:
            while True:
                event = self.queue.get()
                if event is None:
                    break
                self._write(event)
        except BaseException as exc:
            self.error = exc

    def close(self):
        if self.closed:
            return
        self.closed = True
        # Called after robot cleanup; never wait for disk on a motor thread.
        while self.worker.is_alive():
            try:
                self.queue.put(None, timeout=.1)
                break
            except queue.Full:
                continue
        self.worker.join(timeout=30)
        complete = not self.worker.is_alive() and self.error is None
        if not self.worker.is_alive():
            self.stream.flush(timeout_sec=10)
            self.stream.disconnect()
            self.summary.close()
        (self.directory/'rerun-status.json').write_text(json.dumps({
            'complete': complete, 'error': str(self.error or ''),
            'file': 'recording.rrd', 'camera_clock': 'host_receive_not_exposure_synchronized',
            'images': 'source_jpeg90_and_exact_encoded_input_png_tensors'}, indent=2)+'\n')
        if not complete:
            raise RuntimeError('Rerun recording incomplete') from self.error

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
