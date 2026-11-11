import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from openarm_policy.contracts import SensorSample, StreamSpec
from openarm_policy.observations import LatestObservationSource, ObservationBuffer
from openarm_policy.session import PolicySession


class AsyncObservationTests(unittest.TestCase):
    def source(self, fail=False):
        entered, release = threading.Event(), threading.Event()
        sample = SensorSample(1, time.monotonic(), (1.,))
        class Source:
            streams = (StreamSpec('camera/fixture', 'numeric', (1,), '', '', .5),)
            metadata = {}
            calls = 0
            def snapshot(self):
                self.calls += 1
                if self.calls > 1:
                    entered.set()
                    release.wait(2)
                    if fail:
                        raise ValueError('Camera generation changed')
                return {'camera/fixture': sample}
        source = Source()
        reader = LatestObservationSource(source, poll_period_s=.1)
        self.addCleanup(reader.close)
        self.addCleanup(release.set)
        self.assertTrue(entered.wait(1))
        return reader, entered, release, sample

    def test_slow_camera_does_not_block_snapshot_or_refresh_timestamp(self):
        reader, _, release, sample = self.source()
        results = []
        worker = threading.Thread(target=lambda: results.append(reader.snapshot()))
        worker.start(); worker.join(.3)
        self.assertFalse(worker.is_alive(), 'Snapshot must not wait for HTTP completion')
        self.assertEqual(results[0]['camera/fixture'].received_at, sample.received_at)
        with self.assertRaisesRegex(RuntimeError, 'camera/fixture'):
            ObservationBuffer(reader.streams).collect(reader.snapshot(), now=sample.received_at+1.)
        release.set()

    def test_reader_failure_is_latched_not_hidden_by_cached_frames(self):
        reader, _, release, _ = self.source(fail=True)
        release.set(); reader.worker.join(1)
        self.assertFalse(reader.worker.is_alive())
        with self.assertRaisesRegex(RuntimeError, 'generation changed'):
            reader.snapshot()

    def test_collection_refreshes_robot_after_sources_without_loosening_gate(self):
        clock = SimpleNamespace(now=10.)
        specs = (StreamSpec('tcp/left', 'pose', (1,), '', '', .15),
                 StreamSpec('camera/fixture', 'numeric', (1,), '', '', .5))
        class Source:
            def snapshot(self):
                clock.now += .12
                return {'camera/fixture': SensorSample(1, clock.now-.01, (0.,))}
        class Robot:
            age = .04
            def snapshot(self):
                return {'tcp/left': SensorSample(1, clock.now-self.age, (0.,))}
        robot = Robot()
        session = SimpleNamespace(robot=robot, sources=[Source()], buffer=ObservationBuffer(specs))
        with patch('openarm_policy.observations.time.monotonic', lambda: clock.now):
            observation = PolicySession._observe(session)
            self.assertAlmostEqual(observation.captured_at-observation.streams['tcp/left'][-1].received_at, .04)
            robot.age = .20
            with self.assertRaisesRegex(RuntimeError, 'tcp/left'):
                PolicySession._observe(session)
