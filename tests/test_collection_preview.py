"""Collection monitoring/replay reads real-format fixtures, never owns hardware."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from openarm_collection import Collection
from openarm_collection.recording import atomic_json
from openarm_platform.collection import CollectionBrowser
from test_collection import fixture, np


class CollectionPreviewTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def wait_for(self, condition):
        deadline = time.monotonic()+3
        while time.monotonic()<deadline:
            result=condition()
            if result:
                return result
            time.sleep(.02)
        self.fail('Status publisher did not reach expected state')

    def test_live_status_observes_recording_and_close_without_owning_devices(self):
        collection=Collection(self.root)
        self.addCleanup(collection.close)
        collection.register('robot/right',required=False)
        collection.ready=True
        collection.emit('robot/right',time.monotonic(),{})
        path=collection.start('preview test','operator')
        browser=CollectionBrowser(self.root)
        status=self.wait_for(lambda: next((v for v in browser.live() if v['state']=='recording'),None))
        self.assertTrue(status['fresh'])
        self.assertEqual(status['episode_id'],path.name)
        self.assertEqual(status['task'],'preview test')
        self.assertEqual(status['streams']['robot/right']['sequence'],1)
        collection.close()
        status=browser.live()[0]
        self.assertEqual(status['state'],'closed')
        self.assertFalse(collection.status_publisher.thread.is_alive())

    def test_preview_disk_failure_does_not_interrupt_native_recording(self):
        with patch('openarm_collection.status.atomic_json',side_effect=OSError('preview disk denied')):
            collection=Collection(self.root)
            try:
                collection.register('x',required=False)
                collection.ready=True
                collection.start('test','operator')
                collection.emit('x',time.monotonic(),[1])
                self.wait_for(lambda: collection.status_publisher.error)
                path=collection.finish()
                self.assertEqual(json.loads((path/'manifest.json').read_text())['state'],'review')
                self.assertEqual(len((path/'samples.jsonl').read_text().splitlines()),1)
            finally:
                collection.close()

    def test_stale_recording_status_is_not_reported_fresh(self):
        directory=self.root/'.runtime'
        directory.mkdir()
        atomic_json(directory/'one.json',dict(schema='openarm.collection-status/1',publisher='one',
                    updated_wall_time_ns=100_000_000_000,state='recording',streams={}))
        browser=CollectionBrowser(self.root)
        self.assertTrue(browser.live(now_ns=101_000_000_000)[0]['fresh'])
        self.assertFalse(browser.live(now_ns=104_000_000_000)[0]['fresh'])
        self.assertFalse(browser.live(now_ns=99_000_000_000)[0]['fresh'])

    def test_atomic_update_during_read_is_not_mistaken_for_future_status(self):
        directory=self.root/'.runtime';directory.mkdir()
        snapshot=dict(schema='openarm.collection-status/1',publisher='one',
            updated_wall_time_ns=100_000_000_000,state='recording',streams={})
        atomic_json(directory/'one.json',snapshot)
        clock=[99_000_000_000]
        def read_after_publish(path):
            clock[0]=100_000_000_000
            return dict(snapshot)
        with patch('openarm_platform.collection.read_json',side_effect=read_after_publish), \
             patch('openarm_platform.collection.time.time_ns',side_effect=lambda:clock[0]):
            browser=CollectionBrowser(self.root)
            self.assertTrue(browser.live()[0]['fresh'])
            self.assertFalse(browser.live(now_ns=99_000_000_000)[0]['fresh'])

    def test_archive_is_read_only_and_preserves_target_feedback_distinction(self):
        path=fixture(self.root)
        before={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in path.iterdir() if p.is_file()}
        browser=CollectionBrowser(self.root)
        timeline=browser.load(path.name)
        self.assertIn(path.name,browser.episodes())
        row=timeline.joint_rows(timeline.end)[0]
        self.assertNotEqual(row['target'],row['feedback'])
        self.assertEqual(row['target'],f'{math.degrees(15):+.2f}')
        self.assertEqual(row['feedback'],f'{math.degrees(5):+.2f}')
        self.assertEqual(before,{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in path.iterdir() if p.is_file()})

    def test_future_feedback_and_frames_are_not_exposed_before_their_timestamp(self):
        path=self.root/'trial';path.mkdir()
        atomic_json(path/'manifest.json',dict(schema='openarm.collection/1',state='review',started_at=100.,
                    streams={'robot/right':{}}))
        sample=dict(stream='robot/right',sequence=0,timestamp=101.,data=dict(targets_rad={'1':.2},
                    feedback={'1':dict(position=.1,received_at=101.2)}))
        (path/'samples.jsonl').write_text(json.dumps(sample)+'\n')
        timeline=CollectionBrowser(self.root).load('trial')
        self.assertIsNone(timeline.at('robot/right',.5))
        self.assertEqual(timeline.joint_rows(1.1)[0]['feedback'],'—')
        self.assertEqual(timeline.joint_rows(1.3)[0]['feedback'],f'{math.degrees(.1):+.2f}')

    def test_journal_corruption_and_partial_recording_remain_visible(self):
        path=fixture(self.root)
        with (path/'samples.jsonl').open('a') as f:
            f.write((path/'samples.jsonl').read_text().splitlines()[-1]+'\n')
        timeline=CollectionBrowser(self.root).load(path.name)
        self.assertTrue(any('SHA256' in p for p in timeline.problems))
        manifest=json.loads((path/'manifest.json').read_text())
        manifest['state']='recording'
        atomic_json(path/'manifest.json',manifest)
        with (path/'samples.jsonl').open('a') as f:
            f.write('{unfinished')
        timeline=CollectionBrowser(self.root).load(path.name)
        self.assertTrue(any('未写完' in p for p in timeline.problems))
        self.assertIn('未结束',CollectionBrowser(self.root).episodes()[path.name])

    def test_paths_cannot_escape_configured_recording_root(self):
        browser=CollectionBrowser(self.root)
        for name in ('../outside','/tmp',''):
            with self.assertRaises(ValueError):
                browser.load(name)
        (self.root/'alias').symlink_to('/tmp',target_is_directory=True)
        with self.assertRaises(ValueError):
            browser.load('alias')

    @unittest.skipUnless(np is not None and importlib.util.find_spec('cv2'),'Requires video extra')
    def test_camera_and_tactile_preview_from_lossless_sidecars(self):
        path=fixture(self.root,images=True)
        timeline=CollectionBrowser(self.root).load(path.name)
        frame=timeline.camera('camera/head',timeline.end)
        self.assertTrue(frame['source'].startswith('data:image/jpeg;base64,'))
        tactile=timeline.tactile('tactile/left_pad',timeline.end)
        self.assertEqual(tactile['resultant'],[1,2,3])
        self.assertEqual(len(tactile['taxels']),68)
        record=timeline.at('camera/head',timeline.end)
        record['data']['array']='../../outside.npy'
        with self.assertRaisesRegex(ValueError,'越过'):
            timeline.camera('camera/head',timeline.end)

    def test_oversized_metadata_is_rejected(self):
        path=self.root/'large';path.mkdir()
        (path/'manifest.json').write_bytes(b' '* (1024*1024+1))
        with self.assertRaises(ValueError):
            CollectionBrowser(self.root).load('large')
