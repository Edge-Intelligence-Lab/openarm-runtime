import asyncio
from pathlib import Path
import tempfile
import unittest

from openarm_platform.model_stream import input_frames


class InputStreamTests(unittest.IsolatedAsyncioTestCase):
    async def test_stream_keeps_connection_and_only_emits_new_frames(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'image.png'
            path.write_bytes(b'frame-one')
            frames = input_frames(path, lambda: True, period=.001)
            first = await anext(frames)
            self.assertIn(b'Content-Type: image/png', first)
            self.assertTrue(first.endswith(b'frame-one\r\n'))
            pending = asyncio.create_task(anext(frames))
            await asyncio.sleep(.01)
            self.assertFalse(pending.done(), 'Unchanged frames should not be retransmitted')
            temp = path.with_suffix('.tmp')
            temp.write_bytes(b'frame-two')
            temp.replace(path)
            second = await asyncio.wait_for(pending, 1)
            self.assertTrue(second.endswith(b'frame-two\r\n'))
            await frames.aclose()

    async def test_stopped_episode_emits_last_frame_then_closes(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'image.png'
            path.write_bytes(b'last-frame')
            frames = [frame async for frame in input_frames(path, lambda: False)]
            self.assertEqual(len(frames), 1)

    async def test_missing_and_oversize_files_close_without_payload(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'image.png'
            self.assertEqual([f async for f in input_frames(path, lambda: True)], [])
            with path.open('wb') as f:
                f.truncate(4_000_001)
            self.assertEqual([f async for f in input_frames(path, lambda: True)], [])
