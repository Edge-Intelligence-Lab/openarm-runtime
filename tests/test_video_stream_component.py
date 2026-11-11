"""Browser multipart decoder contract, optional Node runner; no UI/hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


@unittest.skipUnless(shutil.which('node'), 'Optional JavaScript test runner')
class VideoParserTests(unittest.TestCase):
    def test_fragmented_frames_and_bounds(self):
        script = r'''
import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
const source=readFileSync(process.argv[2],'utf8');
const {MultipartFrames}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
const encode=new TextEncoder();
const bytes=encode.encode('--frame\r\nContent-Type: image/png\r\nContent-Length: 3\r\n\r\nabc\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 4\r\n\r\ndefg\r\n');
for(const step of [1,2,7,64,bytes.length]) {
 const parser=new MultipartFrames(); const frames=[];
 for(let i=0;i<bytes.length;i+=step)frames.push(...parser.push(bytes.slice(i,i+step)));
 assert.equal(frames.length,2);
 assert.equal(new TextDecoder().decode(frames[0].bytes),'abc');
 assert.equal(new TextDecoder().decode(frames[1].bytes),'defg');
}
for(const header of ['Content-Length: 0','Content-Length: 8000001','Content-Length: -1'])
 assert.throws(()=>new MultipartFrames().push(encode.encode('--frame\r\nContent-Type: image/png\r\n'+header+'\r\n\r\n')));
assert.throws(()=>new MultipartFrames().push(new Uint8Array(16385)));
assert.throws(()=>new MultipartFrames().push(encode.encode('--frame\r\nContent-Type: text/html\r\nContent-Length: 1\r\n\r\nx')));
'''
        source = Path(__file__).resolve().parents[1]/'src/openarm_platform/static/video_stream.js'
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'test.mjs'
            path.write_text(script)
            result = subprocess.run([shutil.which('node'), str(path), str(source)],
                                    capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
