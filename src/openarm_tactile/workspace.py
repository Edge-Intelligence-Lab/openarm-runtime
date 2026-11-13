"""Multi-sensor workspace reusing the maintained single-sensor Viser viewer."""
from html import escape
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import subprocess
import sys
import threading
from urllib.parse import urlsplit

from .profile import POSITIONS, load, save_positions


def page(data, port):
    cards = []
    for index, options in enumerate(data['tactile'].values()):
        identity = options['identity']
        select = ''.join(f'<option value="{key}"'
                         + (' selected' if identity.get('position', 'unassigned') == key else '')
                         + f'>{label}</option>' for key, label in POSITIONS.items())
        cards.append(f'<section><header><strong>{escape(identity["serial"])}</strong>'
                     f'<select data-serial="{escape(identity["serial"], quote=True)}">{select}</select>'
                     f'</header><iframe title="触觉 {escape(identity["serial"], quote=True)}" '
                     f'data-port="{port+10+index}"></iframe></section>')
    return ('''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>四路触觉工作台</title>
<style>body{font:14px system-ui;color:#20342f;background:#f3f5f3;margin:20px}
nav{display:flex;align-items:center;gap:18px;flex-wrap:wrap}h1{font-size:24px}p{color:#60756b}
button,select{font:inherit;padding:8px;border:1px solid #b9ccbf;border-radius:8px;background:white;color:#20342f}
button{cursor:pointer}main{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px}
section{background:white;border-radius:12px;overflow:hidden}header{padding:12px;display:flex;justify-content:space-between;gap:12px}
iframe{width:100%;height:620px;border:0}#status{color:#37664c}@media(max-width:850px){main{grid-template-columns:1fr}}</style>
<nav><h1>四路触觉工作台</h1><button id="save">保存位置配置</button><button id="capture">暂停预览并释放串口</button><span id="status"></span></nav>
<p>按序列号识别设备，为每路选择位置后保存。左右爪统一以“从该手的手腕朝指尖看”为准，不随操作者站位改变。
保存到机器配置，刷新、换浏览器后仍保留；之后新启动的策略记录自动携带位置。试跑前请暂停预览，释放串口。只读采集，不操作电机。单位 counts。</p>
<main>''' + ''.join(cards) + '''</main><script>
document.querySelectorAll('iframe').forEach(f=>{const u=new URL(location.href);if(location.pathname.startsWith('/apps/'))u.hostname='127.0.0.1';u.port=f.dataset.port;u.pathname='/';u.search='';u.hash='';f.src=u.href;});
let capturing=true;
fetch('./state').then(r=>r.json()).then(d=>{capturing=d.enabled;document.querySelector('#capture').textContent=capturing?'暂停预览并释放串口':'开启实时预览';}).catch(()=>{});
document.querySelector('#capture').onclick=async()=>{
 const b=document.querySelector('#capture');b.disabled=true;
 try{const r=await fetch('./capture',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled:!capturing})});
 const d=await r.json();if(!r.ok)throw Error(d.error);capturing=d.enabled;b.textContent=capturing?'暂停预览并释放串口':'开启实时预览';
 document.querySelector('#status').textContent=capturing?'预览已开启':'串口已释放，可以试跑';
 if(capturing)document.querySelectorAll('iframe').forEach(f=>{f.src=f.src;});}
 catch(e){document.querySelector('#status').textContent=e.message;}finally{b.disabled=false;}
};
document.querySelector('#save').onclick=async()=>{
 const button=document.querySelector('#save'),status=document.querySelector('#status');button.disabled=true;
 try{const mapping=Object.fromEntries([...document.querySelectorAll('select')].map(s=>[s.dataset.serial,s.value]));
 const r=await fetch('./positions',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(mapping)});
 const result=await r.json();if(!r.ok)throw Error(result.error);status.textContent='已保存到机器配置';}
 catch(e){status.textContent='保存失败：'+e.message;}finally{button.disabled=false;}
};</script></html>''').encode()


def serve(profile, *, port=8891):
    profile = Path(profile)
    data = load(profile)
    lock = threading.Lock()
    children = []

    def capture(enabled):
        if not enabled:
            for child in children:
                if child.poll() is None:
                    child.terminate()
            for child in children:
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
            children.clear()
        elif not children:
            try:
                for index, options in enumerate(data['tactile'].values()):
                    children.append(subprocess.Popen([
                        sys.executable, '-m', 'openarm_runtime', 'tactile', '--device', options['device'],
                        '--port', str(port+10+index), '--baudrate', str(options.get('baudrate', 921600)),
                        '--device-id', str(options.get('device_id', 1))]))
            except BaseException:
                capture(False)
                raise

    class Handler(BaseHTTPRequestHandler):
        def reply(self, code, content, mime='application/json'):
            self.send_response(code)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self):
            if urlsplit(self.path).path == '/state':
                with lock:
                    self.reply(200, json.dumps({'enabled': bool(children)}).encode())
                return
            if urlsplit(self.path).path != '/':
                self.reply(404, b'{}')
                return
            with lock:
                content = page(load(profile), port)
            self.reply(200, content, 'text/html; charset=utf-8')

        def do_POST(self):
            origin = self.headers.get('Origin')
            if (self.headers.get('Sec-Fetch-Site') == 'cross-site'
                    or (origin and urlsplit(origin).netloc != self.headers.get('Host'))):
                self.reply(403, b'{"error":"Cross-origin write rejected"}')
                return
            route = urlsplit(self.path).path
            if route not in ('/positions', '/capture'):
                self.reply(404, b'{}')
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 8192 or self.headers.get_content_type() != 'application/json':
                    raise ValueError('Expected bounded JSON request')
                mapping = json.loads(self.rfile.read(length))
                with lock:
                    if route == '/positions':
                        save_positions(profile, mapping)
                        result = {'saved': True}
                    else:
                        if not isinstance(mapping, dict) or type(mapping.get('enabled')) is not bool:
                            raise ValueError('enabled must be boolean')
                        capture(mapping['enabled'])
                        result = {'enabled': bool(children)}
                self.reply(200, json.dumps(result).encode())
            except (ValueError, KeyError, TypeError) as exc:
                self.reply(400, json.dumps({'error': str(exc)}, ensure_ascii=False).encode())

        def log_message(self, *args):
            pass

    with ThreadingHTTPServer(('127.0.0.1', port), Handler) as server:
        server.timeout = .25
        try:
            capture(True)
            while True:
                with lock:
                    if any(child.poll() is not None for child in children):
                        capture(False)
                server.handle_request()
        finally:
            with lock:
                capture(False)
