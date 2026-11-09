"""Real ASGI/aiohttp HTTP and WebSocket forwarding; no hardware is opened."""
import asyncio
from dataclasses import replace
import importlib.util
import json
import socket
import unittest
from unittest.mock import patch

from openarm_platform import load_config
from openarm_platform.proxy import Gateway, MAX_WS_MESSAGE, resolve, rewrite_html

HAS_PLATFORM = all(importlib.util.find_spec(name) is not None
                   for name in ('aiohttp', 'starlette', 'uvicorn'))


@unittest.skipUnless(HAS_PLATFORM, 'Optional platform dependencies')
class PlatformProxyTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        import aiohttp
        from aiohttp import web
        from starlette.applications import Starlette
        import uvicorn

        self.release_stream = asyncio.Event()
        self.upstream_disconnected = asyncio.Event()
        self.seen_messages = []
        self.asgi_errors = []
        self.reject_browser_close = False

        async def endpoint(request):
            if request.headers.get('Upgrade', '').lower() == 'websocket':
                websocket = web.WebSocketResponse(protocols=('viser-test',), max_msg_size=MAX_WS_MESSAGE)
                await websocket.prepare(request)
                await websocket.send_bytes(b'initial-binary')
                try:
                    async for message in websocket:
                        self.seen_messages.append(message.data)
                        if message.type == aiohttp.WSMsgType.BINARY:
                            await websocket.send_bytes(message.data)
                        elif message.type == aiohttp.WSMsgType.TEXT:
                            await websocket.send_str(message.data)
                finally:
                    self.upstream_disconnected.set()
                return websocket
            if request.path == '/redirect':
                return web.Response(status=302, headers={'Location': '/next?a=1'})
            if request.path == '/external-redirect':
                return web.Response(status=302, headers={'Location': 'http://example.invalid/no-follow'})
            if request.path == '/chunked':
                response = web.StreamResponse(headers={'Content-Type': 'image/jpeg',
                                                       'Connection': 'X-Internal', 'X-Internal': 'secret'})
                await response.prepare(request)
                await response.write(b'abc')
                await response.write(b'def')
                await response.write_eof()
                return response
            if request.path in ('/stream', '/finite-stream'):
                headers = {'Content-Type': 'multipart/x-mixed-replace; boundary=frame'}
                if request.path == '/finite-stream':
                    headers['Content-Length'] = '20'
                response = web.StreamResponse(headers=headers)
                await response.prepare(request)
                await response.write(b'--frame\r\n')
                await self.release_stream.wait()
                return response
            if request.path == '/cookies':
                response = web.Response(text='ok')
                response.set_cookie('first', '1', path='/')
                response.set_cookie('second', '2', path='/')
                return response
            if request.method not in ('GET', 'HEAD'):
                return web.Response(body=await request.read(), headers={'X-Upstream-Origin': request.headers.get('Origin', '')})
            if request.path == '/':
                return web.Response(text='<html><head><script src="/asset.js"></script></head><body>module</body></html>',
                                    content_type='text/html')
            return web.json_response({'path': request.raw_path, 'host': request.headers['Host'],
                                      'cookie': request.headers.get('Cookie', '')})

        upstream_app = web.Application()
        upstream_app.router.add_route('*', '/{path:.*}', endpoint)
        self.upstream_runner = web.AppRunner(upstream_app)
        await self.upstream_runner.setup()
        upstream_site = web.TCPSite(self.upstream_runner, '127.0.0.1', 0)
        await upstream_site.start()
        self.upstream_port = upstream_site._server.sockets[0].getsockname()[1]
        config = load_config()
        page = replace(config.pages[0], port=self.upstream_port, extra_ports=(self.upstream_port,))
        self.gateway = Gateway(replace(config, pages=(page,)))
        app = Starlette()
        self.gateway.install(app)
        await self.gateway.start()
        self.sock = socket.socket()
        self.sock.bind(('127.0.0.1', 0))
        self.port = self.sock.getsockname()[1]
        self.base = f'http://127.0.0.1:{self.port}'
        async def monitored_app(scope, receive, send):
            async def monitored_send(message):
                if self.reject_browser_close and message['type'] == 'websocket.close':
                    from starlette.websockets import WebSocketDisconnect
                    raise WebSocketDisconnect(code=1006)
                await send(message)
            try:
                await app(scope, receive, monitored_send)
            except BaseException as error:
                self.asgi_errors.append(type(error).__name__)
                raise

        self.server = uvicorn.Server(uvicorn.Config(
            monitored_app, host='127.0.0.1', port=self.port, lifespan='off', access_log=False,
            log_level='critical', timeout_graceful_shutdown=1, ws_max_size=MAX_WS_MESSAGE,
        ))
        self.server_task = asyncio.create_task(self.server.serve(sockets=[self.sock]))
        async with asyncio.timeout(5):
            while not self.server.started:
                await asyncio.sleep(.01)
        self.client = aiohttp.ClientSession(cookie_jar=aiohttp.DummyCookieJar())

    async def asyncTearDown(self):
        self.release_stream.set()
        await self.client.close()
        await self.gateway.close()
        self.server.should_exit = True
        await asyncio.wait_for(self.server_task, 4)
        self.sock.close()
        await self.upstream_runner.cleanup()

    async def request(self, method, path, *, data=None, headers=None):
        async with self.client.request(method, self.base + path, data=data, headers=headers,
                                       allow_redirects=False) as response:
            return response.status, response.headers, await response.read()

    async def websocket(self, **kwargs):
        options = dict(origin=self.base, protocols=('viser-test',), max_msg_size=MAX_WS_MESSAGE)
        options.update(kwargs)
        return await self.client.ws_connect(self.base + '/apps/control', **options)

    async def test_http_html_and_nested_routes_share_one_origin(self):
        code, headers, body = await self.request('GET', '/apps/control/')
        self.assertEqual(code, 200)
        self.assertIn(b'src="/apps/control/asset.js"', body)
        self.assertIn(b'window.fetch=', body)
        self.assertIn(b'/apps/control/_extra/0/', body)
        self.assertEqual(int(headers['Content-Length']), len(body))
        self.assertEqual(headers['X-Frame-Options'], 'SAMEORIGIN')
        for path in ('/apps/control/status?x=2', '/apps/control/_extra/0/status?x=2'):
            code, _, body = await self.request('GET', path)
            self.assertEqual(code, 200)
            self.assertEqual(json.loads(body)['path'], '/status?x=2')
            self.assertEqual(json.loads(body)['host'], f'127.0.0.1:{self.upstream_port}')
        self.assertEqual((await self.request('GET', '/apps/unknown/'))[0], 404)
        self.assertEqual((await self.request('GET', '/apps/control-other/'))[0], 404)

    async def test_redirects_binary_streams_and_head(self):
        code, headers, _ = await self.request('GET', '/apps/control?x=1')
        self.assertEqual(code, 307)
        self.assertEqual(headers['Location'], '/apps/control/?x=1')
        code, headers, _ = await self.request('GET', '/apps/control/redirect')
        self.assertEqual(code, 302)
        self.assertEqual(headers['Location'], '/apps/control/next?a=1')
        code, headers, _ = await self.request('GET', '/apps/control/external-redirect')
        self.assertEqual(code, 302)
        self.assertEqual(headers['Location'], 'http://example.invalid/no-follow')
        code, headers, body = await self.request('GET', '/apps/control/chunked')
        self.assertEqual((code, body), (200, b'abcdef'))
        self.assertNotIn('X-Internal', headers)
        code, _, body = await self.request('HEAD', '/apps/control/')
        self.assertEqual((code, body), (200, b''))

    async def test_mutations_need_matching_origin_and_keep_request_body(self):
        path = '/apps/control/settings'
        self.assertEqual((await self.request('POST', path, data=b'data'))[0], 403)
        self.assertEqual((await self.request('POST', path, data=b'data', headers={'Origin': 'https://evil.invalid'}))[0], 403)
        code, headers, body = await self.request('POST', path, data=b'\x00binary\xff', headers={'origin': self.base})
        self.assertEqual((code, body), (200, b'\x00binary\xff'))
        self.assertEqual(headers['X-Upstream-Origin'], f'http://127.0.0.1:{self.upstream_port}')
        self.assertEqual((await self.request('GET', '/apps/control/', headers={'Host': 'evil.invalid'}))[0], 403)

    async def test_exact_openchamber_preview_host_works_for_module_http(self):
        host = 'openchamber-preview.localhost:50310'
        self.assertEqual((await self.request('GET', '/apps/control/', headers={'Host': host}))[0], 200)
        code, headers, body = await self.request('POST', '/apps/control/settings', data=b'preview',
                                                headers={'Host': host, 'Origin': 'http://' + host})
        self.assertEqual((code, body), (200, b'preview'))
        self.assertEqual(headers['X-Upstream-Origin'], f'http://127.0.0.1:{self.upstream_port}')
        self.assertEqual((await self.request('POST', '/apps/control/settings', data=b'preview',
                         headers={'Host': host, 'Origin': 'http://openchamber-preview.localhost:50311'}))[0], 403)
        for other in ('other.localhost:50310', 'openchamber-preview.localhost.evil.invalid:50310'):
            self.assertEqual((await self.request('GET', '/apps/control/', headers={'Host': other}))[0], 403)

    async def test_openchamber_preview_module_websocket_keeps_origin_boundary(self):
        import aiohttp
        host = 'openchamber-preview.localhost:50310'
        async with await self.websocket(headers={'Host': host}, origin='http://' + host) as websocket:
            self.assertEqual((await websocket.receive()).data, b'initial-binary')
            await websocket.send_str('preview-echo')
            self.assertEqual((await websocket.receive(timeout=3)).data, 'preview-echo')
        with self.assertRaises(aiohttp.WSServerHandshakeError) as failure:
            await self.websocket(headers={'Host': host}, origin='http://localhost:50310')
        self.assertEqual(failure.exception.status, 403)

    async def test_bounded_requests_and_unavailable_upstream(self):
        path = '/apps/control/settings'
        with patch('openarm_platform.proxy.MAX_BODY', 1024):
            code, _, _ = await self.request('POST', path, data=b'x' * 2048,
                                            headers={'Origin': self.base})
            self.assertEqual(code, 413)
            async def chunks():
                yield b'x' * 768
                yield b'y' * 768
            code, _, _ = await self.request('POST', path, data=chunks(),
                                            headers={'Origin': self.base})
            self.assertEqual(code, 413)
        self.release_stream.set()
        await self.upstream_runner.cleanup()
        code, _, body = await self.request('GET', '/apps/control/status')
        self.assertEqual(code, 502)
        self.assertIn('Module service unavailable', json.loads(body)['error'])

    async def test_cookie_isolation_and_multiple_cookie_paths(self):
        _, headers, _ = await self.request('GET', '/apps/control/cookies')
        cookies = headers.getall('Set-Cookie')
        self.assertEqual(len(cookies), 2)
        self.assertTrue(all('Path=/apps/control/' in value for value in cookies))
        _, _, body = await self.request('GET', '/apps/control/status')
        self.assertEqual(json.loads(body)['cookie'], '')
        _, _, body = await self.request('GET', '/apps/control/status', headers={'Cookie': 'first=browser'})
        self.assertEqual(json.loads(body)['cookie'], 'first=browser')

    async def test_websocket_binary_text_subprotocol_and_origin(self):
        import aiohttp
        async with await self.websocket() as websocket:
            self.assertEqual(websocket.protocol, 'viser-test')
            self.assertEqual((await websocket.receive()).data, b'initial-binary')
            # Exceeds aiohttp's default 4 MiB limit; real Viser model payloads can too.
            payload = b'\x00\xff' * (3 * 1024 * 1024)
            await websocket.send_bytes(payload)
            self.assertEqual((await websocket.receive(timeout=5)).data, payload)
            await websocket.send_str('text-message')
            self.assertEqual((await websocket.receive(timeout=3)).data, 'text-message')
        await asyncio.wait_for(self.upstream_disconnected.wait(), 3)
        with self.assertRaises(aiohttp.WSServerHandshakeError) as failure:
            await self.websocket(origin='https://evil.invalid')
        self.assertEqual(failure.exception.status, 403)

    async def test_shutdown_interrupts_live_websocket_and_stream(self):
        import aiohttp
        websocket = await self.websocket()
        await websocket.receive()
        stream = await self.client.get(self.base + '/apps/control/stream')
        self.assertEqual(await stream.content.readexactly(9), b'--frame\r\n')
        finite = await self.client.get(self.base + '/apps/control/finite-stream')
        self.assertEqual(await finite.content.readexactly(9), b'--frame\r\n')
        async with asyncio.timeout(3):
            await self.gateway.close()
            message = await websocket.receive()
        self.assertIn(message.type, (aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.CLOSED, aiohttp.WSMsgType.ERROR))
        self.assertTrue(self.gateway.session.closed)
        self.assertFalse(self.gateway._responses)
        await websocket.close()
        stream.close()
        finite.close()
        self.assertEqual((await self.request('GET', '/apps/control/'))[0], 503)
        self.assertEqual(self.asgi_errors, [])

    async def test_browser_disappearing_during_close_is_normal_cleanup(self):
        websocket = await self.websocket()
        await websocket.receive()
        self.reject_browser_close = True
        await websocket.close()
        await asyncio.wait_for(self.upstream_disconnected.wait(), 3)
        async with asyncio.timeout(3):
            while self.gateway._tasks:
                await asyncio.sleep(.01)
        self.assertEqual(self.asgi_errors, [])
        self.assertFalse(self.gateway._websockets)

    async def test_external_cancellation_is_not_swallowed(self):
        websocket = await self.websocket()
        await websocket.receive()
        task = next(iter(self.gateway._tasks))
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        self.assertEqual(self.asgi_errors, ['CancelledError'])
        self.assertFalse(self.gateway._websockets)
        await websocket.close()


class PlatformRouteTests(unittest.TestCase):
    def test_gateway_never_resolves_arbitrary_destinations(self):
        mapping = {'/apps/control': 8893}
        for path in ('http://evil.invalid/apps/control/', '//evil.invalid/apps/control/', '/apps/other/'):
            self.assertIsNone(resolve(path, mapping))
        self.assertEqual(resolve('/apps/control/?u=http://evil.invalid', mapping),
                         ('/apps/control', 8893, '/?u=http://evil.invalid'))
        content = b'<html><head></head><body><a href="//example.com/a">x</a><img src="data:image/png;base64,a"></body></html>'
        rewritten = rewrite_html(content, '/apps/control', mapping)
        self.assertIn(b'href="//example.com/a"', rewritten)
        self.assertIn(b'src="data:image/png;base64,a"', rewritten)


if __name__ == '__main__':
    unittest.main()
