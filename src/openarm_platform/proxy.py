"""Fixed-route loopback HTTP/WebSocket gateway for the existing module pages.

No arbitrary destination can be supplied by a browser. Application connections
keep their original lifetimes; stopping the gateway closes every proxy socket.
"""
import asyncio
import ipaddress
import json
import re
from urllib.parse import urlsplit

LOCAL_HOSTS = ('127.0.0.1', 'localhost', '::1', 'openchamber-preview.localhost')

HOP_HEADERS = {'connection', 'keep-alive', 'proxy-authenticate', 'proxy-authorization',
               'te', 'trailer', 'transfer-encoding', 'upgrade'}
MAX_BODY = 16 * 1024 * 1024
MAX_WS_MESSAGE = 64 * 1024 * 1024


def routes(config):
    result = {}
    for page in (*config.pages,*(p for p in config.support if p.id=='head_alignment')):
        if page.path is not None:
            continue
        prefix = f'/apps/{page.id}'
        result[prefix] = page.port
        for index, port in enumerate(page.extra_ports):
            result[f'{prefix}/_extra/{index}'] = port
    return result


def resolve(path, mapping):
    """Match the most specific configured prefix, preserving the raw query."""
    parsed = urlsplit(path)
    if parsed.scheme or parsed.netloc:
        return None
    for prefix in sorted(mapping, key=len, reverse=True):
        if parsed.path == prefix or parsed.path.startswith(prefix + '/'):
            suffix = parsed.path[len(prefix):] or '/'
            if parsed.query:
                suffix += '?' + parsed.query
            return prefix, mapping[prefix], suffix
    return None


def bridge(prefix, mapping):
    # Keep legacy camera fetches/downloads and the tactile page's two iframe
    # setters on this origin. Viser's worker already receives the full prefixed
    # page URL, so its binary websocket protocol remains completely untouched.
    ports = {str(port): path + '/' for path, port in mapping.items()}
    return r'''<script>(()=>{'use strict';
const base=BASE,ports=PORTS;
const local=new Set(['127.0.0.1','localhost','[::1]','openchamber-preview.localhost']);
function route(value){
 if(typeof value!=='string'&&!(value instanceof URL))return value;
 const raw=String(value);if(!/^(?:https?:|wss?:|\/)/.test(raw))return value;
 let u;try{u=new URL(raw,location.href);}catch(_){return value;}
 if(!['http:','https:','ws:','wss:'].includes(u.protocol))return value;
 const same=u.host===location.host;
 const path=local.has(u.hostname)&&ports[u.port];
 if(!same&&!path)return value;
 if(same&&(u.pathname===base.slice(0,-1)||u.pathname.startsWith('/apps/')))return value;
 u.host=location.host;u.pathname=(same?base:path)+u.pathname.replace(/^\//,'');
 return u.href;
}
const fetch0=window.fetch;window.fetch=function(input,init){
 if(input instanceof Request){const url=route(input.url);if(url!==input.url)input=new Request(url,input);}
 else input=route(input);return fetch0.call(this,input,init);
};
const open=XMLHttpRequest.prototype.open;XMLHttpRequest.prototype.open=function(method,url,...rest){return open.call(this,method,route(url),...rest);};
for(const [type,key] of [[HTMLIFrameElement,'src'],[HTMLImageElement,'src'],[HTMLScriptElement,'src'],[HTMLLinkElement,'href'],[HTMLAnchorElement,'href'],[HTMLFormElement,'action']]){
 const descriptor=Object.getOwnPropertyDescriptor(type.prototype,key);if(!descriptor||!descriptor.set)continue;
 Object.defineProperty(type.prototype,key,{...descriptor,set(value){descriptor.set.call(this,route(value));}});
}
const set=Element.prototype.setAttribute;Element.prototype.setAttribute=function(key,value){
 if(['src','href','action','poster'].includes(String(key).toLowerCase()))value=route(value);
 return set.call(this,key,value);
};
const ws=window.WebSocket;window.WebSocket=new Proxy(ws,{construct(target,args){args[0]=route(args[0]);return Reflect.construct(target,args);}});
})();</script>'''.replace('BASE', json.dumps(prefix + '/')).replace('PORTS', json.dumps(ports))


def rewrite_html(content, prefix, mapping):
    document = content.decode('utf-8')
    # Parser-created URLs bypass JS property setters. Rewrite only URL-valued
    # attributes, never embedded Viser bundles, data URLs or application text.
    document = re.sub(r'''(\b(?:src|href|action|poster)\s*=\s*["'])(/(?!/|apps/))''',
                      lambda match: match[1] + prefix + '/', document, flags=re.I)
    injected = bridge(prefix, mapping)
    head = re.search(r'<head(?:\s[^>]*)?>', document, re.I)
    if head:
        document = document[:head.end()] + injected + document[head.end():]
    else:
        document = injected + document
    return document.encode('utf-8')


def rewrite_location(value, prefix, port):
    parsed = urlsplit(value)
    if value.startswith('/') and not value.startswith('//'):
        return prefix + value
    if parsed.hostname in ('localhost', '127.0.0.1', '::1') and parsed.port == port:
        return prefix + (parsed.path or '/') + ('?' + parsed.query if parsed.query else '') + ('#' + parsed.fragment if parsed.fragment else '')
    return value


def is_ipv4_host(value):
    try:
        ipaddress.IPv4Address(value)
        return True
    except (ValueError,TypeError):
        return False


def valid_host(headers, allowed_hosts=LOCAL_HOSTS, allow_ip_hosts=False):
    """Accept the local SSH-forward origin, never arbitrary DNS hostnames."""
    try:
        host = urlsplit('http://' + headers.get('host', ''))
        return ((host.hostname in allowed_hosts or (allow_ip_hosts and is_ipv4_host(host.hostname)))
                and host.username is None and host.password is None
                and not host.path and not host.query and not host.fragment
                and (host.port is None or 1 <= host.port <= 65535))
    except ValueError:
        return False


def same_origin(connection, allowed_hosts=LOCAL_HOSTS, allow_ip_hosts=False):
    scheme = 'https' if connection.scope.get('scheme') in ('https', 'wss') else 'http'
    return (valid_host(connection.headers,allowed_hosts,allow_ip_hosts)
            and connection.headers.get('origin') == scheme + '://' + connection.headers.get('host', ''))


def _target(connection, mapping):
    raw = connection.scope.get('raw_path', connection.url.path.encode()).decode('ascii')
    query = connection.scope.get('query_string', b'').decode('ascii')
    return resolve(raw + ('?' + query if query else ''), mapping)


def _headers(connection, port, *, websocket=False):
    tokens = {x.strip().lower() for x in connection.headers.get('connection', '').split(',')}
    omitted = HOP_HEADERS | tokens | {'host', 'accept-encoding', 'content-length', 'origin'}
    if websocket:
        # aiohttp owns the handshake and negotiates compression for its own leg.
        omitted |= {'sec-websocket-key', 'sec-websocket-version', 'sec-websocket-extensions',
                    'sec-websocket-protocol'}
    headers = [(key.decode('latin-1'), value.decode('latin-1'))
               for key, value in connection.scope['headers'] if key.decode('latin-1').lower() not in omitted]
    headers.extend([('Host', f'127.0.0.1:{port}'), ('Accept-Encoding', 'identity')])
    if connection.headers.get('origin') is not None:
        headers.append(('Origin', f'http://127.0.0.1:{port}'))
    return headers


def _response_headers(response, prefix, port, *, html=False):
    omitted = HOP_HEADERS | {'server', 'date', 'x-frame-options'}
    omitted |= {x.strip().lower() for x in response.headers.get('Connection', '').split(',')}
    if html:
        omitted |= {'content-length', 'content-encoding', 'etag', 'content-md5', 'content-security-policy'}
    headers = []
    for raw_key, raw_value in response.raw_headers:
        key, value = raw_key.decode('latin-1'), raw_value.decode('latin-1')
        if key.lower() in omitted:
            continue
        if key.lower() == 'location':
            value = rewrite_location(value, prefix, port)
        if key.lower() == 'set-cookie':
            value = re.sub(r'(?i)(;\s*path=)/', lambda match: match[1] + prefix + '/', value)
        headers.append((key.lower().encode('latin-1'), value.encode('latin-1')))
    headers.append((b'x-frame-options', b'SAMEORIGIN'))
    return headers


class Gateway:
    """ASGI routes using one aiohttp session, owned by the platform lifespan.

    Construction imports no optional framework and opens no connections. Call
    ``install(app)`` before mounting NiceGUI; await ``start``/``close`` from the
    outer ASGI lifespan. Browser disconnects close their corresponding upstream
    promptly. ``close`` also interrupts active streams and WebSocket forwarding.
    """

    def __init__(self, config):
        self.allowed_hosts = (*LOCAL_HOSTS,config.host)
        self.allow_ip_hosts = config.host == '0.0.0.0'
        self.mapping = routes(config)
        self.session = None
        self._responses = set()
        self._websockets = set()
        self._tasks = set()
        self._closed = False

    def install(self, app):
        from starlette.routing import Route, WebSocketRoute
        app.router.routes.extend([
            Route('/apps/{path:path}', self.http,
                  methods=['GET', 'HEAD', 'POST', 'PUT', 'PATCH', 'DELETE', 'OPTIONS']),
            WebSocketRoute('/apps/{path:path}', self.websocket),
        ])

    async def start(self):
        import aiohttp
        if self.session is not None or self._closed:
            raise RuntimeError('Gateway has already started or closed')
        self.session = aiohttp.ClientSession(
            cookie_jar=aiohttp.DummyCookieJar(), auto_decompress=False, trust_env=False,
            timeout=aiohttp.ClientTimeout(total=None, connect=10, sock_read=45),
            connector=aiohttp.TCPConnector(limit=256, limit_per_host=64),
        )

    async def close(self):
        if self._closed:
            return
        self._closed = True
        # Cancel tasks first: neither an idle stream nor a peer waiting for a
        # WebSocket close acknowledgement may delay hardware service cleanup.
        tasks = [task for task in self._tasks if task is not asyncio.current_task()]
        for task in tasks:
            task.cancel()
        for response in list(self._responses):
            response.close()
        if self.session is not None:
            await self.session.close()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        self._tasks.clear()
        self._responses.clear()
        self._websockets.clear()

    async def http(self, request):
        import aiohttp
        from starlette.responses import JSONResponse, RedirectResponse, Response, StreamingResponse
        from starlette.requests import ClientDisconnect

        if not valid_host(request.headers,self.allowed_hosts,self.allow_ip_hosts):
            return JSONResponse({'error': 'Use an allowed platform address'}, status_code=403)
        try:
            route = _target(request, self.mapping)
        except (UnicodeError, ValueError):
            route = None
        if route is None:
            return JSONResponse({'error': 'Unknown module route'}, status_code=404)
        if request.method not in ('GET', 'HEAD') and not same_origin(request,self.allowed_hosts,self.allow_ip_hosts):
            return JSONResponse({'error': 'Same-origin request required'}, status_code=403)
        prefix, port, target = route
        if self._closed or self.session is None:
            return JSONResponse({'error': 'Platform gateway is not running'}, status_code=503)
        if request.url.path == prefix and request.method in ('GET', 'HEAD'):
            return RedirectResponse(prefix + '/' + ('?' + request.url.query if request.url.query else ''), status_code=307)
        lengths = request.headers.getlist('content-length')
        if len(lengths) > 1 or lengths and request.headers.get('transfer-encoding'):
            return JSONResponse({'error': 'Ambiguous request framing'}, status_code=400)
        try:
            expected = int(lengths[0]) if lengths else None
            if expected is not None and not 0 <= expected <= MAX_BODY:
                raise ValueError()
        except ValueError:
            return JSONResponse({'error': 'Invalid request size'}, status_code=413)
        body = bytearray()
        try:
            async for chunk in request.stream():
                body.extend(chunk)
                if len(body) > MAX_BODY:
                    return JSONResponse({'error': 'Request body exceeds gateway limit'}, status_code=413)
        except ClientDisconnect:
            return Response(status_code=400)
        if expected is not None and len(body) != expected:
            return JSONResponse({'error': 'Incomplete request body'}, status_code=400)
        task = asyncio.current_task()
        self._tasks.add(task)
        upstream = None
        handed_off = False
        try:
            upstream = await self.session.request(
                request.method, f'http://127.0.0.1:{port}{target}',
                data=bytes(body) if body else None, headers=_headers(request, port),
                allow_redirects=False,
            )
            self._responses.add(upstream)
            html = (request.method != 'HEAD' and
                    upstream.headers.get('Content-Type', '').split(';')[0].strip().lower() == 'text/html')
            if html:
                if upstream.headers.get('Content-Encoding', 'identity') != 'identity':
                    raise ValueError('Upstream ignored identity content encoding')
                content = bytearray()
                async for chunk in upstream.content.iter_chunked(64 * 1024):
                    content.extend(chunk)
                    if len(content) > MAX_BODY:
                        raise ValueError('Upstream HTML exceeds gateway limit')
                content = rewrite_html(bytes(content), prefix, self.mapping)
                response = Response(content, status_code=upstream.status)
                response.raw_headers = _response_headers(upstream, prefix, port, html=True)
                response.raw_headers.append((b'content-length', str(len(content)).encode('ascii')))
                return response
            if request.method == 'HEAD' or upstream.status in (204, 304):
                response = Response(status_code=upstream.status)
                response.raw_headers = _response_headers(upstream, prefix, port)
                return response

            async def chunks():
                stream_task = asyncio.current_task()
                self._tasks.add(stream_task)
                try:
                    async for chunk in upstream.content.iter_chunked(64 * 1024):
                        if self._closed:
                            break
                        yield chunk
                except asyncio.CancelledError:
                    # Intentional gateway shutdown is a normal end of stream.
                    # Request/server cancellations unrelated to our close still
                    # propagate so their owner can perform its own cleanup.
                    if not self._closed:
                        raise
                finally:
                    upstream.close()
                    self._responses.discard(upstream)
                    self._tasks.discard(stream_task)

            response = StreamingResponse(chunks(), status_code=upstream.status)
            # Let ASGI frame streamed bodies. A gateway shutdown may finish an
            # in-progress frame early; retaining upstream Content-Length would
            # turn that normal cancellation into an invalid HTTP response.
            response.raw_headers = [(key, value) for key, value in
                                    _response_headers(upstream, prefix, port)
                                    if key != b'content-length']
            handed_off = True
            return response
        except asyncio.CancelledError:
            if not self._closed:
                raise
            return JSONResponse({'error': 'Platform gateway is stopping'}, status_code=503)
        except (aiohttp.ClientError, OSError, ValueError, UnicodeError):
            return JSONResponse({'error': 'Module service unavailable; check its platform status'}, status_code=502)
        finally:
            self._tasks.discard(task)
            if upstream is not None and not handed_off:
                upstream.close()
                self._responses.discard(upstream)

    async def websocket(self, downstream):
        import aiohttp
        from starlette.websockets import WebSocketDisconnect

        async def close_browser(code):
            try:
                await downstream.close(code=code)
            except (OSError, RuntimeError, WebSocketDisconnect):
                # The browser can disappear between receive() and close().
                pass

        if not same_origin(downstream,self.allowed_hosts,self.allow_ip_hosts):
            await close_browser(1008)
            return
        try:
            route = _target(downstream, self.mapping)
        except (UnicodeError, ValueError):
            route = None
        if route is None or self._closed or self.session is None:
            await close_browser(1008)
            return
        _prefix, port, target = route
        task = asyncio.current_task()
        self._tasks.add(task)
        upstream = None
        accepted = False
        workers = []
        try:
            upstream = await self.session.ws_connect(
                f'http://127.0.0.1:{port}{target}',
                headers=_headers(downstream, port, websocket=True),
                protocols=downstream.scope.get('subprotocols', ()),
                max_msg_size=MAX_WS_MESSAGE, heartbeat=20,
                timeout=aiohttp.ClientWSTimeout(ws_receive=None, ws_close=2),
            )
            self._websockets.add(upstream)
            await downstream.accept(subprotocol=upstream.protocol)
            accepted = True

            async def from_browser():
                while True:
                    message = await downstream.receive()
                    if message['type'] == 'websocket.disconnect':
                        return
                    if message.get('bytes') is not None:
                        await upstream.send_bytes(message['bytes'])
                    elif message.get('text') is not None:
                        await upstream.send_str(message['text'])

            async def from_module():
                async for message in upstream:
                    if message.type == aiohttp.WSMsgType.BINARY:
                        await downstream.send_bytes(message.data)
                    elif message.type == aiohttp.WSMsgType.TEXT:
                        await downstream.send_text(message.data)
                    elif message.type in (aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.CLOSED,
                                           aiohttp.WSMsgType.ERROR):
                        return

            workers = [asyncio.create_task(from_browser()), asyncio.create_task(from_module())]
            done, _ = await asyncio.wait(workers, return_when=asyncio.FIRST_COMPLETED)
            for worker in done:
                worker.result()
        except asyncio.CancelledError:
            if not self._closed:
                raise
        except (aiohttp.ClientError, OSError, WebSocketDisconnect, RuntimeError):
            pass
        finally:
            for worker in workers:
                worker.cancel()
            if workers:
                await asyncio.gather(*workers, return_exceptions=True)
            if upstream is not None:
                await upstream.close()
                self._websockets.discard(upstream)
            await close_browser(1001 if self._closed else 1000 if accepted else 1011)
            self._tasks.discard(task)
