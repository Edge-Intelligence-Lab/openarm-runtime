"""NiceGUI/FastAPI workspace with one aiohttp gateway and one service owner.

Importing this module is passive and does not require the platform extra.
"""
import asyncio
import ipaddress
from contextlib import asynccontextmanager
import json
import socket
import re
from urllib.parse import urlsplit

from .services import ServiceManager
from .proxy import LOCAL_HOSTS, is_ipv4_host


class Coordinator:
    """Keep blocking hardware-process operations off the ASGI event loop.

    Request cancellation does not cancel an in-progress stop/switch. Shutdown
    waits for those operations before the manager performs its final cleanup.
    """

    def __init__(self, manager, *, attach_only=False, builtin_pages=()):
        self.manager = manager
        self.builtin_pages = tuple(builtin_pages)
        self.attach_only = attach_only
        self.closing = False
        self.models = None
        self._operations = set()

    async def _call(self, function, *args):
        if self.closing:
            raise RuntimeError('平台正在关闭，请等待服务停止。')
        operation = asyncio.create_task(asyncio.to_thread(function, *args))
        self._operations.add(operation)
        def finished(task):
            self._operations.discard(task)
            if not task.cancelled():
                task.exception()  # Retrieve errors even if the request disconnected.
        operation.add_done_callback(finished)
        return await asyncio.shield(operation)

    async def start(self):
        if not self.attach_only:
            await self._call(self.manager.start)

    def with_builtin(self, states):
        return {**states, **{name: dict(reachable=True, state='running', managed=False, generation=0)
                            for name in self.builtin_pages}}

    async def status(self):
        return self.with_builtin(await self._call(self.manager.status))

    async def activate(self, name):
        if self.attach_only or name in self.builtin_pages:
            return await self.status()
        return self.with_builtin(await self._call(self.manager.activate, name))

    async def deactivate(self,name):
        if self.attach_only:raise ValueError('只读模式不能停止服务')
        return self.with_builtin(await self._call(self.manager.deactivate,name))

    async def connection_status(self):
        if self.models is None:
            return dict(state='unmonitored', message='连接在执行前检查')
        if self.attach_only:
            return dict(state='readonly', message='只读平台，不自动查询或控制实物')
        if self.models.simulation:
            return dict(state='simulation', message='显式仿真平台，实物连接禁用')
        return await self._call(self.models.connectivity.snapshot)

    async def stop_motion(self):
        if self.attach_only:
            raise ValueError('只读平台不能停止外部控制')
        if self.models is not None:
            await self._call(self.models.stop)
        return self.with_builtin(await self._call(self.manager.stop_motion))

    def begin_shutdown(self):
        self.closing = True

    async def close(self):
        self.begin_shutdown()
        if self._operations:
            await asyncio.gather(*tuple(self._operations), return_exceptions=True)
        await asyncio.to_thread(self.manager.close)


class LocalBoundary:
    """Protect both framework events and proxied modules on the loopback server."""

    # OpenChamber carries loopback previews through its authenticated desktop
    # tunnel without rewriting Host or Origin. Trust only its exact preview
    # hostname, not arbitrary .localhost names; origin/port equality stays below.
    allowed_hosts = LOCAL_HOSTS

    def __init__(self, app, host='127.0.0.1', allowed_clients=(), port=8888):
        self.app = app
        self.allowed_hosts = (*LOCAL_HOSTS,host)
        self.allow_ip_hosts = host == '0.0.0.0'
        self.lan_host = host if host != '127.0.0.1' else None
        self.port = port
        self.client_networks = tuple(ipaddress.ip_network(n) for n in allowed_clients)

    async def __call__(self, scope, receive, send):
        if scope['type'] not in ('http', 'websocket'):
            return await self.app(scope, receive, send)
        headers = scope.get('headers', [])
        hosts = [v.decode('latin-1') for k, v in headers if k.lower() == b'host']
        origins = [v.decode('latin-1') for k, v in headers if k.lower() == b'origin']
        host = hosts[0] if len(hosts) == 1 else ''
        try:
            address = urlsplit('http://' + host)
            valid = (bool(host) and (address.hostname in self.allowed_hosts or
                     (self.allow_ip_hosts and is_ipv4_host(address.hostname)))
                     and address.netloc == host and not address.username and not address.password
                     and not address.path and not address.query and not address.fragment)
            _ = address.port  # Reject malformed port syntax.
            if (address.hostname == self.lan_host or
                    (self.allow_ip_hosts and address.hostname not in LOCAL_HOSTS)) and address.port != self.port:
                valid = False
        except ValueError:
            valid = False
        unsafe = scope['type'] == 'websocket' or scope.get('method') not in ('GET', 'HEAD', 'OPTIONS')
        client_allowed = True
        if self.client_networks:
            try:
                peer = ipaddress.ip_address(scope['client'][0])
                client_allowed = peer.is_loopback or any(peer in n for n in self.client_networks)
            except (ValueError,TypeError,KeyError):
                client_allowed = False
        if (not valid or not client_allowed or (origins and origins != ['http://' + host])
                or (unsafe and origins != ['http://' + host])):
            if scope['type'] == 'websocket':
                await send({'type': 'websocket.close', 'code': 1008})
            else:
                body = b'{"error":"Use an allowed address, client network and same-origin request"}'
                await send({'type': 'http.response.start', 'status': 403,
                            'headers': [(b'content-type', b'application/json'),
                                        (b'content-length', str(len(body)).encode())]})
                await send({'type': 'http.response.body', 'body': body})
            return

        async def secured(message):
            if message['type'] == 'http.response.start':
                message['headers'] = [(key, value) for key, value in message.get('headers', [])
                                      if key.lower() not in (b'x-frame-options', b'x-content-type-options')]
                message['headers'].extend([(b'x-frame-options', b'SAMEORIGIN'),
                                           (b'x-content-type-options', b'nosniff')])
            await send(message)
        await self.app(scope, receive, secured)


def create_app(config, manager=None, *, attach_only=False, with_ui=True):
    """Construct one ASGI application; hardware starts only in its lifespan.

    ``with_ui=False`` is for focused API/gateway tests. Normal platform launches
    always mount the NiceGUI dashboard. Do not run this app with multiple workers.
    """
    try:
        from fastapi import FastAPI, Request
        from starlette.responses import JSONResponse, Response, FileResponse, StreamingResponse
        from .proxy import Gateway
        if with_ui:
            from nicegui import ui
            from .dashboard import build_dashboard
    except ImportError as exc:
        raise RuntimeError('平台需要可选依赖：pip install "openarm-runtime[platform]"') from exc

    if manager is None:
        manager = ServiceManager(config.services, config.log_dir)
    coordinator = Coordinator(manager, attach_only=attach_only,
                              builtin_pages=(p.id for p in config.pages if p.path))
    gateway = Gateway(config)
    simulation=None
    if config.simulation_profile is not None:
        from .simulation import create_simulation
        simulation=create_simulation(config)
    from .policy import PolicyPreview
    preview = PolicyPreview(config.policy_record_root)
    from .model_workspace import ModelWorkspace
    models = ModelWorkspace(config.model_catalog, config.policy_record_root / 'deployment',
                            manager, readonly=attach_only, simulation=simulation is not None)
    models.video_preview_port = next((p.port for p in config.services if p.id == 'vision'), None)
    coordinator.models = models
    if models.port is not None and models.port in {config.port, *[p.port for p in config.services],
                                                 *[port for p in config.services for port in p.extra_ports]}:
        raise ValueError('模型推理端口必须与平台和其他服务端口不同')
    from .camera_calibration import CalibrationWorkspace
    calibration = CalibrationWorkspace(config.calibration_root,
        next(p.port for p in config.pages if p.id == 'vision'))
    calibration.simulation_backend=simulation
    calibration.teleop_calibration = config.teleop_calibration_profile is not None
    calibration.alignment_coordinator=coordinator
    calibration.attach_only=attach_only

    @asynccontextmanager
    async def lifespan(app):
        try:
            await gateway.start()
            if simulation is not None:
                await asyncio.to_thread(simulation.start)
            await coordinator.start()
            # Missing CAN must not prevent the web server from accepting pages.
            # This starts query-only workers, never a model or motion controller.
            models.start_connectivity()
            yield
        finally:
            # Model execution must release/disable through the existing CLI before
            # the manager closes peer services. Never force-kill a hardware child.
            coordinator.begin_shutdown()
            model_error = None
            try:
                await asyncio.to_thread(models.close)
            except BaseException as exc:
                model_error = exc
            calibration.close()
            try:
                try:
                    await asyncio.to_thread(preview.close)
                finally:
                    try:
                        await gateway.close()
                    finally:
                        try:await coordinator.close()
                        finally:
                            if simulation is not None:await asyncio.to_thread(simulation.close)
            except BaseException as exc:
                app.state.shutdown_error = exc
                raise
            if model_error is not None:
                app.state.shutdown_error = model_error
                raise model_error

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None, openapi_url=None)
    app.state.gateway = gateway
    app.state.policy_preview = preview
    app.state.model_workspace = models
    app.state.camera_calibration = calibration
    app.state.coordinator = coordinator
    app.state.shutdown_error = None
    app.state.simulation_backend=simulation
    app.add_middleware(LocalBoundary,host=config.host,allowed_clients=config.allowed_clients,port=config.port)

    @app.get('/api/config')
    async def configuration():
        return JSONResponse(dict(title=config.title, port=config.port, attach_only=attach_only,
                                 backend='mujoco' if simulation else 'hardware',simulation=simulation is not None,
                                 pages=[p.public() for p in config.pages]),
                            headers={'Cache-Control': 'no-store'})

    @app.get('/api/status')
    async def status():
        try:
            return JSONResponse({'services': await coordinator.status()}, headers={'Cache-Control': 'no-store'})
        except RuntimeError as exc:
            return JSONResponse({'error': str(exc)}, status_code=503)

    @app.get('/api/models')
    async def model_status():
        try:
            return JSONResponse(await asyncio.to_thread(models.snapshot), headers={'Cache-Control': 'no-store'})
        except (OSError, ValueError, TypeError, KeyError) as exc:
            return JSONResponse({'error': '状态暂不可用，请稍后重试：'+str(exc), 'retryable': True},
                                status_code=503, headers={'Cache-Control': 'no-store'})

    @app.get('/api/models/input-image/{key}')
    async def model_input_image(key: str):
        with models.lock:
            root = models.run_root
        if root is None or not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', key):
            return Response(status_code=404)
        path = root / 'episode/model-inputs' / (key+'.png')
        if not path.is_file() or path.is_symlink() or path.stat().st_size > 4_000_000:
            return Response(status_code=404)
        return Response(await asyncio.to_thread(path.read_bytes), media_type='image/png',
                        headers={'Cache-Control': 'no-store'})

    @app.get('/api/models/recording')
    async def model_recording():
        with models.lock:
            root, running = models.run_root, models.process is not None
        path = root / 'episode/recording.rrd' if root is not None else None
        if running or path is None or not path.is_file() or path.is_symlink():
            return Response(status_code=404)
        return FileResponse(path, filename=root.name+'.rrd', media_type='application/octet-stream')

    @app.get('/api/models/input-stream/{key}')
    async def model_input_stream(key: str, run: str = ''):
        from .model_stream import input_frames
        with models.lock:
            root = models.run_root
        if root is None or not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', key) or run and root.name != run:
            return Response(status_code=404)
        path = root/'episode/model-inputs'/(key+'.png')
        if not path.is_file() or path.is_symlink():
            return Response(status_code=404)
        frames = input_frames(path, lambda: not models.closed and models.run_root == root
                              and models.process is not None)
        return StreamingResponse(frames, media_type='multipart/x-mixed-replace; boundary=frame',
                                 headers={'Cache-Control': 'no-store', 'X-Accel-Buffering': 'no'})

    @app.post('/api/select')
    async def select(request: Request):
        if request.headers.get('content-type', '').split(';')[0].strip() != 'application/json':
            return JSONResponse({'error': 'Same-origin JSON request required'}, status_code=403)
        try:
            content = bytearray()
            async for chunk in request.stream():
                content.extend(chunk)
                if len(content) > 1024:
                    raise ValueError('Invalid request size')
            data = json.loads(content)
            if (not isinstance(data, dict) or set(data) != {'id'} or not isinstance(data['id'], str)
                    or data['id'] not in {page.id for page in config.pages}):
                raise ValueError('Select a configured page')
            return JSONResponse({'services': await coordinator.activate(data['id'])},
                                headers={'Cache-Control': 'no-store'})
        except (ValueError, RuntimeError, OSError) as exc:
            return JSONResponse({'error': str(exc)}, status_code=409)

    gateway.install(app)
    if with_ui:
        from .policy_view import build_policy_view
        from .collection_view import build_collection_view
        from .camera_calibration_view import build_camera_calibration_view

        @ui.page('/camera-calibration', title='相机标定 · OpenArm')
        async def camera_calibration_page():
            build_camera_calibration_view(calibration)

        if simulation is not None:
            from .simulation import build_simulation_view
            @ui.page('/simulation/control',title='仿真机械臂 · OpenArm')
            def simulation_control_page():
                build_simulation_view(simulation,attach_only=attach_only)

            @ui.page('/simulation/teleop',title='仿真遥操 · OpenArm')
            def simulation_teleop_page():
                ui.label('仿真模式 · 遥操尚未接入，不会连接主臂串口。')

            @ui.page('/simulation/tactile',title='仿真触觉 · OpenArm')
            def simulation_tactile_page():
                ui.label('仿真模式 · 尚无触觉传感器模型，不提供伪造触觉值。')

        @ui.page('/collection', title='数采可视化 · OpenArm')
        async def collection_page():
            build_collection_view(config.collection_root)

        @ui.page('/policy', title='模型与执行 · OpenArm')
        async def policy_page():
            build_policy_view(preview, models)

        async def dashboard():
            await build_dashboard(config, coordinator, attach_only=attach_only)
        ui.run_with(app, root=dashboard, title=config.title, language='zh-CN',
                    dark=False, show_welcome_message=False, reconnect_timeout=10,
                    binding_refresh_interval=None, on_air=None)
    return app


def serve(config, *, attach_only=False):
    """Run one loopback worker, binding before any service can be launched."""
    try:
        import uvicorn
    except ImportError as exc:
        raise RuntimeError('平台需要可选依赖：pip install "openarm-runtime[platform]"') from exc
    # Bind first: an occupied entry port must never trigger child/device startup.
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((config.host, config.port))
        listener.listen(128)
        app = create_app(config, attach_only=attach_only)

        class PlatformServer(uvicorn.Server):
            async def shutdown(self, sockets=None):
                app.state.coordinator.begin_shutdown()
                await app.state.gateway.close()
                await super().shutdown(sockets=sockets)

        server = PlatformServer(uvicorn.Config(
            app, host=config.host, port=config.port, workers=1, reload=False,
            access_log=False, proxy_headers=False, server_header=False,
            ws_max_size=64 * 1024 * 1024, timeout_graceful_shutdown=10,
        ))
        print(f'OpenArm Platform: http://{config.host}:{config.port}', flush=True)
        if config.host != '127.0.0.1':
            if config.host == '0.0.0.0':
                print('All IPv4 interfaces are exposed without login. Open the workstation IP, not 0.0.0.0. Source IP restrictions: '+(', '.join(config.allowed_clients) or 'NONE'),flush=True)
            else:
                print('LAN access has no login: use only a trusted private network. Allowed clients: '+', '.join(config.allowed_clients),flush=True)
        print(f'服务日志：{config.log_dir} · Ctrl+C 关闭本次启动的服务', flush=True)
        try:
            server.run(sockets=[listener])
        finally:
            # Uvicorn re-raises handled signals after cleanup. A CLI interrupt
            # must not hide a failed hardware-service shutdown.
            if app.state.shutdown_error is not None:
                raise RuntimeError('Platform service cleanup failed') from app.state.shutdown_error
        if not server.started:
            raise RuntimeError('Platform startup failed; see the startup error above')
