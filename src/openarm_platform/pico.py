"""WebXR routes for the existing writable simulation workspace."""
import asyncio
from importlib.resources import files
import time

from openarm_simulation.devices import SimulationBus
from openarm_simulation.pico import PicoTeleop, WATCHDOG, cartesian_start, controller_sample


class PicoSessions:
    def __init__(self):
        self.sessions = set()

    def stop(self):
        for session in self.sessions:
            session.stop()


async def stream_samples(socket, session, closing, *, timeout=WATCHDOG):
    try:
        while not closing():
            message = await asyncio.wait_for(socket.receive(), timeout)
            if message['type'] == 'websocket.disconnect':
                break
            sample = controller_sample(message.get('text') or '')
            active = session.sample(sample, time.monotonic())
            await asyncio.wait_for(socket.send_json({'following': active}), timeout)
    finally:
        session.release()


def install_pico(app, backend):
    from fastapi import WebSocket, WebSocketDisconnect
    from starlette.responses import HTMLResponse, Response

    sessions = PicoSessions()
    static = files('openarm_platform').joinpath('static')

    @app.get('/simulation/teleop')
    async def page():
        return HTMLResponse(static.joinpath('pico.html').read_text(encoding='utf-8'))

    @app.get('/simulation/pico.js')
    async def script():
        return Response(static.joinpath('pico.js').read_text(encoding='utf-8'),
                        media_type='text/javascript')

    @app.websocket('/simulation/pico/input')
    async def input_stream(socket: WebSocket):
        if socket.url.scheme != 'wss' and socket.url.hostname not in ('localhost', '127.0.0.1'):
            await socket.close(code=1008)
            return
        if app.state.coordinator.closing or len(sessions.sessions) >= 4:
            await socket.close(code=1013)
            return
        world = backend.world
        session = PicoTeleop(SimulationBus(world), world.lease,
                             lambda joints: cartesian_start(backend.chain, joints))
        sessions.sessions.add(session)
        try:
            await socket.accept()
            await stream_samples(socket, session, lambda: app.state.coordinator.closing)
        except TimeoutError:
            await socket.close(code=1000)
        except (WebSocketDisconnect, OSError):
            pass
        except (ValueError, RuntimeError, ImportError):
            await socket.close(code=1011, reason='Simulation input rejected')
        finally:
            session.release()
            sessions.sessions.discard(session)

    return sessions
