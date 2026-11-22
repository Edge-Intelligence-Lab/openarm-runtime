"""WebXR routes for the existing writable simulation workspace."""
import asyncio
from contextlib import suppress
from html import escape
from importlib.resources import files
import logging
import math
import os
import time
from urllib.parse import urlsplit

from openarm_simulation.devices import SimulationBus
from openarm_simulation.pico import (
    MAX_ROTATION, MAX_TRANSLATION, RESET_SCENE, START_SESSION, PicoHands, PicoTeleop, WATCHDOG, cartesian_start,
    controller_sample, overview_axes,
)

CONNECTION_TIMEOUT = 6.0
PICO_HOME_LINK = '<a class="home-link" href="/simulation/teleop">← 首页 / Home</a>'
logger = logging.getLogger('uvicorn.error')


class PicoSessions:
    def __init__(self):
        self.sessions = set()

    def stop(self):
        for session in self.sessions:
            session.stop()


async def await_control(operation, session, timeout, connection_timeout):
    """Stop movement promptly while allowing a delayed network operation to finish."""
    pending = asyncio.create_task(operation)
    try:
        done, _ = await asyncio.wait((pending,), timeout=timeout)
        if done:
            return pending.result()
        session.stop()
        return await asyncio.wait_for(pending, connection_timeout - timeout)
    finally:
        if not pending.done():
            pending.cancel()
            with suppress(asyncio.CancelledError):
                await pending


async def stream_samples(socket, session, closing, *, timeout=WATCHDOG,
                         connection_timeout=CONNECTION_TIMEOUT, reset=None, reset_scene=None,
                         select_scene=None):
    first = True
    try:
        while not closing():
            message = await await_control(socket.receive(), session, timeout, connection_timeout)
            if message['type'] == 'websocket.disconnect':
                break
            sample = controller_sample(message.get('text') or '')
            if sample is RESET_SCENE:
                session.release()
                reply = {'reset': reset_scene is not None and reset_scene()}
            elif isinstance(sample, dict) and 'scene' in sample:
                if not first or select_scene is None:
                    raise ValueError('请退出遥操后再切换练习场景')
                scene = sample['scene']
                reply = {'scene': scene if select_scene(scene) else None}
            elif sample is START_SESSION:
                if not first or reset is None:
                    raise ValueError('只能在进入遥操时重置初始姿态')
                reply = {'started': reset()}
            else:
                reply = {'following': session.sample(sample, time.monotonic())}
                if isinstance(session, PicoHands) and session.blocked_sides:
                    reply['blocked'] = session.blocked_sides
                if isinstance(session, PicoHands) and session.paired_origins is not None:
                    reply['paired'] = True
            first = False
            await await_control(socket.send_json(reply), session, timeout, connection_timeout)
            if reply.get('reset') or reply.get('scene'):
                return True
    finally:
        session.release()


def install_pico(app, backend):
    from fastapi import HTTPException, WebSocket, WebSocketDisconnect
    from starlette.responses import HTMLResponse, JSONResponse, Response

    sessions = PicoSessions()
    static = files('openarm_platform').joinpath('static')
    official_sim_url = os.environ.get('OPENARM_PICO_OFFICIAL_SIM_URL')
    if official_sim_url:
        endpoint = urlsplit(official_sim_url)
        if endpoint.scheme != 'https' or not endpoint.hostname or endpoint.username is not None:
            raise ValueError('OPENARM_PICO_OFFICIAL_SIM_URL must be an HTTPS entry point')
    overview = backend.world.config['cameras']['overview']
    axes = overview_axes(overview['position_m'], overview['target_m'])
    chains = {'left': backend.chain}
    fingers = None
    travel = MAX_TRANSLATION
    shared_contact = None
    telemetry = None
    if backend.world.config.get('pico_grasp'):
        from openarm_simulation.grasp import OPEN_FINGERS, SHARED_TRAVEL_M, practice_chain, shared_beam_grasped
        from openarm_simulation.grasp_telemetry import GraspTelemetry
        chains = {side: practice_chain(backend.world.model, side) for side in backend.world.arms}
        fingers = OPEN_FINGERS
        travel = SHARED_TRAVEL_M
        shared_contact = lambda: shared_beam_grasped(backend.world)
        telemetry = GraspTelemetry(backend.world)

    @app.get('/simulation/practice')
    async def page():
        html = static.joinpath('pico.html').read_text(encoding='utf-8')
        if telemetry is not None:
            html = html.replace('<body>', '<body class="practice">', 1)
        if official_sim_url:
            html = html.replace('<!-- home-entry -->', PICO_HOME_LINK, 1)
        return HTMLResponse(html, headers={'Cache-Control': 'no-store'})

    @app.get('/simulation/teleop')
    async def home():
        if not official_sim_url:
            return await page()
        html = static.joinpath('pico-home.html').read_text(encoding='utf-8')
        html = html.replace('href="/simulation/practice"', f'href="{escape(official_sim_url)}"', 1)
        return HTMLResponse(html, headers={'Cache-Control': 'no-store'})

    @app.get('/simulation/pico.webmanifest')
    async def manifest():
        return Response(static.joinpath('pico.webmanifest').read_bytes(),
                        media_type='application/manifest+json')

    @app.get('/simulation/pico-icon-{size}.png')
    async def icon(size: int):
        if size not in (192, 512):
            raise HTTPException(status_code=404)
        return Response(static.joinpath(f'pico-icon-{size}.png').read_bytes(), media_type='image/png')

    @app.get('/simulation/pico.js')
    async def script():
        return Response(static.joinpath('pico.js').read_text(encoding='utf-8'),
                        media_type='text/javascript')

    @app.get('/simulation/pico-telemetry.js')
    async def telemetry_script():
        return Response(static.joinpath('pico-telemetry.js').read_bytes(), media_type='text/javascript')

    @app.get('/simulation/pico/telemetry')
    def telemetry_snapshot():
        if telemetry is None:
            raise HTTPException(status_code=404)
        try:
            data = telemetry.snapshot()
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail='Simulation telemetry unavailable') from exc
        return JSONResponse(data, headers={'Cache-Control': 'no-store'})

    @app.websocket('/simulation/pico/input')
    async def input_stream(socket: WebSocket):
        if socket.url.scheme != 'wss' and socket.url.hostname not in ('localhost', '127.0.0.1'):
            await socket.close(code=1008)
            return
        if app.state.coordinator.closing or len(sessions.sessions) >= 4:
            await socket.close(code=1013)
            return
        world = backend.world
        scene = world.practice_scenes.current if world.practice_scenes else None
        rotation_limit = math.radians(105) if scene in ('cylinder', 'key') else MAX_ROTATION
        scene_travel = .36 if scene == 'cylinder' else travel
        session = PicoHands({
            side: PicoTeleop(SimulationBus(world, side), world.arms[side].lease,
                             lambda joints, side=side: cartesian_start(chains[side], joints, side),
                             axes=axes, open_fingers=fingers, travel=scene_travel,
                             rotation_limit=rotation_limit)
            for side in chains
        }, shared_contact=shared_contact)
        sessions.sessions.add(session)
        try:
            await socket.accept()
            did_reset = await stream_samples(
                socket, session, lambda: app.state.coordinator.closing,
                reset=world.reset_practice_arms,
                reset_scene=lambda: world.reset_practice_arms(reset_objects=True),
                select_scene=lambda scene: world.reset_practice_arms(scene=scene),
            )
            if did_reset:
                await socket.close(code=1000)
        except TimeoutError:
            logger.info('PICO control idle timeout')
            await socket.close(code=1000)
        except (WebSocketDisconnect, OSError):
            pass
        except (ValueError, RuntimeError, ImportError) as exc:
            logger.warning('PICO control rejected: %s', exc)
            await socket.close(code=1011, reason='Simulation input rejected')
        finally:
            session.release()
            sessions.sessions.discard(session)

    return sessions
