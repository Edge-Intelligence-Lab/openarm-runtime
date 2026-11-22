"""Optional HTTPS WebXR entry point for the Classic right arm."""

import asyncio
from contextlib import contextmanager, suppress
from functools import partial
from hmac import compare_digest
from importlib.resources import files
import json
import logging
import math
from pathlib import Path
import secrets
import sys
import threading
import time

from .config import Profile

LOG = logging.getLogger(__name__)
INPUT_TIMEOUT = 0.25


class InputSession:
    """One operator-approved connection, with a latest-only input slot."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.stopped = False
        self.latest = None
        self.last_input = clock()

    def submit(self, pose, grip, trigger=False, joystick_y=0.):
        with self.lock:
            if self.stopped or self.clock() - self.last_input >= INPUT_TIMEOUT:
                self.stopped = True
                return False
            self.latest = (pose, grip, trigger, joystick_y)
            self.last_input = self.clock()
            self.ready.set()
            return True

    def stop(self):
        with self.lock:
            self.stopped = True
            self.ready.set()

    def cancelled(self):
        with self.lock:
            if self.clock() - self.last_input >= INPUT_TIMEOUT:
                self.stopped = True
            return self.stopped

    def sample(self):
        with self.lock:
            return self.latest


def decode_sample(message):
    from openarm_simulation.pico import decode_pose

    if len(message) > 400:
        raise ValueError("Oversized PICO sample")
    value = json.loads(message)
    if not isinstance(value, dict) or value.keys() != {"pose", "grip", "trigger", "joystick_y"}:
        raise ValueError("Expected pose and grip")
    pose = decode_pose(value["pose"])
    if pose is None or len(pose) != 7 or type(value["grip"]) is not bool:
        raise ValueError("Expected a tracked 6DoF controller")
    trigger, joystick = value["trigger"], value["joystick_y"]
    if type(trigger) is not bool or type(joystick) not in (int, float) or not -1. <= joystick <= 1.:
        raise ValueError("Invalid gripper input")
    return pose, value["grip"], trigger, joystick


def load_chain(path):
    from .cartesian import KinematicChain

    options = json.loads(path.read_text())
    options["urdf"] = path.parent / options["urdf"]
    chain = KinematicChain(**options)
    if len(chain.indices) != 7:
        raise ValueError("The right-arm model must map J1 through J7")
    return chain


def record(event):
    if event["event"] not in {"gravity_sample", "hold_sample"}:
        LOG.info("%s", event)


@contextmanager
def classic_right(profile, cancelled, chain, *, allow_unreadable_mode=False):
    from .arm import ArmHold
    from .control import prepare_arm

    with prepare_arm(profile, "right", cancelled=cancelled, record=record,
                     allow_unreadable_mode=allow_unreadable_mode) as (bus, gravity):
        if gravity is None or gravity.mode != "active":
            raise ValueError("The PICO Classic session requires an active gravity profile")
        chain.configuration([bus.state(i).position for i in range(1, 8)])
        with ArmHold(bus, cancelled, record, lambda states: None,
                     tracking=True, gravity=gravity) as arm:
            yield arm


def follow(session, profile, chain, *, open_arm=classic_right):
    """Hold between clutches and solve a bounded Cartesian step from each pose."""
    from scipy.spatial.transform import Rotation
    from openarm_simulation.pico import BASE_AXES, cartesian_start
    from .joints import JOINT_LIMITS

    if not session.ready.wait(INPUT_TIMEOUT) or session.cancelled():
        return
    with open_arm(profile, session.cancelled, chain) as arm:
        reference = dict(arm.base)
        aperture = reference[8]
        grip_min, grip_max = map(math.radians, JOINT_LIMITS["right"][7])
        origin = previous = solver = None
        period = 1 / profile.rate_hz
        while not session.cancelled():
            started = time.monotonic()
            pose, grip, trigger, joystick = session.sample()
            angles = [arm.states[i].position for i in range(1, 8)]
            if not grip:
                origin = previous = None
            elif origin is None:
                origin = previous = pose
                solver = cartesian_start(chain, angles, "right")
            else:
                current = Rotation.from_quat(pose[3:])
                if (math.dist(pose[:3], previous[:3]) > .12
                        or (current * Rotation.from_quat(previous[3:]).inv()).magnitude() > math.pi / 3):
                    raise ValueError("Controller tracking jumped; start a new session")
                previous = pose

                def to_base(vector):
                    return [sum(a * b for a, b in zip(row, vector, strict=True))
                            for row in zip(*BASE_AXES, strict=True)]

                delta = to_base([p - o for p, o in zip(pose[:3], origin[:3], strict=True)])
                delta = [v / max(1., math.hypot(*delta) / .15) for v in delta]
                rotation = to_base((current * Rotation.from_quat(origin[3:]).inv()).as_rotvec())
                rotation = [v / max(1., math.hypot(*rotation) / (math.pi / 3)) for v in rotation]
                joints = solver([reference[i] for i in range(1, 8)], delta, rotation, period)
                reference.update(enumerate(joints, 1))
            if grip:
                if abs(joystick) > .3:
                    aperture = min(grip_max, max(grip_min, aperture + joystick * period))
                desired = grip_max if trigger else aperture
                reference[8] += min(period, max(-period, desired - reference[8]))
            if max(abs(reference[i] - arm.states[i].position) for i in reference) > .15:
                raise RuntimeError("Classic following error exceeded 0.15 rad")
            if session.cancelled():
                break
            arm.track(reference)
            delay = period - (time.monotonic() - started)
            if delay > 0:
                time.sleep(delay)


def application(profile=None, chain=None, *, token=None, camera=None, runner=follow):
    from aiohttp import web

    # A hardware session is an exclusive websocket lifetime, including cleanup.
    owner = asyncio.Lock()
    token = token or secrets.token_urlsafe(24)
    active_session = None
    active_socket = None

    async def index(_request):
        page = files("openarm_runtime").joinpath("static/pico-classic.html")
        return web.Response(text=page.read_text(encoding="utf-8"), content_type="text/html")

    async def video(request):
        if camera is None or request.cookies.get("pico_video") != token:
            raise web.HTTPNotFound()
        import cv2

        def next_frame(device):
            ok, image = device.read()
            if not ok:
                return None
            ok, encoded = cv2.imencode(".jpg", image)
            return encoded.tobytes() if ok else None

        device = await asyncio.to_thread(cv2.VideoCapture, camera, cv2.CAP_V4L2)
        response = web.StreamResponse(headers={"Content-Type": "multipart/x-mixed-replace; boundary=frame",
                                               "Cache-Control": "no-store"})
        try:
            await response.prepare(request)
            while frame := await asyncio.to_thread(next_frame, device):
                await response.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        except ConnectionResetError:
            pass
        finally:
            device.release()
        return response

    async def access(request):
        data = await request.json()
        if not isinstance(data, dict) or not isinstance(data.get("token"), str):
            raise web.HTTPForbidden()
        if not compare_digest(data["token"].encode(), token.encode()):
            raise web.HTTPForbidden()
        response = web.json_response({"camera": camera is not None})
        response.set_cookie("pico_video", token, secure=True, httponly=True, samesite="Strict")
        return response

    async def control(request):
        nonlocal active_session, active_socket
        if (request.headers.get("Origin") != f"https://{request.host}"
                or request.cookies.get("pico_video") != token):
            raise web.HTTPForbidden()
        if owner.locked():
            raise web.HTTPConflict(text="An operator session is active")
        async with owner:
            socket = web.WebSocketResponse(max_msg_size=512)
            await socket.prepare(request)
            session = None
            task = None
            try:
                first = await socket.receive(timeout=2.)
                if first.type != web.WSMsgType.TEXT or first.data != "start":
                    return socket
                if profile is None:
                    await socket.send_json({"error": "Offline smoke mode"})
                    return socket
                session = active_session = InputSession()
                active_socket = socket
                task = asyncio.create_task(asyncio.to_thread(runner, session, profile, chain))
                while True:
                    incoming = asyncio.create_task(socket.receive(timeout=INPUT_TIMEOUT))
                    try:
                        done, _ = await asyncio.wait((incoming, task), return_when=asyncio.FIRST_COMPLETED)
                        if task in done:
                            await task
                            break
                        message = incoming.result()
                    finally:
                        incoming.cancel()
                        with suppress(asyncio.CancelledError, TimeoutError):
                            await incoming
                    if message.type != web.WSMsgType.TEXT or message.data == "stop":
                        break
                    if not session.submit(*decode_sample(message.data)):
                        break
                    await socket.send_str("ok")
            except (TimeoutError, ConnectionError):
                pass
            except Exception as exc:
                LOG.warning("PICO session stopped: %s", exc)
                if not socket.closed:
                    await socket.send_json({"error": str(exc)})
            finally:
                if session is not None:
                    session.stop()
                if task is not None:
                    try:
                        await task
                    except Exception:
                        LOG.exception("Classic session cleanup or tracking failed")
                active_session = active_socket = None
                await socket.close()
            return socket

    async def shutdown(_app):
        if active_session is not None:
            active_session.stop()
        if active_socket is not None:
            await active_socket.close()

    app = web.Application(client_max_size=4096)
    app.on_shutdown.append(shutdown)
    app.add_routes([web.get("/", index), web.post("/access", access),
                    web.get("/input", control), web.get("/video", video)])
    return app


def configure(parser):
    parser.add_argument("--ik-profile", type=Path)
    parser.add_argument("--cert", type=Path)
    parser.add_argument("--key", type=Path)
    parser.add_argument("--token-file", type=Path)
    parser.add_argument("--allow-unreadable-mode", action="store_true",
                        help="Explicit MIT trial when RID10 readback times out")
    parser.add_argument("--camera", help="Linux V4L2 device for the head camera")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8443)
    parser.add_argument("--smoke", action="store_true", help="Check installation without opening devices")


def launch(args):
    import ssl
    from aiohttp import web

    if args.smoke:
        app = application()
        assert any(route.resource.canonical == "/input" for route in app.router.routes())
        assert files("openarm_runtime").joinpath("static/pico-classic.html").is_file()
        if sys.platform == "linux":
            import cv2, daqp, pink, pinocchio  # noqa: F401
            from scipy.spatial.transform import Rotation  # noqa: F401
        if args.ik_profile is not None:
            load_chain(args.ik_profile)
        print("PICO Classic integration import and routes OK; no devices opened")
        return
    if not all((args.config, args.ik_profile, args.cert, args.key, args.token_file)):
        raise ValueError("Set --config, --ik-profile, --cert, --key, and --token-file")
    if sys.platform != "linux":
        raise ValueError("Classic CAN teleoperation requires Linux; use --smoke for offline validation")
    profile = Profile.load(args.config)
    if not profile.mapping_verified:
        raise ValueError("Verify the Classic motor-to-joint mapping first")
    profile.make_gravity("right", required=True)
    chain = load_chain(args.ik_profile)
    token = args.token_file.read_text().strip()
    if len(token) < 24 or not token.isascii() or not all(c.isalnum() or c in "-_" for c in token):
        raise ValueError("Use a random URL-safe token of at least 24 characters")
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(args.cert, args.key)
    logging.basicConfig(level=logging.INFO)
    open_arm = partial(classic_right, allow_unreadable_mode=args.allow_unreadable_mode)
    web.run_app(application(profile, chain, token=token, camera=args.camera,
                            runner=partial(follow, open_arm=open_arm)),
                host=args.host, port=args.port, ssl_context=context)
