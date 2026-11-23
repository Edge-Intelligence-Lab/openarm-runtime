"""Optional HTTPS WebXR entry point for Classic arms."""

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
from .joints import JOINT_LIMITS

LOG = logging.getLogger(__name__)
INPUT_TIMEOUT = 0.25
FIRST_INPUT_TIMEOUT = 2.0
CAMERA_PERIOD = 1 / 12
SIDES = ("left", "right")
TRACKING_LEAD = 0.15
CONTROL_START = "start/2"
PREFERRED_POSTURE_DEG = {
    "left": (0, -12, 0, 40, 0, 0, 0),
    "right": (0, 12, 0, 40, 0, 0, 0),
}


class InputSession:
    """One operator-approved connection, with a latest-only input slot."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.arm_ready = threading.Event()
        self.stopped = False
        self.latest = None
        self.started_at = clock()
        self.last_input = None
        self.stop_reason = ""

    def input_expired(self, now):
        if self.last_input is None:
            return now - self.started_at >= FIRST_INPUT_TIMEOUT
        return now - self.last_input >= INPUT_TIMEOUT

    def submit(self, pose, grip, trigger=False, joystick_y=0.):
        with self.lock:
            if self.stopped or self.input_expired(self.clock()):
                return False
            self.latest = (pose, grip, trigger, joystick_y)
            self.last_input = self.clock()
            self.ready.set()
            return True

    def stop(self, reason=""):
        with self.lock:
            if not self.stopped:
                self.stop_reason = reason
            self.stopped = True
            self.ready.set()

    def cancelled(self):
        with self.lock:
            if not self.stopped and self.input_expired(self.clock()):
                self.stop_reason = self.timeout_reason()
                self.stopped = True
            return self.stopped

    def timeout_reason(self):
        if self.last_input is None:
            return "No right controller tracking"
        if not self.arm_ready.is_set():
            return "Controller input timed out during arm startup"
        return "Controller input timed out"

    def sample(self):
        with self.lock:
            return self.latest


def decode_pose_sample(value):
    from openarm_simulation.pico import decode_pose

    if not isinstance(value, dict) or value.keys() != {"pose", "grip", "trigger", "joystick_y"}:
        raise ValueError("Expected pose and grip")
    pose = decode_pose(value["pose"])
    if pose is None or len(pose) != 7 or type(value["grip"]) is not bool:
        raise ValueError("Expected a tracked 6DoF controller")
    trigger, joystick = value["trigger"], value["joystick_y"]
    if type(trigger) is not bool or type(joystick) not in (int, float) or not -1. <= joystick <= 1.:
        raise ValueError("Invalid gripper input")
    return pose, value["grip"], trigger, joystick


def decode_samples(message, sides):
    if len(message) > 1000:
        raise ValueError("Oversized PICO sample")
    value = json.loads(message)
    if not isinstance(value, dict) or value.keys() != set(sides):
        raise ValueError("Expected a sample for each configured arm")
    return {side: decode_pose_sample(sample) for side, sample in value.items() if sample is not None}


def _load_chain(path, options):
    from .cartesian import KinematicChain

    options["urdf"] = path.parent / options["urdf"]
    chain = KinematicChain(**options)
    if len(chain.indices) != 7:
        raise ValueError("An arm model must map J1 through J7")
    return chain


def load_chain(path):
    return _load_chain(path, json.loads(path.read_text()))


def load_arms(path, *, allow_unreadable_mode=False):
    data = json.loads(path.read_text())
    if "urdf" in data:
        data = {"right": data}
    if not data or set(data) - set(SIDES):
        raise ValueError("PICO IK profile must describe one or both arms")
    chains, runners = {}, {}
    for side, options in data.items():
        velocity = options.pop("velocity_rad_s", .25)
        if not .01 <= velocity <= .5:
            raise ValueError("Physical PICO joint velocity must be in [.01, .5] rad/s")
        chains[side] = _load_chain(path, options)
        open_arm = partial(classic_arm, side=side, allow_unreadable_mode=allow_unreadable_mode)
        runners[side] = partial(follow, side=side, velocity=velocity, open_arm=open_arm)
    return chains, runners


def record(event):
    if event["event"] not in {"gravity_sample", "hold_sample"}:
        LOG.info("%s", event)


@contextmanager
def classic_arm(profile, cancelled, chain, *, side, allow_unreadable_mode=False):
    from .arm import ArmHold
    from .control import prepare_arm

    def record_side(event):
        record({**event, "side": side})

    with prepare_arm(profile, side, cancelled=cancelled, record=record_side,
                     allow_unreadable_mode=allow_unreadable_mode) as (bus, gravity):
        if gravity is None or gravity.mode != "active":
            raise ValueError("The PICO Classic session requires an active gravity profile")
        positions = [bus.state(i).position for i in range(1, 9)]
        chain.configuration(positions[:7])
        for position, (lo, hi) in zip(positions, JOINT_LIMITS[side], strict=True):
            if not lo - .5 <= math.degrees(position) <= hi + .5:
                raise ValueError(f"{side}: initial position outside physical joint limits")
        with ArmHold(bus, cancelled, record_side, lambda states: None,
                     tracking=True, gravity=gravity) as arm:
            yield arm


classic_right = partial(classic_arm, side="right")


def tracking_target(previous, goal, measured, bounds, step):
    lo, hi = bounds
    lower = max(lo, measured - TRACKING_LEAD)
    upper = min(hi, measured + TRACKING_LEAD)
    bounded = min(upper, max(lower, goal))
    return min(hi, max(lo, previous + min(step, max(-step, bounded - previous))))


def preserves_motion(primary, trial, requested, tolerance):
    """Keep the secondary step close to the primary and check signed progress."""
    import numpy as np

    if np.linalg.norm(trial - primary) > tolerance:
        return False
    distance = np.linalg.norm(requested)
    if distance <= 1e-9:
        return True
    direction = requested / distance
    return trial @ direction >= min(0., primary @ direction) - 1e-7


def refine_posture(chain, reference, primary, target, preferred, lower, upper, period, weight):
    """Use the redundant joint direction without changing the primary TCP task."""
    import numpy as np
    import pinocchio as pin
    from scipy.spatial.transform import Rotation

    jacobian = pin.computeFrameJacobian(chain.model, chain.data, chain.configuration(primary),
                                       chain.model.getFrameId(chain.frame), pin.LOCAL)
    _, _, basis = np.linalg.svd(jacobian[:, chain.indices] * chain.signs)
    axis = basis[-1]
    speed = np.clip(3. * (axis @ (preferred - primary)), -.25, .25)
    change = axis * speed * period * weight
    # Limit the step along the null-space ray. Clipping joints individually
    # would rotate that ray back into a direction that moves the TCP.
    active = np.abs(change) > 1e-12
    if not active.any():
        return primary
    room = np.where(change > 0, upper - primary, lower - primary)
    scale = np.clip(np.min(room[active] / change[active]), 0., 1.)
    if scale == 0:
        return primary

    initial_pose = chain.forward(reference)
    primary_pose = chain.forward(primary)
    trial = primary + scale * change
    trial_pose = chain.forward(trial)
    primary_delta = primary_pose[:3, 3] - initial_pose[:3, 3]
    trial_delta = trial_pose[:3, 3] - initial_pose[:3, 3]
    requested = target[:3, 3] - initial_pose[:3, 3]
    if not preserves_motion(primary_delta, trial_delta, requested, .0001):
        return primary

    def rotation_delta(pose):
        return Rotation.from_matrix(pose[:3, :3] @ initial_pose[:3, :3].T).as_rotvec()

    if not preserves_motion(rotation_delta(primary_pose), rotation_delta(trial_pose),
                            rotation_delta(target), .0008):
        return primary
    return trial


def follow(session, profile, chain, *, open_arm=classic_right, side="right", velocity=.25):
    """Hold between clutches and rate-limit progress toward the latest pose."""
    import numpy as np
    from scipy.spatial.transform import Rotation
    from openarm_simulation.pico import xr_to_robot
    from .cartesian import DifferentialIK

    session.ready.wait(FIRST_INPUT_TIMEOUT)
    if session.cancelled() or session.sample() is None:
        return
    bounds = np.deg2rad(JOINT_LIMITS[side])
    model = chain.model
    positions = np.stack((model.lowerPositionLimit[chain.indices],
                          model.upperPositionLimit[chain.indices]), axis=1)
    mapped = np.sort((positions - chain.offsets[:, None]) * chain.signs[:, None], axis=1)
    bounds[:7, 0] = np.maximum(bounds[:7, 0], mapped[:, 0])
    bounds[:7, 1] = np.minimum(bounds[:7, 1], mapped[:, 1])
    if np.any(bounds[:, 0] >= bounds[:, 1]):
        raise ValueError(f"{side}: joint bounds do not intersect")
    period = 1 / profile.rate_hz
    if period > .05:
        raise ValueError("PICO Cartesian control requires at least 20 Hz")
    preferred = np.clip(np.deg2rad(PREFERRED_POSTURE_DEG[side]), bounds[:7, 0], bounds[:7, 1])
    steps = [velocity * period] * 7 + [.5 * period]

    with open_arm(profile, session.cancelled, chain) as arm:
        session.arm_ready.set()
        reference = np.array([arm.base[i] for i in range(1, 9)])
        aperture = reference[7]
        grip_min, grip_max = bounds[7]
        direction = 1 if side == "right" else -1
        closed = grip_max if side == "right" else grip_min
        origin = None
        while not session.cancelled():
            started = time.monotonic()
            pose, grip, trigger, joystick = session.sample()
            measured = np.array([arm.states[i].position for i in range(1, 9)])
            if (np.any(measured < bounds[:, 0] - math.radians(.5))
                    or np.any(measured > bounds[:, 1] + math.radians(.5))):
                raise ValueError(f"{side}: measured joint outside physical limits")
            goal = reference.copy()
            posture_weight = 0.
            if not grip:
                origin = None
            elif origin is None:
                origin = pose
                seed = np.clip(measured[:7], bounds[:7, 0], bounds[:7, 1])
                tcp_origin = chain.forward(seed)
                solver = DifferentialIK(chain, lower=bounds[:7, 0], upper=bounds[:7, 1],
                                        velocity=[velocity] * 7, posture=seed)
            else:
                current = Rotation.from_quat(pose[3:])
                delta = np.asarray(xr_to_robot(np.subtract(pose[:3], origin[:3])))
                rotation = np.asarray(xr_to_robot((current * Rotation.from_quat(origin[3:]).inv()).as_rotvec()))
                travel, turn = math.hypot(*delta), math.hypot(*rotation)
                posture_weight = min(1., max(0., (travel - .002) / .028,
                                             (turn - math.radians(1)) / math.radians(14)))
                target = tcp_origin.copy()
                target[:3, 3] += delta / max(1., travel / .15)
                target[:3, :3] = Rotation.from_rotvec(
                    rotation / max(1., turn / (math.pi / 3))
                ).as_matrix() @ tcp_origin[:3, :3]
                seed = np.clip(reference[:7], bounds[:7, 0], bounds[:7, 1])
                goal[:7] = solver.step(seed, target, period)["joints"]
            if grip:
                if abs(joystick) > .3:
                    aperture = min(grip_max, max(grip_min, aperture + direction * joystick * .5 * period))
                goal[7] = closed if trigger else aperture
            command = np.array([tracking_target(*joint) for joint in zip(
                reference, goal, measured, bounds, steps, strict=True)])
            if posture_weight:
                lower, upper = (np.array([tracking_target(q, extreme, actual, interval, step)
                                          for q, actual, interval, step in zip(
                                              reference[:7], measured[:7], bounds[:7], steps[:7], strict=True)])
                                for extreme in (-math.inf, math.inf))
                command[:7] = refine_posture(chain, seed, command[:7], target, preferred,
                                             lower, upper, period, posture_weight)
            reference = command
            if session.cancelled():
                break
            arm.track(dict(enumerate(reference.tolist(), 1)))
            delay = period - (time.monotonic() - started)
            if delay > 0:
                time.sleep(delay)


def calibrated_home_session(profile, *, allow_unreadable_mode):
    from openarm_policy.calibrated_home import CalibratedZeroReturn
    from openarm_policy.robots import ClassicRobot
    from openarm_policy.session import PolicySession, SessionConfig

    for side in SIDES:
        profile.make_gravity(side, required=True)
    config = SessionConfig(control_hz=profile.rate_hz)
    robot = ClassicRobot(profile, SIDES, watchdog_s=config.command_timeout_s,
                         record=record, allow_unreadable_mode=allow_unreadable_mode,
                         cancelled=lambda: session.cancel.is_set())
    session = PolicySession(robot, None, config=config, return_home=CalibratedZeroReturn())
    return session


async def return_home(socket, session):
    await socket.send_json({"home": "running"})
    worker = asyncio.create_task(asyncio.to_thread(session.run, home_only=True))
    incoming = asyncio.create_task(socket.receive())
    try:
        await asyncio.wait({worker, incoming}, return_when=asyncio.FIRST_COMPLETED)
    finally:
        if not worker.done():
            session.stop()
        incoming.cancel()
        with suppress(asyncio.CancelledError):
            await incoming
        try:
            await asyncio.shield(worker)
        except InterruptedError:
            if not session.cancel.is_set():
                raise
    if not socket.closed:
        await socket.send_json({"home": "stopped" if session.cancel.is_set() else "complete"})


def application(profile=None, chain=None, *, token=None, camera=None, cameras=None,
                runner=follow, passwordless=False, allow_unreadable_mode=False):
    from aiohttp import web

    # A hardware session is an exclusive websocket lifetime, including cleanup.
    owner = asyncio.Lock()
    token = token or secrets.token_urlsafe(24)
    cameras = cameras if cameras is not None else ({"head": camera} if camera is not None else {})
    chains = chain if isinstance(chain, dict) else {"right": chain}
    sides = tuple(side for side in SIDES if side in chains)
    home_available = profile is not None and sides == SIDES
    runners = runner if isinstance(runner, dict) else dict.fromkeys(sides, runner)
    active_sessions = {}
    active_socket = None

    def access_response():
        response = web.json_response({"camera": "head" in cameras, "cameras": list(cameras), "sides": sides})
        response.set_cookie("pico_video", token, secure=True, httponly=True, samesite="Strict")
        return response

    async def index(_request):
        page = files("openarm_runtime").joinpath("static/pico-classic.html")
        text = page.read_text(encoding="utf-8")
        if passwordless:
            text = text.replace("<body>", "<body data-passwordless>", 1)
        if home_available:
            text = text.replace('<button id="home" hidden>', '<button id="home">', 1)
        return web.Response(text=text, content_type="text/html", headers={"Cache-Control": "no-store"})

    async def windows(_request):
        script = files("openarm_runtime").joinpath("static/pico-windows.js")
        return web.Response(text=script.read_text(encoding="utf-8"), content_type="text/javascript",
                            headers={"Cache-Control": "no-store"})

    async def video(request):
        name = request.match_info.get("name", "head")
        device_path = cameras.get(name)
        if device_path is None or request.cookies.get("pico_video") != token:
            raise web.HTTPNotFound()
        import cv2

        def next_frame(device):
            ok, image = device.read()
            if not ok:
                return None
            height, width = image.shape[:2]
            max_width = 640 if name == "head" else 480
            if width > max_width:
                image = cv2.resize(image, (max_width, round(height * max_width / width)), interpolation=cv2.INTER_AREA)
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 75])
            return encoded.tobytes() if ok else None

        device = await asyncio.to_thread(cv2.VideoCapture, device_path, cv2.CAP_V4L2)
        response = web.StreamResponse(headers={"Content-Type": "multipart/x-mixed-replace; boundary=frame",
                                               "Cache-Control": "no-store"})
        try:
            await response.prepare(request)
            device.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            while True:
                started = time.monotonic()
                frame = await asyncio.to_thread(next_frame, device)
                if frame is None:
                    break
                await response.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
                period = CAMERA_PERIOD if name == "head" else 1 / 8
                await asyncio.sleep(max(0., period - (time.monotonic() - started)))
        except ConnectionError:
            pass
        finally:
            device.release()
        return response

    async def access(request):
        if passwordless:
            if request.headers.get("Origin") != f"https://{request.host}":
                raise web.HTTPForbidden()
            return access_response()
        data = await request.json()
        if not isinstance(data, dict) or not isinstance(data.get("token"), str):
            raise web.HTTPForbidden()
        if not compare_digest(data["token"].encode(), token.encode()):
            raise web.HTTPForbidden()
        return access_response()

    async def control(request):
        nonlocal active_sessions, active_socket
        if (request.headers.get("Origin") != f"https://{request.host}"
                or request.cookies.get("pico_video") != token):
            raise web.HTTPForbidden()
        if owner.locked():
            raise web.HTTPConflict(text="An operator session is active")
        async with owner:
            socket = web.WebSocketResponse(max_msg_size=1024, heartbeat=2.)
            await socket.prepare(request)
            sessions, tasks = {}, {}
            reason = ""
            received = False
            try:
                first = await socket.receive(timeout=2.)
                if first.type != web.WSMsgType.TEXT:
                    return socket
                if first.data == "start":
                    await socket.send_json({"error": "请刷新 Pico 页面，再开始遥操。"})
                    return socket
                if first.data not in (CONTROL_START, "home"):
                    return socket
                if first.data == "home":
                    if not home_available:
                        await socket.send_json({"error": "Calibrated zero requires both arms"})
                        return socket
                    active_socket = socket
                    try:
                        session = calibrated_home_session(profile, allow_unreadable_mode=allow_unreadable_mode)
                        await return_home(socket, session)
                    except Exception as exc:
                        LOG.exception("PICO calibrated return failed")
                        if not socket.closed:
                            await socket.send_json({"error": str(exc)})
                    return socket
                if profile is None:
                    await socket.send_json({"error": "Offline smoke mode"})
                    return socket
                sessions = active_sessions = {side: InputSession() for side in sides}
                active_socket = socket
                tasks = {
                    asyncio.create_task(asyncio.to_thread(runners[side], session, profile, chains[side])): side
                    for side, session in sessions.items()
                }
                running = set(tasks)
                while running:
                    timeout = INPUT_TIMEOUT if received else FIRST_INPUT_TIMEOUT
                    incoming = asyncio.create_task(socket.receive(timeout=timeout))
                    try:
                        done, _ = await asyncio.wait(running | {incoming}, return_when=asyncio.FIRST_COMPLETED)
                        for task in done & running:
                            running.remove(task)
                            try:
                                await task
                            except InterruptedError:
                                if not sessions[tasks[task]].cancelled():
                                    raise
                            reason = sessions[tasks[task]].stop_reason or "Control session ended"
                        if not running:
                            break
                        if incoming not in done:
                            continue
                        message = incoming.result()
                    finally:
                        incoming.cancel()
                        with suppress(asyncio.CancelledError, TimeoutError):
                            await incoming
                    if message.type != web.WSMsgType.TEXT or message.data == "stop":
                        reason = ""
                        break
                    samples = decode_samples(message.data, sides)
                    received = True
                    for side, sample in samples.items():
                        sessions[side].submit(*sample)
                    await socket.send_json({"arms": {
                        side: session.stop_reason or "stopped" if session.stopped else
                        "ready" if session.arm_ready.is_set() else
                        "starting" if session.ready.is_set() else "waiting"
                        for side, session in sessions.items()
                    }})
            except TimeoutError:
                if sessions:
                    reason = next((session.timeout_reason() for session in sessions.values()
                                   if session.ready.is_set()), "No controller tracking")
            except (ConnectionError, InterruptedError):
                pass
            except Exception as exc:
                reason = "Control error"
                LOG.warning("PICO session stopped: %s", exc)
                if not socket.closed:
                    await socket.send_json({"error": str(exc)})
            finally:
                for session in sessions.values():
                    session.stop(reason)
                for task in tasks:
                    try:
                        await task
                    except InterruptedError:
                        pass
                    except Exception:
                        LOG.exception("Classic session cleanup or tracking failed")
                if reason:
                    LOG.info("PICO session ended: %s", reason)
                active_sessions, active_socket = {}, None
                await socket.close(message=reason.encode())
            return socket

    async def shutdown(_app):
        for session in active_sessions.values():
            session.stop()
        if active_socket is not None:
            await active_socket.close()

    app = web.Application(client_max_size=4096)
    app.on_shutdown.append(shutdown)
    app.add_routes([web.get("/", index), web.get("/pico-windows.js", windows), web.post("/access", access),
                    web.get("/input", control), web.get("/video", video),
                    web.get("/video/{name}", video)])
    return app


def configure(parser):
    parser.add_argument("--ik-profile", type=Path)
    parser.add_argument("--cert", type=Path)
    parser.add_argument("--key", type=Path)
    access = parser.add_mutually_exclusive_group()
    access.add_argument("--token-file", type=Path)
    access.add_argument("--passwordless", action="store_true",
                        help="Run without a connection password on an isolated lab network")
    parser.add_argument("--allow-unreadable-mode", action="store_true",
                        help="Explicit MIT trial when RID10 readback times out")
    parser.add_argument("--camera", help="Linux V4L2 device for the head camera")
    parser.add_argument("--left-wrist-camera", help="Linux V4L2 device for the left wrist camera")
    parser.add_argument("--right-wrist-camera", help="Linux V4L2 device for the right wrist camera")
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
            load_arms(args.ik_profile)
        print("PICO Classic integration import and routes OK; no devices opened")
        return
    if not all((args.config, args.ik_profile, args.cert, args.key)):
        raise ValueError("Set --config, --ik-profile, --cert, and --key")
    if not (args.token_file or args.passwordless):
        raise ValueError("Choose --token-file or --passwordless")
    if sys.platform != "linux":
        raise ValueError("Classic CAN teleoperation requires Linux; use --smoke for offline validation")
    profile = Profile.load(args.config)
    if not profile.mapping_verified:
        raise ValueError("Verify the Classic motor-to-joint mapping first")
    arms, runners = load_arms(args.ik_profile, allow_unreadable_mode=args.allow_unreadable_mode)
    for side in arms:
        profile.make_gravity(side, required=True)
    token = None
    if args.token_file:
        token = args.token_file.read_text().strip()
        if len(token) < 24 or not token.isascii() or not all(c.isalnum() or c in "-_" for c in token):
            raise ValueError("Use a random URL-safe token of at least 24 characters")
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(args.cert, args.key)
    logging.basicConfig(level=logging.INFO)
    cameras = {name: path for name, path in (
        ("head", args.camera), ("left", args.left_wrist_camera), ("right", args.right_wrist_camera)
    ) if path}
    web.run_app(application(profile, arms, token=token, cameras=cameras, runner=runners,
                            passwordless=args.passwordless, allow_unreadable_mode=args.allow_unreadable_mode),
                host=args.host, port=args.port, ssl_context=context)
