"""Offline PICO session protocol tests; never open CAN or camera devices."""

import asyncio
from contextlib import contextmanager
import json
import time
import unittest
from unittest.mock import Mock, patch

try:
    from aiohttp import WSCloseCode, WSMsgType, WSServerHandshakeError
    from aiohttp.test_utils import TestClient, TestServer
except ImportError:
    TestClient = None

from openarm_runtime.pico_classic import CONTROL_START, RECONNECT_CODE, InputSession, application, classic_right, tracking_target
from openarm_simulation.pico import xr_to_robot

TOKEN = "offline-test-token-at-least-24"
POSE = json.dumps({"pose": [0, 0, 0, 0, 0, 0, 1], "grip": False, "trigger": False, "joystick_y": 0})


class ClassicAdapterTests(unittest.TestCase):
    def test_runtime_preflight_gravity_and_verified_cleanup(self):
        profile, chain = Mock(), Mock()
        bus, gravity = Mock(), Mock(mode="active")
        bus.state.side_effect = lambda mid: Mock(position=mid / 10 if mid < 8 else -.25)
        cleanup = []

        @contextmanager
        def prepared(*_args, **_kwargs):
            try:
                yield bus, gravity
            finally:
                cleanup.append("bus")

        @contextmanager
        def hold(*_args, **_kwargs):
            try:
                yield "right arm"
            finally:
                cleanup.append("disable")

        with patch("openarm_runtime.control.prepare_arm", side_effect=prepared) as prepare:
            with patch("openarm_runtime.arm.ArmHold", side_effect=hold) as arm:
                with classic_right(profile, lambda: False, chain) as session:
                    self.assertEqual(session, "right arm")
                self.assertEqual(prepare.call_args.args, (profile, "right"))
                chain.configuration.assert_called_once_with([i / 10 for i in range(1, 8)])
                self.assertIs(arm.call_args.kwargs["gravity"], gravity)
                self.assertTrue(arm.call_args.kwargs["tracking"])
        self.assertEqual(cleanup, ["disable", "bus"])


class InputTests(unittest.TestCase):
    def test_xr_forward_right_and_up_match_the_robot_base(self):
        self.assertEqual(xr_to_robot((0, 0, -1)), (1, 0, 0))
        self.assertEqual(xr_to_robot((1, 0, 0)), (0, -1, 0))
        self.assertEqual(xr_to_robot((0, 1, 0)), (0, 0, 1))

    def test_quick_target_saturates_while_motor_lags_and_resumes_when_it_catches_up(self):
        bounds = (-1., 1.)
        ref = 0.
        for _ in range(100):
            next_ref = tracking_target(ref, 1., 0., bounds, .01)
            self.assertLessEqual(next_ref - ref, .01000001)
            ref = next_ref
        self.assertAlmostEqual(ref, .15)
        self.assertAlmostEqual(tracking_target(ref, 1., .05, bounds, .01), .16)
        self.assertAlmostEqual(tracking_target(ref, -1., 0., bounds, .01), .14)

    def test_expired_input_cannot_restart(self):
        now = [0.]
        session = InputSession(lambda: now[0])
        self.assertTrue(session.submit((0,) * 7, False))
        now[0] = .30
        self.assertFalse(session.submit((0,) * 7, True))
        self.assertTrue(session.cancelled())
        self.assertEqual(session.stop_reason, "Controller input timed out during arm startup")

    def test_first_pose_has_a_startup_window_before_the_tracking_watchdog(self):
        now = [0.]
        session = InputSession(lambda: now[0])
        now[0] = .5
        self.assertFalse(session.cancelled())
        self.assertTrue(session.submit((0,) * 7, False))
        session.arm_ready.set()
        now[0] = .8
        self.assertTrue(session.cancelled())
        self.assertEqual(session.stop_reason, "Controller input timed out")


@unittest.skipUnless(TestClient is not None, "Install pico-classic for HTTP tests")
class WebTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.events = []

        def worker(session, _profile, _chain):
            if not session.ready.wait(2.) or session.cancelled():
                return
            self.events.append("enabled")
            session.arm_ready.set()
            while not session.cancelled():
                time.sleep(.005)
            self.events.append("disabled")

        self.client = TestClient(TestServer(application(object(), object(), token=TOKEN, runner=worker)))
        await self.client.start_server()

    async def asyncTearDown(self):
        await self.client.close()

    async def connect(self):
        host = self.client.make_url("/").raw_authority
        return await self.client.ws_connect("/input", headers={"Origin": f"https://{host}",
                                                               "Cookie": f"pico_video={TOKEN}"})

    async def start(self, first_pose_delay=0.):
        socket = await self.connect()
        await socket.send_str(CONTROL_START)
        await asyncio.sleep(first_pose_delay)
        await socket.send_json({"right": json.loads(POSE)})
        self.assertIn((await socket.receive_json(timeout=1))["arms"]["right"], ("starting", "ready"))
        for _ in range(50):
            if self.events[-1:] == ["enabled"]:
                break
            await asyncio.sleep(.01)
        self.assertEqual(self.events[-1:], ["enabled"])
        await socket.send_json({"right": json.loads(POSE)})
        self.assertEqual(await socket.receive_json(timeout=1), {"arms": {"right": "ready"}})
        return socket

    async def test_start_exit_loss_and_explicit_restart(self):
        response = await self.client.get("/")
        self.assertEqual(response.status, 200)
        self.assertIn('id="start"', await response.text())
        self.assertEqual((await self.client.post("/access", json={"token": "wrong"})).status, 403)
        self.assertEqual((await self.client.post("/access", json={"token": TOKEN})).status, 200)
        idle = await self.connect()
        await idle.send_str(POSE)
        self.assertEqual((await idle.receive(timeout=1)).type, WSMsgType.CLOSE)
        self.assertEqual(self.events, [])

        first = await self.start()
        await first.send_str("stop")
        self.assertEqual((await first.receive(timeout=1)).type, WSMsgType.CLOSE)
        self.assertEqual(self.events, ["enabled", "disabled"])

        second = await self.start(first_pose_delay=.35)
        closed = await second.receive(timeout=1)
        self.assertEqual(closed.type, WSMsgType.CLOSE)
        self.assertEqual(closed.data, RECONNECT_CODE)
        self.assertEqual(closed.extra, "Controller input timed out")
        self.assertEqual(self.events, ["enabled", "disabled"] * 2)

        third = await self.start()
        await third.close(code=WSCloseCode.GOING_AWAY)
        for _ in range(50):
            if self.events[-1:] == ["disabled"]:
                break
            await asyncio.sleep(.01)
        self.assertEqual(self.events, ["enabled", "disabled"] * 3)

    async def test_home_requires_both_arms(self):
        self.assertIn('id="home" hidden', await (await self.client.get("/")).text())
        async with await self.connect() as socket:
            await socket.send_str("home")
            self.assertIn("both arms", (await socket.receive_json(timeout=1))["error"])
        self.assertEqual(self.events, [])

    async def test_home_holds_motion_owner_until_cleanup_on_stop_or_disconnect(self):
        import threading

        events = []
        arms = {"left": "left", "right": "right"}

        class Home:
            def __init__(self):
                self.cancel = threading.Event()

            def run(self, *, home_only):
                events.append(("started", home_only))
                self.cancel.wait(2)
                events.append("cleaned up")

            def stop(self):
                self.cancel.set()

        with patch("openarm_runtime.pico_classic.calibrated_home_session", side_effect=lambda *a, **k: Home()):
            async with TestClient(TestServer(application(object(), arms, token=TOKEN))) as client:
                host = client.make_url("/").raw_authority
                headers = {"Origin": f"https://{host}", "Cookie": f"pico_video={TOKEN}"}
                self.assertIn('<button id="home">', await (await client.get("/")).text())
                socket = await client.ws_connect("/input", headers=headers)
                await socket.send_str("home")
                self.assertEqual(await socket.receive_json(timeout=1), {"home": "running"})
                with self.assertRaises(WSServerHandshakeError) as busy:
                    await client.ws_connect("/input", headers=headers)
                self.assertEqual(busy.exception.status, 409)
                await socket.send_str("stop")
                self.assertEqual(await socket.receive_json(timeout=1), {"home": "stopped"})
                self.assertEqual(events, [("started", True), "cleaned up"])
                await socket.receive(timeout=1)
                socket = await client.ws_connect("/input", headers=headers)
                await socket.send_str("home")
                await socket.receive_json(timeout=1)
                await socket.close()
                for _ in range(50):
                    if len(events) == 4:
                        break
                    await asyncio.sleep(.01)
                self.assertEqual(events, [("started", True), "cleaned up"] * 2)

    async def test_home_reports_completion_after_worker_returns(self):
        home = Mock()
        home.cancel.is_set.return_value = False
        arms = {"left": "left", "right": "right"}
        with patch("openarm_runtime.pico_classic.calibrated_home_session", return_value=home):
            async with TestClient(TestServer(application(object(), arms, token=TOKEN))) as client:
                host = client.make_url("/").raw_authority
                headers = {"Origin": f"https://{host}", "Cookie": f"pico_video={TOKEN}"}
                async with client.ws_connect("/input", headers=headers) as socket:
                    await socket.send_str("home")
                    self.assertEqual(await socket.receive_json(timeout=1), {"home": "running"})
                    self.assertEqual(await socket.receive_json(timeout=1), {"home": "complete"})
        home.run.assert_called_once_with(home_only=True)
        home.stop.assert_not_called()

    async def test_bimanual_input_loss_leaves_peer_arm_running(self):
        outputs = {}
        starts = {"left": 0, "right": 0}

        def runner(session, _profile, side):
            if not session.ready.wait(2.) or session.cancelled():
                return
            starts[side] += 1
            session.arm_ready.set()
            while not session.cancelled():
                outputs[side] = session.sample()
                time.sleep(.005)

        arms = {"left": "left", "right": "right"}
        app = application(object(), arms, token=TOKEN, runner=runner, passwordless=True,
                          cameras={"left": "/dev/left", "head": "/dev/head", "right": "/dev/right"})
        async with TestClient(TestServer(app)) as client:
            host = client.make_url("/").raw_authority
            headers = {"Origin": f"https://{host}"}
            access = await client.post("/access", headers=headers)
            info = await access.json()
            revision = info.pop("revision")
            self.assertEqual(info, {
                "camera": True, "cameras": ["left", "head", "right"], "sides": ["left", "right"]})
            page = await (await client.get("/")).text()
            self.assertIn("data-passwordless", page)
            self.assertIn(f'<meta name="pico-revision" content="{revision}">', page)
            self.assertEqual((await client.get("/pico-windows.js")).status, 200)
            headers["Cookie"] = f"pico_video={access.cookies['pico_video'].value}"
            async with client.ws_connect("/input", headers=headers) as socket:
                await socket.send_str(CONTROL_START)
                left, right = json.loads(POSE), json.loads(POSE)
                left["pose"][0] = -1.
                right["pose"][0] = 1.
                await socket.send_json({"left": left, "right": right})
                await socket.receive_json(timeout=1)
                for _ in range(35):
                    await asyncio.sleep(.012)
                    await socket.send_json({"left": None, "right": right})
                    status = (await socket.receive_json(timeout=1))["arms"]
                    if status["left"] == "Controller input timed out":
                        break
                self.assertEqual(status, {"left": "Controller input timed out", "right": "ready"})
                self.assertEqual(outputs["left"][0][0], -1.)
                self.assertEqual(outputs["right"][0][0], 1.)
                left["grip"] = True
                for _ in range(3):
                    await socket.send_json({"left": left, "right": right})
                    status = (await socket.receive_json(timeout=1))["arms"]
                    self.assertEqual(status["left"], "Controller input timed out")
                self.assertEqual(starts, {"left": 1, "right": 1})
                left["grip"] = False
                for _ in range(20):
                    await socket.send_json({"left": left, "right": right})
                    status = (await socket.receive_json(timeout=1))["arms"]
                    if status["left"] == "ready":
                        break
                    await asyncio.sleep(.01)
                self.assertEqual(status, {"left": "ready", "right": "ready"})
                self.assertEqual(starts, {"left": 2, "right": 1})
                await socket.send_str("stop")
                self.assertEqual((await socket.receive(timeout=1)).type, WSMsgType.CLOSE)

    async def test_stale_page_must_reload_before_starting_motion(self):
        old = await self.client.get("/?ui=old", allow_redirects=False)
        self.assertEqual(old.status, 302)
        self.assertEqual(old.headers["Location"], "/")
        response = await self.client.get("/")
        self.assertEqual(response.headers["Cache-Control"], "no-store")
        page = await response.text()
        self.assertIn(f'socket.send("{CONTROL_START}")', page)
        for old_start in ("start", "start/2", "start/3"):
            async with await self.connect() as socket:
                await socket.send_str(old_start)
                self.assertIn("刷新", (await socket.receive_json(timeout=1))["error"])
                self.assertEqual((await socket.receive(timeout=1)).type, WSMsgType.CLOSE)
        self.assertEqual(self.events, [])

    async def test_arm_timeout_exception_is_not_a_retryable_input_timeout(self):
        def runner(session, _profile, _chain):
            if session.ready.wait(1.):
                raise TimeoutError("CAN feedback timeout")

        async with TestClient(TestServer(application(object(), object(), token=TOKEN, runner=runner))) as client:
            host = client.make_url("/").raw_authority
            headers = {"Origin": f"https://{host}", "Cookie": f"pico_video={TOKEN}"}
            async with client.ws_connect("/input", headers=headers) as socket:
                await socket.send_str(CONTROL_START)
                await socket.send_json({"right": json.loads(POSE)})
                reply = await socket.receive_json(timeout=1)
                if "arms" in reply:
                    reply = await socket.receive_json(timeout=1)
                self.assertEqual(reply["error"], "CAN feedback timeout")
                self.assertEqual((await socket.receive(timeout=1)).data, 1011)
