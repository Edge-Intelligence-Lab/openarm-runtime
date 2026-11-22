"""Offline PICO session protocol tests; never open CAN or camera devices."""

import asyncio
from contextlib import contextmanager
import json
import time
import unittest
from unittest.mock import Mock, patch

try:
    from aiohttp import WSCloseCode, WSMsgType
    from aiohttp.test_utils import TestClient, TestServer
except ImportError:
    TestClient = None

from openarm_runtime.pico_classic import InputSession, application, classic_right

TOKEN = "offline-test-token-at-least-24"
POSE = json.dumps({"pose": [0, 0, 0, 0, 0, 0, 1], "grip": False, "trigger": False, "joystick_y": 0})


class ClassicAdapterTests(unittest.TestCase):
    def test_runtime_preflight_gravity_and_verified_cleanup(self):
        profile, chain = Mock(), Mock()
        bus, gravity = Mock(), Mock(mode="active")
        bus.state.side_effect = lambda mid: Mock(position=mid / 10)
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
    def test_expired_input_cannot_restart(self):
        now = [0.]
        session = InputSession(lambda: now[0])
        self.assertTrue(session.submit((0,) * 7, False))
        now[0] = .30
        self.assertFalse(session.submit((0,) * 7, True))
        self.assertTrue(session.cancelled())


@unittest.skipUnless(TestClient is not None, "Install pico-classic for HTTP tests")
class WebTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.events = []

        def worker(session, _profile, _chain):
            if not session.ready.wait(.25) or session.cancelled():
                return
            self.events.append("enabled")
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

    async def start(self):
        socket = await self.connect()
        await socket.send_str("start")
        await socket.send_str(POSE)
        self.assertEqual((await socket.receive(timeout=1)).data, "ok")
        for _ in range(50):
            if self.events[-1:] == ["enabled"]:
                break
            await asyncio.sleep(.01)
        self.assertEqual(self.events[-1:], ["enabled"])
        return socket

    async def test_start_exit_loss_and_explicit_restart(self):
        response = await self.client.get("/")
        self.assertEqual(response.status, 200)
        self.assertIn("PICO Classic", await response.text())
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

        second = await self.start()
        self.assertEqual((await second.receive(timeout=1)).type, WSMsgType.CLOSE)
        self.assertEqual(self.events, ["enabled", "disabled"] * 2)

        third = await self.start()
        await third.close(code=WSCloseCode.GOING_AWAY)
        for _ in range(50):
            if self.events[-1:] == ["disabled"]:
                break
            await asyncio.sleep(.01)
        self.assertEqual(self.events, ["enabled", "disabled"] * 3)
