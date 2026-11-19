import unittest
from threading import Lock
from types import SimpleNamespace

from openarm_simulation.pico import PicoTeleop, controller_sample


class SimulatedBus:
    def __init__(self):
        self.enabled = False
        self.commands = []

    def preflight(self):
        if self.enabled:
            raise ValueError('Already active')

    def state(self, mid):
        return SimpleNamespace(position=0.1 * mid)

    def command(self, mid, action, position=0.0):
        if action in ('enable', 'disable'):
            self.enabled = action == 'enable'

    def start_hold(self, targets, kp, kd):
        self.enabled = True
        self.commands.append(targets)

    def exchange(self, targets, kp, kd, feedforward):
        self.commands.append(targets)

    def gravity_feedforward(self):
        return {}


class PicoSimulationTests(unittest.TestCase):
    def setUp(self):
        self.bus = SimulatedBus()
        self.lease = Lock()
        self.calls = []
        self.session = PicoTeleop(self.bus, self.lease, self.begin)
        self.addCleanup(self.session.release)

    def begin(self, joints):
        def step(measured, delta, dt):
            self.calls.append((measured, delta, dt))
            return [value + 0.001 for value in measured]
        return step

    def test_clutch_maps_axes_and_uses_relative_position(self):
        self.session.sample((1., 1., 1.), 1.)
        self.assertTrue(self.bus.enabled)
        self.assertEqual(self.bus.commands, [{i: 0.1 * i for i in range(1, 9)}])
        self.session.sample((1.04, 1.02, .94), 1.05)
        for actual, expected in zip(self.calls[-1][1], (.03, -.02, .01), strict=True):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(self.bus.commands[-1][8], .8)
        self.session.sample(None, 1.06)
        self.assertFalse(self.bus.enabled)
        self.session.sample((2., 1., 1.), 1.07)
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(len(self.bus.commands), 3)

    def test_disconnect_releases_simulation_ownership(self):
        self.session.sample((0., 0., 0.), 1.)
        other = PicoTeleop(self.bus, self.lease, self.begin)
        self.addCleanup(other.release)
        with self.assertRaisesRegex(ValueError, '占用'):
            other.sample((0., 0., 0.), 1.)
        self.session.release()
        self.assertFalse(self.bus.enabled)
        other.sample((0., 0., 0.), 2.)
        self.assertTrue(self.bus.enabled)

    def test_delayed_sample_disarms_until_grip_is_released(self):
        self.session.sample((0., 0., 0.), 1.)
        self.session.sample((.02, 0., 0.), 2.)
        self.assertFalse(self.bus.enabled)
        self.session.sample((.021, 0., 0.), 2.05)
        self.assertFalse(self.bus.enabled)
        self.session.sample(None, 2.10)
        self.session.sample((.021, 0., 0.), 2.15)
        self.assertTrue(self.bus.enabled)

    def test_wire_format_rejects_bad_data(self):
        self.assertIsNone(controller_sample('null'))
        self.assertEqual(controller_sample('[1,2,3]'), (1., 2., 3.))
        for data in ('[true,0,0]', '[0,0,NaN]', '[0,0,1e100]', '[0,0]', '{}', ' ' * 129):
            with self.subTest(data=data), self.assertRaises(ValueError):
                controller_sample(data)


class PicoTransportTests(unittest.IsolatedAsyncioTestCase):
    async def test_https_same_origin_keeps_peer_and_origin_checks(self):
        from openarm_platform.app import LocalBoundary

        async def accepted(scope, receive, send):
            await send({'type': 'websocket.accept'})

        guarded = LocalBoundary(accepted, host='192.168.1.20',
                                allowed_clients=('192.168.1.21/32',), https=True)
        for origin, client, expected in (
            ('https://192.168.1.20:8888', '192.168.1.21', 'websocket.accept'),
            ('http://192.168.1.20:8888', '192.168.1.21', 'websocket.close'),
            ('https://192.168.1.20:8888', '192.168.1.22', 'websocket.close'),
        ):
            messages = []

            async def send(message):
                messages.append(message)

            await guarded({'type': 'websocket', 'scheme': 'wss', 'client': (client, 1234),
                           'headers': [(b'host', b'192.168.1.20:8888'),
                                       (b'origin', origin.encode())]}, None, send)
            self.assertEqual(messages[0]['type'], expected)

    def test_tls_profile_is_paired_and_simulation_only(self):
        from dataclasses import replace
        from pathlib import Path
        from openarm_platform.config import load_config, validate

        config = load_config()
        with self.assertRaisesRegex(ValueError, 'simulation'):
            validate(replace(config, tls_cert=Path('cert.pem'), tls_key=Path('key.pem')))
        with self.assertRaisesRegex(ValueError, 'together'):
            validate(replace(config, tls_cert=Path('cert.pem')))

    async def test_silent_socket_disarms_and_releases_lease(self):
        from asyncio import Event
        from openarm_platform.pico import stream_samples

        bus = SimulatedBus()
        lease = Lock()
        session = PicoTeleop(bus, lease, lambda joints: None)
        replies = []

        async def receive():
            if replies:
                await Event().wait()
            return {'type': 'websocket.receive', 'text': '[0,0,0]'}

        async def send_json(data):
            self.assertTrue(bus.enabled)
            replies.append(data)

        socket = SimpleNamespace(receive=receive, send_json=send_json)
        with self.assertRaises(TimeoutError):
            await stream_samples(socket, session, lambda: False, timeout=0.01)
        self.assertEqual(replies, [{'following': True}])
        self.assertFalse(bus.enabled)
        self.assertFalse(lease.locked())
