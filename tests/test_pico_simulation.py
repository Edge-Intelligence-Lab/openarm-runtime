import unittest
from threading import Lock
from types import SimpleNamespace

from openarm_simulation.pico import PicoHands, PicoTeleop, controller_sample


class SimulatedBus:
    def __init__(self):
        self.enabled = False
        self.commands = []

    def preflight(self):
        if self.enabled:
            raise ValueError('Already active')

    def state(self, mid):
        return SimpleNamespace(position=0.1 * mid)

    def position_snapshot(self):
        return [self.state(mid).position for mid in range(1, 9)]

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
        def step(measured, delta, rotation, dt):
            self.calls.append((measured, delta, rotation, dt))
            return [value + 0.001 for value in measured]
        return step

    def test_clutch_maps_axes_and_uses_relative_position(self):
        self.session.sample((1., 1., 1.), 1.)
        self.assertTrue(self.bus.enabled)
        self.assertEqual(self.bus.commands, [{i: 0.1 * i for i in range(1, 9)}])
        self.session.sample((1.04, 1.02, .94), 1.05)
        for actual, expected in zip(self.calls[-1][1], (.06, -.04, .02), strict=True):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(self.bus.commands[-1][8], .8)
        self.session.sample(None, 1.06)
        self.assertFalse(self.bus.enabled)
        self.session.sample((2., 1., 1.), 1.07)
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(len(self.bus.commands), 3)

    def test_clutch_maps_wrist_rotation_about_the_vertical_axis(self):
        from math import cos, radians, sin

        self.session.sample((0., 0., 0., 0., 0., 0., 1.), 1.)
        half_angle = radians(10)
        self.session.sample((0., 0., 0., 0., sin(half_angle), 0., cos(half_angle)), 1.05)
        _, delta, rotation, _ = self.calls[-1]
        self.assertEqual(tuple(delta), (0., 0., 0.))
        for actual, expected in zip(rotation, (0., 0., radians(20)), strict=True):
            self.assertAlmostEqual(actual, expected)

    def test_overview_screen_right_maps_to_camera_right(self):
        from openarm_simulation.pico import overview_axes

        self.session.axes = overview_axes((1.4, -1.65, .75), (.05, 0., -.18))
        self.session.sample((0., 0., 0.), 1.)
        self.session.sample((.05, 0., 0.), 1.05)
        _, delta, _, _ = self.calls[-1]
        self.assertGreater(delta[1], 0.)
        self.assertAlmostEqual(delta[0] * -1.35 + delta[1] * 1.65, 0.)
        self.assertAlmostEqual(sum(v ** 2 for v in delta), .05 ** 2)

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

    def test_wrist_tracking_jump_disarms_until_release(self):
        self.session.sample((0., 0., 0., 0., 0., 0., 1.), 1.)
        self.assertFalse(self.session.sample((0., 0., 0., 0., 1., 0., 0.), 1.05))
        self.assertFalse(self.bus.enabled)
        self.assertFalse(self.session.sample((0., 0., 0., 0., 0., 0., 1.), 1.1))
        self.session.sample(None, 1.15)
        self.assertTrue(self.session.sample((0., 0., 0., 0., 0., 0., 1.), 1.2))

    def test_right_hand_moves_only_right_arm_and_trigger_controls_gripper(self):
        right_bus = SimulatedBus()
        right = PicoTeleop(right_bus, Lock(), self.begin, open_fingers=.35)
        hands = PicoHands({'left': self.session, 'right': right})
        self.addCleanup(hands.release)
        hands.sample(controller_sample('{"left":null,"right":[0,0,0,0,0,0,1,false]}'), 1.)
        hands.sample(controller_sample('{"left":null,"right":[0,0.02,0,0,0,0,1,true]}'), 1.05)
        self.assertFalse(self.bus.commands)
        self.assertEqual(right_bus.commands[-1][8], 0.)
        hands.sample(controller_sample('{"left":null,"right":[0,0.02,0,0,0,0,1,false]}'), 1.1)
        self.assertEqual(right_bus.commands[-1][8], .35)

    def test_shared_grasp_locks_relative_motion_until_either_trigger_releases(self):
        touching = False
        right = PicoTeleop(SimulatedBus(), Lock(), self.begin, open_fingers=.35)
        hands = PicoHands({'left': self.session, 'right': right}, shared_contact=lambda: touching)
        self.addCleanup(hands.release)

        def pose(y, closed=True):
            return (0., y, 0., 0., 0., 0., 1., closed)

        hands.sample({'left': pose(0.), 'right': pose(0.)}, 1.)
        hands.sample({'left': pose(.02), 'right': pose(0.)}, 1.05)
        self.assertIsNone(hands.paired_origins)
        touching = True
        hands.sample({'left': pose(.02), 'right': pose(0.)}, 1.10)
        hands.sample({'left': pose(.06), 'right': pose(.02)}, 1.15)
        for call in self.calls[-2:]:
            self.assertAlmostEqual(call[1][2], .03)
        hands.sample({'left': pose(.06, False), 'right': pose(.02)}, 1.20)
        self.assertIsNone(hands.paired_origins)
        for call in self.calls[-2:]:
            self.assertAlmostEqual(call[1][2], 0.)

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
        self.assertEqual(controller_sample('[1,2,3,0,0,0,1]'), (1., 2., 3., 0., 0., 0., 1.))
        for data in ('[true,0,0]', '[0,0,NaN]', '[0,0,1e100]', '[0,0]', '[0,0,0,0,0,0,0]', '[0,0,0,0,0,0,2]', '{}', ' ' * 257):
            with self.subTest(data=data), self.assertRaises(ValueError):
                controller_sample(data)


class PicoTransportTests(unittest.IsolatedAsyncioTestCase):
    async def test_scene_selection_is_validated_and_runs_before_control(self):
        from openarm_platform.pico import stream_samples

        self.assertEqual(controller_sample('{"scene":"tube"}'), {'scene': 'tube'})
        for text in ('{"scene":"other"}', '{"scene":[]}', '{"scene":"key","left":null}'):
            with self.assertRaises(ValueError):
                controller_sample(text)

        bus = SimulatedBus()
        session = PicoTeleop(bus, Lock(), lambda joints: None)
        replies = []
        choices = []

        async def receive():
            return {'type': 'websocket.receive', 'text': '{"scene":"key"}'}

        async def send_json(reply):
            replies.append(reply)

        def select_scene(scene):
            choices.append(scene)
            return True

        await stream_samples(SimpleNamespace(receive=receive, send_json=send_json),
                             session, lambda: False, select_scene=select_scene)
        self.assertEqual(choices, ['key'])
        self.assertEqual(replies, [{'scene': 'key'}])
        self.assertFalse(bus.enabled)

    async def test_explicit_scene_reset_releases_control_before_reset(self):
        from openarm_platform.pico import stream_samples

        bus = SimulatedBus()
        lease = Lock()
        session = PicoTeleop(bus, lease, lambda joints: None)
        requests = iter(('[0,0,0]', '{"reset":true}'))
        replies = []

        async def receive():
            return {'type': 'websocket.receive', 'text': next(requests)}

        async def send_json(reply):
            replies.append(reply)

        def reset_scene():
            self.assertFalse(bus.enabled)
            self.assertFalse(lease.locked())
            return True

        await stream_samples(SimpleNamespace(receive=receive, send_json=send_json),
                             session, lambda: False, reset_scene=reset_scene)
        self.assertEqual(replies, [{'following': True}, {'reset': True}])

    async def test_reset_is_only_allowed_at_start_of_connection(self):
        from openarm_platform.pico import stream_samples

        session = PicoTeleop(SimulatedBus(), Lock(), lambda joints: None)
        requests = iter(('{"start":true}', 'null', '{"start":true}'))
        replies = []
        resets = []

        async def receive():
            return {'type': 'websocket.receive', 'text': next(requests)}

        async def send_json(reply):
            replies.append(reply)

        def reset():
            resets.append(True)
            return True

        with self.assertRaisesRegex(ValueError, '重置初始姿态'):
            await stream_samples(SimpleNamespace(receive=receive, send_json=send_json),
                                 session, lambda: False, reset=reset)
        self.assertEqual(resets, [True])
        self.assertEqual(replies, [{'started': True}, {'following': False}])

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
            await stream_samples(socket, session, lambda: False, timeout=0.01, connection_timeout=0.04)
        self.assertEqual(replies, [{'following': True}])
        self.assertFalse(bus.enabled)
        self.assertFalse(lease.locked())

    async def test_brief_gap_stops_arm_but_keeps_socket_and_requires_regrip(self):
        from asyncio import sleep
        from openarm_platform.pico import stream_samples

        bus = SimulatedBus()
        session = PicoTeleop(bus, Lock(), lambda joints: None)
        events = iter(('[0,0,0]', '[0,0,0]', 'null', '[0,0,0]'))
        replies = []

        async def receive():
            if len(replies) == 1:
                await sleep(.04)
                self.assertFalse(bus.enabled)
            message = next(events, None)
            return ({'type': 'websocket.receive', 'text': message} if message else
                    {'type': 'websocket.disconnect'})

        async def send_json(data):
            replies.append(data['following'])

        await stream_samples(SimpleNamespace(receive=receive, send_json=send_json), session,
                             lambda: False, timeout=.01, connection_timeout=.20)
        self.assertEqual(replies, [True, False, False, True])
        self.assertFalse(bus.enabled)
