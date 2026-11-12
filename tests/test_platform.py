"""Configuration and HTTP boundaries for the platform, with no hardware services."""
from dataclasses import replace
import asyncio
import importlib.util
import socket
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from openarm_platform import load_config
from openarm_platform.app import Coordinator, LocalBoundary, create_app, serve


class PlatformTests(unittest.TestCase):
    def profile(self, directory, data):
        path = Path(directory) / 'platform.json'
        path.write_text(json.dumps(data))
        return load_config(path)

    def test_configuration_keeps_commands_private_and_expands_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            config = self.profile(directory, {'services': {'teleop': {
                'command': ['{python}', '-m', 'openarm_runtime', '~/private-config.json'],
                'cwd': 'workspace'}}})
            teleop = config.pages[1]
            self.assertEqual(teleop.command[0], sys.executable)
            self.assertEqual(teleop.command[-1], str(Path.home() / 'private-config.json'))
            self.assertEqual(teleop.cwd, (Path(directory) / 'workspace').resolve())
            self.assertEqual(teleop.exclusive_group, 'motion')
            self.assertNotIn('command', teleop.public())
            self.assertNotIn('cwd', teleop.public())
            self.assertEqual(config.pages[2].extra_ports, (8901, 8902))

    def test_collection_root_follows_teleop_profile_or_explicit_override(self):
        with tempfile.TemporaryDirectory() as directory:
            # macOS temporary directories can be reached through /var symlinks.
            root = Path(directory).resolve()
            (root/'collection.json').write_text(json.dumps({'root': 'episodes'}))
            data = {'services': {'teleop': {'command': ['python', 'leader', '--collection-profile', 'collection.json']}}}
            config = self.profile(directory, data)
            self.assertEqual(config.collection_root, root/'episodes')
            self.assertEqual(next(p.path for p in config.pages if p.id=='collection'), '/collection')
            self.assertNotIn('collection', [p.id for p in config.services])
            data['collection_root'] = 'override'
            self.assertEqual(self.profile(directory, data).collection_root, root/'override')

    def test_rejects_shell_commands_and_port_collisions(self):
        cases = (
            {'services': {'control': {'command': 'echo bad'}}},
            {'services': {'control': {'command': ['']}}},
            {'services': {'control': {'port': True}}},
            {'services': {'typo': {}}},
            {'port': 8891},
            {'services': {'vision': {'port': 8901}}},
            {'services': {'tactile': {'port': 65530}}},
            {'services': {'control': {'cwd': 1}}},
            {'support': {'../outside': {'port': 12000}}},
        )
        with tempfile.TemporaryDirectory() as directory:
            for data in cases:
                with self.subTest(data=data), self.assertRaises(ValueError):
                    self.profile(directory, data)

    def test_imports_and_cli_help_are_passive(self):
        script = """
import sys
from unittest.mock import patch
with patch('socket.socket', side_effect=AssertionError('socket')), patch('subprocess.Popen', side_effect=AssertionError('process')):
    from openarm_platform import load_config
    import openarm_platform.app
    config = load_config()
assert not any(x in sys.modules for x in ('viser', 'serial', 'openarm_can', 'paxini_sdk', 'flask', 'nicegui', 'aiohttp', 'fastapi'))
"""
        subprocess.run([sys.executable, '-I', '-c', script], cwd='/tmp', check=True)
        result = subprocess.run([sys.executable, '-I', '-m', 'openarm_runtime', 'platform', '--help'],
                                cwd='/tmp', check=True, capture_output=True, text=True)
        self.assertIn('--attach-only', result.stdout)


PLATFORM_INSTALLED = all(importlib.util.find_spec(name) is not None
                           for name in ('nicegui', 'aiohttp', 'fastapi'))


class TopStopCoordinatorTests(unittest.IsolatedAsyncioTestCase):
    async def test_model_stop_is_requested_before_owned_service_shutdown(self):
        events = []
        manager, models = Mock(), Mock()
        models.stop.side_effect = lambda: events.append('model-stop')
        manager.stop_motion.side_effect = lambda: events.append('service-stop') or {}
        coordinator = Coordinator(manager)
        coordinator.models = models
        await coordinator.stop_motion()
        self.assertEqual(events, ['model-stop', 'service-stop'])

    async def test_attach_only_never_issues_stop(self):
        manager = Mock()
        coordinator = Coordinator(manager, attach_only=True)
        with self.assertRaises(ValueError): await coordinator.stop_motion()
        manager.stop_motion.assert_not_called()


class LocalBoundaryTests(unittest.IsolatedAsyncioTestCase):
    async def exchange(self, host, *, origin=None, kind='http', method='GET', extra_headers=()):
        messages = []
        async def downstream(scope, receive, send):
            if scope['type'] == 'websocket':
                await send({'type': 'websocket.accept'})
            else:
                await send({'type': 'http.response.start', 'status': 200, 'headers': []})
                await send({'type': 'http.response.body', 'body': b'ok'})
        async def receive():
            return {'type': 'websocket.connect'} if kind == 'websocket' else {'type': 'http.request'}
        async def send(message):
            messages.append(message)
        headers = [(b'host', host.encode())]
        if origin is not None:
            headers.append((b'origin', origin.encode()))
        await LocalBoundary(downstream)({'type': kind, 'method': method,
                                        'headers': headers + list(extra_headers)}, receive, send)
        return messages[0]

    async def test_exact_preview_hostname_accepts_reads_and_same_origin_events(self):
        host = 'openchamber-preview.localhost:50310'
        self.assertEqual((await self.exchange(host))['status'], 200)
        self.assertEqual((await self.exchange(host, method='POST', origin='http://' + host))['status'], 200)
        self.assertEqual((await self.exchange(host, kind='websocket', origin='http://' + host))['type'],
                         'websocket.accept')

    async def test_preview_does_not_allow_lookalikes_malformed_or_duplicate_hosts(self):
        for host in ('other.localhost:50310', 'evil.openchamber-preview.localhost:50310',
                     'openchamber-preview.localhost.evil.invalid:50310',
                     'openchamber-preview.localhost.:50310', 'openchamber-preview.localhost:invalid',
                     'user@openchamber-preview.localhost:50310',
                     'openchamber-preview.localhost:50310/path'):
            with self.subTest(host=host):
                self.assertEqual((await self.exchange(host))['status'], 403)
        result = await self.exchange('openchamber-preview.localhost:50310',
                                     extra_headers=[(b'host', b'localhost:50310')])
        self.assertEqual(result['status'], 403)

    async def test_preview_retains_exact_origin_scheme_port_and_websocket_checks(self):
        host = 'openchamber-preview.localhost:50310'
        for origin in (None, 'null', 'https://evil.invalid', 'http://localhost:50310',
                       'http://openchamber-preview.localhost:50311',
                       'https://' + host):
            with self.subTest(origin=origin):
                self.assertEqual((await self.exchange(host, method='POST', origin=origin))['status'], 403)
                result = await self.exchange(host, kind='websocket', origin=origin)
                self.assertEqual(result, {'type': 'websocket.close', 'code': 1008})
        duplicate = await self.exchange(host, method='POST', origin='http://' + host,
                                        extra_headers=[(b'origin', ('http://' + host).encode())])
        self.assertEqual(duplicate['status'], 403)


@unittest.skipUnless(PLATFORM_INSTALLED, 'Requires the platform extra')
class PlatformApiTests(unittest.TestCase):
    def client(self, manager, *, attach_only=False):
        from starlette.testclient import TestClient
        return TestClient(create_app(load_config(), manager, attach_only=attach_only, with_ui=False),
                          base_url='http://127.0.0.1:8888')

    def test_policy_page_is_available_in_attach_only_without_activating_hardware(self):
        manager = Mock()
        manager.status.return_value = {}
        with self.client(manager, attach_only=True) as client:
            response = client.post('/api/select', json={'id': 'policy'},
                                   headers={'Origin': 'http://127.0.0.1:8888'})
            self.assertEqual(response.status_code, 200)
            self.assertTrue(response.json()['services']['policy']['reachable'])
            manager.start.assert_not_called()
            manager.activate.assert_not_called()
            self.assertIsNone(client.app.state.policy_preview.worker)

    def test_http_status_selection_and_origin_boundary(self):
        manager = Mock()
        manager.status.return_value = {'control': {'state': 'external', 'reachable': True}}
        manager.activate.return_value = manager.status.return_value
        with self.client(manager) as client:
            manager.start.assert_called_once()
            response = client.get('/api/config')
            self.assertEqual(response.status_code, 200)
            self.assertEqual(len(response.json()['pages']), 7)
            self.assertNotIn('command', response.text)
            self.assertNotIn('cwd', response.text)
            self.assertEqual(client.get('/api/status').json()['services'], client.app.state.coordinator.with_builtin(manager.status.return_value))
            self.assertEqual(client.get('/api/status').headers['x-frame-options'], 'SAMEORIGIN')
            self.assertEqual(client.get('/api/status', headers={'Host': 'evil.invalid'}).status_code, 403)
            self.assertEqual(client.get('/_nicegui_ws/socket.io/?EIO=4&transport=polling',
                                        headers={'Origin': 'https://evil.invalid'}).status_code, 403)
            self.assertEqual(client.post('/api/select', json={'id': 'control'},
                                        headers={'Origin': 'https://evil.invalid'}).status_code, 403)
            self.assertEqual(client.post('/api/select', json={'id': 'control'}).status_code, 403)
            manager.activate.assert_not_called()
            headers = {'Origin': 'http://127.0.0.1:8888'}
            self.assertEqual(client.post('/api/select', json={'id': 'control'}, headers=headers).status_code, 200)
            manager.activate.assert_called_once_with('control')
            for data in ({'id': 'unknown'}, {'id': []}, {'id': 'control', 'extra': True}):
                self.assertEqual(client.post('/api/select', json=data, headers=headers).status_code, 409)
            self.assertEqual(client.post('/api/select', content='x' * 2048,
                                        headers={**headers, 'Content-Type': 'application/json'}).status_code, 409)
            manager.activate.side_effect = ValueError('external controller is active')
            response = client.post('/api/select', json={'id': 'teleop'}, headers=headers)
            self.assertEqual(response.status_code, 409)
            self.assertIn('external controller', response.text)
        manager.close.assert_called_once()

    def test_attach_only_never_starts_or_selects_a_service(self):
        manager = Mock()
        manager.status.return_value = {}
        with self.client(manager, attach_only=True) as client:
            self.assertTrue(client.get('/api/config').json()['attach_only'])
            response = client.post('/api/select', json={'id': 'control'},
                                   headers={'Origin': 'http://127.0.0.1:8888'})
            self.assertEqual(response.status_code, 200)
        manager.start.assert_not_called()
        manager.activate.assert_not_called()
        manager.close.assert_called_once()

    def test_openchamber_preview_api_keeps_attach_only_passive(self):
        manager = Mock()
        manager.status.return_value = {}
        host = 'openchamber-preview.localhost:50310'
        with self.client(manager, attach_only=True) as client:
            response = client.get('/api/status', headers={'Host': host})
            self.assertEqual(response.status_code, 200)
            response = client.post('/api/select', json={'id': 'collection'},
                                   headers={'Host': host, 'Origin': 'http://' + host})
            self.assertEqual(response.status_code, 200)
            self.assertTrue(response.json()['services']['collection']['reachable'])
            self.assertEqual(client.post('/api/select', json={'id': 'control'},
                                         headers={'Host': host, 'Origin': 'http://localhost:50310'}).status_code, 403)
        manager.start.assert_not_called()
        manager.activate.assert_not_called()

    def test_failed_startup_still_cleans_up(self):
        manager = Mock()
        manager.start.side_effect = RuntimeError('startup failed')
        with self.assertRaisesRegex(RuntimeError, 'startup failed'), self.client(manager):
            pass
        manager.close.assert_called_once()

    def test_occupied_entry_port_does_not_construct_app_or_start_services(self):
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            listener.listen()
            with patch('openarm_platform.app.create_app') as factory, self.assertRaises(OSError):
                serve(load_config().with_port(listener.getsockname()[1]))
            factory.assert_not_called()


    def test_signal_reraise_cannot_hide_service_cleanup_failure(self):
        application = SimpleNamespace(state=SimpleNamespace(shutdown_error=None))
        class InterruptedServer:
            def __init__(self, config):
                self.config = config
                self.started = True
            def run(self, **kwargs):
                self.config.app.state.shutdown_error = RuntimeError('controller cleanup failed')
                raise KeyboardInterrupt('re-raised SIGTERM')
        with patch('openarm_platform.app.create_app', return_value=application), \
             patch('uvicorn.Server', InterruptedServer), \
             self.assertRaisesRegex(RuntimeError, 'Platform service cleanup failed') as raised:
            serve(replace(load_config(), port=0))
        self.assertIn('controller cleanup failed', str(raised.exception.__cause__))


class CoordinatorTests(unittest.IsolatedAsyncioTestCase):
    async def test_blocking_operations_do_not_block_loop_and_cancelled_switch_is_drained(self):
        entered, release = threading.Event(), threading.Event()
        manager = Mock()
        events = []
        def switch(name):
            entered.set()
            release.wait(2)
            events.append('switched')
            return {}
        manager.activate.side_effect = switch
        manager.close.side_effect = lambda: events.append('closed')
        coordinator = Coordinator(manager)
        operation = asyncio.create_task(coordinator.activate('teleop'))
        try:
            self.assertTrue(await asyncio.to_thread(entered.wait, 1))
            # The event loop remains responsive while the process manager waits.
            self.assertEqual(await asyncio.wait_for(asyncio.sleep(0, result='responsive'), .2), 'responsive')
            operation.cancel()
            with self.assertRaises(asyncio.CancelledError):
                await operation
            closing = asyncio.create_task(coordinator.close())
            await asyncio.sleep(.02)
            self.assertFalse(closing.done())
            with self.assertRaises(RuntimeError):
                await coordinator.status()
            release.set()
            await asyncio.wait_for(closing, 2)
            self.assertEqual(events, ['switched', 'closed'])
        finally:
            release.set()
