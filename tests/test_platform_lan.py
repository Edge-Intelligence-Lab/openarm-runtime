"""Explicit LAN opt-in retains host, peer and same-origin boundaries."""
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from openarm_platform import load_config
from openarm_platform.config import validate
from openarm_platform.app import LocalBoundary, create_app
from openarm_platform.proxy import valid_host, same_origin

HOST='192.168.1.20'
NETWORK='192.168.1.0/24'


class LanConfigTests(unittest.TestCase):
    def test_loopback_default_and_explicit_private_network(self):
        c=load_config()
        self.assertEqual((c.host,c.allowed_clients),('127.0.0.1',()))
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'platform.json'
            p.write_text(json.dumps(dict(host=HOST,allowed_clients=[NETWORK])))
            self.assertEqual(load_config(p).host,HOST)

    def test_wildcards_public_hosts_and_overbroad_or_invalid_clients_rejected(self):
        for host,clients in [('0.0.0.0',(NETWORK,)),('8.8.8.8',(NETWORK,)),('localhost',(NETWORK,)),
                (HOST,()),(HOST,('0.0.0.0/0',)),(HOST,('10.0.0.0/8',)),
                (HOST,('192.168.1.10/24',)),(HOST,('::/0',)),(2130706433,())]:
            with self.subTest(host=host,clients=clients),self.assertRaises(ValueError):
                validate(replace(load_config(),host=host,allowed_clients=clients))

    def test_gateway_only_accepts_explicit_host_and_exact_origin(self):
        allowed=('127.0.0.1',HOST)
        headers={'host':HOST+':8888','origin':'http://'+HOST+':8888'}
        c=SimpleNamespace(headers=headers,scope={'scheme':'http'})
        self.assertFalse(valid_host(headers))
        self.assertTrue(valid_host(headers,allowed))
        self.assertTrue(same_origin(c,allowed))
        for origin in ['http://evil.invalid','http://'+HOST+':8889','null']:
            headers['origin']=origin
            self.assertFalse(same_origin(c,allowed))


class LanBoundaryTests(unittest.IsolatedAsyncioTestCase):
    async def exchange(self,peer='192.168.1.10',host=HOST+':8888',origin=None,kind='http',method='GET',extra=()):
        messages=[]
        async def downstream(scope,receive,send):
            await send({'type':'websocket.accept'} if kind=='websocket' else
                       {'type':'http.response.start','status':200,'headers':[]})
        async def receive():return {'type':'websocket.connect'} if kind=='websocket' else {'type':'http.request'}
        async def send(message):messages.append(message)
        headers=[(b'host',host.encode())]
        if origin is not None:headers.append((b'origin',origin.encode()))
        await LocalBoundary(downstream,host=HOST,allowed_clients=(NETWORK,))(
            {'type':kind,'method':method,'headers':headers+list(extra),'client':(peer,54321)},receive,send)
        return messages[0]

    async def test_allowed_lan_reads_and_same_origin_events(self):
        self.assertEqual((await self.exchange())['status'],200)
        self.assertEqual((await self.exchange(method='POST',origin='http://'+HOST+':8888'))['status'],200)
        self.assertEqual((await self.exchange(kind='websocket',origin='http://'+HOST+':8888'))['type'],'websocket.accept')

    async def test_peer_cannot_be_spoofed_with_forwarding_headers(self):
        for peer in ('192.168.2.10','100.64.0.1','8.8.8.8','testclient'):
            result=await self.exchange(peer=peer,extra=((b'x-forwarded-for',b'192.168.1.10'),))
            self.assertEqual(result['status'],403)
            result=await self.exchange(peer=peer,kind='websocket',origin='http://'+HOST+':8888')
            self.assertEqual(result,{'type':'websocket.close','code':1008})

    async def test_wrong_host_port_origin_and_duplicate_headers_rejected(self):
        for host in ('evil.invalid:8888','192.168.1.21:8888',HOST+':9999',HOST+':bad'):
            self.assertEqual((await self.exchange(host=host))['status'],403)
        for origin in (None,'null','http://evil.invalid','http://'+HOST+':8889'):
            self.assertEqual((await self.exchange(method='POST',origin=origin))['status'],403)
            self.assertEqual((await self.exchange(kind='websocket',origin=origin))['type'],'websocket.close')
        self.assertEqual((await self.exchange(extra=((b'host',b'localhost:8888'),)))['status'],403)


@unittest.skipUnless(importlib.util.find_spec('nicegui'),'platform extra')
class LanApiTests(unittest.TestCase):
    def test_configured_lan_api_without_hardware_and_outside_peer_denied(self):
        from starlette.testclient import TestClient
        c=validate(replace(load_config(),host=HOST,allowed_clients=(NETWORK,)))
        m=Mock();m.status.return_value={}
        with TestClient(create_app(c,m,attach_only=True,with_ui=False),base_url='http://'+HOST+':8888',client=('192.168.1.10',4567)) as client:
            self.assertEqual(client.get('/api/config').status_code,200)
            r=client.post('/api/select',json={'id':'camera_calibration'},headers={'Origin':'http://'+HOST+':8888'})
            self.assertEqual(r.status_code,200)
            self.assertTrue(r.json()['services']['camera_calibration']['reachable'])
            self.assertEqual(client.post('/api/select',json={'id':'camera_calibration'},headers={'Origin':'http://evil.invalid'}).status_code,403)
        with TestClient(create_app(c,m,attach_only=True,with_ui=False),base_url='http://'+HOST+':8888',client=('192.168.2.10',4567)) as client:
            self.assertEqual(client.get('/api/config').status_code,403)
        m.start.assert_not_called();m.activate.assert_not_called()
