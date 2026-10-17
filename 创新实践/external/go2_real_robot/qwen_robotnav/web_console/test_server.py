"""Boundary checks for the local single-window relay (no simulator required)."""
import http.client
import json
from pathlib import Path
import threading
import unittest
from unittest.mock import patch
from http.server import ThreadingHTTPServer
import server


class ConsoleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.http = ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)
        cls.http.state = server.State(225, ':2')
        cls.http.state.window['available'] = True
        cls.http.stopping = threading.Event()
        cls.thread = threading.Thread(target=cls.http.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.http.shutdown()
        cls.http.server_close()
        cls.thread.join(timeout=2)

    def request(self, path, method='GET', headers=None):
        conn = http.client.HTTPConnection('127.0.0.1', self.http.server_port, timeout=2)
        conn.request(method, path, headers=headers or {})
        response = conn.getresponse()
        result = response.status, dict(response.headers), response.read()
        conn.close()
        return result

    def test_real_static_module_available(self):
        path = server.static_file('/vendor/novnc/core/rfb.js')
        self.assertIsNotNone(path, 'Run npm ci before tests')
        self.assertIn('class RFB', path.read_text())

    def test_encoded_and_vendor_traversal_are_rejected(self):
        for path in ['/../server.py', '/%2e%2e/server.py', '/vendor/novnc/../../../../server.py', '/.env', '/package.json']:
            with self.subTest(path=path):
                self.assertEqual(self.request(path)[0], 404)

    def test_static_has_content_type_and_policy(self):
        status, headers, body = self.request('/app.js')
        self.assertEqual(status, 200)
        self.assertTrue(headers['Content-Type'].startswith('text/javascript'))
        self.assertIn("frame-ancestors 'self'", headers['Content-Security-Policy'])

    def test_state_does_not_fabricate_telemetry(self):
        status, _, body = self.request('/api/state')
        data = json.loads(body)
        self.assertEqual(status, 200)
        self.assertEqual(data['telemetry'], {})
        self.assertIsNone(data['bridge_age_s'])
        self.assertTrue(data['read_only'])

    def test_commands_are_not_an_api(self):
        self.assertEqual(self.request('/api/command', 'POST')[0], 405)

    def test_foreign_websocket_origin_is_rejected(self):
        self.assertEqual(self.request('/vnc', headers={'Origin': 'https://example.com', 'Upgrade': 'websocket'})[0], 403)

    def test_missing_websocket_origin_is_rejected(self):
        self.assertEqual(self.request('/vnc', headers={'Upgrade': 'websocket'})[0], 403)

    def test_dns_rebinding_host_is_rejected(self):
        self.assertEqual(self.request('/api/state', headers={'Host': 'example.com'})[0], 403)

    def test_same_origin_non_upgrade_is_rejected(self):
        self.assertEqual(self.request('/vnc', headers={'Origin': f'http://127.0.0.1:{self.http.server_port}'})[0], 426)

    def test_multiple_gazebo_windows_require_selection(self):
        with patch.object(server.subprocess, 'check_output', return_value='0x123 "Gazebo":\n0x456 "Gazebo":'):
            with self.assertRaisesRegex(ValueError, '恰好一个'):
                server.find_window(':2')

    def test_explicit_non_gazebo_window_is_rejected(self):
        with patch.object(server.subprocess, 'check_output', return_value='Window id: 0x123 "Terminal"'):
            with self.assertRaisesRegex(ValueError, '不是 Gazebo'):
                server.find_window(':2', '0x123')

    def test_dead_relay_process_does_not_stay_online(self):
        state = server.State(225, ':2')
        child = unittest.mock.Mock()
        child.poll.return_value = 1
        state.window['available'] = True
        state.window_children = [child]
        self.assertFalse(state.snapshot()['window']['available'])


if __name__ == '__main__':
    unittest.main()
