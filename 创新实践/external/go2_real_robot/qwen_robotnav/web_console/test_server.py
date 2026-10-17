"""Boundary checks for the local single-window relay (no simulator required)."""
import http.client
import json
from pathlib import Path
import threading
import unittest
from unittest.mock import Mock, patch
from http.server import ThreadingHTTPServer
import server


class ConsoleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.http = ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)
        cls.http.state = server.State(225, ':2')
        cls.http.state.window['available'] = True
        cls.http.focus_robot = lambda: (200, {'ok': True})
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
        self.assertFalse(data['read_only'])
        self.assertTrue(data['gui_interactive'])
        self.assertTrue(data['telemetry_read_only'])

    def test_view_only_mode_advertises_disabled_gui_input(self):
        data = server.State(225, ':2', gui_interactive=False).snapshot()
        self.assertTrue(data['read_only'])
        self.assertFalse(data['gui_interactive'])
        self.assertTrue(data['telemetry_read_only'])

    def test_commands_are_not_an_api(self):
        self.assertEqual(self.request('/api/command', 'POST')[0], 405)

    def test_ros_command_wraps_environment_inside_container(self):
        self.assertEqual(server.ros_command('/wrapper', [], ['python3', 'x.py']),
                         ['/wrapper', 'python3', 'x.py'])
        self.assertEqual(server.ros_command('/wrapper', ['A=1', 'B=2'], ['python3', 'x.py']),
                         ['/wrapper', 'env', 'A=1', 'B=2', 'python3', 'x.py'])

    def test_view_robot_requires_same_origin(self):
        self.assertEqual(self.request('/api/view/robot', 'POST')[0], 403)
        self.assertEqual(self.request('/api/view/robot', 'POST', headers={'Origin': 'https://example.com'})[0], 403)

    def test_view_robot_delegates_with_same_origin(self):
        self.http.focus_robot = lambda: (200, {'ok': True, 'model': 'robot1_my_bot'})
        status, _, body = self.request('/api/view/robot', 'POST',
                                       headers={'Origin': f'http://127.0.0.1:{self.http.server_port}'})
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(body)['ok'])

    def test_view_robot_disabled_in_view_only_mode(self):
        self.http.state.gui_interactive = False
        try:
            status, _, _ = self.request('/api/view/robot', 'POST',
                                        headers={'Origin': f'http://127.0.0.1:{self.http.server_port}'})
            self.assertEqual(status, 403)
        finally:
            self.http.state.gui_interactive = True

    def test_view_robot_requires_available_window(self):
        self.http.state.window['available'] = False
        try:
            status, _, _ = self.request('/api/view/robot', 'POST',
                                        headers={'Origin': f'http://127.0.0.1:{self.http.server_port}'})
            self.assertEqual(status, 503)
        finally:
            self.http.state.window['available'] = True

    def test_view_robot_rejects_request_body(self):
        conn = http.client.HTTPConnection('127.0.0.1', self.http.server_port, timeout=2)
        try:
            conn.request('POST', '/api/view/robot', body=b'{}',
                         headers={'Origin': f'http://127.0.0.1:{self.http.server_port}',
                                  'Content-Type': 'application/json'})
            self.assertEqual(conn.getresponse().status, 400)
        finally:
            conn.close()

    def test_focus_robot_reports_missing_model(self):
        with patch.object(server.subprocess, 'run',
                          return_value=Mock(stdout='digraph { 1 [label="world_demo (1)"]; }', stderr='')) as run:
            code, payload = server.focus_robot('/usr/bin/env', 'go2obj', 'world_demo', 'robot1_my_bot')
        self.assertEqual(code, 409)
        self.assertIn('没有找到机器狗', payload['error'])
        command = run.call_args.args[0]
        self.assertEqual(command[1:3], ['env', 'IGN_PARTITION=go2obj'])
        self.assertEqual(command[command.index('-s') + 1], '/world/world_demo/scene/graph')

    def test_focus_robot_moves_camera_for_existing_model(self):
        def reply(command, **kwargs):
            service = command[command.index('-s') + 1]
            if service == '/world/world_demo/scene/graph':
                return Mock(stdout='1 [label="world_demo (1)"];\n7 [label="robot1_my_bot (7)"];', stderr='')
            if service == '/gui/move_to':
                return Mock(stdout='data: true', stderr='')
            raise AssertionError(service)
        with patch.object(server.subprocess, 'run', side_effect=reply):
            code, payload = server.focus_robot('/usr/bin/env', 'go2obj', 'world_demo', 'robot1_my_bot')
        self.assertEqual(code, 200)
        self.assertEqual(payload, {'ok': True, 'model': 'robot1_my_bot'})

    def test_focus_robot_rejects_unconfirmed_camera_move(self):
        def reply(command, **kwargs):
            if command[command.index('-s') + 1] == '/world/world_demo/scene/graph':
                return Mock(stdout='7 [label="robot1_my_bot (7)"];', stderr='')
            return Mock(stdout='data: false', stderr='')
        with patch.object(server.subprocess, 'run', side_effect=reply):
            code, payload = server.focus_robot('/usr/bin/env', 'go2obj', 'world_demo', 'robot1_my_bot')
        self.assertEqual(code, 502)
        self.assertIn('未确认', payload['error'])

    def test_focus_robot_reports_service_timeout(self):
        with patch.object(server.subprocess, 'run',
                          return_value=Mock(stdout='', stderr='Service call timed out')):
            with self.assertRaises(OSError):
                server.focus_robot('/usr/bin/env', 'go2obj', 'world_demo', 'robot1_my_bot')

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
