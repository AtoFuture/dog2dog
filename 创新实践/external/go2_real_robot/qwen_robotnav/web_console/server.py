"""Local, single-window Gazebo relay with read-only ROS telemetry."""
import argparse
import json
import os
from pathlib import Path
import re
import select
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parent
MIME = {'.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.svg': 'image/svg+xml'}


def static_file(path):
    path = unquote(urlsplit(path).path)
    base = ROOT / ('node_modules/@novnc/novnc' if path.startswith('/vendor/novnc/') else 'public')
    relative = path.removeprefix('/vendor/novnc/') if path.startswith('/vendor/novnc/') else path.lstrip('/') or 'index.html'
    result = (base / relative).resolve()
    if not result.is_relative_to(base.resolve()) or result.suffix not in MIME or not result.is_file():
        return None
    return result


def same_origin(headers):
    origin = headers.get('Origin')
    parsed = urlsplit(origin or '')
    return bool(origin and parsed.scheme in ('http', 'https') and parsed.netloc == headers.get('Host'))


def find_window(display, explicit=None):
    env = dict(os.environ, DISPLAY=display, XAUTHORITY=os.environ.get('XAUTHORITY', str(Path.home() / '.Xauthority')))
    command = ['xwininfo', '-id', explicit] if explicit else ['xwininfo', '-root', '-tree']
    text = subprocess.check_output(command, env=env, text=True, stderr=subprocess.DEVNULL, timeout=5)
    if explicit:
        if not re.search(r'Window id: (0x[0-9a-f]+) "Gazebo"', text):
            raise ValueError('指定窗口不是 Gazebo，拒绝转发其他应用')
        return explicit, env
    ids = re.findall(r'(0x[0-9a-f]+)\s+"Gazebo":', text)
    if len(ids) != 1:
        raise ValueError('需要恰好一个 Gazebo 窗口；有多个时请指定 --window-id')
    return ids[0], env


def fit_window(window, env):
    """XGetImage fails when a drawable extends beyond the X screen."""
    def info(*args):
        return subprocess.check_output(['xwininfo', *args], env=env, text=True, stderr=subprocess.DEVNULL, timeout=5)
    text, root = info('-id', window), info('-root')
    def value(source, name):
        return int(re.search(r'^\s*' + re.escape(name) + r':\s*(-?\d+)', source, re.M)[1])
    if 'Map State: IsViewable' not in text:
        raise ValueError('Gazebo 窗口未显示，请先取消最小化')
    x, y = value(text, 'Absolute upper-left X'), value(text, 'Absolute upper-left Y')
    w, h, rw, rh = value(text, 'Width'), value(text, 'Height'), value(root, 'Width'), value(root, 'Height')
    if x < 0 or y < 0 or x+w > rw or y+h > rh:
        subprocess.run(['xdotool', 'windowsize', window, str(min(w, rw-40)), str(min(h, rh-100))], env=env, check=True, timeout=5)
        subprocess.run(['xdotool', 'windowmove', window, '20', '40'], env=env, check=True, timeout=5)
        time.sleep(.2)


class State:
    def __init__(self, domain, display):
        self.lock = threading.Lock()
        self.data = {}
        self.received = None
        self.window = {'available': False, 'message': '正在连接 Gazebo 窗口', 'display': display}
        self.domain = domain
        self.ros_process = None
        self.window_children = []

    def snapshot(self):
        with self.lock:
            age = time.monotonic() - self.received if self.received is not None else None
            window = dict(self.window)
            if self.window_children and any(c.poll() is not None for c in self.window_children):
                window.update(available=False, message='窗口转发进程已退出，请检查日志并重新启动控制台')
            return {'window': window, 'domain': self.domain, 'telemetry': self.data,
                    'bridge_age_s': age, 'bridge_alive': self.ros_process is not None and self.ros_process.poll() is None,
                    'read_only': True}

    def consume(self, child):
        for line in child.stdout:
            try:
                data = json.loads(line)
                if not isinstance(data, dict) or data.get('type') != 'telemetry':
                    continue
                with self.lock:
                    self.data = data
                    self.received = time.monotonic()
            except (ValueError, TypeError):
                continue


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def reply(self, code, body, content_type='application/json'):
        self.send_response(code)
        self.send_header('Content-Type', content_type + '; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' data: blob:; style-src 'self'; connect-src 'self'; frame-ancestors 'self'")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        try:
            host = urlsplit('//' + self.headers.get('Host', '')).hostname
        except ValueError:
            host = None
        if host not in ('127.0.0.1', 'localhost', '::1'):
            return self.reply(403, b'{"error":"loopback host required"}')
        path = urlsplit(self.path).path
        if path == '/vnc':
            if not self.server.state.snapshot()['window']['available']:
                return self.reply(503, b'{"error":"window relay unavailable"}')
            if not same_origin(self.headers):
                return self.reply(403, b'{"error":"same-origin connection required"}')
            if self.headers.get('Upgrade', '').lower() != 'websocket':
                return self.reply(426, b'{"error":"WebSocket required"}')
            return self.tunnel()
        if path == '/api/state':
            return self.reply(200, json.dumps(self.server.state.snapshot(), allow_nan=False).encode())
        target = static_file(self.path)
        if target:
            return self.reply(200, target.read_bytes(), MIME[target.suffix])
        self.reply(404, b'{"error":"not found"}')

    def do_POST(self):
        self.close_connection = True
        self.reply(405, b'{"error":"read-only console"}')

    def tunnel(self):
        # Keep one public HTTP origin; the private WebSocket proxy owns RFB framing.
        self.close_connection = True
        try:
            with socket.create_connection(('127.0.0.1', self.server.ws_port), timeout=3) as upstream:
                headers = ''.join(f'{key}: {value}\r\n' for key, value in self.headers.items())
                upstream.sendall(('GET /websockify HTTP/1.1\r\n' + headers + '\r\n').encode('latin-1'))
                self.connection.settimeout(10)
                upstream.settimeout(10)
                while not self.server.stopping.is_set():
                    ready, _, _ = select.select([self.connection, upstream], [], [], 1)
                    for source in ready:
                        block = source.recv(65536)
                        if not block:
                            return
                        (upstream if source is self.connection else self.connection).sendall(block)
        except (OSError, ValueError):
            return

    def log_message(self, fmt, *args):
        if '/api/state' not in self.path:
            super().log_message(fmt, *args)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--vnc-port', type=int, default=5909)
    parser.add_argument('--ws-port', type=int, default=8766)
    parser.add_argument('--display', default=':2')
    parser.add_argument('--window-id')
    parser.add_argument('--domain', type=int, default=225)
    parser.add_argument('--profile', type=Path, default=ROOT.parent / 'config/sim_runtime.yaml')
    parser.add_argument('--ros-wrapper', type=Path, default=Path('/home/wy/.enroot/ros'))
    parser.add_argument('--no-ros', action='store_true')
    parser.add_argument('--no-window', action='store_true')
    parser.add_argument('--log-dir', type=Path, default=Path('/tmp/go2-web-console-' + str(os.getuid())))
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or any(not 1024 <= p <= 65535 for p in (args.port, args.ws_port, args.vnc_port)) or len({args.port, args.ws_port, args.vnc_port}) != 3:
        parser.error('Require a valid ROS domain and three distinct unprivileged ports')
    # Bind before spawning children, so a second launch cannot duplicate relays.
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.daemon_threads = True
    server.state = state = State(args.domain, args.display)
    server.ws_port = args.ws_port
    server.stopping = threading.Event()
    children, streams = [], []
    args.log_dir.mkdir(parents=True, exist_ok=True)

    def start(command, name, env=None, pipe=False):
        log = (args.log_dir / (name + '.log')).open('w')
        streams.append(log)
        child = subprocess.Popen(command, env=env, stdout=subprocess.PIPE if pipe else log,
                                 stderr=log, text=True, start_new_session=True)
        children.append(child)
        return child

    try:
        if not args.no_window:
            try:
                window, env = find_window(args.display, args.window_id)
                fit_window(window, env)
                for port in (args.vnc_port, args.ws_port):
                    with socket.socket() as probe:
                        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                        probe.bind(('127.0.0.1', port))
                # Never export the root desktop or input events; only the Gazebo drawable.
                vnc = start(['x11vnc', '-display', args.display, '-auth', env['XAUTHORITY'], '-id', window,
                             '-localhost', '-rfbport', str(args.vnc_port), '-forever', '-shared', '-nopw',
                             '-noshm', '-noxdamage', '-noscr', '-viewonly', '-quiet'], 'window', env)
                ws = start([sys.executable, '-m', 'websockify', '--heartbeat', '15',
                            '127.0.0.1:' + str(args.ws_port), '127.0.0.1:' + str(args.vnc_port)], 'websocket')
                state.window_children = [vnc, ws]
                time.sleep(.5)
                state.window = {'available': vnc.poll() is None and ws.poll() is None,
                                'message': '当前 Gazebo 窗口 · 只读镜像', 'id': window, 'display': args.display}
            except (OSError, ValueError, subprocess.SubprocessError) as exc:
                state.window = {'available': False, 'message': str(exc), 'display': args.display}
        else:
            state.window['message'] = '未启用窗口转发'
        if not args.no_ros:
            state.ros_process = start([str(args.ros_wrapper), 'python3', str(ROOT / 'ros_telemetry.py'),
                                       '--domain', str(args.domain), '--profile', str(args.profile.resolve())], 'ros', pipe=True)
            threading.Thread(target=state.consume, args=(state.ros_process,), daemon=True).start()
        print(f'Go2 console: http://127.0.0.1:{args.port} · ROS domain {args.domain} · read only', flush=True)
        def stop(*_):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, stop)
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.stopping.set()
        server.server_close()
        for child in reversed(children):
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5)
        for stream in streams:
            stream.close()


if __name__ == '__main__':
    main()
