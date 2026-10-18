"""Track bounded, correlated G3 command receipts; never publish actuator commands."""
import threading
import time
import uuid

COMMANDS = {'EXPLORE', 'PAUSE', 'RESUME', 'RETURN', 'ESTOP', 'RESET', 'STANDBY'}
ALLOWED = {
    'standby': ['EXPLORE', 'ESTOP'],
    'exploring': ['PAUSE', 'RETURN', 'STANDBY', 'ESTOP'],
    'paused': ['RESUME', 'RETURN', 'STANDBY', 'ESTOP'],
    'returning': ['STANDBY', 'ESTOP'],
    'estop': ['RESET', 'ESTOP'],
}


class TaskControl:
    def __init__(self, enabled=False, clock=time.monotonic):
        self.enabled, self.clock = enabled, clock
        self.lock = threading.Lock()
        self.records = []

    def _expire(self):
        for record in self.records:
            if record['status'] == 'sent' and self.clock() - record['_sent'] > 12:
                record.update(status='unconfirmed', reason='未收到 G3 回执，请检查状态；不会自动重发。')

    def snapshot(self, state):
        data = state.get('telemetry', {})
        age = (state.get('bridge_age_s') or 0) + data.get('ages', {}).get('mission', float('inf'))
        ready = state.get('bridge_alive') and state.get('bridge_age_s') is not None and age < 3
        mission = data.get('values', {}).get('mission') if ready else None
        with self.lock:
            self._expire()
            pending = any(r['status'] == 'sent' for r in self.records)
            allowed = ALLOWED.get(mission, []) if self.enabled else []
            if pending:
                allowed = [c for c in allowed if c == 'ESTOP']
            # A stop remains available with stale mission telemetry if the bridge is alive.
            if self.enabled and state.get('bridge_alive') and 'ESTOP' not in allowed:
                allowed = [*allowed, 'ESTOP']
            return {'enabled': self.enabled, 'mission_fresh': bool(ready), 'allowed': allowed,
                    'records': [{k: v for k, v in r.items() if not k.startswith('_')} for r in self.records]}

    def submit(self, command, state, send):
        if command not in COMMANDS:
            return 400, {'error': '不支持的任务指令。'}
        if not self.enabled:
            return 403, {'error': '任务控制未启用。'}
        # Serialize check + send across HTTP clients; ESTOP can preempt an outstanding request.
        with self.lock:
            self._expire()
            data = state.get('telemetry', {})
            age = (state.get('bridge_age_s') or 0) + data.get('ages', {}).get('mission', float('inf'))
            fresh = state.get('bridge_age_s') is not None and age < 3
            mission = data.get('values', {}).get('mission') if fresh else None
            if not state.get('bridge_alive'):
                return 503, {'error': 'ROS 连接已断开。'}
            if command != 'ESTOP' and command not in ALLOWED.get(mission, []):
                return 409, {'error': '当前状态不允许此操作，或任务状态已过期。'}
            if command != 'ESTOP' and any(r['status'] == 'sent' for r in self.records):
                return 409, {'error': '正在等待上一条指令回执。'}
            request = {'request_id': uuid.uuid4().hex, 'command': command}
            try:
                send(request)
            except (OSError, ValueError):
                return 503, {'error': '指令未发送，ROS 连接不可用。'}
            record = dict(request, status='sent', reason='', _sent=self.clock())
            self.records = [*self.records[-19:], record]
            return 202, {k: v for k, v in record.items() if not k.startswith('_')}

    def observe(self, receipts):
        with self.lock:
            for receipt in receipts:
                for record in self.records:
                    if (receipt.get('request_id') == record['request_id']
                            and receipt.get('command') == record['command']
                            and isinstance(receipt.get('accepted'), bool)):
                        record.update(status='accepted' if receipt['accepted'] else 'rejected',
                                      state=receipt.get('state'), reason=receipt.get('reason', ''))
