"""Mission commands require fresh state and a matching G3 receipt."""
import unittest
from task_control import TaskControl


def snapshot(mission='standby', age=0., alive=True):
    return {'bridge_alive': alive, 'bridge_age_s': .1,
            'telemetry': {'values': {'mission': mission}, 'ages': {'mission': age}}}


class TaskTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.control = TaskControl(True, clock=lambda: self.now)
        self.sent = []

    def submit(self, command, state=None):
        return self.control.submit(command, state or snapshot(), self.sent.append)

    def test_sent_is_not_accepted_without_receipt(self):
        code, record = self.submit('EXPLORE')
        self.assertEqual(code, 202)
        self.assertEqual(record['status'], 'sent')
        self.assertEqual(self.sent[0]['request_id'], record['request_id'])

    def test_receipt_must_match_id_and_command(self):
        _, record = self.submit('EXPLORE')
        self.control.observe([dict(request_id='other', command='EXPLORE', accepted=True),
                              dict(request_id=record['request_id'], command='ESTOP', accepted=True)])
        self.assertEqual(self.control.snapshot(snapshot())['records'][0]['status'], 'sent')
        self.control.observe([dict(record, accepted=True, state='exploring')])
        self.assertEqual(self.control.snapshot(snapshot())['records'][0]['status'], 'accepted')

    def test_rejection_reason_is_preserved(self):
        _, record = self.submit('EXPLORE')
        self.control.observe([dict(record, accepted=False, reason='health not stable')])
        result = self.control.snapshot(snapshot())['records'][0]
        self.assertEqual(result['status'], 'rejected')
        self.assertEqual(result['reason'], 'health not stable')

    def test_timeout_is_unconfirmed_and_never_republished(self):
        self.submit('EXPLORE')
        self.now += 13
        self.assertEqual(self.control.snapshot(snapshot())['records'][0]['status'], 'unconfirmed')
        self.assertEqual(len(self.sent), 1)

    def test_fresh_state_and_live_bridge_are_required(self):
        for state in [snapshot(age=4), snapshot(alive=False)]:
            self.assertNotEqual(self.submit('EXPLORE', state)[0], 202)
        self.assertEqual(self.sent, [])

    def test_state_matrix_and_invalid_command(self):
        self.assertEqual(self.submit('RETURN')[0], 409)
        self.assertEqual(self.submit('cmd_vel')[0], 400)
        self.assertEqual(self.sent, [])

    def test_estop_can_preempt_pending_and_stale_mission(self):
        self.submit('EXPLORE')
        self.assertEqual(self.submit('STANDBY', snapshot('exploring'))[0], 409)
        self.assertEqual(self.submit('ESTOP', snapshot(age=100))[0], 202)
        self.assertEqual([r['command'] for r in self.sent], ['EXPLORE', 'ESTOP'])

    def test_disabled_control_never_sends(self):
        self.control.enabled = False
        self.assertEqual(self.submit('ESTOP')[0], 403)
        self.assertEqual(self.control.snapshot(snapshot())['allowed'], [])

    def test_pipe_failure_never_records_sent(self):
        def broken(_):
            raise BrokenPipeError()
        self.assertEqual(self.control.submit('EXPLORE', snapshot(), broken)[0], 503)
        self.assertEqual(self.control.snapshot(snapshot())['records'], [])


if __name__ == '__main__':
    unittest.main()
