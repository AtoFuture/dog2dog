"""Completion requests are session-scoped and reuse the terminal-ack RETURN handoff."""
import json
from unittest.mock import Mock
import pytest
rclpy = pytest.importorskip('rclpy')
from std_msgs.msg import String
from g3_integration_eval.state_machine_node import G3StateMachineNode
from g3_integration_eval.state_machine import Event, State


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args', '-p', 'enable_exploration_completion:=true',
                     '-p', 'require_navigation_handoff:=true'])
    n = G3StateMachineNode()
    n._navigator.request_return = Mock()
    n._navigator.cancel_current_goal = Mock(return_value=False)
    n._apply_event(Event.EXPLORE, source='test')
    yield n
    n.destroy_node()
    rclpy.shutdown()


def request(node, **kw):
    payload = dict(session_id=node._exploration_session, reason='no_frontiers', completed=True)
    payload.update(kw)
    node._on_exploration_completion(String(data=json.dumps(payload)))


def test_completion_waits_for_terminal_release_and_sends_return_once(node):
    request(node)
    assert node._state is State.RETURN and node._exploration_result['completed']
    node._navigator.request_return.assert_not_called()
    request(node)
    token = node._handoff_pending[0]
    node._on_handoff_ack(String(data=json.dumps(dict(token=token, released=True))))
    node._navigator.request_return.assert_called_once()


@pytest.mark.parametrize('kw', [dict(session_id='old'), dict(reason='unknown'),
    dict(completed=False), dict(completed=1), dict(reason=['invalid'])])
def test_invalid_completion_does_not_change_mission(node, kw):
    request(node, **kw)
    assert node._state is State.EXPLORE and node._exploration_result is None


def test_old_completion_after_new_mission_is_rejected(node):
    old = node._exploration_session
    node._apply_event(Event.STANDBY, source='test')
    node._apply_event(Event.EXPLORE, source='test')
    assert node._exploration_session != old
    request(node, session_id=old)
    assert node._state is State.EXPLORE


def test_pause_resume_preserves_session_and_ignores_completion_while_paused(node):
    token = node._exploration_session
    node._apply_event(Event.PAUSE, source='command')
    request(node)
    assert node._state is State.PAUSED and node._exploration_result is None
    node._on_handoff_ack(String(data=json.dumps(dict(token=node._handoff_pending[0], released=True))))
    node._apply_event(Event.RESUME, source='command')
    assert node._state is State.EXPLORE and node._exploration_session == token
    request(node, reason='coverage_stalled', completed=False)
    assert node._state is State.RETURN and not node._exploration_result['completed']


def test_estop_cannot_be_overridden_by_completion(node):
    node._apply_event(Event.ESTOP, source='test')
    request(node)
    assert node._state is State.ESTOP
