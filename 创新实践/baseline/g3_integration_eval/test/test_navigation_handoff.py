"""RETURN must never race the exploration client's asynchronous cancellation."""
import json
import time
from unittest.mock import Mock

import pytest

rclpy=pytest.importorskip('rclpy')
pytest.importorskip('nav2_msgs')
from std_msgs.msg import String
from g3_integration_eval.state_machine_node import G3StateMachineNode
from g3_integration_eval.state_machine import Event, State


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args','-p','require_navigation_handoff:=true'])
    obj=G3StateMachineNode()
    obj._navigator=Mock()
    obj._state=State.EXPLORE
    yield obj
    obj.destroy_node()
    rclpy.shutdown()


def test_return_waits_for_matching_release_and_ignores_duplicate_ack(node):
    node._apply_event(Event.RETURN,source='test_battery')
    node._navigator.request_return.assert_not_called()
    token=node._handoff_pending[0]
    node._on_handoff_ack(String(data=json.dumps({'token':'old','released':True})))
    node._navigator.request_return.assert_not_called()
    ack=String(data=json.dumps({'token':token,'released':True}))
    node._on_handoff_ack(ack)
    node._navigator.request_return.assert_called_once()
    node._on_handoff_ack(ack)
    node._navigator.request_return.assert_called_once()


def test_handoff_timeout_enters_estop_without_sending_home_goal(node):
    node._apply_event(Event.RETURN,source='test_battery')
    token,trigger,_=node._handoff_pending
    node._handoff_pending=(token,trigger,time.monotonic()-1)
    node._check_handoff()
    assert node._state is State.ESTOP
    node._navigator.request_return.assert_not_called()


def test_estop_invalidates_pending_release(node):
    node._apply_event(Event.RETURN,source='test_battery')
    token=node._handoff_pending[0]
    node._apply_event(Event.ESTOP,source='test_estop')
    node._on_handoff_ack(String(data=json.dumps({'token':token,'released':True})))
    node._navigator.request_return.assert_not_called()
