from concurrent.futures import Future
from types import SimpleNamespace
import pytest

pytest.importorskip('nav2_msgs')
from qwen_robotnav_ros.nav2_waypoint_bridge import Nav2WaypointBridge


def ready_future(value):
    f = Future()
    f.set_result(value)
    return f


def fixture_node(attempt):
    events = []
    obj = SimpleNamespace(_attempt=attempt, _pending=None, _allowed=True, _handoff_token=None,
                          _event=lambda status,*args,**kwargs:events.append(status),events=events)
    obj._on_result = lambda a,f: Nav2WaypointBridge._on_result(obj,a,f)
    obj._on_cancel = lambda a,f: Nav2WaypointBridge._on_cancel(obj,a,f)
    obj._cancel_handle = lambda a: Nav2WaypointBridge._cancel_handle(obj,a)
    obj.cancel_navigation = lambda reason: Nav2WaypointBridge.cancel_navigation(obj,reason)
    return obj


def test_estop_during_goal_acceptance_cancels_late_handle():
    attempt = {'id':1,'cancel':False,'handle':None,'target':{}}
    node = fixture_node(attempt)
    result = Future()
    calls = []
    handle = SimpleNamespace(accepted=True,get_result_async=lambda:result,
                             cancel_goal_async=lambda:calls.append('cancel') or ready_future(SimpleNamespace(goals_canceling=[1])))
    Nav2WaypointBridge._on_mission(node,SimpleNamespace(data='estop'))
    Nav2WaypointBridge._on_response(node,attempt,ready_future(handle))
    assert not node._allowed
    assert calls == ['cancel']
    assert node._attempt is attempt
    result.set_result(SimpleNamespace(status=5))
    assert node._attempt is None
    assert 'canceled' in node.events


def test_cancel_is_idempotent_and_waits_for_terminal_result():
    calls=[]
    handle=SimpleNamespace(cancel_goal_async=lambda:calls.append(1) or ready_future(SimpleNamespace(goals_canceling=[1])))
    attempt={'id':1,'cancel':False,'handle':handle,'target':{}}
    node=fixture_node(attempt)
    node.cancel_navigation('vision_timeout')
    node.cancel_navigation('estop')
    assert calls == [1]
    assert node._attempt is attempt
    Nav2WaypointBridge._on_result(node,attempt,ready_future(SimpleNamespace(status=4)))
    assert node._attempt is None


def test_old_result_cannot_clear_a_new_attempt():
    old={'id':1,'target':{}}
    current={'id':2,'target':{}}
    node=fixture_node(current)
    Nav2WaypointBridge._on_result(node,old,ready_future(SimpleNamespace(status=5)))
    assert node._attempt is current


def test_shutdown_drops_queued_prediction_and_blocks_new_goals():
    node=fixture_node(None)
    node._pending=object()
    Nav2WaypointBridge.stop_navigation(node)
    assert node._pending is None
    assert not node._allowed


def test_handoff_waits_for_late_goal_acceptance_and_terminal_result():
    import json
    attempt={'id':1,'cancel':False,'handle':None,'target':{}}
    node=fixture_node(attempt)
    node.stop_navigation=lambda reason:Nav2WaypointBridge.stop_navigation(node,reason)
    node._warn=lambda message:None
    node._handoff_ack=SimpleNamespace(publish=lambda message:node.acks.append(json.loads(message.data)))
    node.acks=[]
    node._input_timeout=10
    node._goal_timeout=100
    node._last_input=None
    attempt['started']=__import__('time').monotonic()
    Nav2WaypointBridge._on_handoff(node,SimpleNamespace(data='{"token":"return-123"}'))
    Nav2WaypointBridge._tick(node)
    assert node.acks==[]
    result=Future()
    calls=[]
    handle=SimpleNamespace(accepted=True,get_result_async=lambda:result,
        cancel_goal_async=lambda:calls.append(1) or ready_future(SimpleNamespace(goals_canceling=[1])))
    Nav2WaypointBridge._on_response(node,attempt,ready_future(handle))
    assert calls==[1]
    Nav2WaypointBridge._tick(node)
    assert node.acks==[]
    result.set_result(SimpleNamespace(status=5))
    Nav2WaypointBridge._tick(node)
    assert node.acks==[{'token':'return-123','released':True}]
    assert not node._allowed
