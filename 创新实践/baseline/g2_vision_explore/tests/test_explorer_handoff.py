"""Asynchronous cancellation must release ownership only after a terminal result."""
from concurrent.futures import Future
from types import SimpleNamespace
import json
import time

import pytest
pytest.importorskip('nav2_msgs')
from g2_core.explorer import Goal
from g2_core.state_machine import ExplorerStateMachine, Phase
from vision_explorer.explorer_node import VisionExplorer


def ready(value):
    f=Future();f.set_result(value);return f


def fixture_node():
    node=VisionExplorer.__new__(VisionExplorer)
    node._attempt={'id':1,'goal':Goal(1,2,0,1,1,1,.5),'handle':None,'cancel':False,
                   'cancel_sent':False,'started':time.monotonic(),'reason':None}
    node._goal=node._attempt['goal'];node._goal_handle=None
    node._sm=ExplorerStateMachine(phase=Phase.NAVIGATING)
    node._handoff_token=None;node._mission_state='exploring';node._stopped=False
    node.events=[];node.acks=[];node.results=[]
    node._event=lambda status,*a,**kw:node.events.append(status)
    node._handoff_ack=SimpleNamespace(publish=lambda msg:node.acks.append(json.loads(msg.data)))
    node._selector=SimpleNamespace(on_result=lambda *a:node.results.append(a),mark_visited=lambda *a:None)
    node._now=lambda:10
    node._p=lambda:{'max_nav_timeout_s':120,'map_frame':'map','robot_frame':'base_link'}
    return node


def test_handoff_cancels_late_acceptance_and_waits_for_terminal_result():
    node=fixture_node();attempt=node._attempt;result=Future();calls=[]
    node._on_handoff(SimpleNamespace(data='{"token":"return-A"}'))
    node._watch_navigation()
    assert not node.acks
    handle=SimpleNamespace(accepted=True,get_result_async=lambda:result,
        cancel_goal_async=lambda:calls.append(1) or ready(SimpleNamespace(goals_canceling=[1])))
    node._on_goal_accepted(attempt,ready(handle))
    assert calls==[1]
    node._watch_navigation()
    assert not node.acks
    result.set_result(SimpleNamespace(status=5))
    node._watch_navigation()
    assert node.acks==[{'token':'return-A','released':True}]
    assert not node._sm.should_explore
    assert not node.results  # mission cancellation is not a failed frontier


def test_old_result_cannot_clear_new_attempt():
    node=fixture_node();old=dict(node._attempt);old['id']=0;current=node._attempt
    node._on_goal_done(old,ready(SimpleNamespace(status=5)))
    assert node._attempt is current


def test_cancel_acceptance_does_not_release_action_ownership():
    node=fixture_node();attempt=node._attempt
    attempt['handle']=SimpleNamespace(cancel_goal_async=lambda:ready(SimpleNamespace(goals_canceling=[1])))
    node._cancel_goal(reason='timeout');node._cancel_goal(reason='timeout')
    assert node._attempt is attempt
    assert node.events.count('cancel_requested')==1


def test_missing_tf_does_not_fall_back_to_map_origin():
    node=fixture_node()
    node._tf=SimpleNamespace(lookup_transform=lambda *a:(_ for _ in ()).throw(RuntimeError('no transform')))
    assert node._robot_xy() is None
