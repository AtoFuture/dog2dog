"""Delayed model decisions must never escape mission and current-map guards."""
import json
import time
from types import SimpleNamespace

import pytest
pytest.importorskip('nav2_msgs')
from g2_core.explorer import Goal
from g2_core.state_machine import ExplorerStateMachine, Phase
from vision_explorer.explorer_node import VisionExplorer


def fixture_node():
    node = VisionExplorer.__new__(VisionExplorer)
    node._sm = ExplorerStateMachine(phase=Phase.SENDING)
    node._stopped = False
    node._handoff_token = None
    node._attempt = None
    node._brain_pending = dict(id='current', deadline=time.monotonic()+10,
                              robot_xy=(0, 0), goals={'f0': Goal(1,0,0,1,1,1,.5)})
    node._map_pose_healthy = lambda xy: True
    node._grid = object()
    node._grid_stamp = time.monotonic()
    node._robot_xy = lambda: (0, 0)
    node._p = lambda: {'map_timeout_s': 20}
    node._now = lambda: 1
    node._selector = SimpleNamespace(is_goal_valid=lambda *args: True)
    node.events = []
    node.sent = []
    node._event = lambda status, **kw: node.events.append((status, kw))
    node._send_goal = lambda goal, now: node.sent.append(goal)
    return node


def decision(request_id='current', candidate_id='f0'):
    return SimpleNamespace(data=json.dumps(dict(request_id=request_id, action='SELECT_FRONTIER',
                          candidate_id=candidate_id, fallback=False)))


def test_old_request_id_is_ignored_and_does_not_consume_current_request():
    node = fixture_node()
    node._on_brain_decision(decision('previous'))
    assert node._brain_pending is not None and not node.sent
    node._on_brain_decision(decision())
    assert len(node.sent) == 1 and node._brain_pending is None


def test_return_during_model_request_discards_late_reply():
    node = fixture_node()
    node._sm.on_mission_state('returning')
    node._on_brain_decision(decision())
    assert not node.sent and node._sm.phase is Phase.IDLE


def test_latest_map_rejects_previously_safe_candidate():
    node = fixture_node()
    node._selector.is_goal_valid = lambda *args: False
    node._on_brain_decision(decision())
    assert not node.sent and node._sm.phase is Phase.IDLE


def test_deadline_uses_local_validated_fallback_and_ignores_late_model():
    node = fixture_node()
    node._brain_pending['deadline'] = time.monotonic()-1
    node._on_brain_decision(decision())
    node._on_brain_decision(decision())
    assert len(node.sent) == 1
    assert node.events[0][1]['source'] == 'deadline'


def test_invented_candidate_cannot_create_coordinates():
    node = fixture_node()
    node._on_brain_decision(decision(candidate_id='invented'))
    assert node.sent == [Goal(1,0,0,1,1,1,.5)]
    assert node.events[0][1]['fallback']


def test_viewpoint_survives_short_map_flicker_but_safety_is_rechecked(monkeypatch):
    node=fixture_node();goal=Goal(2,0,0,1,1,2,.5,purpose='observe_occlusion')
    node._p=lambda:{'active_observation':True,'brain_max_candidates':6}
    node._recon_status={'state':'searching','missing_counts':{'person':1},'targets':{}}
    node._recon_status_at=time.monotonic()
    node._selector._expire=lambda now:None;node._selector._visits=[]
    node._selector.params=SimpleNamespace(safety_radius_m=.45)
    batches=iter([(goal,),(),(),()])
    monkeypatch.setattr('vision_explorer.explorer_node.observation_candidates',lambda *a,**kw:next(batches))
    assert node._observation_goals((0,0),1)==(goal,)
    assert node._observation_goals((0,0),1)==(goal,)
    node._selector.is_goal_valid=lambda *args:False
    assert not node._observation_goals((0,0),1)
    node._selector.is_goal_valid=lambda *args:True
    at=time.monotonic()+9.;node._recon_status_at=at
    monkeypatch.setattr('vision_explorer.explorer_node.time.monotonic',lambda:at)
    assert not node._observation_goals((0,0),1)


def test_recon_status_rejects_invalid_counts_and_stale_observation_context():
    node=fixture_node();node._recon_status=None
    node._on_recon_status(SimpleNamespace(data='{"missing_counts":{"person":"unknown"}}'))
    assert node._recon_status is None
    node._p=lambda:{'active_observation':True}
    node._recon_status={'state':'searching','missing_counts':{'person':1}}
    node._recon_status_at=time.monotonic()-6
    assert not node._observation_goals((0,0),1)
