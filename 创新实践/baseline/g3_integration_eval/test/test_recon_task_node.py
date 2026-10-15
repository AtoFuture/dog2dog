import json
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from g3_integration_eval.recon_task_node import ReconTaskNode


def node_fixture():
    node=ReconTaskNode.__new__(ReconTaskNode)
    node.state='searching';node.mission='exploring';node.started_stamp=0
    node.required=['person'];node.minimum_goals=1;node.targets={}
    node.completed_goals=set();node.return_requested=False;node.return_attempt=None;node.history=[]
    node.saved=[];node.commands=[]
    node._save=lambda:node.saved.append(node.state)
    node.command=SimpleNamespace(publish=lambda msg:node.commands.append(msg.data))
    return node


def message(**value):return SimpleNamespace(data=json.dumps(value))


def test_only_confirmed_objective_and_goal_gate_request_return_once():
    node=node_fixture()
    node._navigation(message(status='succeeded',attempt_id=1,geodesic_dist_m=1.))
    assert not node.commands
    node.targets={'person':{'evidence_ids':[1,2]}}
    node._maybe_return();node._maybe_return()
    assert node.commands==['RETURN']
    assert node.state=='return_requested'


def test_return_success_must_match_current_attempt():
    node=node_fixture();node.return_requested=True;node.state='return_requested'
    node._return_event(message(status='succeeded',attempt_id='unrelated'))
    assert node.state=='return_requested'
    node._return_event(message(status='accepted',attempt_id='current'))
    node._return_event(message(status='succeeded',attempt_id='old'))
    assert node.state=='return_requested'
    node._return_event(message(status='succeeded',attempt_id='current'))
    assert node.state=='returned'


def test_external_battery_return_is_not_reported_as_objective_completion():
    node=node_fixture()
    node._mission(SimpleNamespace(data='returning'))
    node._return_event(message(status='succeeded',attempt_id='battery'))
    assert node.state=='interrupted' and not node.commands


def test_two_person_objective_does_not_return_after_only_one():
    node=node_fixture();node.required_person_count=2;node.completed_goals={1}
    node.targets={'person':{'class_name':'person'}}
    node._maybe_return();assert not node.commands
    node.targets['person:2']={'class_name':'person'}
    node._maybe_return();assert node.commands==['RETURN']


def test_budget_exhaustion_requests_return_once_and_reports_incomplete(monkeypatch):
    node=node_fixture();node.search_started_at=10.;node.search_timeout=20.;node.unavailable_since=None
    node.no_candidate_timeout=5.;node.return_reason=None
    monkeypatch.setattr('g3_integration_eval.recon_task_node.time.monotonic',lambda:31.)
    node._check_budget();node._check_budget()
    assert node.commands==['RETURN'] and node.return_reason=='search_budget_exhausted'
    node._return_event(message(status='accepted',attempt_id='current'))
    node._return_event(message(status='succeeded',attempt_id='current'))
    assert node.state=='returned_incomplete' and node.missing_counts()=={'person':1}


def test_no_candidates_must_persist_but_pose_failure_does_not_claim_exhausted(monkeypatch):
    node=node_fixture();node.search_started_at=0.;node.search_timeout=600.;node.unavailable_since=None
    node.no_candidate_timeout=5.;node.return_reason=None
    monkeypatch.setattr('g3_integration_eval.recon_task_node.time.monotonic',lambda:10.)
    node._navigation(message(status='search_unavailable',reason='map_or_pose_unavailable'))
    node._check_budget();assert not node.commands
    node._navigation(message(status='search_unavailable',reason='NO_CANDIDATE'))
    monkeypatch.setattr('g3_integration_eval.recon_task_node.time.monotonic',lambda:16.)
    node._check_budget();assert node.return_reason=='no_reachable_viewpoints'


def test_startup_hold_pauses_budget_and_pose_loss_resets_no_candidate_timer(monkeypatch):
    node=node_fixture();node.search_started_at=0.;node.search_timeout=1.;node.unavailable_since=0.
    node.search_budget_paused=True;node.no_candidate_timeout=5.;node.return_reason=None
    monkeypatch.setattr('g3_integration_eval.recon_task_node.time.monotonic',lambda:10.)
    node._check_budget();assert not node.commands
    node._navigation(message(status='search_unavailable',reason='NO_ROBOT_POSE'))
    assert node.unavailable_since is None
    node.search_budget_paused=False;node.search_started_at=10.;node._check_budget()
    assert not node.commands
