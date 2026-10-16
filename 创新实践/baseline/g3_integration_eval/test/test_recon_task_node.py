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


def test_moving_person_changed_track_does_not_complete_two_person_objective():
    from g3_integration_eval.moving_identity import MovingIdentityResolver
    node=node_fixture();node.required_person_count=2;node.completed_goals={1}
    node.identity_policy='motion_appearance_v1';node.identity_resolver=MovingIdentityResolver()
    observations=[dict(evidence_id=i+1,stamp_ns=i*10**9,track_id=i//3+1,
        class_name='person',confidence=.9,source='coco',frame_id='map',position_m=[i*.4,0,.8],
        snapshot_status='available',snapshot_file=f'{i}.jpg') for i in range(10)]
    node._evidence(message(observations=observations,ready_evidence_ids=list(range(1,11))))
    assert len(node.targets)==1 and not node.commands and node.missing_counts()=={'person':1}


def test_moving_two_people_still_need_navigation_before_exactly_one_return():
    from g3_integration_eval.moving_identity import MovingIdentityResolver
    node=node_fixture();node.required_person_count=2
    node.identity_policy='motion_appearance_v1';node.identity_resolver=MovingIdentityResolver()
    observations=[dict(evidence_id=2*i+p+1,stamp_ns=i*10**9,track_id=p+1,
        class_name='person',confidence=.9,source='coco',frame_id='map',position_m=[i*.3,p*1.5,.8],
        snapshot_status='available',snapshot_file=f'{i}-{p}.jpg') for i in range(4) for p in range(2)]
    node._evidence(message(observations=observations,ready_evidence_ids=list(range(1,9))))
    assert len(node.targets)==2 and not node.commands
    node._navigation(message(status='succeeded',attempt_id=1,geodesic_dist_m=1.))
    node._navigation(message(status='succeeded',attempt_id=1,geodesic_dist_m=1.))
    assert node.commands==['RETURN'] and node.return_reason=='objective_completed'


def test_minimum_observation_time_delays_only_successful_completion():
    node=node_fixture();node.required_person_count=2;node.completed_goals={1}
    node.targets={f'person:{i}':{'class_name':'person'} for i in (1,2)}
    node.minimum_observation_duration_s=50.
    now=SimpleNamespace(nanoseconds=49_000_000_000)
    node.get_clock=lambda:SimpleNamespace(now=lambda:now)
    node._maybe_return();assert not node.commands
    now.nanoseconds=50_000_000_000
    node._maybe_return();node._maybe_return()
    assert node.commands==['RETURN'] and node.return_reason=='objective_completed'


def test_observation_minimum_cannot_delay_budget_or_safety_return(monkeypatch):
    node=node_fixture();node.minimum_observation_duration_s=50.
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_000_000_000))
    node.search_started_at=0.;node.search_timeout=10.;node.unavailable_since=None;node.no_candidate_timeout=5.
    monkeypatch.setattr('g3_integration_eval.recon_task_node.time.monotonic',lambda:11.)
    node._check_budget()
    assert node.commands==['RETURN'] and node.return_reason=='search_budget_exhausted'


def test_old_confirmed_people_cannot_satisfy_recent_completion_gate():
    node=node_fixture();node.required_person_count=2;node.required_target_recency_s=5.
    node.completed_goals={1};node.targets={f'person:{i}':{'class_name':'person','last_seen_stamp_ns':10_000_000_000} for i in (1,2)}
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=20_000_000_000))
    node._maybe_return();assert not node.commands
    node.targets['person:1']['last_seen_stamp_ns']=19_000_000_000
    node._maybe_return();assert not node.commands
    node.targets['person:2']['last_seen_stamp_ns']=19_000_000_000
    node._maybe_return();assert node.commands==['RETURN']


def test_short_views_count_separately_without_replacing_required_long_goal():
    node=node_fixture();node.completed_views=set();node.minimum_observation_viewpoints=3
    node.targets={'person':{'class_name':'person'}}
    node._navigation(message(status='succeeded',attempt_id=1,purpose='observe_targets',geodesic_dist_m=.55))
    node._navigation(message(status='succeeded',attempt_id=1,purpose='observe_targets',geodesic_dist_m=.55))
    assert len(node.completed_views)==1 and not node.completed_goals and not node.commands
    node._navigation(message(status='succeeded',attempt_id=2,purpose='observe_targets',geodesic_dist_m=.8))
    assert len(node.completed_views)==2 and len(node.completed_goals)==1 and not node.commands
    node._navigation(message(status='succeeded',attempt_id=3,purpose='observe_targets',geodesic_dist_m=.5))
    assert node.commands==['RETURN']


def test_pause_resume_preserves_progress_and_excludes_paused_budget(monkeypatch):
    node=node_fixture();node.search_started_at=10.;node.unavailable_since=11.
    node.targets={'person':{'class_name':'person'}};node.completed_goals={'completed'}
    node.completed_views={'view'};node.search_timeout=20.;node.no_candidate_timeout=5.
    clock=[12.];ros=[12*10**9]
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=ros[0]))
    monkeypatch.setattr('g3_integration_eval.recon_task_node.time.monotonic',lambda:clock[0])
    node._mission(SimpleNamespace(data='paused'))
    clock[0]=112.;ros[0]=112*10**9
    node._check_budget()
    assert node.state=='paused' and not node.commands
    node._mission(SimpleNamespace(data='exploring'))
    assert node.state=='searching' and node.started_stamp==0
    assert node.search_started_at==110. and node.unavailable_since is None
    assert node.paused_duration_s==100. and node.paused_sim_duration_s==100.
    assert node.targets=={'person':{'class_name':'person'}}
    assert node.completed_goals=={'completed'} and node.completed_views=={'view'}
    node.minimum_observation_duration_s=15.
    assert not node._observation_duration_complete()
    ros[0]=115*10**9
    assert node._observation_duration_complete()


def test_recovery_timeline_is_saved_and_bad_messages_ignored():
    node=node_fixture();node.recovery_history=[]
    node._recovery(SimpleNamespace(data='[]'))
    node._recovery(SimpleNamespace(data='broken'))
    node._recovery(message(status='unknown'))
    assert not node.recovery_history
    node._recovery(message(status='fault',missing=['scan'],stamp_ns=123))
    node._recovery(message(status='resumed',stamp_ns=456))
    assert [e['status'] for e in node.recovery_history]==['fault','resumed']
    assert len(node.saved)==2
