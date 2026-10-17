"""Real explorer node: termination stops new goals while cancellation retains ownership."""
import json
import time
from types import SimpleNamespace
import numpy as np
import pytest
rclpy = pytest.importorskip('rclpy')
from std_msgs.msg import String
from g2_core.explorer import Goal
from g2_core.grid import GridMap
from g2_core.mission_progress import MissionProgress, ProgressLimits
from g2_core.state_machine import Phase
from vision_explorer.explorer_node import VisionExplorer


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args','-p','task_completion_enabled:=true','-p','require_mission_state:=true'])
    n = VisionExplorer()
    n._on_mission_state(String(data='exploring'))
    n._on_exploration_session(String(data=json.dumps(dict(session_id='a'*32, state='exploring'))))
    n._grid = GridMap(np.zeros((40,40),dtype=np.int8), .1)
    n._grid_stamp = time.monotonic()
    n._robot_xy = lambda: (1.,1.)
    n.events=[]
    n._event=lambda status,*a,**kw:n.events.append((status,kw))
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_time_budget_cancels_existing_goal_without_early_release(node):
    node._progress.elapsed=901
    node._sm.phase=Phase.NAVIGATING
    attempt=dict(id=1,goal=Goal(2,2,0,1,1,1,.5),handle=None,cancel=False,
                 cancel_sent=False,started=time.monotonic(),reason=None)
    node._attempt=attempt
    node._watch_navigation()
    assert node._completion_pending['reason']=='time_budget'
    assert not node._completion_pending['completed']
    assert node._attempt is attempt and attempt['cancel']
    assert not node._sm.should_explore
    node._on_mission_state(String(data='exploring'))  # late heartbeat cannot revive
    assert not node._sm.should_explore
    node._on_handoff(String(data='{"token":"return-test"}'))
    node._watch_navigation()
    assert node._handoff_token=='return-test'
    node._finish_attempt(attempt,'canceled')
    node._watch_navigation()
    assert node._handoff_token is None and node._progress.failed_goals==0


def test_no_frontier_confirmation_waits_for_different_map_stamps(node):
    for i in range(3):
        node._progress.elapsed=i*6
        node._map_frame_id=(i,0)
        node._record_select_status(SimpleNamespace(value='NO_FRONTIER'))
    node._watch_navigation()
    assert node._completion_pending['reason']=='no_frontiers'
    assert node._completion_pending['completed']


def test_new_session_clears_quarantine_and_restarts_budgets(node):
    node._selector._region_failures=[(1,1,2)]
    node._progress.elapsed=901
    node._watch_navigation()
    node._on_exploration_session(String(data=json.dumps(dict(session_id='b'*32,state='exploring'))))
    assert node._progress.session_id=='b'*32 and node._progress.elapsed<1
    assert node._completion_pending is None and not node._selector._region_failures
    node._on_mission_state(String(data='exploring'))
    assert node._sm.should_explore


def test_pause_does_not_create_new_budget_or_new_failure_allowance(node):
    old=node._progress
    node._on_mission_state(String(data='paused'))
    node._on_exploration_session(String(data=json.dumps(dict(session_id='a'*32,state='exploring'))))
    node._on_mission_state(String(data='exploring'))
    assert node._progress is old


def test_stale_map_cannot_report_exploration_completed(node):
    node._progress.elapsed=901
    node._grid_stamp=time.monotonic()-21
    node._watch_navigation()
    assert node._completion_pending is None


def test_pending_completion_retries_after_safety_pause_instead_of_resuming_goals(node):
    node._progress.elapsed=901
    node._watch_navigation()
    result=node._completion_pending
    node._on_mission_state(String(data='paused'))
    node._on_mission_state(String(data='exploring'))
    node._completion_published_at=-float('inf')
    node._watch_navigation()
    assert node._completion_pending is result and node._mission_state=='exploring'
    assert not node._sm.should_explore and node._completion_published_at>0


def test_shutdown_cannot_publish_new_completion_or_resume_from_late_heartbeat(node):
    node._progress.elapsed=901
    node.stop_navigation()
    node._on_mission_state(String(data='exploring'))
    node._watch_navigation()
    assert node._completion_pending is None and not node._sm.should_explore


def test_in_place_scan_uses_same_goal_owner_without_submitting_frontier_to_brain(node):
    from pathlib import Path
    from g2_core.explorer import ExplorerParams,GoalSelector
    values=np.load(Path(__file__).parent/'fixtures/static_narrow_map.npz')
    node._grid=GridMap(values['data'],float(values['resolution']),values['origin'])
    node._selector=GoalSelector(ExplorerParams(require_path_clearance=True,max_bootstrap_scans=3,
        min_goal_distance_m=.5,min_frontier_area_m2=.04))
    sent=[];node._send_goal=lambda goal,now:sent.append(goal)
    node._sm.phase=Phase.SENDING
    node._request_brain(tuple(values['robot_xy']),0)
    assert len(sent)==1 and sent[0].purpose=='scan'
    assert node._brain_pending is None


@pytest.mark.parametrize('stop_signal',['SIGINT','SIGTERM'])
def test_explorer_subprocess_stops_without_interrupting_native_dds_take(tmp_path,stop_signal):
    import signal,subprocess,sys
    path=tmp_path/'explorer.log'
    with path.open('w') as log:
        child=subprocess.Popen([sys.executable,'-m','vision_explorer.explorer_node'],stdout=log,stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+15
            while time.monotonic()<deadline and '任务状态订阅已启用' not in path.read_text() and child.poll() is None:time.sleep(.05)
            assert '任务状态订阅已启用' in path.read_text()
            time.sleep(.1)
            child.send_signal(getattr(signal,stop_signal))
            assert child.wait(timeout=5)==0
            assert 'Traceback' not in path.read_text()
        finally:
            if child.poll() is None:child.kill();child.wait()
