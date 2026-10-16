"""Regress the real joint-trial startup race and observation hold behavior."""
import time
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from vision_explorer.explorer_node import VisionExplorer
from g2_core.state_machine import Command


def test_measured_targets_never_fall_back_to_unrelated_frontiers():
    sent=[];failures=[];events=[]
    fake=SimpleNamespace(_last_select_status=None,_robot_xy=lambda:(0.,0.),_map_pose_healthy=lambda xy:True,
        _p=lambda:dict(track_observation=True,use_mission_brain=True),
        _sm=SimpleNamespace(on_select_failed=failures.append),_event=lambda *a,**kw:events.append(kw),
        _request_brain=lambda *a:sent.append(a))
    VisionExplorer._select_and_send(fake,1.)
    assert not sent and failures==[False] and events[-1]['reason']=='awaiting_recon_status'
    fake._recon_status=dict(targets={'person':dict(position_m=[3.,0.,.8])})
    fake._recon_status_at=time.monotonic();fake._observation_goals=lambda *a:()
    VisionExplorer._select_and_send(fake,1.)
    assert not sent and events[-1]['reason']=='no_safe_target_viewpoint'
    fake._observation_goals=lambda *a:('safe candidate',)
    VisionExplorer._select_and_send(fake,1.)
    assert len(sent)==1


def test_observation_hold_still_runs_navigation_watch_and_handoff(monkeypatch):
    monkeypatch.setattr('vision_explorer.explorer_node.rclpy.ok',lambda:True)
    watched=[];selected=[];commands=[]
    def next_command():commands.append(True);return Command.SEND_GOAL
    fake=SimpleNamespace(_now=lambda:1.,_watch_navigation=lambda:watched.append(True),_stopped=False,
        _attempt=None,_handoff_token=None,_brain_pending=None,
        _recon_status=dict(state='searching',observation_window_pending=True),_recon_status_at=time.monotonic(),
        _p=lambda:dict(track_observation=True),_sm=SimpleNamespace(next_command=next_command),
        _select_and_send=selected.append)
    VisionExplorer._tick(fake)
    assert watched and not selected and not commands
    fake._recon_status['observation_window_pending']=False
    VisionExplorer._tick(fake)
    assert selected==[1.] and len(watched)==2
