"""Explore completion must distinguish exhausted frontiers from partial termination."""
import pytest
from g2_core.mission_progress import MissionProgress, ProgressLimits


def mission(**kw):
    p = MissionProgress('a'*32, ProgressLimits(**kw), now=0)
    p.advance(0, active=True)
    p.observe_area(20)
    return p


def test_exhaustion_requires_three_maps_and_confirmation_not_ticks():
    p = mission()
    p.selection('NO_FRONTIER', frame_id=1)
    p.advance(11, active=True)
    for _ in range(10): p.selection('NO_FRONTIER', frame_id=1)
    assert p.evaluate(healthy=True) is None
    p.selection('NO_FRONTIER', frame_id=2)
    assert p.evaluate(healthy=True) is None
    p.selection('NO_FRONTIER', frame_id=3)
    assert p.evaluate(healthy=True)['completed'] is True
    assert p.result['reason'] == 'no_frontiers'


@pytest.mark.parametrize('status', ['NO_CANDIDATE', 'SMALL_FRONTIERS'])
def test_blacklist_or_small_frontier_is_partial_return_after_wait(status):
    p = mission()
    p.selection(status)
    p.advance(31, active=True)
    assert p.evaluate(healthy=True) is None
    p.advance(90, active=True)
    result = p.evaluate(healthy=True)
    assert result['reason'] == 'candidates_unavailable' and not result['completed']


def test_missing_pose_clears_exhaustion_and_wait_confirmation():
    p = mission()
    for i in range(3): p.selection('NO_FRONTIER', frame_id=i)
    p.advance(11, active=True)
    assert p.evaluate(healthy=False) is None
    assert p.evaluate(healthy=True) is None
    p.selection('NO_ROBOT_POSE')
    p.advance(20, active=True)
    assert p.evaluate(healthy=True) is None


def test_pause_freezes_active_budget_and_preserves_progress():
    p = mission(exploration_budget_s=50)
    p.advance(20, active=False)
    p.advance(1000, active=True)
    assert p.elapsed == 20 and p.evaluate(healthy=True) is None
    p.advance(1030, active=True)
    assert p.evaluate(healthy=True)['reason'] == 'time_budget'


def test_growth_uses_area_not_map_array_size_and_accumulates_small_gains():
    p = mission()
    p.advance(100, active=True)
    p.observe_area(20.2)
    p.observe_area(20.6)
    p.advance(200, active=True)
    assert p.evaluate(healthy=True) is None
    p.observe_area(19)  # SLAM corrections do not count as new growth
    p.advance(280, active=True)
    assert p.evaluate(healthy=True)['reason'] == 'coverage_stalled'


def test_success_resets_consecutive_failures_and_terminal_result_is_latched():
    p = mission(max_consecutive_failures=2)
    p.navigation_result(False)
    p.navigation_result(True)
    p.navigation_result(False)
    assert p.evaluate(healthy=True) is None
    p.navigation_result(False)
    result = p.evaluate(healthy=True)
    assert result['reason'] == 'navigation_failures' and not result['completed']
    assert result['succeeded_goals'] == 1 and result['failed_goals'] == 3
    p.advance(1000, active=True)
    assert p.evaluate(healthy=True) == result


@pytest.mark.parametrize('kw', [dict(candidate_wait_s=30),dict(max_region_failures=1.5),
    dict(coverage_min_gain_m2=float('nan')),dict(exploration_budget_s=True)])
def test_invalid_limits_rejected(kw):
    with pytest.raises(ValueError): ProgressLimits(**kw)


def test_old_empty_selection_cannot_complete_after_new_map_has_arrived():
    p = mission()
    for i in range(3):
        p.advance(i*6, active=True)
        p.selection('NO_FRONTIER', frame_id=i)
    assert p.evaluate(healthy=True, selection_current=False) is None
    p.selection('GOAL', frame_id=3)
    assert p.evaluate(healthy=True) is None


def test_three_early_frames_followed_by_frozen_map_cannot_confirm_exhaustion():
    p = mission()
    for i in range(3):
        p.advance(i, active=True)
        p.selection('NO_FRONTIER', frame_id=i)
    p.advance(11, active=True)
    p.selection('NO_FRONTIER', frame_id=2)
    assert p.evaluate(healthy=True) is None
    p.selection('NO_FRONTIER', frame_id=3)
    assert p.evaluate(healthy=True)['reason'] == 'no_frontiers'


def test_new_map_must_be_selected_before_declaring_candidates_unavailable():
    p=mission()
    p.selection('NO_CANDIDATE',frame_id=1)
    p.advance(91,active=True)
    assert p.evaluate(healthy=True,selection_current=False) is None
    p.selection('GOAL',frame_id=2)
    assert p.evaluate(healthy=True) is None
