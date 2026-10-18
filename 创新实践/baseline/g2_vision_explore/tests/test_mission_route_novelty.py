"""Real travel persists for a mission without forbidding transit through old paths."""
import numpy as np

from g2_core.explorer import ExplorerParams, Goal, GoalSelector, SelectStatus
from g2_core.grid import GridMap


def selector():
    return GoalSelector(ExplorerParams(require_path_clearance=True,
        min_goal_distance_m=.5, min_frontier_area_m2=.01))


def test_actual_travel_remains_excluded_after_short_cooldowns_and_resets():
    grid = GridMap(np.zeros((100, 100), dtype=np.int8), .1, (-5., -5.))
    s = selector()
    goal = Goal(1., 0., 0., 1, 1., 1., .5)
    s.observe_position(1., 0., 0.)
    s.observe_position(0., 0., 65.)
    assert len(s._visits) == 1
    assert not s.is_goal_valid(grid, (0., 0.), goal, 66.)
    s.reset_mission()
    assert s.is_goal_valid(grid, (0., 0.), goal, 67.)


def test_requested_goal_without_real_travel_does_not_enter_route_memory():
    grid = GridMap(np.zeros((100, 100), dtype=np.int8), .1, (-5., -5.))
    s = selector()
    goal = Goal(1., 0., 0., 1, 1., 1., .5)
    s.on_result(goal, True, 0.)
    assert not s._mission_route
    assert s.is_goal_valid(grid, (0., 0.), goal, 65.)


def test_old_route_can_be_crossed_to_reach_an_unvisited_branch():
    data = np.full((80, 120), 100, dtype=np.int8)
    data[30:50, 5:110] = 0
    data[5:50, 80:100] = 0
    data[5:15, 80:100] = -1
    grid = GridMap(data, .1)
    s = selector()
    for x in np.arange(1., 10., .2):
        s.observe_position(float(x), 4., 0.)
    result = s.select(grid, (1., 4.), 65.)
    assert result.status is SelectStatus.GOAL
    assert result.goal.y < 3.
    assert s.is_goal_valid(grid, (1., 4.), result.goal, 65.)


def test_stationary_positions_are_deduplicated_and_legacy_stays_unchanged():
    s = selector()
    for t in range(200):
        s.observe_position(1., 0., float(t))
    assert len(s._mission_route) == 1
    legacy = GoalSelector()
    legacy.observe_position(1., 0., 0.)
    assert not legacy._mission_route
    s.observe_position(float('nan'), 0., 201.)
    assert len(s._mission_route) == 1
