import numpy as np
from g2_core.explorer import ExplorerParams, Goal, GoalSelector
from g2_core.grid import GridMap


def test_failed_region_retries_after_ttl_then_stays_excluded_until_next_mission():
    s = GoalSelector(ExplorerParams(max_region_failures=2))
    grid = GridMap(np.zeros((50, 50), dtype=np.int8), .1)
    cells = np.array([[10, 10], [40, 40]])
    x, y = grid.grid_to_world(10, 10)
    g = Goal(x, y, 0, 1, 1, 1, .5)
    s.on_result(g, False, 0)
    assert len(s._filter_blacklisted(grid, cells)) == 1
    s._expire(31)
    assert len(s._filter_blacklisted(grid, cells)) == 2
    s.on_result(g, False, 31)
    s._expire(100)
    assert len(s._filter_blacklisted(grid, cells)) == 1
    assert not s.is_goal_valid(grid, (2, 2), g, 100)
    s.reset_mission()
    assert len(s._filter_blacklisted(grid, cells)) == 2
