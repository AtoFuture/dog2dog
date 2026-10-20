"""Turning evidence is stronger than straight-line footprint clearance."""
import numpy as np
import pytest
from g2_core.explorer import ExplorerParams, GoalSelector, SelectStatus
from g2_core.grid import GridMap


def corridor(width):
    data=np.full((50,100),100,dtype=np.int8)
    lo=25-width//2
    data[lo:lo+width,2:98]=0
    data[lo:lo+width,75:98]=-1
    return GridMap(data,.1)


def selector(radius):
    return GoalSelector(ExplorerParams(min_frontier_area_m2=.01,
        require_path_clearance=True,goal_obstacle_clearance_m=.65,
        goal_turn_clearance_m=radius))


def test_narrow_straight_route_is_not_a_turning_endpoint():
    grid=corridor(14);xy=(2.,2.5)
    previous=selector(0.).select(grid,xy,0.)
    assert previous.status is SelectStatus.GOAL
    guarded=selector(.75)
    result=guarded.select(grid,xy,0.)
    assert result.status is SelectStatus.NO_CANDIDATE and not result.is_complete
    assert not guarded.is_goal_valid(grid,xy,previous.goal,1.)


def test_wide_known_turning_disk_remains_available():
    grid=corridor(20);s=selector(.75);xy=(2.,2.5)
    result=s.select(grid,xy,0.)
    assert result.status is SelectStatus.GOAL
    assert s.is_goal_valid(grid,xy,result.goal,0.)
    row,col=(int(v) for v in grid.world_to_grid(result.goal.x,result.goal.y))
    assert grid.clearance_m()[row,col]>=.75
    # A delayed provider reply must be rechecked when new unknown space or a
    # newly observed obstacle invalidates the turning disk.
    for value in (-1,100):
        data=grid.data.copy();data[row+6,col]=value
        assert not s.is_goal_valid(GridMap(data,.1),xy,result.goal,1.)


@pytest.mark.parametrize('radius',[-.1,float('nan'),float('inf')])
def test_invalid_turning_radius_is_rejected(radius):
    with pytest.raises(ValueError):selector(radius).select(corridor(20),(2.,2.5),0.)
