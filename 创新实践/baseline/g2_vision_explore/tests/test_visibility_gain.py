"""Known walls must occlude utility; unknown space must never become traversable."""
import math
import numpy as np
from g2_core.grid import GridMap
from g2_core.visibility import visible_unknown_area
from g2_core.explorer import ExplorerParams, GoalSelector, SelectStatus


def room(gap=0):
    data=np.full((120,120),-1,dtype=np.int8)
    data[29:91,29:91]=100
    data[30:90,30:90]=0
    if gap:
        data[60-gap//2:60+(gap+1)//2,90]=0
    return GridMap(data,.1,(-6.,-6.))


def test_unknown_behind_closed_wall_has_zero_visibility_gain():
    grid=room()
    candidates=np.array([[60,80],[60,60]])
    assert np.all(visible_unknown_area(grid,candidates,2.)==0)


def test_real_opening_exposes_unknown_and_map_is_unchanged():
    grid=room(14);before=grid.data.copy()
    result=visible_unknown_area(grid,np.array([[60,80]]),2.)
    assert result[0]>.25
    assert np.array_equal(grid.data,before)


def test_all_unknown_counts_distinct_visible_cells_and_known_free_has_zero_gain():
    unknown=GridMap(np.full((100,100),-1,dtype=np.int8),.1)
    free=GridMap(np.zeros((100,100),dtype=np.int8),.1)
    candidates=np.array([[50,50]])
    assert .6*math.pi*4 < visible_unknown_area(unknown,candidates,2.)[0] < math.pi*(2.+math.sqrt(2)*.1/2)**2
    assert visible_unknown_area(free,candidates,2.)[0]==0


def test_unknown_slivers_do_not_borrow_gain_from_outside_a_closed_room():
    grid=room();grid.data[40:60,88]=-1
    params=ExplorerParams(require_path_clearance=True,goal_obstacle_clearance_m=.65,min_goal_distance_m=.5)
    result=GoalSelector(params).select(grid,(0.,0.),0.,max_candidates=6)
    assert result.status is SelectStatus.NO_CANDIDATE
    # The frozen historical heuristic remains available and is not redefined.
    params.require_path_clearance=False
    assert GoalSelector(params).select(grid,(0.,0.),0.).status is SelectStatus.GOAL
