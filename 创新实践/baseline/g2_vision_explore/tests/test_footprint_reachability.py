"""A safe endpoint behind a lidar-visible slit is not a footprint-reachable goal."""
import numpy as np
from g2_core.explorer import ExplorerParams, GoalSelector, SelectStatus
from g2_core.grid import GridMap


def rooms(gap_cells):
    data=np.full((120,160),100,dtype=np.int8)
    data[10:110,10:60]=0
    data[10:110,61:150]=0
    data[60-gap_cells//2:60+(gap_cells+1)//2,60]=0
    data[10:110,120:150]=-1
    return GridMap(data,.1)


def selector():
    params=ExplorerParams(min_frontier_area_m2=.01)
    params.require_path_clearance=True
    return GoalSelector(params)


def test_lidar_slit_does_not_make_other_room_reachable():
    result=selector().select(rooms(3),(3.,6.),0)
    assert result.status is SelectStatus.NO_CANDIDATE


def test_wide_passage_remains_reachable():
    s=selector();grid=rooms(14)
    result=s.select(grid,(3.,6.),0)
    assert result.status is SelectStatus.GOAL and result.goal.x>6
    assert s.is_goal_valid(grid,(3.,6.),result.goal,0)


def test_latest_map_can_invalidate_path_while_endpoint_stays_clear():
    s=selector();result=s.select(rooms(14),(3.,6.),0)
    assert result.goal is not None
    assert not s.is_goal_valid(rooms(3),(3.,6.),result.goal,1)


def test_measured_startup_unknown_gap_allows_bounded_egress():
    from pathlib import Path
    values=np.load(Path(__file__).parent/'fixtures/static_startup_map.npz')
    grid=GridMap(values['data'],float(values['resolution']),values['origin'])
    xy=tuple(values['robot_xy'])
    s=selector();s.params.min_goal_distance_m=.5
    start=grid.nearest_traversable_index(*xy)
    assert .26<float(grid.clearance_m()[start])<s.params.safety_radius_m
    result=s.select(grid,xy,0)
    assert result.status is SelectStatus.GOAL
    assert s.is_goal_valid(grid,xy,result.goal,0)
    mask=s._path_free(grid,start,grid.clearance_m())
    extra=mask & (grid.clearance_m()<s.params.safety_radius_m)
    rr,cc=np.where(extra)
    assert np.all(np.hypot(rr-start[0],cc-start[1])*grid.resolution<=.30)


def test_observed_obstacle_cannot_use_unknown_space_egress_exception():
    grid=rooms(3);s=selector()
    start=grid.nearest_traversable_index(5.8,6.)
    # Near the wall, even a free start cell must retain full known-obstacle clearance.
    assert not s._path_free(grid,start,grid.clearance_m())[start]


def test_measured_sparse_narrow_map_requests_bounded_safe_scans_then_refuses():
    from pathlib import Path
    values=np.load(Path(__file__).parent/'fixtures/static_narrow_map.npz')
    grid=GridMap(values['data'],float(values['resolution']),values['origin'])
    xy=tuple(values['robot_xy']);s=selector();s.params.min_goal_distance_m=.5;s.params.max_bootstrap_scans=3
    headings=[]
    for i in range(3):
        result=s.select(grid,xy,i)
        assert result.status is SelectStatus.GOAL and result.goal.purpose=='scan'
        assert (result.goal.x,result.goal.y)==xy and result.goal.geodesic_dist_m==0
        assert s.is_goal_valid(grid,xy,result.goal,i)
        headings.append(result.goal.yaw)
        s.on_result(result.goal,success=i!=1,now=i)
    assert len(set(headings))==3
    assert s.select(grid,xy,4).status is SelectStatus.NO_CANDIDATE
    assert not s._blacklist and not s._visits
    s.reset_mission()
    assert s.select(grid,xy,5).goal.purpose=='scan'


def test_scan_cannot_relax_clearance_to_an_observed_wall():
    grid=rooms(3);s=selector();s.params.max_bootstrap_scans=3
    start=grid.nearest_traversable_index(5.8,6.)
    assert s._scan_goal(grid,start,(5.8,6.)) is None


def test_manhattan_path_length_cannot_hide_a_too_close_endpoint():
    from g2_core.explorer import Goal
    grid=GridMap(np.zeros((100,100),dtype=np.int8),.1,(-5.,-5.))
    s=selector();s.params.min_goal_distance_m=.5
    close=Goal(.3,.3,0,1,1,.6,.5)
    assert not s.is_goal_valid(grid,(0.,0.),close,0)
    far=Goal(1.,0.,0,1,1,1.,.5)
    assert s.is_goal_valid(grid,(0.,0.),far,0)


def test_success_cooldown_expires_and_does_not_become_a_permanent_blacklist():
    from g2_core.explorer import Goal
    grid=GridMap(np.zeros((100,100),dtype=np.int8),.1,(-5.,-5.))
    s=selector();s.params.min_goal_distance_m=.5
    goal=Goal(1.,0.,0,1,1,1.,.5)
    s.on_result(goal,True,0)
    assert not s.is_goal_valid(grid,(0.,0.),goal,1)
    assert s.is_goal_valid(grid,(0.,0.),goal,31)
    s.on_result(goal,True,32);s.reset_mission()
    assert s.is_goal_valid(grid,(0.,0.),goal,33)
