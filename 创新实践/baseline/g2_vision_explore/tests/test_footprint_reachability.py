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
    params=ExplorerParams(min_frontier_area_m2=.01,goal_obstacle_clearance_m=.65)
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


def test_measured_corridor_offers_novel_forward_goal_over_traversed_path():
    from pathlib import Path
    from g2_core.explorer import Goal
    v=np.load(Path(__file__).parent/'fixtures/static_corridor_progress_map.npz')
    grid=GridMap(v['data'],float(v['resolution']),v['origin'])
    s=selector();s.params.min_goal_distance_m=.5
    for xy in v['travel_xy']:s.observe_position(*xy,0)
    s.on_result(Goal(*v['last_goal'],0,1,1,1,1),True,0)
    result=s.select(grid,tuple(v['robot_xy']),1,max_candidates=6)
    assert result.goal is not None and result.goal.x>4.
    forward=[g for g in result.candidates if g.x>4.]
    behind=[g for g in result.candidates if g.x<1.6]
    assert forward
    if behind:
        assert max(g.unknown_fraction/g.geodesic_dist_m for g in forward)>max(g.unknown_fraction/g.geodesic_dist_m for g in behind)


def test_recent_visit_discount_is_preserved_in_model_information_gain():
    from g2_core.explorer import Goal
    grid=GridMap(np.zeros((100,100),dtype=np.int8),.1,(-5.,-5.))
    grid.data[:,80:]=-1
    s=selector();s.params.min_goal_distance_m=.5
    before=s.select(grid,(0.,0.),0,max_candidates=16)
    s.on_result(Goal(.9,0.,0,1,1,1,1),True,0)
    after=s.select(grid,(0.,0.),1,max_candidates=16)
    old={(g.x,g.y):g.unknown_fraction for g in before.candidates}
    shared=[g for g in after.candidates if (g.x,g.y) in old and .5<np.hypot(g.x-.9,g.y)<2.]
    assert shared
    assert all(g.unknown_fraction<old[(g.x,g.y)] for g in shared)


def test_actual_travel_history_is_bounded_and_disabled_for_frozen_mode():
    s=selector();s.observe_position(0.,0.,0.);s.observe_position(.1,0.,1.)
    assert len(s._visits)==1
    s.observe_position(.3,0.,2.)
    assert len(s._visits)==2
    s.observe_position(.3,0.,65.)
    assert len(s._visits)==1
    s.reset_mission();assert not s._visits
    legacy=GoalSelector(ExplorerParams())
    legacy.observe_position(1.,0.,1.);assert not legacy._visits


def test_execution_endpoint_standoff_is_revalidated_without_weakening_path_clearance():
    from g2_core.explorer import Goal
    grid=GridMap(np.zeros((100,100),dtype=np.int8),.1,(-5.,-5.))
    grid.data[:,70]=100
    s=selector();s.params.min_goal_distance_m=.5
    near=Goal(1.5,0.,0,1,1,1.5,.5)
    far=Goal(1.2,0.,0,1,1,1.2,.5)
    assert not s.is_goal_valid(grid,(0.,0.),near,0)
    assert s.is_goal_valid(grid,(0.,0.),far,0)
