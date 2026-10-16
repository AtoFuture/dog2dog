"""Observed targets guide safe camera-facing motion, never unknown-space travel."""
import math
import numpy as np
import pytest
from g2_core.grid import GridMap
from g2_core.observation import tracked_observation_candidates


def test_forward_views_keep_measured_people_in_camera_and_outside_standoff():
    grid=GridMap(np.zeros((100,100),dtype=np.int8),.1,(-5.,-5.))
    targets=[(3.5,-.4),(3.8,.4)]
    goals=tracked_observation_candidates(grid,(0.,0.),anchors=targets)
    assert goals and goals[0].purpose=='observe_targets'
    assert goals[0].x>.7 and abs(goals[0].y)<.15
    for g in goals:
        assert .75<=g.geodesic_dist_m<=1.4
        driving_heading=math.atan2(g.y,g.x)
        for x,y in targets:
            for ox,oy in [(0.,0.),(g.x,g.y)]:
                error=math.atan2(y-oy,x-ox)-driving_heading
                assert abs(math.atan2(math.sin(error),math.cos(error)))<=.35
            assert math.hypot(x-g.x,y-g.y)>=2.5
            relative=math.atan2(y-g.y,x-g.x)-g.yaw
            assert abs(math.atan2(math.sin(relative),math.cos(relative)))<=.45


def test_unknown_travel_and_blocked_travel_do_not_generate_target_views():
    data=np.full((100,100),-1,dtype=np.int8);data[49:52,49:52]=0
    assert not tracked_observation_candidates(GridMap(data,.1,(-5.,-5.)),(0.,0.),anchors=[(3.5,0.)])
    data[:]=0;data[:,54:58]=100
    assert not tracked_observation_candidates(GridMap(data,.1,(-5.,-5.)),(0.,0.),anchors=[(3.5,0.)])
    assert not tracked_observation_candidates(GridMap(data,.1,(-5.,-5.)),(0.,0.),anchors=[])


@pytest.mark.parametrize('name',['joint_initial_map.npz','joint_initial_map_v4.npz'])
def test_sparse_real_initial_map_uses_short_safe_observation_step(name):
    from pathlib import Path
    fixture=np.load(Path(__file__).with_name('fixtures')/name)
    grid=GridMap(fixture['data'],float(fixture['resolution']),fixture['origin'])
    goals=tracked_observation_candidates(grid,fixture['robot'],anchors=fixture['anchors'])
    assert goals and all(g.geodesic_dist_m>=.5 for g in goals)
    if name=='joint_initial_map_v4.npz':assert all(g.geodesic_dist_m<.75 for g in goals)
    from g2_core.explorer import GoalSelector,ExplorerParams
    selector=GoalSelector(ExplorerParams(min_goal_distance_m=.5))
    assert all(selector.is_goal_valid(grid,fixture['robot'],g,0.) for g in goals)
    for g in goals:
        cell=tuple(math.floor(v) for v in grid.world_to_grid(g.x,g.y))
        assert grid.free[cell] and grid.clearance_m()[cell]>=.45


def test_blocked_forward_view_allows_only_short_side_reposition():
    data=np.zeros((100,100),dtype=np.int8);data[48:53,58:63]=100
    grid=GridMap(data,.1,(-5.,-5.))
    goals=tracked_observation_candidates(grid,(0.,0.),anchors=[(3.5,0.)])
    assert goals and all(.5<=g.geodesic_dist_m<=.65 for g in goals)
    assert all(abs(g.y)>.1 for g in goals)
    for g in goals:
        assert g.yaw==pytest.approx(math.atan2(-g.y,3.5-g.x))
        cell=tuple(math.floor(v) for v in grid.world_to_grid(g.x,g.y))
        assert grid.clearance_m()[cell]>=.45
