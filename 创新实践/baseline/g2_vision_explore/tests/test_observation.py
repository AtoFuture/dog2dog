import math
import numpy as np
from g2_core.grid import GridMap
from g2_core.observation import observation_candidates,ray_clear


def scene():
    data=np.zeros((160,160),np.int8)
    data[66:90,90:93]=100
    data[60:105,93:125]=-1
    return GridMap(data,.05,(-2.,-4.))


def test_viewpoints_reveal_shadow_and_have_clearance_inflated_routes():
    grid=scene();robot=(0.,0.)
    goals=observation_candidates(grid,robot)
    assert goals
    for g in goals:
        assert g.purpose=='observe_occlusion' and g.geodesic_dist_m>=.75
        assert math.dist((g.x,g.y),g.look_at)>=2.8
        r,c=(math.floor(v) for v in grid.world_to_grid(g.x,g.y))
        assert grid.traversable[r,c] and grid.clearance_m()[r,c]>=.45
        assert not ray_clear(grid,robot,g.look_at)
        assert ray_clear(grid,(g.x,g.y),g.look_at)
        assert math.atan2(g.look_at[1]-g.y,g.look_at[0]-g.x)==g.yaw


def test_unknown_space_and_narrow_routes_never_become_motion_goals():
    grid=scene();grid.data[grid.free]=-1
    assert not observation_candidates(grid,(0.,0.))
    grid=scene();grid.data[:]=0
    assert not observation_candidates(grid,(0.,0.))
    assert not ray_clear(grid,(0.,0.),(100.,100.))


def test_narrow_door_is_not_a_clearance_inflated_motion_route():
    grid=scene();grid.data[:,60]=100;grid.data[74:86,60]=0
    goals=observation_candidates(grid,(0.,0.))
    assert all(g.x<1.0 for g in goals)
