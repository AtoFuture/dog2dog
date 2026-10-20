import math
from pathlib import Path
import numpy as np
import pytest
from qwen_robotnav.corridor import CorridorGuidance, detect_corridor
from qwen_robotnav.runtime_profile import load_profile, validate_profile, ros_parameters

PROFILE = Path(__file__).parents[1]/'config/sim_runtime.yaml'


def settings():
    return dict(load_profile(PROFILE)['corridor'], steering_enabled=True)


def test_sim_profile_enables_guidance_but_legacy_profile_remains_observe_only():
    config=load_profile(PROFILE)
    assert config['corridor']['enabled'] is True
    assert config['corridor']['steering_enabled'] is True
    guide=CorridorGuidance(config['corridor'])
    guide.observe(walls(offset=.2),1);guide.observe(walls(offset=.2),2)
    assert guide.guide(.32,-.6,None)==(0.,0.,'awaiting_path')
    legacy=load_profile(PROFILE);legacy['corridor'].pop('steering_enabled')
    validate_profile(legacy)
    guide=CorridorGuidance(legacy['corridor'])
    guide.observe(walls(offset=.2),1);guide.observe(walls(offset=.2),2)
    assert guide.guide(.32,-.6,None)==(.32,-.6,'observing')
    assert guide.guide(0.,0.)==(0.,0.,'observing')


def walls(width=1.4, offset=0., angle=0., noise=0.):
    x = np.linspace(-3., 3., 121)
    points = np.concatenate([np.column_stack((x, np.full_like(x, side*width/2+offset))) for side in (-1,1)])
    c,s = math.cos(angle),math.sin(angle)
    points = points @ np.array([[c,s],[-s,c]])
    return points + np.random.default_rng(8).normal(0,noise,points.shape)


def entrance_walls(width=1.4, start=.7, end=3.):
    x=np.linspace(start,end,61)
    return np.concatenate([np.column_stack((x,np.full_like(x,side*width/2))) for side in (-1,1)])


@pytest.mark.parametrize('angle',[-1.2,-.35,0.,.4,1.3])
def test_measured_parallel_walls_recover_width_heading_and_center(angle):
    result = detect_corridor(walls(offset=.12, angle=angle, noise=.006),settings())
    assert result is not None
    assert result.width_m == pytest.approx(1.4,abs=.025)
    assert result.heading_rad == pytest.approx(angle,abs=.025)
    assert result.center_offset_m == pytest.approx(.12,abs=.025)


@pytest.mark.parametrize('points',[(),((1.,1.),),((math.nan,0.),),walls(4.),walls()[:121],walls()[:8]])
def test_wide_room_one_wall_sparse_or_invalid_evidence_is_not_a_corridor(points):
    assert detect_corridor(points,settings()) is None


def test_short_doorway_and_perpendicular_corner_are_not_a_corridor():
    doorway = walls();doorway = doorway[np.abs(doorway[:,0])<.35]
    assert detect_corridor(doorway,settings()) is None
    one = walls()[:121]
    corner = np.concatenate((one,one @ np.array([[0.,1.],[-1.,0.]])))
    assert detect_corridor(corner,settings()) is None


def test_parallel_walls_are_confirmed_before_the_body_crosses_the_entrance():
    result=detect_corridor(entrance_walls(),settings())
    assert result is not None
    assert result.positive_wall_start_m==pytest.approx(.7,abs=.03)
    assert result.negative_wall_start_m is None


def confirmed(offset=0.,angle=0.):
    guide=CorridorGuidance(settings())
    guide.observe(walls(offset=offset,angle=angle),1)
    assert guide.geometry is None
    assert not guide.observe(walls(offset=offset,angle=angle),1)
    guide.observe(walls(offset=offset,angle=angle),2)
    assert guide.geometry is not None
    return guide


def stabilize(guide, points, target=(3.,0.), linear=.32, angular=0.):
    assert guide.guide(linear,angular,target)[2]=='entry_stabilizing'
    guide.observe(points,3)
    assert guide.guide(linear,angular,target)[2]=='entry_stabilizing'
    guide.observe(points,4)
    return guide.guide(linear,angular,target)


def test_guidance_corrects_toward_center_and_never_initiates_a_stopped_command():
    for offset in (-.02,.02):
        guide=confirmed(offset)
        x,z,mode=stabilize(guide,walls(offset=offset))
        assert x==.32 and mode=='centering' and z*offset>0
        assert abs(z)<=settings()['turn_rate_radps']
        assert guide.guide(0.,0.)==(0.,0.,'stationary')
        assert guide.guide(0.,.8)==(0.,.4,'turning')
        assert guide.guide(0.,-.8)==(0.,-.4,'turning')


def test_large_heading_or_sharp_requested_turn_stops_forward_motion():
    assert confirmed(angle=.8).guide(.32,.3,(3.,3.))==(0.,.4,'entry_aligning')
    assert confirmed().guide(.32,-.6,(-3.,0.))==(0.,-.4,'entry_aligning')


def test_path_goal_selects_the_return_direction_without_fighting_nav2_curvature():
    guide=confirmed()
    assert stabilize(guide,walls(),angular=1.2)==(.32,0.,'centering')
    assert guide.guide(.32,-.2,(-3.,0.))==(0.,-.4,'entry_aligning')
    assert guide.guide(.32,.2,None)==(0.,0.,'awaiting_path')
    assert guide.guide(.32,.2,(float('nan'),0.))==(0.,0.,'awaiting_path')


def test_interior_obstacle_yields_to_planner_only_in_obstructed_direction():
    guide=CorridorGuidance(settings())
    points=np.concatenate((walls(),np.array([[.8,-.1],[.8,0.],[.8,.1]])))
    guide.observe(points,1);guide.observe(points,2)
    assert guide.geometry is not None and guide.geometry.forward_obstacle
    assert guide.guide(.32,.6,(3.,0.))==(.32,.6,'planner_obstacle_avoidance')
    assert guide.guide(.32,-.4,(-3.,0.))==(0.,-.4,'entry_aligning')
    # A wall beyond the requested stopping point does not require a detour.
    assert stabilize(guide,points,target=(.6,0.),angular=.6)==(.32,0.,'centering')


def test_centering_continues_through_goal_approach_without_overriding_stop():
    guide=confirmed(offset=.02)
    x,z,mode=stabilize(guide,walls(offset=.02),target=(.4,0.),angular=-.6)
    assert x==.32 and z>0 and mode=='goal_approach'
    assert guide.guide(0.,0.,(.4,0.))==(0.,0.,'stationary')


def test_parallel_axis_representation_flip_during_turn_preserves_confirmation():
    guide=confirmed(offset=.12,angle=1.5)
    guide.observe(walls(offset=-.12,angle=-1.5),3)
    assert guide.geometry is not None
    assert guide.confirmations==3


def test_lost_wall_removes_confirmation_and_requires_two_new_observations():
    guide=confirmed()
    guide.observe(walls()[:121],3)
    assert guide.geometry is None
    assert guide.guide(.32,.1)==(.32,.1,'unconfirmed')
    guide.observe(walls(),4);assert guide.geometry is None
    guide.observe(walls(),5);assert guide.geometry is not None


def test_old_profiles_and_legacy_scaler_keep_guidance_disabled():
    p=load_profile(PROFILE)
    params=ros_parameters(p)['cmd_vel_scaler']['ros__parameters']
    assert 'corridor_config' in params
    p.pop('corridor')
    validate_profile(p)
    assert 'corridor_config' not in ros_parameters(p)['cmd_vel_scaler']['ros__parameters']


@pytest.mark.parametrize('key,value',[('enabled',1),('max_width_m',5.),('wall_tolerance_m',.3),('turn_rate_radps',1.),('heading_gain',float('nan'))])
def test_unvalidated_corridor_parameters_are_rejected(key,value):
    p=load_profile(PROFILE);p['corridor'][key]=value
    with pytest.raises(ValueError):validate_profile(p)


def test_shorter_centering_lookahead_is_bounded_without_changing_path_or_turn_limits():
    p=load_profile(PROFILE)
    p['corridor']['lookahead_m']=.25
    validate_profile(p)
    g=CorridorGuidance(p['corridor'])
    g.observe(walls(offset=.02),1);g.observe(walls(offset=.02),2)
    x,z,mode=stabilize(g,walls(offset=.02))
    assert x==.32 and mode=='centering'
    assert z==pytest.approx(1.2*math.atan2(.02,.25),abs=.002)
    assert p['corridor']['path_lookahead_m']==1.
    assert p['corridor']['turn_rate_radps']==.4
    assert load_profile(PROFILE)['corridor']['lookahead_m']==.5
    p['corridor']['lookahead_m']=.249
    with pytest.raises(ValueError):validate_profile(p)
