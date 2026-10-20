import math
from dataclasses import replace
from pathlib import Path
import pytest
from qwen_robotnav.corridor import Corridor, CorridorGuidance, CorridorTurnRecovery
from qwen_robotnav.runtime_profile import load_profile, validate_profile
from qwen_robotnav.velocity_safety import VelocitySafety

PROFILE=Path(__file__).parents[1]/'config/sim_runtime.yaml'


def geometry(offset=.15, heading=-math.pi/2):
    return Corridor(1.4,heading,offset,3.,80,positive_wall_start_m=-3.,negative_wall_start_m=-3.)


def step(recovery,t,g=None,**kw):
    args=dict(geometry=g or geometry(),sensor_radius=.22,range_min=.3,requested=True,
              now=t,current_linear=0.,current_angular=0.,enabled=True)
    args.update(kw)
    return recovery.filter(0.,.4,'turning',**args)


def test_turn_brakes_recenters_and_settles_before_resuming_with_measured_progress():
    r=CorridorTurnRecovery()
    assert step(r,0.)==(0.,0.,'turn_recovery_settling')
    assert step(r,.79)[0]==0.
    assert step(r,.81)==(.32,0.,'turn_recentering')
    assert step(r,1.,geometry(.09))==(.32,0.,'turn_recentering')
    assert step(r,1.1,geometry(.03))==(0.,0.,'turn_recovery_settling')
    assert step(r,1.2,geometry(.03),current_linear=.02)[0]==0.
    assert step(r,1.3,geometry(.03))[0]==0.
    assert step(r,2.11,geometry(.03))[2]=='turn_recovery_complete'
    assert step(r,2.2,geometry(.03))==(0.,.4,'turning')


def test_never_translates_toward_near_wall_and_has_bounded_failure():
    r=CorridorTurnRecovery();g=geometry(heading=math.pi/2)
    step(r,0.,g)
    x,z,mode=step(r,1.,g)
    assert x==0 and mode=='turn_recovery_orienting'
    assert step(r,13.,g)==(0.,0.,'turn_recovery_timeout')
    assert r.blocked


def test_no_self_start_and_missing_geometry_cancels_recovery():
    r=CorridorTurnRecovery();step(r,0.)
    assert step(r,.1,requested=False)==(0.,.4,'turning')
    assert r.phase is None
    step(r,1.)
    assert step(r,2.,geometry=None)==(0.,.4,'turning')
    assert r.phase is None


def test_measured_sensor_blind_range_limits_turn_offset():
    r=CorridorTurnRecovery()
    assert step(r,0.,geometry(.01),sensor_radius=.30,range_min=.4)[2]=='turn_sensor_clearance'
    assert step(r,1.,geometry(.01))[2]=='turning'
    assert r.center_limit==pytest.approx(.08)


def test_decimal_ros_time_does_not_look_like_a_clock_reset_on_episode_start():
    r=CorridorTurnRecovery()
    assert step(r,120.54)==(0.,0.,'turn_recovery_settling')
    assert not r.blocked
    assert step(r,121.35)==(.32,0.,'turn_recentering')


def test_real_clock_reset_stops_recovery():
    r=CorridorTurnRecovery();step(r,120.54)
    assert step(r,100.)==(0.,0.,'turn_recovery_timeout')


def test_short_zero_requests_reuse_one_episode_without_refreshing_its_deadline():
    r=CorridorTurnRecovery()
    for attempt in range(8):
        assert step(r,float(attempt))[2]=='turn_recovery_settling'
        step(r,attempt+.1,requested=False)
    assert r.episodes==1
    assert step(r,8.)[2]=='turn_recovery_settling'
    assert r.started_at==0.
    assert step(r,12.1)==(0.,0.,'turn_recovery_timeout')


def test_distinct_interrupted_episodes_still_exhaust_the_eight_episode_budget():
    r=CorridorTurnRecovery()
    for attempt in range(8):
        assert step(r,float(attempt)*13)[2]=='turn_recovery_settling'
        step(r,attempt*13+.1,requested=False)
    assert r.episodes==8
    assert step(r,104.)==(0.,0.,'turn_recovery_exhausted')
    r.reset()
    assert step(r,105.)[2]=='turn_recovery_settling'


def test_resume_brakes_again_before_using_fresh_geometry_and_original_time_budget():
    r=CorridorTurnRecovery();step(r,0.)
    assert step(r,.81)[2]=='turn_recentering'
    assert r.filter(0.,0.,'stationary',geometry=geometry(),sensor_radius=.22,range_min=.3,
                    requested=False,now=.9,current_linear=.02,current_angular=0.,enabled=True)==(0.,0.,'stationary')
    assert r.phase is None and r.snapshot()['recovery_resume_pending']
    assert step(r,1.,geometry(-.15,math.pi/2),current_linear=.02)[2]=='turn_recovery_settling'
    assert step(r,1.1,geometry(-.15,math.pi/2))[2]=='turn_recovery_settling'
    assert step(r,1.91,geometry(-.15,math.pi/2))[2]=='turn_recentering'
    assert r.episodes==1 and r.started_at==0.
    assert step(r,12.1)[2]=='turn_recovery_timeout'


def test_clock_reset_during_interruption_does_not_create_a_new_episode():
    r=CorridorTurnRecovery();step(r,120.)
    step(r,120.1,requested=False)
    assert step(r,100.)==(0.,0.,'turn_recovery_timeout')
    assert r.episodes==1 and r.blocked


@pytest.mark.parametrize('width,start,offset',[(1.014,.377,-.0786),(1.56,.893,.288)])
def test_entrance_ahead_is_not_a_turn_envelope_around_the_body(width,start,offset):
    r=CorridorTurnRecovery()
    g=replace(geometry(offset,.19),width_m=width,positive_wall_start_m=start,negative_wall_start_m=None)
    assert step(r,0.,g)==(0.,.4,'turning')
    assert r.episodes==0 and r.center_limit is None
    # Once the same narrow walls actually surround the body, the original
    # sensor turn-clearance restriction remains in force.
    if width<1.1:
        g=replace(g,positive_wall_start_m=-3.)
        assert step(r,1.,g)==(0.,0.,'turn_sensor_clearance')


def test_completed_episode_is_not_reused_for_the_next_recovery():
    r=CorridorTurnRecovery();step(r,0.);step(r,.81)
    step(r,1.,geometry(.03));step(r,1.1,geometry(.03))
    assert step(r,1.91,geometry(.03))[2]=='turn_recovery_complete'
    assert step(r,2.)[2]=='turn_recovery_settling'
    assert r.episodes==2


def test_axis_representation_flip_preserves_inward_translation():
    r=CorridorTurnRecovery()
    step(r,0.,geometry(.15,-math.pi/2))
    assert step(r,.81,geometry(-.15,math.pi/2))==(.32,0.,'turn_recentering')


def test_reversal_enters_inward_half_plane_with_recenter_enabled():
    settings=load_profile(PROFILE)['corridor']
    settings.update(reversal_turn_enabled=True,turn_recenter_enabled=True)
    for offset,sign in ((.025,1.),(-.025,-1.)):
        guide=CorridorGuidance(settings);guide.geometry=geometry(offset,0.)
        assert guide.guide(0.,-.4,(-2.,0.))==(0.,sign*.4,'reversal_turning')


def ready(points):
    p=load_profile(PROFILE);g=dict(p['guard'],clearance_recovery_enabled=True)
    s=VelocitySafety(g,True)
    for k in ('nav_command','command','mission','scan','odometry','imu','battery'):s.observe(k,10.)
    s.mission='returning';s.command=(.024,0.);s.front=2.;s.tilt=0.;s.obstacles=points
    return s


def test_guard_allows_only_clearance_increasing_straight_escape():
    # Wall behind the robot: a forward correction grows the current margin.
    s=ready(tuple((-.435,i*.025) for i in range(-60,61)))
    assert s.output(10.1)==(.024,0.,'clearance_recovery')
    s.command=(.024,.1)
    assert s.output(10.1)==(0.,0.,'footprint_clearance')
    s.command=(.024,0.);s.measured_velocity=(-.05,0.,0.)
    assert s.output(10.1)==(0.,0.,'footprint_clearance')


def test_escape_includes_scan_age_in_its_forward_envelope():
    s=ready(((-.435,0.),(.70,0.)))
    assert s.output(10.)==(.024,0.,'clearance_recovery')
    # The scan is still fresh, but uncertainty extends the sweep into the
    # obstacle ahead. A short escape must not use the nominal horizon alone.
    assert s.output(10.3)==(0.,0.,'footprint_clearance')


@pytest.mark.parametrize('points',[
    ((.435,0.),), tuple((i*.025,.435) for i in range(-60,61)),
    ((-.415,0.),), ((-.435,0.),(.60,0.))])
def test_guard_rejects_approach_parallel_motion_body_intrusion_and_blocked_escape(points):
    assert ready(points).output(10.1)==(0.,0.,'footprint_clearance')


@pytest.mark.parametrize('source',['scan','imu','odometry','mission','command','nav_command','battery'])
def test_escape_never_bypasses_watchdogs(source):
    s=ready(((-.435,0.),));s.invalidate(source)
    assert s.output(10.1)==(0.,0.,source+'_unavailable')


@pytest.mark.parametrize('section,key',[('corridor','turn_recenter_enabled'),('guard','clearance_recovery_enabled')])
def test_new_options_are_strict_booleans(section,key):
    p=load_profile(PROFILE);p[section][key]=True;validate_profile(p)
    p[section][key]='true'
    with pytest.raises(ValueError):validate_profile(p)
