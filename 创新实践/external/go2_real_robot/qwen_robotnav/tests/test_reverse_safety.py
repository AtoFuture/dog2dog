from pathlib import Path
import pytest
from qwen_robotnav.runtime_profile import load_profile,validate_profile
from qwen_robotnav.velocity_safety import VelocitySafety,scan_points

PROFILE=Path(__file__).parents[1]/'config/sim_runtime.yaml'


def ready(points,*,enabled=True,mission='returning'):
    g=load_profile(PROFILE)['guard']
    s=VelocitySafety(g,True,reverse_enabled=enabled)
    for k in ('nav_command','command','mission','scan','odometry','imu','battery'):s.observe(k,10.)
    s.command=(-.024,0.);s.mission=mission;s.obstacles=points;s.front=.3;s.tilt=0.
    return s


def test_reverse_is_opt_in_and_return_only():
    assert ready(((2.,0.),),enabled=False).output(10.1)[2]=='reverse_not_supported'
    assert ready(((2.,0.),),mission='exploring').output(10.1)[2]=='reverse_not_supported'
    assert ready(((2.,0.),(-2.,0.))).output(10.1)==(-.024,0.,'allowed')


def test_negative_command_is_clamped_and_curved_reverse_is_rejected():
    s=ready(((2.,0.),(-2.,0.)));s.command=(-2.,0.)
    assert s.output(10.1)==(-.03,0.,'allowed')
    s.command=(-.02,.2)
    assert s.output(10.1)==(0.,0.,'reverse_turn_not_supported')


def test_rear_sweep_uses_physical_speed_and_scan_age():
    s=ready(((-.7,0.),(2.,0.)))
    assert s.output(10.1)[0]<0
    assert s.output(10.3)==(0.,0.,'predicted_footprint_collision')
    s=ready(((-.6,0.),(2.,0.)))
    assert s.output(10.1)==(0.,0.,'predicted_footprint_collision')


def test_residual_sideways_motion_still_blocks_reverse():
    s=ready(((0.,.46),(-2.,0.)))
    s.measured_velocity=(0.,.1,0.)
    assert s.output(10.1)==(0.,0.,'predicted_footprint_collision')


def test_reverse_recovery_only_increases_measured_clearance():
    s=ready(tuple((.435,i*.025)for i in range(-40,41)))
    assert s.output(10.1)==(-.024,0.,'clearance_recovery')
    s.measured_velocity=(.05,0.,0.)
    assert s.output(10.1)==(0.,0.,'footprint_clearance')
    assert ready(((-.435,0.),)).output(10.1)==(0.,0.,'footprint_clearance')
    assert ready(((.415,0.),)).output(10.1)==(0.,0.,'footprint_clearance')


@pytest.mark.parametrize('source',['scan','odometry','imu','battery','mission','command','nav_command'])
def test_reverse_never_bypasses_watchdogs(source):
    s=ready(((-2.,0.),));s.invalidate(source)
    assert s.output(10.1)==(0.,0.,source+'_unavailable')


def test_full_scan_is_required_and_the_feature_flag_is_strict():
    assert ready(None).output(10.1)==(0.,0.,'footprint_evidence_unavailable')
    p=load_profile(PROFILE);p['navigation']['reverse_retreat_enabled']=True;validate_profile(p)
    p['navigation']['reverse_retreat_enabled']='true'
    with pytest.raises(ValueError):validate_profile(p)
