from pathlib import Path
import copy
import pytest
from qwen_robotnav.runtime_profile import load_profile,profile_hash,ros_parameters
from qwen_robotnav.velocity_safety import VelocitySafety

PROFILE=Path(__file__).parents[1]/'config/sim_runtime.yaml'


def ready():
    p=load_profile(PROFILE);s=VelocitySafety(p['guard'],True)
    for k in ('command','mission','scan','odometry','imu','battery'):s.observe(k,10.)
    s.mission='exploring';s.command=(.1,2.);s.front=1.;s.tilt=0.
    return s


def test_motion_is_clipped_only_after_all_guards_pass():
    s=ready();assert s.output(10.1)==(.03,.8,'allowed')
    s.enabled=False;assert s.output(10.1)==(0.,0.,'motion_disabled')


@pytest.mark.parametrize('source',['command','mission','scan','odometry','imu','battery'])
def test_every_lost_input_stops_both_linear_and_angular_motion(source):
    s=ready();s.invalidate(source)
    x,z,reason=s.output(10.1)
    assert x==z==0 and reason==source+'_unavailable'


def test_frozen_timestamps_do_not_refresh_sensor_watchdog():
    s=ready();s.observe('scan',10.1,100)
    assert not s.observe('scan',12.,100)
    for k in ('command','mission','odometry','imu','battery'):s.observe(k,12.)
    assert s.output(12.)==(0.,0.,'scan_unavailable')


@pytest.mark.parametrize('state',['paused','estop','standby','unknown'])
def test_task_states_never_allow_late_commands(state):
    s=ready();s.mission=state
    assert s.output(10.1)==(0.,0.,'mission:'+state)


def test_obstacle_allows_only_turning_and_reverse_is_refused():
    s=ready();s.front=.2
    assert s.output(10.1)==(0.,.8,'front_obstacle')
    s.command=(-.02,.1);assert s.output(10.1)==(0.,0.,'reverse_not_supported')


def test_excessive_tilt_and_foreign_publishers_stop_output():
    s=ready();s.tilt=.7;assert s.output(10.1)==(0.,0.,'excessive_tilt')
    s.tilt=0;s.publisher_conflict=True
    assert s.output(10.1)==(0.,0.,'multiple_actuator_publishers')


def test_resolved_configuration_connects_one_velocity_chain():
    p=load_profile(PROFILE);params=ros_parameters(p)
    scaler=params['cmd_vel_scaler']['ros__parameters']
    assert scaler['output_topic']==p['topics']['guarded_input']
    assert scaler['input_topic']==p['topics']['nav_command']
    assert p['topics']['actuator_command']!=scaler['output_topic']
    assert params['g3_state_machine']['ros__parameters']['state_publish_period_sec']<p['guard']['mission_timeout_s']/2
    assert profile_hash(p)==profile_hash(copy.deepcopy(p))


@pytest.mark.parametrize('mutation',[lambda p:p.update(backend='real_go2'),lambda p:p['guard'].update(linear_max=.3),
    lambda p:p['topics'].update(guarded_input=p['topics']['actuator_command']),lambda p:p['guard'].update(command_timeout_s=float('nan')),
    lambda p:p['mission'].update(heartbeat_period_s=1.),lambda p:p.update(unknown=True)])
def test_unmeasured_or_inconsistent_profiles_are_rejected(tmp_path,mutation):
    import yaml
    p=load_profile(PROFILE);mutation(p);path=tmp_path/'bad.yaml';path.write_text(yaml.safe_dump(p))
    with pytest.raises(ValueError):load_profile(path)


def test_generated_nav2_uses_profile_geometry_and_speed(tmp_path):
    import subprocess,sys,yaml
    pytest.importorskip('ament_index_python')
    from qwen_robotnav.runtime_profile import configure_navigation
    script=PROFILE.parents[1]/'scripts/prepare_amcl_navigation.py'
    subprocess.run([sys.executable,str(script),'--localization','slam','--output-dir',str(tmp_path)],check=True,capture_output=True,text=True)
    p=load_profile(PROFILE);p['navigation'].update(robot_radius_m=.45,inflation_radius_m=.7,desired_linear_velocity_mps=.15)
    configure_navigation(tmp_path,p)
    config=yaml.safe_load((tmp_path/'nav2.yaml').read_text())
    for name in ('local_costmap','global_costmap'):
        layer=config[name][name]['ros__parameters']
        assert layer['robot_radius']==.45 and layer['inflation_layer']['inflation_radius']==.7
    assert config['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel']==.15
    assert yaml.safe_load((tmp_path/'slam.yaml').read_text())['slam_toolbox']['ros__parameters']['scan_topic']==p['topics']['scan']
