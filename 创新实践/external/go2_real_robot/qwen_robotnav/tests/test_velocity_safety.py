from pathlib import Path
import copy
import pytest
from qwen_robotnav.runtime_profile import load_profile,profile_hash,ros_parameters
from qwen_robotnav.velocity_safety import VelocitySafety

PROFILE=Path(__file__).parents[1]/'config/sim_runtime.yaml'


def ready():
    p=load_profile(PROFILE);s=VelocitySafety(p['guard'],True)
    for k in ('nav_command','command','mission','scan','odometry','imu','battery'):s.observe(k,10.)
    s.mission='exploring';s.command=(.1,2.);s.front=1.;s.tilt=0.;s.obstacles=()
    return s


def test_motion_is_clipped_only_after_all_guards_pass():
    s=ready();assert s.output(10.1)==(.03,.8,'allowed')
    s.enabled=False;assert s.output(10.1)==(0.,0.,'motion_disabled')


@pytest.mark.parametrize('source',['nav_command','command','mission','scan','odometry','imu','battery'])
def test_every_lost_input_stops_both_linear_and_angular_motion(source):
    s=ready();s.invalidate(source)
    x,z,reason=s.output(10.1)
    assert x==z==0 and reason==source+'_unavailable'


def test_frozen_timestamps_do_not_refresh_sensor_watchdog():
    s=ready();s.observe('scan',10.1,100)
    assert not s.observe('scan',12.,100)
    for k in ('nav_command','command','mission','odometry','imu','battery'):s.observe(k,12.)
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
    lambda p:p['mission'].update(heartbeat_period_s=1.),lambda p:p.update(unknown=True),lambda p:p['navigation'].update(desired_linear_velocity_mps=.15),
    lambda p:p['navigation'].update(desired_linear_velocity_mps=.5),lambda p:p['scaler'].update(linear_scale=.01),
    lambda p:p['scaler'].update(linear_scale=.2)])
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
    p=load_profile(PROFILE);p['navigation'].update(robot_radius_m=.45,inflation_radius_m=.7,desired_linear_velocity_mps=.30)
    configure_navigation(tmp_path,p)
    config=yaml.safe_load((tmp_path/'nav2.yaml').read_text())
    for name in ('local_costmap','global_costmap'):
        layer=config[name][name]['ros__parameters']
        assert layer['robot_radius']==.45 and layer['inflation_layer']['inflation_radius']==.7
    assert config['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel']==.30
    controller=config['controller_server']['ros__parameters']
    assert controller['general_goal_checker']['yaw_goal_tolerance']==6.28
    assert controller['general_goal_checker']['xy_goal_tolerance']==.25
    assert controller['FollowPath']['rotate_to_heading_min_angle']==pytest.approx(3.141592653589793/2)
    assert controller['FollowPath']['min_approach_linear_velocity']==.30
    assert controller['FollowPath']['regulated_linear_scaling_min_speed']==.30
    assert controller['FollowPath']['max_angular_accel']==3.2
    assert config['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_topic']==p['topics']['planning_map']
    assert ros_parameters(p)['vision_explorer']['ros__parameters']['planning_map_topic']==p['topics']['planning_map']
    assert ros_parameters(p)['vision_explorer']['ros__parameters']['goal_obstacle_clearance_m']==pytest.approx(.7)
    assert config['planner_server']['ros__parameters']['GridBased']['tolerance']==.05
    assert config['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False
    assert controller['scan_goal_checker']['yaw_goal_tolerance']==.15
    assert 'scan_goal_checker' in (tmp_path/'scan_bt.xml').read_text()
    assert ros_parameters(p,tmp_path)['vision_explorer']['ros__parameters']['scan_bt_path']==str(tmp_path/'scan_bt.xml')
    assert yaml.safe_load((tmp_path/'slam.yaml').read_text())['slam_toolbox']['ros__parameters']['scan_topic']==p['topics']['scan']
    import ast
    from qwen_robotnav.runtime_profile import direct_navigation_commands
    source=(tmp_path/'navigation.launch.py').read_text();tree=ast.parse(source)
    packages=[k.value.value for call in ast.walk(tree) if isinstance(call,ast.Call)
              for k in call.keywords if k.arg=='package' and isinstance(k.value,ast.Constant)]
    assert 'nav2_velocity_smoother' not in packages and 'nav2_smoother' in packages
    assert 'velocity_smoother' not in config
    assert direct_navigation_commands(source,p['topics']['nav_command'])==source
    assert 'cmd_vel_nav' not in source



@pytest.mark.parametrize('mission',['exploring','returning'])
def test_side_wall_inside_full_footprint_stops_translation_and_rotation(mission):
    s=ready();s.mission=mission;s.obstacles=((0.,.42),)
    assert s.output(10.1)==(0.,0.,'footprint_clearance')


def test_obstacle_on_predicted_arc_stops_forward_but_allows_safe_in_place_turn():
    s=ready();s.command=(.02,.4);s.obstacles=((.58,0.),)
    assert s.output(10.1)==(0.,.4,'predicted_footprint_collision')
    s.command=(0.,.4)
    assert s.output(10.1)==(0.,.4,'allowed')


def test_wall_parallel_to_body_is_not_mistaken_for_forward_collision():
    s=ready();s.command=(.02,0.);s.obstacles=tuple((i*.05,.7) for i in range(-20,21))
    assert s.output(10.1)==(.02,0.,'allowed')


def test_measured_lateral_drift_is_included_in_prediction():
    s=ready();s.command=(.02,0.);s.obstacles=((0.,.55),);s.measured_velocity=(.2,.3,0.)
    assert s.output(10.1)==(0.,0.,'predicted_footprint_collision')


def test_missing_full_body_evidence_never_allows_motion():
    s=ready();s.obstacles=None
    assert s.output(10.1)==(0.,0.,'footprint_evidence_unavailable')


def test_scan_projection_uses_real_extrinsic_and_rejects_missing_coverage():
    import math
    from qwen_robotnav.velocity_safety import scan_points
    rays=[math.inf]*360;rays[180]=.3
    points=scan_points(rays,-math.pi,math.pi/180,.1,10.,(.22,0.,0.))
    assert points[0]==pytest.approx((.52,0.))
    assert scan_points([2.,2.,2.],-.5,.5,.1,10.,(0,0,0)) is None
    rays[:60]=[math.nan]*60
    assert scan_points(rays,-math.pi,math.pi/180,.1,10.,(0,0,0)) is None
    assert scan_points([math.inf]*360,-math.pi,math.pi/180,.1,10.,(0,0,0))==()


@pytest.mark.parametrize('key,value',[('clearance_margin_m',.01),('prediction_horizon_s',.1),('prediction_speed_mps',.2)])
def test_footprint_envelope_cannot_be_weakened(key,value):
    from qwen_robotnav.runtime_profile import validate_profile
    p=load_profile(PROFILE);p['guard'][key]=value
    with pytest.raises(ValueError):validate_profile(p)


def test_in_place_turn_does_not_ignore_measured_gait_drift_toward_wall():
    s=ready();s.command=(0.,.4);s.obstacles=((0.,.55),);s.measured_velocity=(0.,.3,0.)
    assert s.output(10.1)==(0.,0.,'predicted_footprint_collision')


def test_soft_clearance_boundary_keeps_safe_in_place_recovery_available():
    s=ready();s.command=(0.,.4);s.obstacles=((0.,.447),)
    assert s.output(10.1)==(0.,.4,'allowed')
    s.obstacles=((0.,.43),)
    assert s.output(10.1)==(0.,0.,'footprint_clearance')


def test_recovery_band_allows_retreat_from_wall_but_refuses_approach():
    s=ready();s.obstacles=tuple((i*.025,.447) for i in range(-20,21))
    s.command=(.02,-.4)
    assert s.output(10.1)==(.02,-.4,'allowed')
    s.command=(.02,.4)
    assert s.output(10.1)==(0.,.4,'predicted_footprint_collision')


def test_standby_does_not_wait_for_nav_command_before_first_goal():
    s=ready();s.mission='standby';s.invalidate('nav_command')
    assert s.output(10.1)==(0.,0.,'mission:standby')
