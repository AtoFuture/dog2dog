"""Only G2 owns exploration; selectors share one runtime and one effective config."""
import importlib.util
from pathlib import Path
import subprocess
import os
import pytest
from qwen_robotnav.runtime_profile import load_profile,ros_parameters,expected_runtime_roles,runtime_exit_ok,profile_hash

ROOT=Path(__file__).parents[1]


def module(path):
    spec=importlib.util.spec_from_file_location('convergence_'+path.stem,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result


@pytest.mark.parametrize('selector',['deepseek','deterministic'])
def test_selector_changes_only_brain_ranking_not_goal_or_velocity_owners(selector):
    p=load_profile(ROOT/'config/sim_runtime.yaml');p['decision']['candidate_selector']=selector
    roles=expected_runtime_roles(p);params=ros_parameters(p)
    assert {'frontier','g3','guard','scaler','navigation','battery'}<=roles
    assert ('mission_brain' in roles)==(selector=='deepseek')
    assert not {'nav_node','waypoint_bridge','monitor'} & roles
    assert params['vision_explorer']['ros__parameters']['use_mission_brain']==(selector=='deepseek')
    assert ('mission_brain' in params)==(selector=='deepseek')


def test_resolved_cli_decision_is_saved_in_profile_before_hash(monkeypatch):
    run=module(ROOT/'scripts/run_navigation_runtime.py')
    for key in ['MISSION_BRAIN_BASE_URL','DEEPSEEK_VLM_BASE_URL','MISSION_BRAIN_MODEL','DEEPSEEK_VLM_MODEL']:monkeypatch.delenv(key,raising=False)
    args=run._parser().parse_args(['--profile','unused','--navigation-dir','unused','--expected-domain','226','--candidate-selector','deterministic','--mission-instruction','Inspect the corridor.'])
    p=load_profile(ROOT/'config/sim_runtime.yaml');before=profile_hash(p);run._resolve_decision(args,p)
    assert p['decision']['candidate_selector']=='deterministic' and profile_hash(p)!=before
    assert ros_parameters(p)['vision_explorer']['ros__parameters']['mission_instruction']=='Inspect the corridor.'
    assert p['decision']['brain_model']=='deepseek-flash'


@pytest.mark.parametrize('codes',[{'guard':1},{'guard':0},{'guard':0,'frontier':0,'extra':0}])
def test_supervisor_zero_cannot_hide_failed_or_missing_children(codes):
    assert not runtime_exit_ok(dict(exit_code=0,child_exit_codes=codes),{'guard','frontier'})
    assert runtime_exit_ok(dict(exit_code=0,child_exit_codes={'guard':0,'frontier':0}),{'guard','frontier'})


def test_current_validator_cannot_own_actuator_publisher():
    source=(ROOT/'scripts/validate_slam_frontier.py').read_text()
    assert 'from validate_runtime import main' in source and 'class Monitor' not in source
    assert "create_publisher(Twist" not in (ROOT/'scripts/validate_runtime.py').read_text()
    assert (ROOT/'scripts/legacy/validate_slam_frontier.py').is_file()


def test_frozen_scenario_requires_explicit_historical_mode(tmp_path):
    env={k:v for k,v in os.environ.items() if not k.startswith('SLAM_')};env['SLAM_RECON_TASK']='1'
    result=subprocess.run(['bash',str(ROOT/'scripts/run_slam_frontier.sh'),str(tmp_path/'never'),'1'],env=env,capture_output=True,text=True)
    assert result.returncode==2 and 'SLAM_LEGACY_VALIDATION=1' in result.stderr
    assert not (tmp_path/'never').exists()


def test_waypoint_launch_is_disabled_by_default_and_explicit_mode_still_checks_exclusivity(monkeypatch):
    pytest.importorskip('launch')
    from launch import LaunchContext
    launch=module(ROOT/'ros2_ws/src/qwen_robotnav_ros/launch/deepseek_nav2.launch.py')
    context=LaunchContext();context.launch_configurations['enable_experimental_waypoints']='false'
    with pytest.raises(RuntimeError,match='archived experimental'):launch._experimental_gate(context)
    context.launch_configurations['enable_experimental_waypoints']='true'
    def occupied():raise RuntimeError('another runtime exists')
    monkeypatch.setattr(launch,'_assert_experimental_exclusive',occupied)
    with pytest.raises(RuntimeError,match='another runtime'):launch._experimental_gate(context)


@pytest.mark.parametrize('stop_signal',['SIGINT','SIGTERM'])
def test_guard_process_sends_zero_and_exits_cleanly_before_ros_shutdown(tmp_path,stop_signal):
    pytest.importorskip('rclpy')
    import signal,sys,time
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    rclpy.init();peer=rclpy.create_node('guard_shutdown_observer');outputs=[];statuses=[]
    p=load_profile(ROOT/'config/sim_runtime.yaml')
    peer.create_subscription(Twist,p['topics']['actuator_command'],lambda m:outputs.append((m.linear.x,m.angular.z)),10)
    peer.create_subscription(String,p['topics']['guard_status'],lambda m:statuses.append(m.data),10)
    with (tmp_path/'guard.log').open('w') as log:
        child=subprocess.Popen([sys.executable,'-m','qwen_robotnav_ros.velocity_guard','--profile',str(ROOT/'config/sim_runtime.yaml'),
            '--ros-args','-p','use_sim_time:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            end=time.monotonic()+6
            while time.monotonic()<end and not statuses:rclpy.spin_once(peer,timeout_sec=.03)
            assert statuses and outputs
            os.killpg(child.pid,getattr(signal,stop_signal))
            end=time.monotonic()+5
            while time.monotonic()<end and child.poll() is None:rclpy.spin_once(peer,timeout_sec=.03)
            assert child.poll()==0
            assert outputs[-1]==(0.,0.)
        finally:
            if child.poll() is None:os.killpg(child.pid,signal.SIGKILL);child.wait()
            peer.destroy_node();rclpy.shutdown()
    assert 'Traceback' not in (tmp_path/'guard.log').read_text()


def test_direct_velocity_launch_cannot_start_by_default():
    pytest.importorskip('launch')
    from launch import LaunchContext
    launch=module(ROOT/'ros2_ws/src/qwen_robotnav_ros/launch/robot_nav.launch.py')
    context=LaunchContext();context.launch_configurations['enable_legacy_direct_control']='false'
    with pytest.raises(RuntimeError,match='archived'):launch._legacy_gate(context)


@pytest.mark.parametrize('key,value',[('candidate_selector','waypoints'),('brain_base_url','http://example.invalid'),('mission_instruction','   '),('brain_max_candidates',True)])
def test_decision_profile_refuses_unsupported_or_unbounded_modes(key,value):
    from qwen_robotnav.runtime_profile import validate_profile
    p=load_profile(ROOT/'config/sim_runtime.yaml');p['decision'][key]=value
    with pytest.raises(ValueError):validate_profile(p)


def test_public_validation_scripts_do_not_publish_velocity_and_old_amcl_requires_opt_in(tmp_path):
    import sys
    for path in (ROOT/'scripts').glob('validate_*.py'):
        assert 'create_publisher(Twist' not in path.read_text(),path.name
    result=subprocess.run([sys.executable,str(ROOT/'scripts/validate_amcl_navigation.py')],capture_output=True,text=True)
    assert result.returncode!=0 and '--legacy-validation' in result.stderr
    assert 'Traceback' not in result.stderr


@pytest.mark.parametrize('field,value', [('task_completion_enabled',False),('max_region_failures',1.5),
    ('max_consecutive_failures',True),('candidate_wait_s',30),('exploration_budget_s',float('inf'))])
def test_exploration_limits_cannot_silently_disable_or_escape_bounds(field,value):
    from qwen_robotnav.runtime_profile import load_profile,validate_profile
    profile=load_profile(Path(__file__).parents[1]/'config/sim_runtime.yaml')
    profile['explorer'][field]=value
    with pytest.raises(ValueError): validate_profile(profile)


def test_completion_is_enabled_in_both_task_owners_from_same_profile():
    from qwen_robotnav.runtime_profile import load_profile,ros_parameters
    profile=load_profile(Path(__file__).parents[1]/'config/sim_runtime.yaml')
    params=ros_parameters(profile)
    assert params['g3_state_machine']['ros__parameters']['enable_exploration_completion'] is True
    assert params['vision_explorer']['ros__parameters']['max_region_failures']==2
    assert params['vision_explorer']['ros__parameters']['task_completion_enabled'] is True
