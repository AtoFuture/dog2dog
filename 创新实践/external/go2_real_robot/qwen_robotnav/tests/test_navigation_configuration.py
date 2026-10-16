"""Every velocity consumer must use the EKF output after raw odometry was renamed."""
from pathlib import Path
import subprocess
import sys
import pytest
import yaml


def test_generated_controller_receives_filtered_odometry(tmp_path):
    pytest.importorskip('ament_index_python')
    script=Path(__file__).resolve().parents[1]/'scripts/prepare_amcl_navigation.py'
    subprocess.run([sys.executable,str(script),'--localization','slam','--output-dir',str(tmp_path)],
                   check=True,capture_output=True,text=True)
    config=yaml.safe_load((tmp_path/'nav2.yaml').read_text())
    for node in ('controller_server','velocity_smoother','bt_navigator'):
        assert config[node]['ros__parameters']['odom_topic']=='/robot1/odometry/filtered'

    import xml.etree.ElementTree as ET
    controller=config['controller_server']['ros__parameters']
    assert controller['progress_checker']['plugin']=='qwen_robotnav_progress::StableProgressChecker'
    assert controller['progress_checker']['median_window_s']==2.
    assert controller['progress_checker']['movement_time_allowance']==12.
    for name in ('local_costmap','global_costmap'):
        layer=config[name][name]['ros__parameters']
        assert layer['robot_radius']==.4 and layer['inflation_layer']['inflation_radius']==.65
    assert controller['general_goal_checker']['xy_goal_tolerance']==.25
    assert controller['general_goal_checker']['yaw_goal_tolerance']==6.28
    assert controller['observation_goal_checker']['yaw_goal_tolerance']==.25
    tree=ET.parse(tmp_path/'observation_bt.xml')
    assert tree.find('.//FollowPath').get('goal_checker_id')=='observation_goal_checker'
    assert tree.find('.//Sequence[@name="NavigateThenObserve"]/Wait').get('wait_duration')=='3.0'
    assert ET.parse(tmp_path/'nav2_go2_bt.xml').find('.//FollowPath').get('goal_checker_id')=='general_goal_checker'
