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
    assert controller['general_goal_checker']['yaw_goal_tolerance']==6.28
    assert controller['observation_goal_checker']['yaw_goal_tolerance']==.25
    tree=ET.parse(tmp_path/'observation_bt.xml')
    assert tree.find('.//FollowPath').get('goal_checker_id')=='observation_goal_checker'
    assert tree.find('.//Sequence[@name="NavigateThenObserve"]/Wait').get('wait_duration')=='3.0'
    assert ET.parse(tmp_path/'nav2_go2_bt.xml').find('.//FollowPath').get('goal_checker_id')=='general_goal_checker'
