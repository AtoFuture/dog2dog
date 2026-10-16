"""The fault reaches real consumer channels and removes only the selected source."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
pytest.importorskip('rclpy')
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
path=Path(__file__).parents[1]/'scripts/fault_recovery_trial.py'
spec=importlib.util.spec_from_file_location('fault_trial',path);trial=importlib.util.module_from_spec(spec);spec.loader.exec_module(trial)


def test_scan_cut_preserves_raw_message_and_recovers():
    relay=SimpleNamespace(drop='scan',dropped_scan=0,scan_pub=Mock());msg=object()
    trial.SensorRelay.scan(relay,msg)
    assert relay.dropped_scan==1;relay.scan_pub.publish.assert_not_called()
    relay.drop=None;trial.SensorRelay.scan(relay,msg)
    relay.scan_pub.publish.assert_called_once_with(msg)


def test_tf_cut_removes_map_odom_but_keeps_leg_odometry():
    relay=SimpleNamespace(drop='localization',dropped_tf=0,tf_pub=Mock())
    a=TransformStamped();a.header.frame_id='map';a.child_frame_id='odom'
    b=TransformStamped();b.header.frame_id='odom';b.child_frame_id='base_link'
    msg=TFMessage(transforms=[a,b]);trial.SensorRelay.tf(relay,msg)
    assert relay.dropped_tf==1 and len(msg.transforms)==2
    assert relay.tf_pub.publish.call_args.args[0].transforms==[b]
    relay.drop=None;trial.SensorRelay.tf(relay,msg)
    assert relay.tf_pub.publish.call_args.args[0].transforms==[a,b]


def test_private_configs_route_nav2_and_slam_scan_without_changing_slam_tf(tmp_path):
    (tmp_path/'nav2.yaml').write_text('scan: /robot1/velodyne')
    (tmp_path/'slam.yaml').write_text('scan: /robot1/velodyne')
    (tmp_path/'navigation.launch.py').write_text("remappings=[('/tf', '/robot1/tf')]")
    trial.prepare_navigation(tmp_path)
    assert trial.SCAN in (tmp_path/'slam.yaml').read_text()
    assert trial.SCAN in (tmp_path/'nav2.yaml').read_text()
    assert trial.TF in (tmp_path/'navigation.launch.py').read_text()
    assert not (tmp_path/'slam_navigation.launch.py').exists()


@pytest.mark.parametrize('overrides',[{'SLAM_FAULT_RECOVERY':'invalid'},
    {'SLAM_FAULT_RECOVERY':'short','SLAM_RECON_TASK':'1'},
    {'SLAM_FAULT_RECOVERY':'long','SLAM_DYNAMIC_OBSTACLE':'crossing'}])
def test_invalid_fault_combinations_rejected_before_start(tmp_path,overrides):
    import os,subprocess
    env=dict(os.environ,SLAM_RECON_TASK='0',SLAM_MISSION_BRAIN='0',SLAM_PERCEPTION='0',
        SLAM_JOINT_RECON='0',SLAM_DYNAMIC_OBSTACLE='',SLAM_INJECT_RETURN_HOLD_S='0')
    env.update(overrides)
    output=tmp_path/'not_created'
    result=subprocess.run(['bash',str(path.with_name('run_slam_frontier.sh')),str(output),'1','--prepare-only'],
        env=env,capture_output=True,text=True)
    assert result.returncode==2 and not output.exists()
