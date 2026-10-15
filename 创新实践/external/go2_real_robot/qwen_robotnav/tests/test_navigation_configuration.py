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
