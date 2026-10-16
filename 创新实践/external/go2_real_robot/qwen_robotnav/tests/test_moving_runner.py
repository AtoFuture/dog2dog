"""Reject inconsistent moving-person configuration before launching a simulator."""
import os
from pathlib import Path
import subprocess
import pytest

SCRIPT=Path(__file__).resolve().parents[1]/'scripts/run_slam_frontier.sh'


@pytest.mark.parametrize('settings,error',[
    ({'SLAM_RECON_TASK':'0','SLAM_RECON_SCENARIO':'moving_crossing'},'Moving scenes require recon'),
    ({'SLAM_RECON_TASK':'1','SLAM_RECON_SCENARIO':'moving_crossing','SLAM_RECON_IDENTITY_POLICY':'stationary'},'Moving scenes require recon'),
    ({'SLAM_RECON_IDENTITY_POLICY':'unknown'},'Unknown identity policy'),
])
def test_invalid_policy_fails_before_starting_simulator(tmp_path,settings,error):
    env={k:v for k,v in os.environ.items() if not k.startswith('SLAM_')}
    env.update(settings)
    env['G1_ROS_WRAPPER']='/not/an/executable'
    output=tmp_path/'must_not_be_created'
    result=subprocess.run(['bash',str(SCRIPT),str(output),'1'],env=env,capture_output=True,text=True,timeout=5)
    assert result.returncode==2 and error in result.stderr
    assert not output.exists()
