"""Stop the scaler during real high-rate DDS traffic, without actuators or Gazebo."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
import pytest
pytest.importorskip('rclpy')
import rclpy
from geometry_msgs.msg import Twist

SCALER = Path(__file__).resolve().parents[4]/'baseline/g1_mapping_nav/tools/cmd_vel_scaler.py'


@pytest.mark.parametrize('signals',[(signal.SIGINT,), (signal.SIGTERM,), (signal.SIGINT,signal.SIGTERM)])
@pytest.mark.parametrize('role',['scaler','guard'])
def test_high_rate_input_shutdown_keeps_a_clean_exit(tmp_path, signals, role):
    rclpy.init()
    peer=rclpy.create_node('scaler_shutdown_test_'+uuid.uuid4().hex[:8])
    prefix='/scaler_shutdown_'+uuid.uuid4().hex[:8]
    publisher=peer.create_publisher(Twist,prefix+'/input',10)
    outputs=[]
    peer.create_subscription(Twist,prefix+'/output',lambda m:outputs.append(m.linear.x),10)
    def publish():
        msg=Twist();msg.linear.x=.1;publisher.publish(msg)
    peer.create_timer(.002,publish)
    log=tmp_path/'scaler.log'
    command=[sys.executable,str(SCALER),'--ros-args',
        '-p','input_topic:='+prefix+'/input','-p','output_topic:='+prefix+'/output']
    if role=='guard':
        import yaml
        from qwen_robotnav.runtime_profile import load_profile
        profile=load_profile(Path(__file__).parents[1]/'config/sim_runtime.yaml')
        profile['topics'].update(guarded_input=prefix+'/input',actuator_command=prefix+'/output')
        path=tmp_path/'guard.yaml';path.write_text(yaml.safe_dump(profile))
        command=[sys.executable,'-m','qwen_robotnav_ros.velocity_guard','--profile',str(path)]
    with log.open('w') as stream:
        child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and len(outputs)<20:
            rclpy.spin_once(peer,timeout_sec=.01)
        assert len(outputs)>=20,log.read_text()
        for sig in signals:
            if child.poll() is None:child.send_signal(sig)
            # Keep traffic active while shutdown is requested.
            for _ in range(5):rclpy.spin_once(peer,timeout_sec=.001)
        assert child.wait(timeout=5)==0,log.read_text()
        assert 'Traceback' not in log.read_text()
    finally:
        if child.poll() is None:child.kill();child.wait()
        peer.destroy_node();rclpy.shutdown()
