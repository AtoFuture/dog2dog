"""Real DDS takes remain valid when mission nodes receive stop signals."""
import os
import signal
import subprocess
import sys
import time
import uuid

import pytest
pytest.importorskip('rclpy')
import rclpy
from std_msgs.msg import String


@pytest.mark.parametrize('role',['mission_brain_node','state_machine_node'])
@pytest.mark.parametrize('signals',[(signal.SIGINT,),(signal.SIGTERM,),(signal.SIGINT,signal.SIGTERM)])
def test_signal_during_high_rate_mission_traffic(tmp_path, role, signals):
    rclpy.init()
    peer=rclpy.create_node('lifecycle_test_'+uuid.uuid4().hex[:8])
    prefix='/lifecycle_'+uuid.uuid4().hex[:8]
    input_topic=prefix+'/context' if role=='mission_brain_node' else prefix+'/command'
    publisher=peer.create_publisher(String,input_topic,10)
    # No candidates or EXPLORE commands: this test never calls an API or moves.
    payload='{"mission_state":"STANDBY"}' if role=='mission_brain_node' else 'standby'
    peer.create_timer(.002,lambda:publisher.publish(String(data=payload)))
    params={'context_topic':input_topic,'candidates_topic':prefix+'/candidates',
            'decision_topic':prefix+'/decision','mission_state_topic':prefix+'/mission'}
    if role=='state_machine_node':
        params={'command_topic':input_topic,'state_topic':prefix+'/state',
                'mission_state_topic':prefix+'/mission','battery_topic':prefix+'/battery',
                'return_event_topic':prefix+'/return','navigate_action':prefix+'/navigate'}
    command=[sys.executable,'-m','g3_integration_eval.'+role,'--ros-args']
    for key,value in params.items():command+=['-p',key+':='+value]
    log=tmp_path/'node.log'
    env=dict(os.environ,MISSION_BRAIN_API_KEY='local-test-no-network')
    with log.open('w') as stream:
        child=subprocess.Popen(command,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and publisher.get_subscription_count()==0:
            rclpy.spin_once(peer,timeout_sec=.01)
            assert child.poll() is None,log.read_text()
        assert publisher.get_subscription_count()>0,log.read_text()
        for _ in range(100):rclpy.spin_once(peer,timeout_sec=.002)
        for sig in signals:
            if child.poll() is None:child.send_signal(sig)
            for _ in range(5):rclpy.spin_once(peer,timeout_sec=.001)
        assert child.wait(timeout=5)==0,log.read_text()
        assert 'Traceback' not in log.read_text()
    finally:
        if child.poll() is None:child.kill();child.wait()
        peer.destroy_node();rclpy.shutdown()
