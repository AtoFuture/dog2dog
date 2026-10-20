"""Recovery ramp ordering over private DDS topics, without an actuator."""
import json
import math
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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from qwen_robotnav.runtime_profile import load_profile


def test_measured_recenter_uses_settled_ramps_and_cancels_on_zero_or_stale_scan(tmp_path):
    import yaml
    nav=Path(__file__).parents[1]
    rclpy.init()
    peer=rclpy.create_node('recenter_transport_'+uuid.uuid4().hex[:8])
    prefix='/recenter_'+uuid.uuid4().hex[:8]
    settings=dict(load_profile(nav/'config/sim_runtime.yaml')['corridor'],
                  turn_recenter_enabled=True,turn_settle_enabled=True)
    cfg=dict(settings=settings,scan_topic=prefix+'/scan',mission_topic=prefix+'/mission',
             path_topic=prefix+'/path',status_topic=prefix+'/status',base_frame='base_link',sensor_timeout_s=.5)
    params=tmp_path/'params.yaml'
    params.write_text(yaml.safe_dump({'cmd_vel_scaler':{'ros__parameters':dict(
        input_topic=prefix+'/input',output_topic=prefix+'/output',corridor_config=json.dumps(cfg),
        linear_scale=.075,linear_min=.02,linear_max=.03)}}))
    command=peer.create_publisher(Twist,prefix+'/input',10)
    scans=peer.create_publisher(LaserScan,prefix+'/scan',10)
    mission=peer.create_publisher(String,prefix+'/mission',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    outputs=[];statuses=[]
    peer.create_subscription(Twist,prefix+'/output',lambda m:outputs.append((time.monotonic(),m.linear.x,m.angular.z)),10)
    peer.create_subscription(String,prefix+'/status',lambda m:statuses.append((time.monotonic(),json.loads(m.data))),10)
    state=dict(offset=.15,scan=True,angular=.4,entrance_start=.8)
    def publish():
        msg=Twist();msg.angular.z=state['angular'];command.publish(msg)
        mission.publish(String(data='returning'))
        if state['scan']:
            scan=LaserScan();scan.header.stamp=peer.get_clock().now().to_msg();scan.header.frame_id='base_link'
            scan.angle_min=-math.pi;scan.angle_increment=math.pi/180;scan.range_min=.3;scan.range_max=10.
            # Corridor normal points forward in the body frame. Forward motion
            # moves away from the closer wall behind the robot.
            for i in range(360):
                c=math.cos(scan.angle_min+i*scan.angle_increment)
                distance=(state['offset']+(.7 if c>0 else -.7))/c if abs(c)>1e-8 else math.inf
                if (state['entrance_start'] is not None and math.isfinite(distance)
                        and distance*math.sin(scan.angle_min+i*scan.angle_increment)<state['entrance_start']):
                    distance=math.inf
                scan.ranges.append(distance if distance<10. else math.inf)
            scans.publish(scan)
    peer.create_timer(.08,publish)
    log=tmp_path/'scaler.log'
    with log.open('w') as stream:
        child=subprocess.Popen([sys.executable,str(nav.parents[2]/'baseline/g1_mapping_nav/tools/cmd_vel_scaler.py'),
            '--ros-args','--params-file',str(params)],stdout=stream,stderr=subprocess.STDOUT)
    def wait(predicate,timeout=7.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(peer,timeout_sec=.02)
            if predicate():return
        pytest.fail(str(statuses[-3:])+log.read_text()[-1500:])
    def stopped():
        return len(outputs)>=4 and all(abs(x)+abs(z)<1e-7 for _,x,z in outputs[-4:])
    try:
        wait(lambda:statuses and statuses[-1][1]['confirmed']
             and statuses[-1][1]['mode']=='turning' and outputs[-1][2]>.1)
        assert statuses[-1][1]['recovery_episodes']==0
        state['entrance_start']=None
        wait(lambda:any(s['mode']=='turn_recovery_settling' for _,s in statuses))
        began=next(t for t,s in statuses if s['mode']=='turn_recovery_settling')
        wait(lambda:statuses[-1][1]['mode']=='turn_recentering' and outputs[-1][1]>.019)
        assert outputs[-1][0]-began>=.7
        assert abs(outputs[-1][2])<1e-7
        state['offset']=.025
        wait(lambda:stopped())
        wait(lambda:statuses[-1][1]['mode']=='turning' and outputs[-1][2]>.1)
        state['offset']=.15
        wait(lambda:statuses[-1][1]['mode']=='turn_recentering' and outputs[-1][1]>.019)
        episodes=statuses[-1][1]['recovery_episodes']
        state['angular']=0.
        wait(lambda:stopped() and statuses[-1][1]['requested']==[0.,0.]
             and statuses[-1][1]['recovery_phase'] is None)
        assert statuses[-1][1]['recovery_resume_pending']
        state['angular']=.4
        wait(lambda:statuses[-1][1]['mode']=='turn_recentering' and outputs[-1][1]>.019)
        assert statuses[-1][1]['recovery_episodes']==episodes
        state['angular']=.4;state['scan']=False
        wait(lambda:statuses[-1][1]['mode']=='scan_unavailable' and stopped())
    finally:
        child.send_signal(signal.SIGTERM)
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:child.kill();child.wait()
        peer.destroy_node();rclpy.shutdown()
    assert child.returncode==0,log.read_text()
