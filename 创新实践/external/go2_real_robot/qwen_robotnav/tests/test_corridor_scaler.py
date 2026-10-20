"""Real DDS transport on private topics; never connects to an actuator."""
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
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from qwen_robotnav.runtime_profile import load_profile

NAV=Path(__file__).resolve().parents[1]
SCALER=NAV.parents[2]/'baseline/g1_mapping_nav/tools/cmd_vel_scaler.py'


def test_real_corridor_guidance_before_ramp_and_fail_closed_sensor_lifecycle(tmp_path):
    import yaml
    rclpy.init()
    peer=rclpy.create_node('corridor_scaler_test_'+uuid.uuid4().hex[:8])
    prefix='/corridor_test_'+uuid.uuid4().hex[:8]
    config=dict(settings=load_profile(NAV/'config/sim_runtime.yaml')['corridor'],
        scan_topic=prefix+'/scan',base_frame='base_link',mission_topic=prefix+'/mission',
        path_topic=prefix+'/plan',
        status_topic=prefix+'/status',sensor_timeout_s=.5)
    config['settings']['steering_enabled']=True
    config['settings']['turn_settle_enabled']=True
    config['settings']['reversal_turn_enabled']=True
    params=tmp_path/'params.yaml'
    params.write_text(yaml.safe_dump({'cmd_vel_scaler':{'ros__parameters':dict(
        input_topic=prefix+'/input',output_topic=prefix+'/output',corridor_config=json.dumps(config),
        linear_scale=.075,linear_min=.02,linear_max=.03)}}))
    commands=peer.create_publisher(Twist,prefix+'/input',10)
    scans=peer.create_publisher(LaserScan,prefix+'/scan',10)
    plans=peer.create_publisher(NavPath,prefix+'/plan',10)
    missions=peer.create_publisher(String,prefix+'/mission',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    outputs=[];statuses=[]
    peer.create_subscription(Twist,prefix+'/output',lambda m:outputs.append((m.linear.x,m.angular.z)),10)
    peer.create_subscription(String,prefix+'/status',lambda m:statuses.append(json.loads(m.data)),10)
    state=dict(scan=True,mission='exploring',linear=.32,angular=0.,frame='base_link',freeze=False,plan=True,offset=.15,target_x=3.)
    stamp=peer.get_clock().now().to_msg()
    def publish():
        nonlocal stamp
        msg=Twist();msg.linear.x=state['linear'];msg.angular.z=state['angular'];commands.publish(msg)
        missions.publish(String(data=state['mission']))
        if state['plan']:
            path=NavPath();path.header.frame_id='base_link';path.header.stamp=peer.get_clock().now().to_msg()
            end=PoseStamped();end.pose.position.x=state['target_x'];path.poses.append(end);plans.publish(path)
        if state['scan']:
            msg=LaserScan();msg.header.frame_id=state['frame']
            if not state['freeze']:stamp=peer.get_clock().now().to_msg()
            msg.header.stamp=stamp
            msg.angle_min=-math.pi;msg.angle_increment=math.pi/180
            msg.range_min=.1;msg.range_max=10.
            for i in range(360):
                sine=math.sin(msg.angle_min+i*msg.angle_increment)
                distance=(state['offset']+(.7 if sine>0 else -.7))/sine if abs(sine)>1e-8 else math.inf
                msg.ranges.append(distance if distance<=10. else math.inf)
            scans.publish(msg)
    peer.create_timer(.08,publish)
    log=tmp_path/'scaler.log'
    with log.open('w') as stream:
        child=subprocess.Popen([sys.executable,str(SCALER),'--ros-args','--params-file',str(params)],
            stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    def wait_for(predicate,timeout=6.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(peer,timeout_sec=.02)
            if predicate():return
        pytest.fail(log.read_text()[-2500:]+'\n'+str(statuses[-2:]))
    def stopped():
        return len(outputs)>=4 and all(abs(x)+abs(z)<1e-7 for x,z in outputs[-4:])
    try:
        # Model fresh geometry after turning: a fixed .15m offset never
        # becomes aligned merely because more DDS messages arrive.
        wait_for(lambda: statuses and statuses[-1]['mode']=='entry_aligning'
                 and len(outputs)>=4 and all(abs(x)<1e-7 for x,_ in outputs[-4:]))
        state['offset']=.03
        wait_for(lambda: statuses and statuses[-1]['mode']=='centering' and outputs and outputs[-1][0]>.019 and outputs[-1][1]>.05)
        assert statuses[-1]['geometry']['width_m']==pytest.approx(1.4,abs=.03)
        # Exercise actual timer/ramp ordering: turn only after both command
        # ramps stop and the backend receives a full auto-rest interval.
        state.update(linear=0.,angular=.4);outputs.clear();statuses.clear()
        wait_for(lambda: statuses and statuses[-1]['mode']=='turn_settling' and stopped())
        zero_at=time.monotonic()
        assert statuses[-1]['guided']==[0.,0.]
        wait_for(lambda: statuses[-1]['mode']=='turning' and outputs[-1][1]>.1)
        assert time.monotonic()-zero_at>.5
        state.update(linear=.32,angular=0.)
        wait_for(lambda: statuses[-1]['mode']=='centering' and outputs[-1][0]>.019)
        state.update(linear=0.,angular=.4,offset=.12,target_x=-3.)
        wait_for(lambda: statuses[-1]['mode']=='reversal_turning' and outputs[-1][1]<-.2)
        assert statuses[-1]['reversal_turn_sign']==-1.
        assert all(abs(x)<1e-7 for x,_ in outputs[-4:])
        # Replanning to the forward axis releases the latched direction.
        state.update(linear=.32,angular=0.,offset=.03,target_x=3.)
        wait_for(lambda: statuses[-1]['mode']=='centering' and outputs[-1][0]>.019)
        assert statuses[-1]['reversal_turn_sign'] is None
        # A raw zero must never become a self-starting centering motion.
        state['linear']=0.;outputs.clear()
        wait_for(lambda: stopped() and statuses[-1]['mode']=='stationary')
        state['linear']=.32;state['scan']=False;outputs.clear()
        wait_for(lambda: stopped() and statuses[-1]['mode']=='scan_unavailable')
        state['scan']=True
        wait_for(lambda: outputs[-1][0]>.019 and statuses[-1]['mode']=='centering')
        state['plan']=False;outputs.clear()
        wait_for(lambda: stopped() and statuses[-1]['mode']=='awaiting_path')
        state['plan']=True
        wait_for(lambda: outputs[-1][0]>.019 and statuses[-1]['mode']=='centering')
        state['mission']='paused';outputs.clear()
        wait_for(lambda: stopped() and statuses[-1]['mode']=='inactive')
        state['mission']='returning';state['freeze']=True;outputs.clear()
        wait_for(lambda: stopped() and statuses[-1]['mode']=='scan_unavailable')
        state['freeze']=False;state['frame']='missing_laser_transform';outputs.clear()
        wait_for(lambda: stopped() and statuses[-1]['mode']=='scan_unavailable')
    finally:
        if child.poll() is None:child.send_signal(signal.SIGTERM)
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:child.kill();child.wait()
        peer.destroy_node();rclpy.shutdown()
    assert child.returncode==0,log.read_text()
    assert 'Traceback' not in log.read_text()
