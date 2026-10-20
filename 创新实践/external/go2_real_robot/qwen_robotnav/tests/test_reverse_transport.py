"""Private DDS retreat integration; no robot actuator publisher."""
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
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry,Path as NavPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.qos import QoSProfile,DurabilityPolicy
from qwen_robotnav.runtime_profile import load_profile


def test_reverse_follows_visited_path_and_stops_for_rear_obstacle_zero_and_wide_exit(tmp_path):
    import yaml
    nav=Path(__file__).parents[1];p=load_profile(nav/'config/sim_runtime.yaml');prefix='/reverse_'+uuid.uuid4().hex[:8]
    cfg=dict(settings=p['corridor'],scan_topic=prefix+'/scan',mission_topic=prefix+'/mission',path_topic=prefix+'/path',
        status_topic=prefix+'/status',base_frame='base_link',odom_frame='odom',odometry_topic=prefix+'/odom',sensor_timeout_s=.5,
        reverse_retreat=dict(robot_radius_m=.4,desired_linear_velocity_mps=.32,
            **{k:p['guard'][k]for k in ('clearance_margin_m','prediction_speed_mps','prediction_horizon_s')}))
    params=tmp_path/'params.yaml';params.write_text(yaml.safe_dump({'cmd_vel_scaler':{'ros__parameters':dict(
        input_topic=prefix+'/input',output_topic=prefix+'/output',corridor_config=json.dumps(cfg),
        linear_scale=.075,linear_min=.02,linear_max=.03)}}))
    rclpy.init();n=rclpy.create_node('reverse_transport_'+uuid.uuid4().hex[:8]);state=dict(mission='exploring',x=0.,angular=.4,width=.6,rear=False,scan=True)
    pubs={k:n.create_publisher(t,prefix+'/'+k,10)for k,t in [('input',Twist),('scan',LaserScan),('path',NavPath),('odom',Odometry)]}
    pubs['mission']=n.create_publisher(String,prefix+'/mission',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    outputs=[];statuses=[]
    n.create_subscription(Twist,prefix+'/output',lambda m:outputs.append((time.monotonic(),m.linear.x,m.angular.z)),10)
    n.create_subscription(String,prefix+'/status',lambda m:statuses.append(json.loads(m.data)),10)
    def publish():
        stamp=n.get_clock().now().to_msg();cmd=Twist();cmd.angular.z=state['angular'];pubs['input'].publish(cmd)
        pubs['mission'].publish(String(data=state['mission']))
        odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom';odom.child_frame_id='base_link'
        odom.pose.pose.position.x=state['x'];odom.pose.pose.orientation.w=1.;pubs['odom'].publish(odom)
        path=NavPath();path.header.stamp=stamp;path.header.frame_id='base_link'
        for x in (0.,-.5,-1.,-1.5):
            pose=PoseStamped();pose.pose.position.x=x;pose.pose.orientation.w=1.;path.poses.append(pose)
        pubs['path'].publish(path)
        if state['scan']:
            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='base_link';scan.angle_min=-math.pi
            scan.angle_increment=math.pi/180;scan.range_min=.3;scan.range_max=10.
            for i in range(360):
                a=scan.angle_min+i*scan.angle_increment;s=math.sin(a);c=math.cos(a)
                d=state['width']/abs(s)if abs(s)>1e-8 else math.inf
                if state['rear']and c<0:d=min(d,-.55/c)
                scan.ranges.append(d if d<10. else math.inf)
            pubs['scan'].publish(scan)
    n.create_timer(.05,publish)
    log=tmp_path/'scaler.log';stream=log.open('w')
    child=subprocess.Popen([sys.executable,str(nav.parents[2]/'baseline/g1_mapping_nav/tools/cmd_vel_scaler.py'),
        '--ros-args','--params-file',str(params)],stdout=stream,stderr=subprocess.STDOUT)
    def wait(predicate,timeout=8.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.01)
            if predicate():return
        pytest.fail(str(statuses[-3:])+log.read_text()[-1200:])
    def stop():return len(outputs)>=4 and all(abs(x)+abs(z)<1e-6 for _,x,z in outputs[-4:])
    try:
        wait(lambda:bool(statuses))
        for i in range(11):
            state['x']=i*.05
            until=time.monotonic()+.07
            while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.01)
        state['mission']='returning'
        wait(lambda:statuses[-1]['mode']=='reverse_retreat' and outputs[-1][1]<-.019)
        assert abs(outputs[-1][2])<1e-6 and statuses[-1]['reverse_episodes']==1
        state['rear']=True
        wait(lambda:statuses[-1]['mode']=='reverse_retreat_path_blocked' and stop())
        state['rear']=False
        wait(lambda:statuses[-1]['mode']=='reverse_retreat' and outputs[-1][1]<-.019)
        state['angular']=0.
        wait(lambda:stop() and statuses[-1]['requested']==[0.,0.])
        state['angular']=.4
        wait(lambda:statuses[-1]['mode']=='reverse_retreat' and outputs[-1][1]<-.019)
        assert statuses[-1]['reverse_episodes']==1
        state['width']=.8
        wait(lambda:statuses[-1]['reverse_phase']is None and statuses[-1]['mode']=='turning' and outputs[-1][2]>.1)
        assert any(s['mode']=='reverse_retreat_complete'for s in statuses)
        assert statuses[-1]['reverse_episodes']==1
        state['width']=.6
        wait(lambda:statuses[-1]['mode']=='reverse_retreat' and outputs[-1][1]<-.019)
        state['scan']=False
        wait(lambda:statuses[-1]['mode']=='scan_unavailable' and stop())
    finally:
        child.send_signal(signal.SIGTERM)
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:child.kill();child.wait()
        stream.close();n.destroy_node();rclpy.shutdown()
    assert child.returncode==0,log.read_text()
