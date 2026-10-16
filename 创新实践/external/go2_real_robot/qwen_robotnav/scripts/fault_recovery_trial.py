"""Simulator-only sensor relay and moving G2 fault acceptance; no synthetic motion."""
import json
import math
import time
from pathlib import Path
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String

SCAN='/frontier_trial/recovery_scan'
TF='/frontier_trial/recovery_tf'


def prepare_navigation(root):
    """Only private generated configs change; SLAM retains source odometry TF."""
    for name in ('nav2.yaml','slam.yaml'):
        p=Path(root)/name
        p.write_text(p.read_text().replace('/robot1/velodyne',SCAN))
    p=Path(root)/'navigation.launch.py'
    source=p.read_text()
    if "('/tf', '/robot1/tf')" not in source:raise ValueError('Nav2 TF layout changed')
    p.write_text(source.replace("('/tf', '/robot1/tf')",repr(('/tf',TF))))


class SensorRelay(Node):
    def __init__(self):
        super().__init__('fault_recovery_sensor_relay')
        self.drop=None;self.events=[];self.dropped_scan=0;self.dropped_tf=0
        self.scan_pub=self.create_publisher(LaserScan,SCAN,qos_profile_sensor_data)
        self.tf_pub=self.create_publisher(TFMessage,TF,100)
        self.event_pub=self.create_publisher(String,'/frontier_trial/recovery_fault',30)
        self.create_subscription(LaserScan,'/robot1/velodyne',self.scan,qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/robot1/tf',self.tf,100)

    def scan(self,msg):
        if self.drop=='scan':self.dropped_scan+=1
        else:self.scan_pub.publish(msg)

    def tf(self,msg):
        transforms=[]
        for tr in msg.transforms:
            if self.drop=='localization' and tr.header.frame_id.strip('/')=='map' and tr.child_frame_id.strip('/')=='odom':
                self.dropped_tf+=1
            else:transforms.append(tr)
        if transforms:self.tf_pub.publish(TFMessage(transforms=transforms))

    def event(self,status,**extra):
        event=dict(status=status,stamp_ns=self.get_clock().now().nanoseconds,
                   wall_time_s=time.time(),monotonic_time=time.monotonic(),**extra)
        self.events.append(event)
        self.event_pub.publish(String(data=json.dumps(event)))
        print(json.dumps({'fault_trial':event}),flush=True)


def run_trial(monitor,g3,explorer,relay,root,home,baseline,spin_until,mode):
    from g3_integration_eval.state_machine import State
    episodes=[]
    def completed():return {e['attempt_id'] for e in monitor.events if e['status']=='succeeded' and e.get('geodesic_dist_m',0)>=.75}
    def wait(predicate,seconds,label,allow_estop=False):
        def checked():
            if not allow_estop and g3._state is State.ESTOP:raise RuntimeError('Unexpected ESTOP: '+label)
            return predicate()
        if not spin_until(checked,seconds):raise RuntimeError('Fault trial deadline: '+label)
    def movement_since(index):
        poses=monitor.poses[index:]
        return bool(len(poses)>2 and math.dist(poses[0][1:3],poses[-1][1:3])>=.12
                    and monitor.command is not None and monitor.command.linear.x>0
                    and explorer._attempt is not None)
    try:
        wait(lambda:g3._health.ready(time.monotonic()),30,'startup stable health')
        monitor.running=True;monitor.battery();monitor.command_pub.publish(String(data='EXPLORE'))
        wait(lambda:g3._state is State.EXPLORE,5,'EXPLORE command')
        if mode=='short':wait(lambda:len(completed())>=1,240,'first completed frontier')
        for source in (['scan','localization'] if mode=='short' else ['scan']):
            index=len(monitor.poses)
            wait(lambda:movement_since(index),120,'actual forward movement before '+source)
            before=completed();attempt=explorer._attempt['id'] if 'id' in explorer._attempt else explorer._attempt.get('attempt_id')
            start=time.monotonic()
            relay.drop=source
            relay.event('drop_started',source=source,mode=mode,completed_before=sorted(before),active_attempt=attempt)
            wait(lambda:g3._state is State.PAUSED,8,'fault pause')
            wait(lambda:g3._pause_released,8,'real G2 cancellation release')
            if mode=='short':
                wait(lambda:time.monotonic()-start>=7.,8,'short dropout duration')
            else:
                wait(lambda:g3._state is State.ESTOP,70,'long dropout timeout',allow_estop=True)
            relay.drop=None;relay.event('drop_released',source=source,mode=mode)
            if mode=='short':
                wait(lambda:g3._state is State.EXPLORE,20,'stable recovery resume')
                if completed()!=before:raise RuntimeError('Completed goals changed while paused')
                relay.event('resume_verified',source=source,completed_after=sorted(completed()))
            else:
                spin_until(lambda:False,8)
                if g3._state is not State.ESTOP:raise RuntimeError('Long fault automatically restarted')
                relay.event('estop_latch_verified',source=source)
            episodes.append(dict(source=source,mode=mode))
        if mode=='short':
            index=len(monitor.poses)
            wait(lambda:movement_since(index),120,'actual movement after final resume')
            wait(lambda:len(completed())>=2,240,'completed frontier after recovery')
            relay.event('task_progress_verified',completed_after=sorted(completed()))
            monitor.battery_level=.1;monitor.battery()
            wait(lambda:any(e['status']=='succeeded' for e in monitor.returns),180,'RETURN success')
            monitor.running=False;explorer.stop_navigation();spin_until(lambda:False,2)
            error=math.dist(monitor.poses[-1][1:3],baseline[1:3])
            if error>.4:raise RuntimeError('Return home error exceeds .4m')
        else:error=None
        return dict(success=True,fault_recovery_mode=mode,fault_episodes=episodes,
                    truth_home_error_m=error,home_map=home,truth_start=baseline[1:],
                    fault_relay_events=relay.events)
    finally:
        relay.drop=None
        (Path(root)/'fault_stimulus.json').write_text(json.dumps(dict(mode=mode,events=relay.events,
            dropped_scan_messages=relay.dropped_scan,dropped_map_odom_transforms=relay.dropped_tf),indent=2))
