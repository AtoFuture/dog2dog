"""Observe a standalone navigation runtime; this process cannot publish velocity."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer,TransformListener
from qwen_robotnav.runtime_profile import load_profile,profile_hash


class Observer(Node):
    def __init__(self,profile):
        super().__init__('runtime_validation_observer');self.profile=profile
        self.poses=[];self.events=[];self.returns=[];self.states=[];self.status=[];self.command_samples=[];self.latest_map=None;self.first_area=None;self.exploration_results=[];self.operator_commands=[]
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.create_subscription(String,'/g3/exploration_result',lambda m:self.exploration_results.append(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.command_pub=self.create_publisher(String,profile['topics']['command'],10)
        self.create_subscription(TFMessage,'/frontier_trial/gt_pose',self.truth,10)
        self.create_subscription(String,'/exploration/events',lambda m:self.events.append(json.loads(m.data)),30)
        self.create_subscription(String,'/g3/return_event',lambda m:self.returns.append(json.loads(m.data)),30)
        self.create_subscription(String,profile['topics']['guard_status'],lambda m:self.status.append(json.loads(m.data)),30)
        self.create_subscription(String,profile['topics']['mission'],lambda m:self.states.append(m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(OccupancyGrid,profile['topics']['map'],self.mapping,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist,profile['topics']['actuator_command'],lambda m:self.command_samples.append((time.monotonic(),m.linear.x,m.angular.z)),10)

    def command(self, value):
        self.operator_commands.append(value)
        self.command_pub.publish(String(data=value))

    def truth(self,msg):
        for t in msg.transforms:
            if t.child_frame_id=='robot1_my_bot':
                p=t.transform.translation;self.poses.append((time.monotonic(),p.x,p.y,p.z))

    def mapping(self,msg):
        self.latest_map=msg
        if self.first_area is None:self.first_area=sum(v>=0 for v in msg.data)*msg.info.resolution**2


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--profile',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--expected-domain',type=int,required=True)
    p.add_argument('--goals',type=int,default=2);p.add_argument('--exploration-timeout',type=float,default=300.)
    p.add_argument('--candidate-selector',choices=('deepseek','deterministic'))
    p.add_argument('--mission-instruction')
    p.add_argument('--autonomous-completion',action='store_true',help='Observe runtime termination; never send RETURN')
    a=p.parse_args();profile=load_profile(a.profile);root=a.output_dir.resolve();topics=profile['topics']
    if a.expected_domain in (0,42,43) or os.environ.get('ROS_DOMAIN_ID')!=str(a.expected_domain) or not os.environ.get('IGN_PARTITION') or a.goals<1:p.error('Require isolated Gazebo domain and positive goals')
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-r','/tf:='+topics['tf'],'-r','/tf_static:='+topics['tf_static']],signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(sig,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    node=Observer(profile);executor=SingleThreadedExecutor();executor.add_node(node)
    children=[];streams=[];recorder=None;runtime=None;report=dict(success=False);baseline=None
    def start(command,name):
        stream=(root/name).open('w');streams.append(stream)
        child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True);children.append(child);return child
    def wait(predicate,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.05)
            if runtime is not None and runtime.poll() is not None:raise RuntimeError('Standalone runtime exited')
            if recorder is not None and recorder.poll() is not None:raise RuntimeError('Recorder exited early')
            if predicate():return
        raise RuntimeError('Runtime validation deadline')
    runtime_args=[]
    if a.candidate_selector:runtime_args+=['--candidate-selector',a.candidate_selector]
    if a.mission_instruction:runtime_args+=['--mission-instruction',a.mission_instruction]
    try:
        start(['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge','/world/world_demo/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '--ros-args','-r','__node:=runtime_truth_observer','-r','/world/world_demo/pose/info:=/frontier_trial/gt_pose'],'gt_bridge.log')
        wait(lambda:node.poses and .18<node.poses[-1][3]<.35,150)
        baseline=node.poses[-1]
        recorder=start(['ros2','bag','record','--include-hidden-topics','-o',str(root/'bag'),'/clock',
            *topics.values(),'/exploration/events','/g3/return_event','/g3/recovery_event','/frontier_trial/gt_pose',
            '/navigation/handoff_request','/navigation/handoff_ack','/g3/exploration_session','/exploration/completion','/g3/exploration_result','/brain/decision','/mission/context','/exploration/candidates',topics['action']+'/_action/status'],'bag.log')
        runtime=start(['python3',str(Path(__file__).with_name('run_navigation_runtime.py')),'--profile',str(a.profile),
            '--navigation-dir',str(root),'--expected-domain',str(a.expected_domain),
            *runtime_args,'--enable-motion'],'runtime_supervisor.log')
        def ready():
            if not node.status or not node.latest_map or not node.states:return False
            try:tr=node.tf.lookup_transform(profile['frames']['map'],profile['frames']['base'],Time()).transform
            except Exception:return False
            if math.dist((tr.translation.x,tr.translation.y),(profile['mission']['home_x'],profile['mission']['home_y']))>.1:raise RuntimeError('Configured home differs from initial map pose')
            return node.status[-1]['reason']=='mission:standby'
        wait(ready,90)
        effective=load_profile(root/'runtime_profile.yaml')
        profile=effective
        publishers=node.get_publishers_info_by_topic(topics['actuator_command'])
        if len(publishers)!=1 or publishers[0].node_name!='velocity_guard':raise RuntimeError('Actuator publisher is not the standalone guard')
        # Wait for G3 localization debounce before allowing exploration.
        end=time.monotonic()+4
        while time.monotonic()<end:executor.spin_once(timeout_sec=.05)
        node.command('EXPLORE')
        wait(lambda:'exploring' in node.states,10)
        if a.autonomous_completion:
            wait(lambda:bool(node.exploration_results),a.exploration_timeout)
        else:
            wait(lambda:sum(e['status']=='succeeded' and e.get('geodesic_dist_m',0)>=.75 for e in node.events)>=a.goals,a.exploration_timeout)
            node.command('RETURN')
        wait(lambda:any(e['status']=='succeeded' for e in node.returns),180)
        wait(lambda:node.states[-1]=='standby',5)
        end=time.monotonic()+2
        while time.monotonic()<end:executor.spin_once(timeout_sec=.05)
        error=math.dist(node.poses[-1][1:3],baseline[1:3])
        report=dict(success=error<=.4,candidate_selector=profile['decision']['candidate_selector'],truth_home_error_m=error,profile_sha256=profile_hash(profile),
            completed_goals=sum(e['status']=='succeeded' and e.get('geodesic_dist_m',0)>=.75 for e in node.events),
            observer_publishes_velocity=False,actuator_publishers=[v.node_name for v in publishers],
            runtime_uses_ground_truth=False,guard_moving_outputs=max((s['moving_outputs'] for s in node.status),default=0),
            initial_known_area_m2=node.first_area,final_known_area_m2=sum(v>=0 for v in node.latest_map.data)*node.latest_map.info.resolution**2)
    except (Exception,KeyboardInterrupt) as exc:report=dict(success=False,error=str(exc))
    finally:
        node.command('ESTOP')
        end=time.monotonic()+1
        while time.monotonic()<end:executor.spin_once(timeout_sec=.05)
        for child in reversed(children):
            if child.poll() is None:
                try:os.killpg(child.pid,signal.SIGINT)
                except ProcessLookupError:pass
        for child in reversed(children):
            try:child.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        report.update(autonomous_completion=a.autonomous_completion,operator_commands=node.operator_commands,exploration_results=node.exploration_results,recorder_exit_code=recorder.returncode if recorder else None,runtime_exit_code=runtime.returncode if runtime else None,
            recording_complete=(root/'bag/metadata.yaml').is_file(),return_events=node.returns,guard_status=node.status[-1] if node.status else None)
        if report['recorder_exit_code']!=0 or report['runtime_exit_code']!=0 or not report['recording_complete']:report['success']=False
        (root/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
        for stream in streams:stream.close()
        executor.remove_node(node);node.destroy_node();executor.shutdown();rclpy.shutdown()
    return 0 if report['success'] else 1


if __name__=='__main__':raise SystemExit(main())
