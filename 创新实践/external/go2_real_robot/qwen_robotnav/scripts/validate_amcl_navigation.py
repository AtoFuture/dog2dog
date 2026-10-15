"""Run a bounded, simulator-only AMCL / DeepSeek / G3 return regression.

Requires generated AMCL navigation files, a running isolated Go2 simulator,
ROS Humble, G3 Python package and DeepSeek environment variables. Ground truth
is recorded for evaluation and the standing-posture guard; never published as
localization TF or odometry. No hardware SDK is imported.
"""

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, BatteryState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rclpy.signals import SignalHandlerOptions

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output-dir',type=Path,required=True)
parser.add_argument('--expected-domain',type=int,required=True)
parser.add_argument('--cycles',type=int,default=3)
parser.add_argument('--home-x',type=float,default=6.86)
parser.add_argument('--home-y',type=float,default=6.0)
args=parser.parse_args()
if int(os.environ.get('ROS_DOMAIN_ID','0'))!=args.expected_domain or args.expected_domain in (0,42,43):
    parser.error('Select an isolated simulator ROS domain, matching --expected-domain')
if not os.environ.get('IGN_PARTITION'):
    parser.error('IGN_PARTITION must identify the isolated simulator')
if args.cycles<1:
    parser.error('--cycles must be positive')
ROOT=args.output_dir.resolve()
if not (ROOT/'amcl_navigation.launch.py').is_file():
    parser.error('Generate AMCL navigation into --output-dir first')
PROJECT=Path(__file__).resolve().parents[4]


class Monitor(Node):
    def __init__(self):
        super().__init__('deepseek_nav2_trial_monitor')
        self.return_events=[];self.states=[];self.battery_level=0.8;self.cycle_results=[]
        self.images=0;self.scans=0;self.poses=[];self.events=[];self.plans=[]
        self.cmd=None;self.cmd_time=0;self.scan_time=0;self.front=float('inf')
        self.minimum_front=float('inf');self.forwarded=0;self.blocked=0
        self.frame=None;self.running=False;self.costmap_messages=0;self.costmap_obstacles=0
        self.cv=CvBridge();self.estimates=[];self.odom_samples=[]
        self.battery_pub=self.create_publisher(BatteryState,'/nav2_trial/battery',10)
        self.command_pub=self.create_publisher(String,'/g3/command',10)
        self.create_subscription(String,'/g3/return_event',self.return_event,10)
        self.create_subscription(String,'/mission_state',self.state,10)
        self.create_timer(0.5,self.battery)
        self.pub=self.create_publisher(Twist,'/robot1/cmd_vel',10)
        self.create_subscription(PoseWithCovarianceStamped,'/robot1/amcl_pose',self.estimate,10)
        self.create_subscription(Odometry,'/robot1/odometry/filtered',self.odometry,10)
        self.create_subscription(Image,'/robot1/color/image_raw',self.image,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/robot1/velodyne',self.scan,qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/nav2_trial/gt_pose',self.pose,10)
        self.create_subscription(Twist,'/nav2_trial/scaled_cmd',self.command,10)
        self.create_subscription(String,'/deepseek/nav2_events',self.event,10)
        self.create_subscription(NavPath,'/robot1/plan',self.plan,10)
        self.create_subscription(OccupancyGrid,'/robot1/local_costmap/costmap',self.costmap,10)
        self.create_timer(0.05,self.control)

    def battery(self):
        msg=BatteryState();msg.present=True;msg.percentage=self.battery_level
        self.battery_pub.publish(msg)

    def return_event(self,msg):
        payload=json.loads(msg.data);self.return_events.append(payload)
        print(json.dumps({'event':'return',**payload}),flush=True)

    def state(self,msg):
        self.states.append((time.monotonic(),msg.data))
        if msg.data=='estop':self.running=False
        print(json.dumps({'mission_state':msg.data}),flush=True)

    def image(self,msg):
        self.images+=1;self.frame=self.cv.imgmsg_to_cv2(msg,'bgr8')
        if self.images==1:cv2.imwrite(str(ROOT/'first_frame.png'),self.frame)

    def scan(self,msg):
        self.scans+=1;self.scan_time=time.monotonic();ranges=[]
        for i,d in enumerate(msg.ranges):
            a=msg.angle_min+i*msg.angle_increment
            if abs(math.atan2(math.sin(a),math.cos(a)))<0.5 and math.isfinite(d) and msg.range_min<d<msg.range_max:ranges.append(d)
        self.front=min(ranges) if ranges else float('inf')
        self.minimum_front=min(self.minimum_front,self.front)

    def pose(self,msg):
        for tr in msg.transforms:
            if tr.child_frame_id!='robot1_my_bot':continue
            p=tr.transform.translation;q=tr.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            self.poses.append((time.monotonic(),p.x,p.y,p.z,yaw))

    def estimate(self,msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.estimates.append((time.monotonic(),p.x,p.y,yaw))

    def odometry(self,msg):
        p=msg.pose.pose.position
        self.odom_samples.append((time.monotonic(),p.x,p.y))

    def command(self,msg):self.cmd=msg;self.cmd_time=time.monotonic()

    def control(self):
        out=Twist();now=time.monotonic()
        if self.running and self.cmd is not None and now-self.cmd_time<0.5 and now-self.scan_time<1 and self.poses and now-self.poses[-1][0]<1.0 and self.poses[-1][3]>0.15:
            out.angular.z=max(-0.8,min(0.8,self.cmd.angular.z))
            if self.front>0.24:out.linear.x=max(-0.03,min(0.03,self.cmd.linear.x))
            elif abs(self.cmd.linear.x)>0:self.blocked+=1
            if abs(out.linear.x)+abs(out.angular.z)>0:self.forwarded+=1
        self.pub.publish(out)

    def event(self,msg):
        payload=json.loads(msg.data);self.events.append(payload)
        print(json.dumps({'event':'nav2',**payload}),flush=True)
        if self.frame is not None:cv2.imwrite(str(ROOT/f"event_{len(self.events):03d}.png"),self.frame)

    def plan(self,msg):
        self.plans.append({'time':time.monotonic(),'points':[[p.pose.position.x,p.pose.position.y] for p in msg.poses]})

    def costmap(self,msg):
        self.costmap_messages+=1
        self.costmap_obstacles=max(self.costmap_obstacles,sum(v>=65 for v in msg.data))


rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p','camera_topic:=/robot1/color/image_raw',
                '-p','require_navigation_handoff:=true','-p','require_mission_state:=true','-p','battery_topic:=/nav2_trial/battery','-p','enable_navigation_transmission:=true',
                '-p',f'home_x:={args.home_x}','-p',f'home_y:={args.home_y}','-p','home_yaw:=0.0',
                '-p','history_length:=1','-p','inference_period:=4.0','-p','inference_timeout_sec:=8.0',
                '-p','max_frame_age_sec:=12.0','-p','waypoint_timeout_sec:=15.0','-p','goal_timeout_sec:=180.0',
                '-p','instruction:=Explore open space ahead. Choose a destination beyond nearby obstacles; Nav2 will plan a safe route.',
                '-r','/tf:=/robot1/tf','-r','/tf_static:=/robot1/tf_static'],signal_handler_options=SignalHandlerOptions.NO)
from g3_integration_eval.state_machine_node import G3StateMachineNode
g3=G3StateMachineNode()
monitor=Monitor();executor=SingleThreadedExecutor();executor.add_node(monitor);executor.add_node(g3)
children=[];logs=[];nav=waypoint_bridge=None;result={}

def start(args,name):
    log=(ROOT/name).open('w');logs.append(log)
    p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT);children.append(p);return p

try:
    start(['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge','/world/world_demo/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V','--ros-args','-r','__node:=nav2_trial_gt_bridge','-r','/world/world_demo/pose/info:=/nav2_trial/gt_pose'],'gt_bridge.log')
    deadline=time.monotonic()+75
    while time.monotonic()<deadline and not (monitor.images and monitor.scans and monitor.poses and 0.18<monitor.poses[-1][3]<0.35):executor.spin_once(timeout_sec=0.1)
    if not (monitor.images and monitor.scans and monitor.poses and 0.18<monitor.poses[-1][3]<0.35):raise RuntimeError('Simulator sensors or standing posture unavailable')
    print(json.dumps({'phase':'standing','truth':monitor.poses[-1][1:]}),flush=True)
    start(['/opt/ros/humble/bin/ros2','launch',str(ROOT/'amcl_navigation.launch.py')],'nav2.log')
    start(['python3',str(PROJECT/'baseline/g1_mapping_nav/tools/cmd_vel_scaler.py'),'--ros-args','-p','use_sim_time:=true','-p','input_topic:=/deepseek/nav_cmd','-p','output_topic:=/nav2_trial/scaled_cmd'],'scaler.log')
    from qwen_robotnav_ros.nav2_waypoint_bridge import Nav2WaypointBridge
    waypoint_bridge=Nav2WaypointBridge();executor.add_node(waypoint_bridge)
    clients=[monitor.create_client(GetState,'/robot1/'+name+'/get_state') for name in ['bt_navigator','controller_server','planner_server','map_server','amcl']]
    deadline=time.monotonic()+45;active=False
    while time.monotonic()<deadline:
        executor.spin_once(timeout_sec=0.1)
        if all(c.service_is_ready() for c in clients):
            futures=[c.call_async(GetState.Request()) for c in clients]
            end=time.monotonic()+2
            while time.monotonic()<end and not all(f.done() for f in futures):executor.spin_once(timeout_sec=0.05)
            if all(f.done() and f.result().current_state.id==3 for f in futures):active=True;break
    if not active:raise RuntimeError('Nav2 lifecycle nodes did not all become active')
    deadline=time.monotonic()+20
    while time.monotonic()<deadline and not monitor.estimates:executor.spin_once(timeout_sec=0.1)
    if not monitor.estimates:raise RuntimeError('AMCL pose unavailable')
    print(json.dumps({'phase':'amcl_ready','estimate':monitor.estimates[-1],'truth':monitor.poses[-1]}),flush=True)
    from qwen_robotnav_ros.nav_node import NavNode
    nav=NavNode();executor.add_node(nav)
    started=time.monotonic();baseline=monitor.poses[-1];monitor.running=True
    print(json.dumps({'phase':'navigating','truth_start':baseline[1:]}),flush=True)
    for cycle in range(args.cycles):
        monitor.battery_level=0.8;monitor.battery()
        deadline=time.monotonic()+1
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=0.05)
        monitor.command_pub.publish(String(data='EXPLORE'))
        count=sum(e['status']=='succeeded' for e in monitor.events)
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=0.05)
            if sum(e['status']=='succeeded' for e in monitor.events)>count:break
        else:raise RuntimeError(f'Cycle {cycle+1} outbound timed out')
        outbound=monitor.poses[-1]
        # The production battery monitor initiates the handoff, not a direct home action.
        monitor.battery_level=0.1;monitor.battery()
        returns=sum(e['status']=='succeeded' for e in monitor.return_events)
        return_event_start=len(monitor.return_events)
        deadline=time.monotonic()+150
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=0.05)
            if sum(e['status']=='succeeded' for e in monitor.return_events)>returns:break
            if any(e['status'] in ('aborted','rejected','canceled','server_unavailable') for e in monitor.return_events[return_event_start:]):
                raise RuntimeError(f'Cycle {cycle+1} RETURN failed')
        else:raise RuntimeError(f'Cycle {cycle+1} battery RETURN timed out')
        end=monitor.poses[-1];home_error=math.hypot(end[1]-args.home_x,end[2]-args.home_y)
        outbound_goal=[e['target'] for e in monitor.events if e['status']=='succeeded'][-1]
        outbound_error=math.hypot(outbound[1]-outbound_goal['x'],outbound[2]-outbound_goal['y'])
        monitor.cycle_results.append({'cycle':cycle+1,'outbound_goal':outbound_goal,'outbound_error_m':outbound_error,'outbound_truth':outbound[1:],'home_truth':end[1:],'home_error_m':home_error})
        print(json.dumps({'cycle_complete':monitor.cycle_results[-1]}),flush=True)
        if home_error>0.4:raise RuntimeError('Home truth error exceeded 0.4 m')
    waypoint_bridge.stop_navigation('trial_end');monitor.running=False
    deadline=time.monotonic()+3
    while time.monotonic()<deadline:executor.spin_once(timeout_sec=0.05)
    end=monitor.poses[-1]
    successes=[e for e in monitor.events if e['status']=='succeeded']
    target=successes[0]['target'] if successes else None
    trace=[p for p in monitor.poses if p[0]>=baseline[0]]
    lateral=max(abs(p[2]-baseline[2]) for p in trace)
    result={'success':len(monitor.cycle_results)==args.cycles,'elapsed_seconds':time.monotonic()-started,
            'truth_start':baseline[1:],'truth_end':end[1:],'truth_home_error_m':math.hypot(end[1]-args.home_x,end[2]-args.home_y),
            'net_displacement_m':math.hypot(end[1]-baseline[1],end[2]-baseline[2]),
            'maximum_lateral_motion_m':lateral,'goal_target':target,
            'passed_previous_rack_post_x':any(p[1]>8.8 for p in trace),'localization':'AMCL + foot odometry/EKF; truth excluded from navigation localization'}
except (Exception,KeyboardInterrupt) as exc:
    result={'success':False,'error':str(exc),'localization':'AMCL + foot odometry/EKF'}
finally:
    monitor.running=False;monitor.pub.publish(Twist())
    if waypoint_bridge is not None:
        waypoint_bridge.stop_navigation('shutdown')
        deadline=time.monotonic()+2
        while waypoint_bridge._attempt is not None and time.monotonic()<deadline:executor.spin_once(timeout_sec=0.05)
    g3._navigator.cancel_current_goal(reason='trial_shutdown')
    for node in (nav,waypoint_bridge,g3):
        if node is not None:executor.remove_node(node);node.destroy_node()
    for p in reversed(children):
        if p.poll() is None:p.send_signal(signal.SIGINT)
    for _ in range(20):monitor.pub.publish(Twist());executor.spin_once(timeout_sec=0.05)
    for p in children:
        try:p.wait(timeout=5)
        except subprocess.TimeoutExpired:p.terminate()
    for log in logs:log.close()
    if monitor.frame is not None:cv2.imwrite(str(ROOT/'last_frame.png'),monitor.frame)
    with (ROOT/'trajectory.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['monotonic_seconds','x_m','y_m','z_m','yaw_rad']);w.writerows(monitor.poses)
    (ROOT/'localization.json').write_text(json.dumps({'amcl':monitor.estimates,'filtered_odom':monitor.odom_samples}))
    (ROOT/'plans.json').write_text(json.dumps(monitor.plans))
    result.update(acceptance_criteria={'outbound':'Nav2 SUCCEEDED (truth error reported separately)',
                  'return':'Nav2 SUCCEEDED and truth distance to dock <= 0.4 m','cycles':args.cycles},
                  requested_cycles=args.cycles,cycles=monitor.cycle_results,return_events=monitor.return_events,mission_states=monitor.states,image_messages=monitor.images,scan_messages=monitor.scans,truth_samples=len(monitor.poses),
                  nav2_events=monitor.events,planner_messages=len(monitor.plans),
                  local_costmap_messages=monitor.costmap_messages,local_costmap_obstacle_cells_max=monitor.costmap_obstacles,
                  minimum_front_range_m=monitor.minimum_front if math.isfinite(monitor.minimum_front) else None,
                  forwarded_nonzero_commands=monitor.forwarded,emergency_guard_interventions=monitor.blocked,
                  final_command_zero=True)
    if monitor.estimates and monitor.poses:
        import bisect
        times=[p[0] for p in monitor.poses]
        errors=[]
        for sample in monitor.estimates:
            i=min(bisect.bisect_left(times,sample[0]),len(times)-1)
            truth=monitor.poses[i]
            errors.append(math.hypot(sample[1]-truth[1],sample[2]-truth[2]))
        result.update(amcl_samples=len(errors),amcl_max_position_error_m=max(errors),
                      amcl_final_position_error_m=errors[-1])
    (ROOT/'report.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    executor.remove_node(monitor);monitor.destroy_node();executor.shutdown();rclpy.shutdown()
raise SystemExit(0 if result['success'] else 1)
