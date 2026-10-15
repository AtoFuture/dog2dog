"""Validate online SLAM, G2 frontier navigation and G3 battery RETURN in simulation.

Start the generated isolated simulator first. No static map, AMCL, truth TF or
hardware SDK is used. Ground truth measures errors and guards standing posture.
Optional DeepSeek ranking uses only safe G2 candidates; vision is a separate stage.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from g3_integration_eval.state_machine_node import G3StateMachineNode
from vision_explorer.explorer_node import VisionExplorer


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class Monitor(Node):
    def __init__(self):
        super().__init__('slam_frontier_validation')
        self.poses=[];self.events=[];self.returns=[];self.maps=[];self.states=[];self.perception=[]
        self.latest_map=None;self.command=None;self.command_at=-math.inf
        self.scan_at=-math.inf;self.front=math.inf;self.minimum_front=math.inf
        self.running=False;self.battery_level=.8;self.blocked=0;self.forwarded=0
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pub=self.create_publisher(Twist,'/robot1/cmd_vel',10)
        self.battery_pub=self.create_publisher(BatteryState,'/frontier_trial/battery',10)
        self.command_pub=self.create_publisher(String,'/g3/command',10)
        self.create_subscription(TFMessage,'/frontier_trial/gt_pose',self.truth,10)
        self.create_subscription(Twist,'/frontier_trial/scaled_cmd',self.velocity,10)
        self.create_subscription(LaserScan,'/robot1/velodyne',self.scan,qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/robot1/map',self.mapping,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String,'/exploration/events',self.event,10)
        self.create_subscription(String,'/g3/return_event',self.return_event,10)
        self.create_subscription(String,'/mission_state',self.state,10)
        self.create_subscription(String,'/perception/status',lambda m:self.perception.append(json.loads(m.data)),10)
        self.create_timer(.05,self.control)
        self.create_timer(.5,self.battery)

    def truth(self,msg):
        for tr in msg.transforms:
            if tr.child_frame_id=='robot1_my_bot':
                p=tr.transform.translation
                self.poses.append((time.monotonic(),p.x,p.y,p.z,yaw(tr.transform.rotation)))

    def velocity(self,msg):
        self.command=msg;self.command_at=time.monotonic()

    def scan(self,msg):
        self.scan_at=time.monotonic()
        values=[d for i,d in enumerate(msg.ranges)
                if abs(math.atan2(math.sin(msg.angle_min+i*msg.angle_increment),
                                  math.cos(msg.angle_min+i*msg.angle_increment)))<.5
                and math.isfinite(d) and msg.range_min<d<msg.range_max]
        self.front=min(values,default=math.inf)
        self.minimum_front=min(self.front,self.minimum_front)

    def mapping(self,msg):
        self.latest_map=msg
        known=sum(v>=0 for v in msg.data)*msg.info.resolution**2
        free=sum(0<=v<50 for v in msg.data)*msg.info.resolution**2
        self.maps.append((time.monotonic(),known,free,msg.info.width,msg.info.height))

    def event(self,msg):
        e=json.loads(msg.data)
        if self.poses:e['truth_at_receipt']=self.poses[-1][1:]
        self.events.append(e)
        print(json.dumps({'exploration':e}),flush=True)

    def return_event(self,msg):
        e=json.loads(msg.data);self.returns.append(e)
        print(json.dumps({'return':e}),flush=True)

    def state(self,msg):
        self.states.append((time.monotonic(),msg.data))
        if msg.data=='estop':self.running=False
        print(json.dumps({'mission_state':msg.data}),flush=True)

    def battery(self):
        msg=BatteryState(present=True,percentage=self.battery_level)
        self.battery_pub.publish(msg)

    def control(self):
        out=Twist();now=time.monotonic()
        if self.running and self.command is not None and now-self.command_at<.5 and now-self.scan_at<1 \
                and self.poses and now-self.poses[-1][0]<1 and self.poses[-1][3]>.15:
            out.angular.z=max(-.8,min(.8,self.command.angular.z))
            if self.front>.24:out.linear.x=max(-.03,min(.03,self.command.linear.x))
            elif abs(self.command.linear.x)>0:self.blocked+=1
            if abs(out.linear.x)+abs(out.angular.z)>0:self.forwarded+=1
        self.pub.publish(out)

    def map_pose(self):
        tr=self.buffer.lookup_transform('map','base_link',Time()).transform
        return tr.translation.x,tr.translation.y,yaw(tr.rotation)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--expected-domain',type=int,required=True)
    parser.add_argument('--mission-brain',action='store_true')
    parser.add_argument('--perception',action='store_true')
    parser.add_argument('--recon-task',action='store_true')
    parser.add_argument('--active-observation',action='store_true')
    parser.add_argument('--required-person-count',type=int,default=1)
    parser.add_argument('--goals',type=int,default=5)
    parser.add_argument('--exploration-timeout',type=float,default=600)
    args=parser.parse_args()
    if args.expected_domain in (0,42,43) or int(os.environ.get('ROS_DOMAIN_ID','0'))!=args.expected_domain:
        parser.error('Use a matching isolated simulator ROS domain')
    if not os.environ.get('IGN_PARTITION') or args.goals<1 or args.required_person_count<1:
        parser.error('Set IGN_PARTITION and a positive --goals')
    if args.active_observation and not (args.recon_task and args.mission_brain):parser.error('Active observation requires recon and mission brain')
    if args.recon_task and not args.perception:parser.error('--recon-task requires --perception')
    root=args.output_dir.resolve();project=Path(__file__).resolve().parents[4]
    if not (root/'slam_navigation.launch.py').exists():
        parser.error('Generate --localization slam into --output-dir first')
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p','require_mission_state:=true',
        '-p','require_navigation_handoff:=true','-p','enable_navigation_transmission:=true',
        '-p','active_observation:='+str(args.active_observation).lower(),
        '-p','observation_bt_path:='+str(root/'observation_bt.xml'),
        '-p','search_timeout_s:='+str(float(args.exploration_timeout)),
        '-p','required_person_count:='+str(args.required_person_count),'-p','recon_output_dir:='+str(root),'-p','minimum_exploration_goals:='+str(args.goals),
        '-p','battery_topic:=/frontier_trial/battery','-p','map_topic:=/robot1/map',
        '-p','use_mission_brain:='+str(args.mission_brain).lower(),
        '-p','nav_action:=/robot1/navigate_to_pose','-p','tick_period_s:=0.75','-p','min_goal_distance_m:=0.5',
        '-r','/tf:=/robot1/tf','-r','/tf_static:=/robot1/tf_static'],signal_handler_options=SignalHandlerOptions.NO)
    # Background shells can inherit SIGINT=SIG_IGN. Restore graceful shutdown so
    # interruption still reaches finally and finalizes the recorder.
    signal.signal(signal.SIGINT,signal.default_int_handler)
    def interrupted(signum, frame):raise KeyboardInterrupt('Validation interrupted')
    signal.signal(signal.SIGTERM,interrupted)
    monitor=Monitor();g3=G3StateMachineNode();explorer=None;brain=None;recon=None
    executor=SingleThreadedExecutor();executor.add_node(monitor);executor.add_node(g3)
    children=[];logs=[];report={};baseline=None;home=None;anchor=None;recorder=None

    def start(command,name):
        stream=(root/name).open('w');logs.append(stream)
        child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(child);return child

    def spin_until(condition,timeout,check_recorder=True):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.05)
            if check_recorder and recorder is not None and recorder.poll() is not None:
                raise RuntimeError('Recorder exited before validation finished; inspect bag.log')
            if condition():return True
        return False

    try:
        if args.mission_brain:
            from g3_integration_eval.mission_brain_node import MissionBrainNode
            brain=MissionBrainNode();executor.add_node(brain)
        if args.recon_task:
            from g3_integration_eval.recon_task_node import ReconTaskNode
            recon=ReconTaskNode();executor.add_node(recon)
        start(['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge',
            '/world/world_demo/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '--ros-args','-r','__node:=frontier_truth_evaluation',
            '-r','/world/world_demo/pose/info:=/frontier_trial/gt_pose'],'gt_bridge.log')
        if not spin_until(lambda:monitor.poses and .18<monitor.poses[-1][3]<.35 and time.monotonic()-monitor.scan_at<1,150):
            raise RuntimeError('Simulator sensors or standing posture unavailable')
        recorder=start(['ros2','bag','record','--include-hidden-topics','-o',str(root/'bag'),'/clock','/robot1/map','/robot1/tf','/robot1/tf_static',
               '/robot1/velodyne','/robot1/odometry/filtered','/robot1/cmd_vel','/deepseek/nav_cmd','/robot1/cmd_vel_nav','/frontier_trial/scaled_cmd','/exploration/events',
               '/mission_state','/g3/state','/g3/return_event','/frontier_trial/gt_pose','/frontier_trial/battery',
               '/robot1/navigate_to_pose/_action/status','/robot1/navigate_to_pose/_action/feedback',
               '/navigation/handoff_request','/navigation/handoff_ack',
               '/exploration/candidates','/mission/context','/brain/decision',
               '/detections_3d','/detections_snapshot','/mission/detection_evidence','/perception/status','/mission/recon_status',
               *(['/robot1/rgbd_d435/image','/robot1/rgbd_d435/depth_image','/robot1/rgbd_d435/camera_info'] if args.recon_task else [])],'bag.log')
        start(['ros2','launch',str(root/'slam_navigation.launch.py')],'nav2.log')
        start(['python3',str(project/'baseline/g1_mapping_nav/tools/cmd_vel_scaler.py'),
            '--ros-args','-p','use_sim_time:=true','-p','input_topic:=/deepseek/nav_cmd',
            '-p','output_topic:=/frontier_trial/scaled_cmd'],'scaler.log')
        clients=[monitor.create_client(GetState,'/robot1/'+name+'/get_state')
                 for name in ['bt_navigator','controller_server','planner_server']]
        def ready():
            if monitor.latest_map is None or not all(c.service_is_ready() for c in clients):return False
            try:monitor.map_pose()
            except Exception:return False
            return True
        if not spin_until(ready,60):raise RuntimeError('SLAM map, map TF or Nav2 services unavailable')
        active=False;activation_deadline=time.monotonic()+30
        while time.monotonic()<activation_deadline:
            futures=[c.call_async(GetState.Request()) for c in clients]
            if spin_until(lambda:all(f.done() for f in futures),2) and all(f.result().current_state.id==3 for f in futures):
                active=True;break
            spin_until(lambda:False,1)
        if not active:raise RuntimeError('Nav2 did not activate')
        home=monitor.map_pose();baseline=monitor.poses[-1];anchor=baseline[4]-home[2]
        g3.set_parameters([Parameter('home_x',value=float(home[0])),Parameter('home_y',value=float(home[1]))])
        initial_area=monitor.maps[-1][1]
        if args.perception:
            start(['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge',
                '/robot1/rgbd_d435/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '--ros-args','-r','__node:=rgbd_intrinsics_bridge'],'rgbd_bridge.log')
            start(['python3','-m','g3_integration_eval.detection_evidence_node','--ros-args',
                '-p','evidence_output_dir:='+str(root/'detections')],'evidence.log')
            start(['env','OMP_NUM_THREADS=2','MKL_NUM_THREADS=2','OPENBLAS_NUM_THREADS=2',
                'python3','-m','vision_detector.detector_node','--ros-args',
                '-p','use_sim_time:=true','-p','color_topic:=/robot1/rgbd_d435/image',
                '-p','depth_topic:=/robot1/rgbd_d435/depth_image',
                '-p','camera_info_topic:=/robot1/rgbd_d435/camera_info',
                '-p','weights:='+str(project/'baseline/g2_vision_explore'/('yolo26n-seg.pt' if args.recon_task else 'yolo26n.pt')),
                '-p','mask_depth_cluster_gap_m:='+('0.35' if args.recon_task else '0.0'),
                '-p','tf_fallback_to_latest:=false',
                '-r','/tf:=/robot1/tf','-r','/tf_static:=/robot1/tf_static'],'detector.log')
        started=time.monotonic()
        if args.recon_task:
            recon.search_budget_paused=True
            monitor.battery();monitor.command_pub.publish(String(data='EXPLORE'))
            initial_view_start=monitor.get_clock().now().nanoseconds
            spin_until(recon.objective_confirmed,30)
            (root/'recon_initial_view.json').write_text(json.dumps(dict(
                start_stamp_ns=initial_view_start,end_stamp_ns=monitor.get_clock().now().nanoseconds,
                motion_enabled=False,targets=recon.targets,required_person_count=args.required_person_count,
                objective_confirmed=recon.objective_confirmed()),indent=2))
        if recon is not None:
            recon.search_budget_paused=False;recon.search_started_at=time.monotonic()
        explorer=VisionExplorer();executor.add_node(explorer)
        monitor.running=True;monitor.battery()
        if not args.recon_task:monitor.command_pub.publish(String(data='EXPLORE'))
        print(json.dumps({'phase':'exploring','home_map':home,'truth_start':baseline[1:],
                          'initial_known_area_m2':initial_area}),flush=True)
        def explored():
            if monitor.states and monitor.states[-1][1]=='estop':raise RuntimeError('G3 entered ESTOP during exploration')
            if args.recon_task:
                if recon.state in ('failed','interrupted'):raise RuntimeError('Recon task '+recon.state)
                return recon.return_requested
            return sum(e['status']=='succeeded' and e['geodesic_dist_m']>=.75 for e in monitor.events)>=args.goals
        if not spin_until(explored,args.exploration_timeout+5):
            raise RuntimeError((f'Recon deadline: targets_confirmed={recon.objective_confirmed()}, '
                                f'exploration_goals={len(recon.completed_goals)}/{args.goals}')
                               if args.recon_task else 'Insufficient successful frontier goals before deadline')
        # Exercise an active/late-accepted frontier cancellation at low-battery handoff.
        if not args.recon_task:
            spin_until(lambda:explorer._attempt is not None or explorer._brain_pending is not None,3)
            monitor.battery_level=.1;monitor.battery()
        def returned():
            if any(e['status'] in ('aborted','canceled','rejected','server_unavailable') for e in monitor.returns):
                raise RuntimeError('Battery RETURN failed')
            return any(e['status']=='succeeded' for e in monitor.returns)
        if not spin_until(returned,180):raise RuntimeError('Battery RETURN timed out')
        monitor.running=False;explorer.stop_navigation()
        spin_until(lambda:False,2)
        end=monitor.poses[-1];home_error=math.hypot(end[1]-baseline[1],end[2]-baseline[2])
        final_area=monitor.maps[-1][1]
        report=dict(success=home_error<=.4 and final_area>initial_area,elapsed_seconds=time.monotonic()-started,
            home_map=home,truth_start=baseline[1:],truth_end=end[1:],truth_home_error_m=home_error,
            initial_known_area_m2=initial_area,final_known_area_m2=final_area,
            known_area_growth_m2=final_area-initial_area)
    except (Exception,KeyboardInterrupt) as exc:
        report=dict(success=False,error=str(exc) or 'Validation interrupted')
        if recon is not None and recon.state not in ('returned','failed','interrupted'):
            recon._transition('failed')
    finally:
        monitor.running=False;monitor.pub.publish(Twist())
        if explorer is not None:explorer.stop_navigation()
        g3._navigator.cancel_current_goal(reason='trial_shutdown')
        spin_until(lambda:False,2,check_recorder=False)
        if monitor.latest_map is not None:
            m=monitor.latest_map
            np.save(root/'map.npy',np.asarray(m.data,dtype=np.int8).reshape(m.info.height,m.info.width))
            (root/'map.json').write_text(json.dumps({'origin':[m.info.origin.position.x,m.info.origin.position.y],
                                                   'resolution':m.info.resolution,'frame':m.header.frame_id}))
        if monitor.latest_map is not None:
            saver=start(['ros2','run','nav2_map_server','map_saver_cli','-f',str(root/'slam_map'),
                         '-t','/robot1/map','--ros-args','-p','use_sim_time:=true',
                         '-p','map_subscribe_transient_local:=true','-p','save_map_timeout:=10.0',
                         '-p','free_thresh_default:=0.196'],'map_saver.log')
            spin_until(lambda:saver.poll() is not None,20,check_recorder=False)
        for child in reversed(children):
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
        for child in children:
            try:child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL)
        for stream in logs:stream.close()
        with (root/'trajectory.csv').open('w') as stream:
            w=csv.writer(stream);w.writerow(['monotonic_seconds','x_m','y_m','z_m','yaw_rad']);w.writerows(monitor.poses)
        if baseline is not None and home is not None:
            for e in monitor.events:
                if e['status']=='succeeded' and 'truth_at_receipt' in e:
                    target=e['target'];dx=target['x']-home[0];dy=target['y']-home[1]
                    wx=baseline[1]+math.cos(anchor)*dx-math.sin(anchor)*dy
                    wy=baseline[2]+math.sin(anchor)*dx+math.cos(anchor)*dy
                    e['truth_goal_error_m']=math.hypot(wx-e['truth_at_receipt'][0],wy-e['truth_at_receipt'][1])
        if baseline is not None:
            report.setdefault('truth_start',baseline[1:])
            report.setdefault('home_map',home)
            report['world_map_yaw_offset']=anchor
        if args.mission_brain and not any(e['status']=='brain_selected' and not e['fallback'] for e in monitor.events):
            report['success']=False
            report.setdefault('error','No real DeepSeek decision reached navigation')
        if baseline is not None and monitor.poses:
            report.setdefault('truth_home_error_m',math.hypot(monitor.poses[-1][1]-baseline[1],monitor.poses[-1][2]-baseline[2]))
        report['recording_complete']=(root/'bag/metadata.yaml').is_file()
        report['recorder_exit_code']=recorder.returncode if recorder is not None else None
        if not report['recording_complete']:
            report['success']=False;report.setdefault('error','Recording did not finalize metadata')
        report['return_trigger']=(getattr(recon,'return_reason',None) or 'not_triggered') if args.recon_task else 'simulated_low_battery'
        report['active_observation']=args.active_observation
        report['navigation_return_success']=bool(home is not None and any(e['status']=='succeeded' for e in monitor.returns)
                                                 and report.get('truth_home_error_m',float('inf'))<=.4)
        if args.recon_task:
            report['recon_mission']=json.loads((root/'mission_report.json').read_text()) if (root/'mission_report.json').exists() else {'state':'not_started'}
            if report['recon_mission']['state']!='returned':
                report['success']=False;report.setdefault('error','Recon search incomplete; RETURN action completed' if report['recon_mission']['state']=='returned_incomplete' else 'Recon mission did not confirm RETURN success')
        report['perception_enabled']=args.perception
        report['perception_status']=monitor.perception[-1] if monitor.perception else None
        if args.perception and not (monitor.perception and monitor.perception[-1]['stats']['frames']>0 and monitor.perception[-1]['map_tf_available']):
            report['success']=False
            report.setdefault('error','Perception frames or image-time map TF unavailable')
        if (root/'detections/evidence.json').exists():
            report['detection_evidence']=json.loads((root/'detections/evidence.json').read_text())
        report.update(saved_map_yaml=(root/'slam_map.yaml').exists(),goal_source='DeepSeek-ranked G2 safe frontiers' if args.mission_brain else 'G2 deterministic frontier',
            deepseek_selections=sum(e['status']=='brain_selected' and not e['fallback'] for e in monitor.events),
            brain_fallback_selections=sum(e['status']=='brain_selected' and e['fallback'] for e in monitor.events),localization='online SLAM + foot-position / IMU EKF',
            static_map_loaded=False,truth_in_navigation_tf=False,requested_goals=args.goals,
            counted_goal_minimum_geodesic_distance_m=.75,node_minimum_goal_geodesic_distance_m=.5,home_error_limit_m=.4,
            meaningful_frontier_goals=sum(e['status']=='succeeded' and e['geodesic_dist_m']>=.75 for e in monitor.events),
            successful_frontier_goals=sum(e['status']=='succeeded' for e in monitor.events),
            exploration_events=monitor.events,return_events=monitor.returns,mission_states=monitor.states,
            map_samples=monitor.maps,emergency_guard_interventions=monitor.blocked,
            minimum_front_range_m=monitor.minimum_front if math.isfinite(monitor.minimum_front) else None,
            final_zero_command_published=True)
        (root/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
        for node in [explorer,recon,brain,g3,monitor]:
            if node is not None:executor.remove_node(node);node.destroy_node()
        executor.shutdown();rclpy.shutdown()
    return 0 if report['success'] else 1


if __name__=='__main__':
    raise SystemExit(main())
