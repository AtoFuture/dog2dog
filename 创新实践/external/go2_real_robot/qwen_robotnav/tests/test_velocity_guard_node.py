"""Actual ROS traffic verifies independent command gating without GT input."""
from pathlib import Path
import math
import time
import pytest
pytest.importorskip('rclpy')
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan,Imu,BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from qwen_robotnav.runtime_profile import load_profile
from qwen_robotnav_ros.velocity_guard import VelocityGuard


@pytest.fixture
def transport():
    rclpy.init();profile=load_profile(Path(__file__).parents[1]/'config/sim_runtime.yaml')
    guard=VelocityGuard(profile,True);peer=Node('guard_transport_peer');executor=SingleThreadedExecutor()
    executor.add_node(guard);executor.add_node(peer);t=profile['topics'];outputs=[]
    peer.create_subscription(Twist,t['actuator_command'],lambda msg:outputs.append((time.monotonic(),msg.linear.x,msg.angular.z)),10)
    pubs={k:peer.create_publisher(typ,t[k],QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if k=='mission' else qos_profile_sensor_data if k in ('scan','imu','odometry','battery') else 10)
          for k,typ in [('nav_command',Twist),('guarded_input',Twist),('scan',LaserScan),('imu',Imu),('odometry',Odometry),('battery',BatteryState),('mission',String)]}
    sending={k:True for k in pubs};state=['exploring'];tilt=[0.];bad_scan=[False]
    def publish():
        stamp=peer.get_clock().now().to_msg()
        scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='base_link';scan.angle_min=-math.pi;scan.angle_increment=math.pi/180;scan.range_min=.1;scan.range_max=10.;scan.ranges=[float('nan')]*360 if bad_scan[0] is True else [2.]*360
        if bad_scan[0] in ('laser_missing','laser'):
            scan.header.frame_id='guard_test_laser'
            if bad_scan[0]=='laser':scan.ranges[180]=.3
        elif bad_scan[0]=='side_wall':scan.ranges[270]=.42
        elif bad_scan[0] in ('recovery_rear','recovery_rear_turn'):scan.ranges[0]=.435
        elif bad_scan[0]=='recovery_front':scan.ranges[180]=.435
        elif bad_scan[0]=='front_only':scan.angle_min=-.5;scan.angle_increment=.5;scan.ranges=[2.]*3
        odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom';odom.child_frame_id='base_link';odom.pose.pose.orientation.w=1.
        imu=Imu();imu.header.stamp=stamp;imu.orientation.x=math.sin(tilt[0]/2);imu.orientation.w=math.cos(tilt[0]/2)
        command=Twist();command.linear.x=.02;command.angular.z=.2
        if bad_scan[0] in ('recovery_rear','recovery_front'):command.angular.z=0.
        battery=BatteryState(present=True,percentage=.8)
        for key,msg in dict(scan=scan,odometry=odom,imu=imu,nav_command=command,guarded_input=command,battery=battery,mission=String(data=state[0])).items():
            if sending[key]:pubs[key].publish(msg)
    peer.create_timer(.04,publish)
    def until(predicate,timeout=4.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.02)
            if predicate():return
        raise AssertionError((guard.last_reason,outputs[-3:]))
    try:
        until(lambda:outputs and outputs[-1][1]>.01)
        yield guard,peer,outputs,sending,state,tilt,bad_scan,until
    finally:
        executor.remove_node(guard);executor.remove_node(peer);guard.destroy_node();peer.destroy_node();executor.shutdown();rclpy.shutdown()


@pytest.mark.parametrize('source',['nav_command','guarded_input','scan','odometry','imu','battery','mission'])
def test_input_loss_stops_real_ros_output(transport,source):
    guard,peer,outputs,sending,state,tilt,bad_scan,until=transport
    sending[source]=False
    expected={'guarded_input':'command','mission':'mission'}.get(source,source)+'_unavailable'
    until(lambda:guard.last_reason==expected and outputs[-1][1:]==(0.,0.))


def test_pause_tilt_and_invalid_scan_each_stop_real_output(transport):
    guard,peer,outputs,sending,state,tilt,bad_scan,until=transport
    state[0]='paused';until(lambda:guard.last_reason=='mission:paused' and outputs[-1][1:]==(0.,0.))
    state[0]='exploring';tilt[0]=.8;until(lambda:guard.last_reason=='excessive_tilt' and outputs[-1][1:]==(0.,0.))
    tilt[0]=0.;bad_scan[0]=True;until(lambda:guard.last_reason=='scan_unavailable' and outputs[-1][1:]==(0.,0.))


def test_duplicate_actuator_publisher_latches_stop(transport):
    guard,peer,outputs,sending,state,tilt,bad_scan,until=transport
    foreign=peer.create_publisher(Twist,guard.profile['topics']['actuator_command'],10)
    until(lambda:guard.last_reason=='multiple_actuator_publishers' and outputs[-1][1:]==(0.,0.))
    peer.destroy_publisher(foreign)
    assert guard.policy.publisher_conflict


@pytest.mark.parametrize('mode,reason',[('side_wall','footprint_clearance'),('front_only','scan_unavailable')])
def test_side_wall_and_incomplete_scan_stop_real_output(transport,mode,reason):
    guard,peer,outputs,sending,state,tilt,bad_scan,until=transport
    bad_scan[0]=mode
    until(lambda:guard.last_reason==reason and outputs[-1][1:]==(0.,0.))


def test_scan_requires_measured_extrinsic_before_allowing_output(transport):
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    guard,peer,outputs,sending,state,tilt,bad_scan,until=transport
    bad_scan[0]='laser_missing'
    until(lambda:guard.last_reason=='scan_unavailable' and outputs[-1][1:]==(0.,0.))
    broadcaster=StaticTransformBroadcaster(peer)
    tr=TransformStamped();tr.header.stamp=peer.get_clock().now().to_msg()
    tr.header.frame_id='base_link';tr.child_frame_id='guard_test_laser'
    tr.transform.translation.x=.22;tr.transform.rotation.w=1.
    broadcaster.sendTransform(tr);bad_scan[0]='laser'
    until(lambda:guard.last_reason=='predicted_footprint_collision' and outputs[-1][1]==0 and outputs[-1][2]>.1)


def test_protected_stop_can_only_resume_along_certified_escape(transport):
    guard,peer,outputs,sending,state,tilt,bad_scan,until=transport
    guard.policy.settings['clearance_recovery_enabled']=False
    state[0]='returning';bad_scan[0]='recovery_rear'
    until(lambda:guard.last_reason=='footprint_clearance' and outputs[-1][1:]==(0.,0.))
    guard.policy.settings['clearance_recovery_enabled']=True
    until(lambda:guard.last_reason=='clearance_recovery' and outputs[-1][1:]==(.02,0.))
    bad_scan[0]='recovery_rear_turn'
    until(lambda:guard.last_reason=='footprint_clearance' and outputs[-1][1:]==(0.,0.))
    bad_scan[0]='recovery_front'
    until(lambda:guard.last_reason=='footprint_clearance' and outputs[-1][1:]==(0.,0.))
    bad_scan[0]=True
    until(lambda:guard.last_reason=='scan_unavailable' and outputs[-1][1:]==(0.,0.))
