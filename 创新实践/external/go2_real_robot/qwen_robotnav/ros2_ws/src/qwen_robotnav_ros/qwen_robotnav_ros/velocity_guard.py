"""Independent final velocity publisher, with steady-time fail-closed watchdogs."""
import argparse
import json
import math
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan,Imu,BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from qwen_robotnav.runtime_profile import load_profile,profile_hash
from qwen_robotnav.velocity_safety import VelocitySafety,scan_points


class VelocityGuard(Node):
    def __init__(self,profile,enabled=False):
        super().__init__('velocity_guard')
        self.profile=profile;self.policy=VelocitySafety(profile['guard'],enabled,profile['navigation']['robot_radius_m'])
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        t=profile['topics'];self.outputs=0;self.moving=0;self.last_reason=None;self.status_at=-math.inf
        self.pub=self.create_publisher(Twist,t['actuator_command'],10)
        self.status_pub=self.create_publisher(String,t['guard_status'],10)
        self.create_subscription(Twist,t['guarded_input'],self.command,10)
        self.create_subscription(Twist,t['nav_command'],self.nav_command,10)
        self.create_subscription(LaserScan,t['scan'],self.scan,qos_profile_sensor_data)
        self.create_subscription(Imu,t['imu'],self.imu,qos_profile_sensor_data)
        self.create_subscription(Odometry,t['odometry'],self.odometry,qos_profile_sensor_data)
        self.create_subscription(BatteryState,t['battery'],self.battery,qos_profile_sensor_data)
        self.create_subscription(String,t['mission'],self.mission,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.steady=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1./profile['guard']['rate_hz'],self.tick,clock=self.steady)
        self.create_timer(.5,self.check_publishers,clock=self.steady)

    def fresh(self,msg):
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age=(self.get_clock().now().nanoseconds-stamp)*1e-9
        return stamp if -self.profile['guard']['stamp_future_tolerance_s']<=age<=self.profile['guard']['sensor_timeout_s'] else None

    def command(self,msg):
        values=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        if not all(math.isfinite(v) for v in values) or any(abs(values[i])>1e-9 for i in (1,2,3,4)):
            self.policy.invalidate('command');return
        self.policy.command=(msg.linear.x,msg.angular.z);self.policy.observe('command',time.monotonic())

    def nav_command(self,msg):
        values=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        if not all(math.isfinite(v) for v in values) or any(abs(values[i])>1e-9 for i in (1,2,3,4)):
            self.policy.invalidate('nav_command');return
        self.policy.observe('nav_command',time.monotonic())

    def mission(self,msg):
        self.policy.mission=msg.data;self.policy.observe('mission',time.monotonic())

    def battery(self,msg):
        if not msg.present or not math.isfinite(msg.percentage) or not 0<=msg.percentage<=1:
            self.policy.invalidate('battery');return
        self.policy.observe('battery',time.monotonic())

    def odometry(self,msg):
        stamp=self.fresh(msg);p=msg.pose.pose.position;q=msg.pose.pose.orientation
        values=(p.x,p.y,p.z,q.x,q.y,q.z,q.w)
        f=self.profile['frames']
        if stamp is None or msg.header.frame_id!=f['odom'] or msg.child_frame_id!=f['base'] or not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1.)>.01:
            self.policy.invalidate('odometry');return
        velocity=(msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z)
        if not all(math.isfinite(v) for v in velocity):
            self.policy.invalidate('odometry');return
        if self.policy.observe('odometry',time.monotonic(),stamp):self.policy.measured_velocity=velocity

    def imu(self,msg):
        stamp=self.fresh(msg);q=msg.orientation;values=(q.x,q.y,q.z,q.w)
        if stamp is None or msg.orientation_covariance[0]<0 or not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values)-1.)>.01:
            self.policy.invalidate('imu');return
        tilt=math.acos(max(-1.,min(1.,1-2*(q.x*q.x+q.y*q.y))))
        if self.policy.observe('imu',time.monotonic(),stamp):self.policy.tilt=tilt

    def scan(self,msg):
        stamp=self.fresh(msg);g=self.profile['guard'];values=[]
        if stamp is not None and all(math.isfinite(v) for v in (msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max)) and msg.angle_increment>0 and 0<=msg.range_min<msg.range_max:
            for i,d in enumerate(msg.ranges):
                a=msg.angle_min+i*msg.angle_increment
                if abs(math.atan2(math.sin(a),math.cos(a)))<=g['front_half_angle_rad']:
                    if math.isfinite(d) and msg.range_min<=d<=msg.range_max:values.append(d)
                    elif d==math.inf:values.append(msg.range_max)
        if not values:self.policy.invalidate('scan');return
        try:
            if msg.header.frame_id==self.profile['frames']['base']:sensor=(0.,0.,0.)
            else:
                tr=self.tf.lookup_transform(self.profile['frames']['base'],msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
                q=tr.rotation
                if abs(q.x)+abs(q.y)>.01 or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.01:
                    raise ValueError('Require measured planar laser extrinsic')
                sensor=(tr.translation.x,tr.translation.y,2*math.atan2(q.z,q.w))
            points=scan_points(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,sensor)
        except Exception:
            points=None
        if points is None:self.policy.invalidate('scan');return
        if self.policy.observe('scan',time.monotonic(),stamp):
            self.policy.front=min(values);self.policy.obstacles=points

    def check_publishers(self):
        if len(self.get_publishers_info_by_topic(self.profile['topics']['actuator_command']))>1:
            self.policy.publisher_conflict=True

    def tick(self):
        now=time.monotonic();x,z,reason=self.policy.output(now)
        out=Twist();out.linear.x=x;out.angular.z=z;self.pub.publish(out)
        self.outputs+=1;self.moving+=int(abs(x)+abs(z)>0)
        if reason!=self.last_reason or now-self.status_at>=.5:
            self.status_at=now;self.last_reason=reason
            self.status_pub.publish(String(data=json.dumps(dict(reason=reason,enabled=self.policy.enabled,
                stamp_ns=self.get_clock().now().nanoseconds,monotonic_time=now,profile_sha256=profile_hash(self.profile),
                outputs=self.outputs,moving_outputs=self.moving,front_m=self.policy.front,
                footprint_clearance_m=min((math.hypot(x,y) for x,y in self.policy.obstacles),default=None) if self.policy.obstacles is not None else None,
                tilt_rad=self.policy.tilt if math.isfinite(self.policy.tilt) else None,
                inputs_age_s={k:now-v for k,v in self.policy.seen.items()},publisher_conflict=self.policy.publisher_conflict))))

    def destroy_node(self):
        self.policy.enabled=False;self.pub.publish(Twist())
        return super().destroy_node()


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--profile',required=True)
    parser.add_argument('--enable-motion',action='store_true');a,ros=parser.parse_known_args(args)
    profile=load_profile(a.profile);rclpy.init(args=ros,signal_handler_options=SignalHandlerOptions.NO)
    stopping=False
    def request_stop(sig,frame):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
    node=VelocityGuard(profile,a.enable_motion)
    try:
        while rclpy.ok() and not stopping:rclpy.spin_once(node,timeout_sec=.05)
    except KeyboardInterrupt:pass
    finally:
        node.policy.enabled=False
        for _ in range(3):
            node.tick();rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
