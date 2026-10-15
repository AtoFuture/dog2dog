"""static_sensor_frame: 把 utlidar 传感器系刚性挂接到导航系。

odom(导航系, sport腿部里程计) 与 odom_utlidar(体素地图系) 是两个独立原点。
挂接变换 = T_sport(启动时) ∘ T_utlidar(启动时)⁻¹，使点云经
odom_utlidar → odom → base_link 正确变换到机器人身上。
（体素地图是 utlidar 系累积图，刚性；两系相对漂移短期可忽略）
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


rclpy.init()
n = rclpy.create_node("static_sensor_frame")
br = StaticTransformBroadcaster(n)
got = {"u": None, "s": None}


def on_pose(m):
    if got["u"] is None:
        got["u"] = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))


def on_odom(m):
    if got["s"] is None:
        got["s"] = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))


n.create_subscription(PoseStamped, "/baiyanggo/webrtc/pose", on_pose, 10)
n.create_subscription(Odometry, "/baiyanggo/webrtc/sport_odom", on_odom, 10)
t0 = time.time()
while (got["u"] is None or got["s"] is None) and time.time() - t0 < 150 and rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.2)

if got["u"] is None or got["s"] is None:
    print(f"150秒未收齐位姿 u={got['u']} s={got['s']},退出")
    raise SystemExit(1)

xu, yu, tu = got["u"]
xs, ys, ts = got["s"]
# T_mount = T_s ∘ T_u⁻¹: rotation = ts-tu, translation = p_s - R(ts-tu)·p_u
dth = ts - tu
mx = xs - (math.cos(dth) * xu - math.sin(dth) * yu)
my = ys - (math.sin(dth) * xu + math.cos(dth) * yu)
t = TransformStamped()
t.header.stamp = n.get_clock().now().to_msg()
t.header.frame_id = "odom"
t.child_frame_id = "odom_utlidar"
t.transform.translation.x = mx
t.transform.translation.y = my
t.transform.translation.z = 0.0
dyaw = ts - tu
t.transform.rotation.x = 0.0
t.transform.rotation.y = 0.0
t.transform.rotation.z = math.sin(dyaw / 2.0)
t.transform.rotation.w = math.cos(dyaw / 2.0)
br.sendTransform(t)
print(f"静态TF odom->odom_utlidar: u=({xu:.3f},{yu:.3f},{tu:.3f}) s=({xs:.3f},{ys:.3f},{ts:.3f}) "
      f"-> mount=({mx:.3f},{my:.3f},dyaw={dyaw:.3f})")
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=1.0)
