"""pose_tf_relay: 运动控制器腿部里程计 → TF(odom→base_link) 直发。

输入: /baiyanggo/webrtc/sport_odom (桥接器从 rt/lf/sportmodestate 20Hz 发布：
      position/机体系velocity/yaw_speed，与 L1 扫描匹配完全独立)
输出: TF odom→base_link（publish_tf:=true，默认开）

BYY修复⑭ 定版架构：主导航里程计 = 狗运动控制器腿部里程计（无航向翻转问题）；
utlidar 只贡献障碍点云（odom_utlidar 系，经 static_sensor_frame 挂接进 odom）。
历史教训：utlidar robot_pose 在慢速机动中航向漂移/翻转（⑨），桥重连归零会污染
EKF 状态（⑫），IMU 航向与 utlidar 同源无增益——均废弃。
"""
import time as _time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped


class PoseTfRelay(Node):
    def __init__(self):
        super().__init__("pose_tf_relay")
        self.declare_parameter("publish_tf", True)
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/baiyanggo/webrtc/sport_odom",
                                 self.on_odom, 10)
        self.get_logger().info("pose_tf_relay 启动: sport_odom → TF odom→base_link")

    def on_odom(self, od: Odometry):
        if not self.get_parameter("publish_tf").value:
            return
        t = TransformStamped()
        # 用接收时刻打戳：源戳与 scan 戳有几十毫秒错位，按消息戳会让下游查 TF 外推到未来
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = od.header.frame_id      # odom
        t.child_frame_id = od.child_frame_id        # base_link
        t.transform.translation.x = od.pose.pose.position.x
        t.transform.translation.y = od.pose.pose.position.y
        t.transform.translation.z = 0.0             # 2D 导航不需要高度
        # 只保留航向分量（平面导航；原始四元数含俯仰/翻滚）
        q = od.pose.pose.orientation
        import math
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = PoseTfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
