#!/usr/bin/env python3
"""把桥进来的 Gazebo 真值转成带仿真时间戳的 nav_msgs/Odometry。

为什么需要这一步：
  ros_gz_bridge 把 gz 的 Pose_V 桥成 tf2_msgs/TFMessage 时**不填 header.stamp**
  （实测全为 0）。时间戳为 0 的轨迹没法跟 /robot1/odom 逐帧对齐，
  离线算出来的「到点误差」就是错位样本相减的假数。

  本节点用 rclpy 的时钟（use_sim_time=true 时即仿真时间）重新打戳，
  输出一条与 /robot1/odom 同时基、同消息类型的真值轨迹。

话题：
  订阅  /world/world_demo/pose/info   tf2_msgs/msg/TFMessage
  发布  /g1/ground_truth              nav_msgs/msg/Odometry

用法：
  ros2 run ... 不适用（这是独立脚本），直接：
  python3 tools/gt_relay.py --ros-args -p robot_name:=robot1_my_bot -p use_sim_time:=true
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

GT_INPUT_TOPIC = "/world/world_demo/pose/info"
GT_OUTPUT_TOPIC = "/g1/ground_truth"


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class GroundTruthRelay(Node):
    def __init__(self):
        super().__init__("gt_relay")

        self.declare_parameter("robot_name", "robot1_my_bot")
        self.declare_parameter("input_topic", GT_INPUT_TOPIC)
        self.declare_parameter("output_topic", GT_OUTPUT_TOPIC)
        self.declare_parameter("frame_id", "gt_odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("scan_topic", "/robot1/velodyne")
        # 原来这里还有个 `publish_tf` 参数：声明了但代码从不读它，
        # 传 true 既不发 TF 也不报错（问题清单「低」）。要么实现要么删，
        # 这里选择删掉 —— 本节点的唯一职责是给真值打仿真时间戳。

        self._robot_name = str(self.get_parameter("robot_name").value)
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._child_frame_id = str(self.get_parameter("child_frame_id").value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(Odometry, output_topic, qos)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._sub = self.create_subscription(
            TFMessage, input_topic, self._on_pose, qos
        )
        scan_topic = str(self.get_parameter("scan_topic").value)
        self._scan_stamp = None
        self._sub_scan = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, 10
        )

        self._seen = 0
        self._last_warn = 0
        self.get_logger().info(
            f"真值中转就绪：{input_topic} → {output_topic}"
            f"（robot_name={self._robot_name}，scan同步={scan_topic}）"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan_stamp = msg.header.stamp

    def _on_pose(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if transform.child_frame_id != self._robot_name:
                continue

            out = Odometry()
            sim_now = self.get_clock().now().to_msg()
            if self._scan_stamp is not None:
                out.header.stamp = self._scan_stamp
            else:
                out.header.stamp = sim_now
            out.header.frame_id = self._frame_id
            out.child_frame_id = self._child_frame_id
            out.pose.pose.position.x = transform.transform.translation.x
            out.pose.pose.position.y = transform.transform.translation.y
            out.pose.pose.position.z = transform.transform.translation.z
            out.pose.pose.orientation = transform.transform.rotation
            # 真值位姿是精确的；协方差给小值而非 0，避免下游当成「无信息」
            out.pose.covariance[0] = 1e-6
            out.pose.covariance[7] = 1e-6
            out.pose.covariance[14] = 1e-6
            out.pose.covariance[21] = 1e-6
            out.pose.covariance[28] = 1e-6
            out.pose.covariance[35] = 1e-6
            self._pub.publish(out)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = out.header.stamp
            tf_msg.header.frame_id = self._frame_id
            tf_msg.child_frame_id = self._child_frame_id
            tf_msg.transform = transform.transform
            self._tf_broadcaster.sendTransform(tf_msg)

            self._seen += 1
            if self._seen == 1:
                self.get_logger().info(
                    f"首帧真值：x={out.pose.pose.position.x:.4f} "
                    f"y={out.pose.pose.position.y:.4f} "
                    f"z={out.pose.pose.position.z:.4f}"
                )
            return


def main():
    rclpy.init()
    node = GroundTruthRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
