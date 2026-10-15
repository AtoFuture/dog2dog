"""ROS2 controller: converts Qwen-RobotNav waypoints to Unitree Go2 velocity commands."""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, Twist
from rclpy.node import Node

from qwen_robotnav import WaypointDecoder


class Go2Controller(Node):
    """Subscribe to predicted waypoints and publish cmd_vel to Go2.

    The node can operate in two modes:
      - ``cmd_vel`` mode: publish ``geometry_msgs/Twist`` on the dedicated
        ``/qwen_robotnav/cmd_vel`` topic.
      - ``unitree_sdk2`` mode: directly call Unitree SDK2 sport API (stub).
    """

    def __init__(self):
        super().__init__("qwen_robotnav_go2_controller")

        # Keep model-generated velocity commands off the shared /cmd_vel topic.
        # The latter may still be owned by teleop or Nav2; publishing zeros there
        # while the VLM is warming up would continuously cancel those commands.
        self.declare_parameter("cmd_vel_topic", "/qwen_robotnav/cmd_vel")
        self.declare_parameter("waypoint_topic", "/qwen_robotnav/waypoints")
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("max_linear", 0.8)
        self.declare_parameter("max_angular", 0.8)
        self.declare_parameter("waypoint_dt", 0.5)
        self.declare_parameter("waypoint_timeout_sec", 2.5)
        self.declare_parameter("controller_mode", "cmd_vel")

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.waypoint_dt = float(self.get_parameter("waypoint_dt").value)
        self.waypoint_timeout_sec = float(
            self.get_parameter("waypoint_timeout_sec").value
        )
        self.controller_mode = self.get_parameter("controller_mode").value

        self.decoder = WaypointDecoder()
        self.latest_waypoints: Optional[np.ndarray] = None
        self.last_waypoint_time = None
        self.has_control = False

        self.waypoint_sub = self.create_subscription(
            PoseArray,
            self.get_parameter("waypoint_topic").value,
            self._waypoint_callback,
            1,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 1)

        self.timer = self.create_timer(1.0 / self.control_rate, self._control_loop)

        self.get_logger().info(
            f"Go2Controller started. mode={self.controller_mode}, "
            f"cmd_vel_topic={self.cmd_vel_topic}"
        )

    def _waypoint_callback(self, msg: PoseArray):
        arr = np.zeros((len(msg.poses), 3), dtype=np.float64)
        for i, pose in enumerate(msg.poses):
            arr[i, 0] = pose.position.x
            arr[i, 1] = pose.position.y
            # Recover yaw from quaternion (assume z-up rotation).
            z = pose.orientation.z
            w = pose.orientation.w
            arr[i, 2] = 2.0 * np.arctan2(z, w)
        self.latest_waypoints = arr
        self.last_waypoint_time = self.get_clock().now()

    def _control_loop(self):
        if self.latest_waypoints is None or len(self.latest_waypoints) == 0:
            self._release_control()
            return

        age = (
            self.get_clock().now() - self.last_waypoint_time
        ).nanoseconds / 1e9
        if age > self.waypoint_timeout_sec:
            self.get_logger().warning(
                f"Waypoints expired after {age:.2f}s; releasing model control"
            )
            self.latest_waypoints = None
            self.last_waypoint_time = None
            self._release_control()
            return

        cmd = self.decoder.to_velocity_command(
            self.latest_waypoints,
            dt=self.waypoint_dt,
            max_linear=self.max_linear,
            max_angular=self.max_angular,
        )
        self._publish_cmd_vel(cmd["linear_x"], cmd["angular_z"])
        self.has_control = True

    def _release_control(self):
        """Stop once after model control, then stay silent for other controllers."""
        if not self.has_control:
            return
        self._publish_cmd_vel(0.0, 0.0)
        self.has_control = False

    def _publish_cmd_vel(self, linear_x: float, angular_z: float):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Go2Controller()
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
