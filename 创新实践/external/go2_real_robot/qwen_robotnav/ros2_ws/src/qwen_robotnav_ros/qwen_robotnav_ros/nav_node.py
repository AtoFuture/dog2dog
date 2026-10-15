"""ROS2 navigation node: receives images, calls Qwen-RobotNav, publishes waypoints."""

from __future__ import annotations

import os
import threading
import traceback
from collections import deque
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, Pose
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from qwen_robotnav import NavigationConfig, NavigationEngine, TaskMode


class NavNode(Node):
    """Subscribe to camera images, maintain history, run Qwen-RobotNav inference."""

    def __init__(self):
        super().__init__("qwen_robotnav_nav_node")

        # Parameters
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("instruction", "Move forward safely.")
        self.declare_parameter("task_mode", "VLN")
        self.declare_parameter("history_length", 4)
        self.declare_parameter("inference_period", 1.0)  # seconds
        self.declare_parameter("vllm_base_url", "https://api.deepseek.com")
        self.declare_parameter("vllm_model", "deepseek-flash")
        self.declare_parameter("camera_name", "Front")
        self.declare_parameter("publish_debug_image", True)

        self.camera_topic = self.get_parameter("camera_topic").value
        self.instruction = self.get_parameter("instruction").value
        self.history_length = int(self.get_parameter("history_length").value)
        self.inference_period = float(self.get_parameter("inference_period").value)
        self.camera_name = self.get_parameter("camera_name").value
        self.publish_debug_image = self.get_parameter("publish_debug_image").value

        mode_str = self.get_parameter("task_mode").value
        try:
            task_mode = TaskMode(mode_str)
        except ValueError:
            self.get_logger().warn(f"Unknown task_mode {mode_str}; defaulting to VLN")
            task_mode = TaskMode.VLN

        # Configure navigation engine
        config = NavigationConfig(
            task_mode=task_mode,
            instruction=self.instruction,
            embodiment="quadruped robot",
            vlm=self._make_vlm_config(),
        )
        config.observation.max_frames = self.history_length
        # Single camera on Go2 (head camera). Update weights if multi-cam is added.
        config.cameras = [
            type("CameraConfig", (), {"name": self.camera_name, "azimuth_deg": 0.0, "default_weight": 1.0})()
        ]
        config.observation.camera_weights = {self.camera_name: 1.0}

        self.engine = NavigationEngine(config)
        self.bridge = CvBridge()
        self.frame_history: deque = deque(maxlen=self.history_length)
        self.latest_frame: Optional[np.ndarray] = None
        self.lock = threading.Lock()

        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self._image_callback, 1
        )
        self.instruction_sub = self.create_subscription(
            String, "/qwen_robotnav/instruction", self._instruction_callback, 1
        )
        self.waypoint_pub = self.create_publisher(PoseArray, "/qwen_robotnav/waypoints", 1)
        self.debug_image_pub = self.create_publisher(Image, "/qwen_robotnav/debug_image", 1)

        # Timer for periodic inference
        self.timer = self.create_timer(self.inference_period, self._inference_callback)

        self.get_logger().info(
            f"NavNode started. task_mode={task_mode.value}, "
            f"camera_topic={self.camera_topic}, vllm={config.vlm.base_url}"
        )

    def _make_vlm_config(self):
        from qwen_robotnav.config import VLMConfig

        return VLMConfig(
            base_url=self.get_parameter("vllm_base_url").value,
            model=self.get_parameter("vllm_model").value,
            api_key=os.environ.get("DEEPSEEK_API_KEY") or os.environ.get("VLLM_API_KEY") or None,
        )

    def _image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        with self.lock:
            self.latest_frame = cv_image
            from PIL import Image as PILImage

            rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb)
            self.frame_history.append({self.camera_name: pil_image})

    def _instruction_callback(self, msg: String):
        self.instruction = msg.data
        self.engine.config.instruction = msg.data
        self.get_logger().info(f"Updated instruction: {msg.data}")

    def _inference_callback(self):
        with self.lock:
            if len(self.frame_history) == 0:
                self.get_logger().debug("No frames received yet; skipping inference")
                return
            frames = list(self.frame_history)

        try:
            waypoints, raw_text = self.engine(frames)
            self._publish_waypoints(waypoints)
            self.get_logger().info(
                f"Predicted {len(waypoints)} waypoints; first={waypoints[0].tolist() if len(waypoints) else None}"
            )
            if self.publish_debug_image and self.latest_frame is not None:
                try:
                    self._publish_debug_image(self.latest_frame, waypoints)
                except Exception as exc:
                    self.get_logger().warning(f"Debug image publish failed: {exc!r}")
        except Exception as exc:
            self.get_logger().error(
                f"Inference failed: {exc!r}\n{traceback.format_exc()}"
            )

    def _publish_waypoints(self, waypoints: np.ndarray):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        for w in waypoints:
            pose = Pose()
            pose.position.x = float(w[0])
            pose.position.y = float(w[1])
            pose.position.z = 0.0
            # theta as yaw
            yaw = float(w[2])
            pose.orientation.z = np.sin(yaw / 2.0)
            pose.orientation.w = np.cos(yaw / 2.0)
            msg.poses.append(pose)
        self.waypoint_pub.publish(msg)

    def _publish_debug_image(self, frame: np.ndarray, waypoints: np.ndarray):
        vis = frame.copy()
        h, w = vis.shape[:2]
        # Project first waypoint forward into the image for visualisation.
        if len(waypoints) > 0:
            wx, wy = waypoints[0, 0], waypoints[0, 1]
            # Simple bird's-eye projection onto image center-bottom.
            px = int(w / 2 + wy * 50)
            py = int(h - wx * 50)
            cv2.circle(vis, (px, py), 8, (0, 255, 0), -1)
            cv2.putText(
                vis,
                f"({wx:.2f}, {wy:.2f})",
                (px + 10, py),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )
        # Construct the message directly.  Some ROS Humble cv_bridge builds do
        # not populate the CV_8UC3 (16) reverse mapping used by cv2_to_imgmsg.
        vis = np.ascontiguousarray(vis, dtype=np.uint8)
        debug_msg = Image()
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.height = h
        debug_msg.width = w
        debug_msg.encoding = "bgr8"
        debug_msg.is_bigendian = False
        debug_msg.step = w * 3
        debug_msg.data = vis.tobytes()
        self.debug_image_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
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
