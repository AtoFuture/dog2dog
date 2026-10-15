"""ROS2 navigation node: receives images, calls DeepSeek, publishes waypoints."""

from __future__ import annotations

import os
import threading
import time
import traceback
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, Pose
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import Image
from std_msgs.msg import String

from qwen_robotnav import NavigationConfig, NavigationEngine, TaskMode


class NavNode(Node):
    """Receive images while a single worker runs DeepSeek inference."""

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
        self.declare_parameter("inference_timeout_sec", 15.0)
        self.declare_parameter("max_frame_age_sec", 15.0)
        self.declare_parameter("require_mission_state", False)
        self._gate_by_mission = bool(self.get_parameter("require_mission_state").value)
        self._mission_allowed = not self._gate_by_mission

        self.camera_topic = self.get_parameter("camera_topic").value
        self.instruction = self.get_parameter("instruction").value
        self.history_length = int(self.get_parameter("history_length").value)
        self.inference_period = float(self.get_parameter("inference_period").value)
        self.camera_name = self.get_parameter("camera_name").value
        self.publish_debug_image = self.get_parameter("publish_debug_image").value
        self.max_frame_age_sec = float(self.get_parameter("max_frame_age_sec").value)
        if not np.isfinite(self.max_frame_age_sec) or self.max_frame_age_sec <= 0:
            raise ValueError("max_frame_age_sec must be finite and positive")

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
        self._last_frame_time = None
        self._last_frame_stamp = None
        self.lock = threading.Lock()
        self._executor = ThreadPoolExecutor(max_workers=1)
        self._inference_future = None
        self._inference_frame = None
        self._inference_frame_time = None
        self._inference_frame_stamp = None
        self._instruction_revision = 0
        self._inference_revision = 0

        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self._image_callback, 1
        )
        self.instruction_sub = self.create_subscription(
            String, "/qwen_robotnav/instruction", self._instruction_callback, 1
        )
        self.mission_sub = self.create_subscription(
            String, '/mission_state', self._mission_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.waypoint_pub = self.create_publisher(PoseArray, "/qwen_robotnav/waypoints", 1)
        self.debug_image_pub = self.create_publisher(Image, "/qwen_robotnav/debug_image", 1)

        # Timer for periodic inference
        self._inference_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(self.inference_period, self._inference_callback, clock=self._inference_clock)
        self.result_timer = self.create_timer(0.1, self._poll_inference, clock=self._inference_clock)

        self.get_logger().info(
            f"NavNode started. task_mode={task_mode.value}, "
            f"camera_topic={self.camera_topic}, provider=DeepSeek, "
            f"endpoint={config.vlm.base_url}, model={config.vlm.model}"
        )

    def _make_vlm_config(self):
        from qwen_robotnav.config import VLMConfig

        return VLMConfig(
            base_url=self.get_parameter("vllm_base_url").value,
            model=self.get_parameter("vllm_model").value,
            api_key=os.environ.get("DEEPSEEK_API_KEY") or os.environ.get("VLLM_API_KEY") or None,
            timeout=float(self.get_parameter("inference_timeout_sec").value),
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
            self._last_frame_time = time.monotonic()
            self._last_frame_stamp = msg.header.stamp

    def _instruction_callback(self, msg: String):
        self.instruction = msg.data
        self._instruction_revision += 1
        self.engine.config.instruction = msg.data
        self.get_logger().info(f"Updated instruction: {msg.data}")

    def _mission_callback(self, msg: String):
        if not self._gate_by_mission:
            return
        allowed = msg.data.strip().lower() == 'exploring'
        if allowed != self._mission_allowed:
            self._instruction_revision += 1
        self._mission_allowed = allowed

    def _inference_callback(self):
        if not self._mission_allowed:
            return
        if self._inference_future is not None:
            return
        with self.lock:
            if len(self.frame_history) == 0:
                self.get_logger().debug("No frames received yet; skipping inference")
                return
            if time.monotonic() - self._last_frame_time > self.max_frame_age_sec:
                return
            frames = list(self.frame_history)
            self._inference_frame = self.latest_frame.copy()
            self._inference_frame_time = self._last_frame_time
            self._inference_frame_stamp = self._last_frame_stamp
            self._inference_revision = self._instruction_revision

        self._inference_future = self._executor.submit(self.engine, frames)

    def _poll_inference(self):
        future = self._inference_future
        if future is None or not future.done():
            return
        self._inference_future = None
        try:
            waypoints, raw_text = future.result()
            if self._inference_revision != self._instruction_revision:
                self.get_logger().warning("Discarding DeepSeek result for a superseded instruction")
                return
            if time.monotonic() - self._inference_frame_time > self.max_frame_age_sec:
                self.get_logger().warning("Discarding DeepSeek result for an expired camera frame")
                return
            self._publish_waypoints(waypoints)
            self.get_logger().info(
                f"Predicted {len(waypoints)} waypoints; first={waypoints[0].tolist() if len(waypoints) else None}"
            )
            if self.publish_debug_image and self._inference_frame is not None:
                try:
                    self._publish_debug_image(self._inference_frame, waypoints)
                except Exception as exc:
                    self.get_logger().warning(f"Debug image publish failed: {exc!r}")
        except Exception as exc:
            self.get_logger().error(
                f"Inference failed: {exc!r}\n{traceback.format_exc()}"
            )
        finally:
            self._inference_frame = None
            self._inference_frame_time = None
            self._inference_frame_stamp = None

    def destroy_node(self):
        self.timer.cancel()
        self.result_timer.cancel()
        self._executor.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()

    def _publish_waypoints(self, waypoints: np.ndarray):
        msg = PoseArray()
        # Relative goals belong to the image acquisition pose, not the pose
        # reached after several seconds of model inference.
        msg.header.stamp = self._inference_frame_stamp or self.get_clock().now().to_msg()
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
