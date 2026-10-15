#!/usr/bin/env python3
"""Bridge model cmd_vel and camera images to a standalone Unitree SDK2 process."""

import json
import socket
import struct
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


CMD_HOST = "127.0.0.1"
CMD_PORT = 5000
IMAGE_BIND = ("0.0.0.0", 5001)
CMD_TIMEOUT_SEC = 0.5


class Go2RosBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2_ros_bridge")

        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("cmd_vel_topic", "/qwen_robotnav/cmd_vel")
        self.declare_parameter("cmd_timeout_sec", CMD_TIMEOUT_SEC)

        camera_topic = self.get_parameter("camera_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.cmd_timeout_sec = self.get_parameter("cmd_timeout_sec").value

        self.cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_socket.setblocking(False)

        self.image_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.image_socket.bind(IMAGE_BIND)
        self.image_socket.setblocking(False)

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, camera_topic, 1)
        self.cmd_sub = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_callback, 1
        )

        self.last_cmd_time = self.get_clock().now()
        # Start silent.  Until this bridge receives a model command it must not
        # interfere with a teleop/Nav2 controller using the Unitree Sport API.
        self.timeout_stop_sent = True
        self.image_connected = False
        self.cmd_timer = self.create_timer(0.05, self.cmd_timer_callback)
        self.registration_timer = self.create_timer(
            1.0, self.registration_timer_callback
        )
        self.image_thread = threading.Thread(target=self.image_loop, daemon=True)
        self.image_thread.start()
        self._send_registration()

        self.get_logger().info(
            f"Go2RosBridge ready. cmd_vel={cmd_vel_topic}, camera={camera_topic}"
        )

    def cmd_callback(self, msg: Twist) -> None:
        self.last_cmd_time = self.get_clock().now()
        self.timeout_stop_sent = (
            abs(msg.linear.x) < 1e-4
            and abs(msg.linear.y) < 1e-4
            and abs(msg.angular.z) < 1e-4
        )
        payload = json.dumps(
            {"vx": msg.linear.x, "vy": msg.linear.y, "vyaw": msg.angular.z}
        )
        try:
            self.cmd_socket.sendto(payload.encode(), (CMD_HOST, CMD_PORT))
        except BlockingIOError:
            pass

    def cmd_timer_callback(self) -> None:
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout_sec and not self.timeout_stop_sent:
            # Send exactly one zero when an active command stream times out.
            payload = json.dumps({"vx": 0.0, "vy": 0.0, "vyaw": 0.0})
            try:
                self.cmd_socket.sendto(payload.encode(), (CMD_HOST, CMD_PORT))
                self.timeout_stop_sent = True
            except BlockingIOError:
                pass

    def _send_registration(self) -> None:
        """Register the image return address without claiming motion control."""
        payload = json.dumps({"vx": 0.0, "vy": 0.0, "vyaw": 0.0})
        try:
            self.cmd_socket.sendto(payload.encode(), (CMD_HOST, CMD_PORT))
        except BlockingIOError:
            pass

    def registration_timer_callback(self) -> None:
        if not self.image_connected:
            self._send_registration()

    def image_loop(self) -> None:
        while rclpy.ok():
            try:
                # Receive 4-byte length header.
                data, _ = self.image_socket.recvfrom(65535)
                if len(data) != 4:
                    continue
                total = struct.unpack(">I", data)[0]

                # Receive chunks until complete.
                buffer = bytearray()
                while len(buffer) < total:
                    chunk, _ = self.image_socket.recvfrom(65535)
                    buffer.extend(chunk)

                jpeg = np.frombuffer(buffer, dtype=np.uint8)
                bgr = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)
                if bgr is None:
                    continue

                self.image_connected = True
                msg = self.bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "go2_camera"
                self.image_pub.publish(msg)
            except BlockingIOError:
                time.sleep(0.005)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"Image loop error: {e}")


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = Go2RosBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
