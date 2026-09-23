#!/usr/bin/env python3
# Republish /robot1/color/camera_info as /robot1/rgbd_d435/camera_info with frame_id=camera_d435
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
class Relay(Node):
    def __init__(self):
        super().__init__('d435_camera_info_relay')
        self.pub = self.create_publisher(CameraInfo, '/robot1/rgbd_d435/camera_info', 10)
        self.sub = self.create_subscription(CameraInfo, '/robot1/color/camera_info', self.cb, 10)
    def cb(self, msg):
        msg.header.frame_id = 'camera_d435'
        self.pub.publish(msg)
def main():
    rclpy.init(); n = Relay(); rclpy.spin(n)
if __name__ == '__main__':
    main()
