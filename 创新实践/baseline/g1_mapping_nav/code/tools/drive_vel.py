#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import time

MODE = sys.argv[1]
DUR = float(sys.argv[2])


def main():
    rclpy.init()
    n = Node('cmd_driver')
    pub = n.create_publisher(Twist, '/robot1/cmd_vel', 10)
    msg = Twist()
    if MODE == 'turnl':
        msg.angular.z = 0.2
    elif MODE == 'turnl_slow':
        msg.angular.z = 0.10
    elif MODE == 'fwd':
        msg.linear.x = 0.2
    elif MODE == 'fwd_slow':
        msg.linear.x = 0.1
    elif MODE == 'arc':                 # 边走边转：避免原地旋转导致的扫描涂抹
        msg.linear.x = 0.08
        msg.angular.z = 0.06
    elif MODE == 'arc_l':
        msg.linear.x = 0.08
        msg.angular.z = 0.10
    end = time.time() + DUR
    while time.time() < end:
        pub.publish(msg)
        time.sleep(0.1)
    z = Twist()
    stop_at = time.time() + 3.0
    while time.time() < stop_at:
        pub.publish(z)
        time.sleep(0.1)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
