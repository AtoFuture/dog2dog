import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from quadropted_msgs.msg import RobotModeCommand
import sys
import time

MODE = sys.argv[1]


def main():
    rclpy.init()
    n = Node('mode_driver')
    mp = n.create_publisher(RobotModeCommand, '/robot1/robot_mode', 10)
    vp = n.create_publisher(Twist, '/robot1/cmd_vel', 10)
    m = RobotModeCommand()
    m.robot_id = 1
    m.mode = MODE
    z = Twist()
    end = time.time() + 10
    while time.time() < end:
        mp.publish(m)
        vp.publish(z)
        time.sleep(0.2)
    time.sleep(5)
    end = time.time() + 3
    while time.time() < end:
        vp.publish(z)
        time.sleep(0.2)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
