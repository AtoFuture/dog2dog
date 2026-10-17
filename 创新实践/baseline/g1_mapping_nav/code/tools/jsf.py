import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
import threading

res = {}


class S(Node):
    def __init__(self):
        super().__init__('jsf')
        self.create_subscription(JointState, '/robot1/joint_states', self.cb, 10)
        self.create_subscription(Float64MultiArray, '/robot1/joint_group_controller/commands', self.cb2, 10)

    def cb(self, m):
        res['js'] = res.get('js', 0) + 1

    def cb2(self, m):
        res['cmd'] = res.get('cmd', 0) + 1


def main():
    rclpy.init()
    n = S()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    time.sleep(20)
    with open('/tmp/jsf.txt', 'w') as f:
        f.write("JS=%s CMD=%s\n" % (res.get('js', 0), res.get('cmd', 0)))


if __name__ == '__main__':
    main()
