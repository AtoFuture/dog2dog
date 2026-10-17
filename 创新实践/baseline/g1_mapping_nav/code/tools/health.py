import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import OccupancyGrid
import time
import threading
import math

res = {}


class S(Node):
    def __init__(self):
        super().__init__('health')
        self.create_subscription(Imu, '/robot1/imu_plugin/out', self.cb, 10)
        self.create_subscription(JointState, '/robot1/joint_states', self.cb2, 10)
        self.create_subscription(OccupancyGrid, '/robot1/map', self.cb3, 10)
        self.create_subscription(OccupancyGrid, '/map', self.cb4, 10)

    def cb(self, m):
        q = m.orientation
        r = math.degrees(math.atan2(
            2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
        s = 2 * (q.w * q.y - q.z * q.x)
        p = math.degrees(math.asin(max(-1, min(1, s))))
        res['imu'] = (round(r, 2), round(p, 2))

    def cb2(self, m):
        res['js'] = len(m.name)

    def cb3(self, m):
        res['map_r'] = (m.info.width, m.info.height)

    def cb4(self, m):
        res['map_o'] = (m.info.width, m.info.height)


def main():
    rclpy.init()
    n = S()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    time.sleep(25)
    with open('/tmp/health.txt', 'w') as f:
        f.write("IMU=%s JS=%s MAP_R=%s MAP_O=%s\n" % (
            res.get('imu'), res.get('js'), res.get('map_r'), res.get('map_o')))


if __name__ == '__main__':
    main()
