import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
import math
import time
import threading
import sys

RUN_SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 1200.0

LIN = 0.08
ANG = 0.06
FRONT_STOP = 0.45
SETTLE = 6.0
FWD_MAX = 9.0
TURN_TOL = 0.06


def norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.vel = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.create_subscription(Odometry, '/robot1/odometry/filtered', self.cb_odom, 20)
        self.create_subscription(LaserScan, '/robot1/scan', self.cb_scan, 10)
        self.create_subscription(Float64MultiArray, '/robot1/joint_group_controller/commands',
                                 self.cb_cmd, 10)
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.23
        self.have_odom = False
        self.front = 99.0
        self.n_cmd = 0
        self.log = []

    def cb_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.z = m.pose.pose.position.z
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))
        self.have_odom = True

    def cb_scan(self, m):
        n = len(m.ranges)
        if n < 5:
            return
        half = max(3, int(n * 0.10))
        seg = [r for r in (m.ranges[:half] + m.ranges[-half:])
               if r == r and m.range_min < r < m.range_max]
        self.front = min(seg) if seg else 99.0

    def cb_cmd(self, m):
        self.n_cmd += 1

    def send(self, lin=0.0, ang=0.0):
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self.vel.publish(t)

    def note(self, s):
        self.log.append('[%6.1fs] %s' % (time.time() - self.t0, s))

    def sleep_pub(self, dur, lin=0.0, ang=0.0, stop_early=None):
        end = time.time() + dur
        while time.time() < end:
            if stop_early and stop_early():
                break
            self.send(lin, ang)
            time.sleep(0.1)
        self.send(0.0, 0.0)

    def turn_deg(self, deg):
        target = norm(self.yaw + math.radians(deg))
        self.note('TURN %+d from yaw=%.3f' % (deg, self.yaw))
        t_end = time.time() + 40.0
        while time.time() < t_end:
            err = norm(target - self.yaw)
            if abs(err) < TURN_TOL:
                break
            a = ANG if err > 0 else -ANG
            self.send(0.0, a)
            time.sleep(0.05)
        self.send(0.0, 0.0)
        self.note('  -> yaw=%.3f err=%.3f' % (self.yaw, norm(target - self.yaw)))

    def run(self, total):
        self.t0 = time.time()
        # 等 odom
        while not self.have_odom and time.time() - self.t0 < 20:
            time.sleep(0.2)
        self.note('odom ready z=%.3f cmd=%d' % (self.z, self.n_cmd))
        turns = 0
        while time.time() - self.t0 < total:
            if self.z < 0.16:
                self.note('FALLEN z=%.3f - ABORT' % self.z)
                break
            if self.n_cmd == 0 and time.time() - self.t0 > 30:
                self.note('NO CMD FLOW - ABORT')
                break
            # 1) 站立静止，让 SLAM 收敛
            self.note('settle(stand still)')
            self.sleep_pub(SETTLE)
            # 2) 前进直到遇墙或超时
            x0, y0 = self.x, self.y
            t_seg = time.time()
            while time.time() - t_seg < FWD_MAX:
                if self.front < FRONT_STOP:
                    break
                if self.z < 0.16:
                    break
                self.send(LIN, 0.0)
                time.sleep(0.1)
            self.send(0.0, 0.0)
            d = math.hypot(self.x - x0, self.y - y0)
            self.note('forward %.2fm, front=%.2f' % (d, self.front))
            # 3) 遇墙则后退脱困
            if self.front < FRONT_STOP:
                self.note('blocked -> backup')
                self.sleep_pub(2.0)
                self.turn_deg(180)
                turns += 1
            else:
                self.turn_deg(90)
                turns += 1
        self.send(0.0, 0.0)
        time.sleep(3)
        self.note('DONE turns=%d pos=(%.2f,%.2f)' % (turns, self.x, self.y))
        with open('/tmp/explore_log.txt', 'w') as f:
            f.write('\n'.join(self.log))


def main():
    rclpy.init()
    n = Explorer()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    n.run(RUN_SECONDS)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
