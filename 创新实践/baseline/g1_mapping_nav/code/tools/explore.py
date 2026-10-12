import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Float64MultiArray
import math
import time
import threading
import sys
import os as _os

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
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.create_subscription(Imu, '/robot1/imu_plugin/out', self.cb_imu, 20)
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

    def cb_imu(self, m):
        q = m.orientation
        import math as _m
        self.roll_deg = _m.degrees(_m.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)))
        s = 2*(q.w*q.y-q.z*q.x)
        self.pitch_deg = _m.degrees(_m.asin(max(-1,min(1,s))))

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
        line = '[%6.1fs] %s' % (time.time() - self.t0, s)
        self.log.append(line)
        try:
            print(line, flush=True)
        except Exception:
            pass

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
        t_end = time.time() + 55.0
        while time.time() < t_end:
            if abs(self.roll_deg) > 30 or abs(self.pitch_deg) > 30:
                self.note('  !! tilt roll=%.1f pitch=%.1f - stop turn' % (self.roll_deg, self.pitch_deg))
                break
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
            # NOTE: EKF 是 two_d_mode，z 恒为 0，不能用 z 判摔倒。改为看 IMU 倾角。
            if abs(self.roll_deg) > 40 or abs(self.pitch_deg) > 40 or self.z < -0.5:
                self.note('TILTED/FALLEN roll=%.1f pitch=%.1f - ABORT' % (self.roll_deg, self.pitch_deg))
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
                if abs(self.roll_deg) > 40 or abs(self.pitch_deg) > 40:
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
    # 用 os._exit 规避 rclpy 关闭时的 C++ abort（terminate called ...）
    _os._exit(0)


if __name__ == '__main__':
    main()
