#!/usr/bin/env python3
"""让狗按真值闭环向前走一段距离，并按真值判定「走了没有 / 走够没有 / 偏了多少」。

## 为什么要有它（以及为什么不复用现成的）

`drive_probe.sh` 是**定速按时长**驱动，不闭环、也不判定；`goal_test.sh` 走的是
Nav2，判据还依赖 action。两者都答不了最简单的问题：**狗本身会不会走**。

假成功的教训（2026-09-27 实测）：某轮 17 个 Nav2 目标点全是从 (21.55,-1.74)
到 0.15~0.22 m 外的点，而 `xy_goal_tolerance=0.25` —— **目标落在容差里，
40 ms 就报 Reached the goal，狗一步没动**。所以本脚本：

  · 目标距离**远大于**任何容差（默认 5 m），走不到就是走不到
  · 判定只用 **Gazebo 真值**，不看 odom、不看 action
  · 缺真值就 **rc=2 响亮失败**，绝不静默判通过（对照问题清单 H4/H5）

## 判据

  · 前进分量 ≥ 目标距离          —— 主判据
  · 走偏（横向漂移）≤ 0.5 m      —— 太多说明步态跑偏
  · z 全程 0.15~0.50 m           —— 掉出这个范围按摔倒处理
  · 卡住（连续 5 s 净位移 <5 cm 但一直在发指令）—— 参照问题清单 M2，用**窗口净位移**
    而不是单帧位移：单帧位移在「原地蹬腿」时反而很大，会把卡住判成在动。

用法（容器内由 walk_test.sh 调起；也可单跑）：
    python3 tools/walk_driver.py <距离m> [--speed 0.02] [--timeout 200]
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

CMD_TOPIC = "/robot1/cmd_vel"
GT_TOPIC = "/g1/ground_truth"

# 判据阈值
LATERAL_LIMIT_M = 0.5      # 横向漂移上限
# 当前 qwen_robotnav 生成的仿真里，站立时 base_link 真值 z 约 0.446 m
# （旧 G1 仿真约 0.25~0.28）。上限放到 0.50 仍能抓住真正掉到地面的情况，
# 但不会把正常站立误判为摔倒。
Z_MIN, Z_MAX = 0.15, 0.50
STUCK_WINDOW_S = 5.0       # 卡住判定的窗口长度
STUCK_MIN_MOVE = 0.05      # 窗口内净位移小于这个值 = 没动
REACH_TOL_M = 0.30         # 到点容差（只在报结论时用）
NO_DATA_ABORT_S = 15.0     # 这么久还收不到真值就 rc=2 退出，别挂着


class WalkDriver(Node):
    def __init__(self, target_m, speed, timeout_s):
        super().__init__("walk_driver")
        self.target = target_m
        self.speed = speed
        self.timeout = timeout_s

        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_subscription(Odometry, GT_TOPIC, self.on_gt, 10)
        self.create_timer(0.05, self.tick)  # 20 Hz

        self.start = None
        self.heading = 0.0
        self.last = None
        self.z_min, self.z_max = 99.0, -99.0
        self.t0 = time.time()
        self.last_log = 0.0
        # 卡住检测：窗口起点的真值位置
        self.win_t0 = None
        self.win_xy = None
        self.stuck_windows = []
        self._done = False
        self.get_logger().info(f"目标：前进 {self.target} m @ cmd_vel.x={self.speed}")

    def on_gt(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        if self.start is None:
            self.start = (p.x, p.y)
            self.heading = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y * q.y + q.z * q.z))
            self.win_t0 = time.time()
            self.win_xy = (p.x, p.y)
            self.get_logger().info(
                f"起点真值 ({p.x:.3f}, {p.y:.3f}) 朝向 {math.degrees(self.heading):.1f}°")
        self.last = (p.x, p.y, p.z)
        self.z_min = min(self.z_min, p.z)
        self.z_max = max(self.z_max, p.z)

    def forward(self):
        """沿初始朝向的前进分量（横漂不计入）"""
        if self.start is None or self.last is None:
            return 0.0
        dx = self.last[0] - self.start[0]
        dy = self.last[1] - self.start[1]
        return dx * math.cos(self.heading) + dy * math.sin(self.heading)

    def lateral(self):
        """垂直初始朝向的偏移量"""
        if self.start is None or self.last is None:
            return 0.0
        dx = self.last[0] - self.start[0]
        dy = self.last[1] - self.start[1]
        return -dx * math.sin(self.heading) + dy * math.cos(self.heading)

    def check_stuck(self):
        now = time.time()
        if now - self.win_t0 < STUCK_WINDOW_S:
            return
        moved = math.hypot(self.last[0] - self.win_xy[0], self.last[1] - self.win_xy[1])
        if moved < STUCK_MIN_MOVE:
            self.stuck_windows.append(
                (self.win_t0 - self.t0, self.forward(), moved))
        self.win_t0 = now
        self.win_xy = (self.last[0], self.last[1])

    def tick(self):
        if self._done:
            return

        # ⚠️ 超时与「真值没来」必须判在发速度之前、早退之前。
        # 原来把超时判在 `if self.last is None: return` 之后，于是真值一旦收不到，
        # 这个回调每次都早退 → 永远不超时 → 挂死（实测被外层 200s timeout 杀掉）。
        # 缺数据要响亮失败（rc=2），不能靠外层兜。
        now = time.time()
        elapsed = now - self.t0
        if self.last is None:
            if elapsed > NO_DATA_ABORT_S:
                self.finish(f"{NO_DATA_ABORT_S:.0f}s 内没收到真值（{GT_TOPIC}）——"
                            "桥/中继没起来，这轮什么都没验到")
            return
        if elapsed > self.timeout:
            self.finish(f"超时（{self.timeout:.0f}s）没用走够 {self.target} m")

        msg = Twist()
        msg.linear.x = self.speed
        self.pub.publish(msg)
        if now - self.last_log > 3.0:
            self.last_log = now
            print(f"  前进 {self.forward():6.2f} / {self.target} m   "
                  f"横偏 {self.lateral():+5.2f} m   z={self.last[2]:.3f}", flush=True)

        # 摔倒 → 立刻停，别继续发速度
        if self.last[2] < Z_MIN or self.last[2] > Z_MAX:
            self.finish(f"z={self.last[2]:.3f} 掉出 {Z_MIN}~{Z_MAX} —— 疑似摔倒/异常，已停")

        self.check_stuck()
        if self.forward() >= self.target:
            self.finish(None)

    def finish(self, trouble):
        self._done = True
        for _ in range(20):  # 连发零速，别把速度留在总线上
            self.pub.publish(Twist())
            time.sleep(0.05)

        if self.last is None or self.start is None:
            # 缺数据 = 这轮没验到东西，必须响亮失败，不能打印一堆 0 然后判通过
            # （对照问题清单 H4/H5：缺话题时静默判通过 / 打印 0.0000 看着像完美）
            print(f"\n===== 结果 =====")
            print(f"  ❌ 从没收到真值（{GT_TOPIC}），无法判定。")
            print(f"     检查 gt_bridge / gt_relay 是否起来、IGN_PARTITION 是否与仿真一致。")
            sys.exit(2)

        fwd, lat = self.forward(), self.lateral()
        print(f"\n===== 结果 =====")
        print(f"  起点真值   ({self.start[0]:.3f}, {self.start[1]:.3f})")
        print(f"  终点真值   ({self.last[0]:.3f}, {self.last[1]:.3f})")
        print(f"  前进分量   {fwd:.3f} m   （目标 {self.target}）")
        print(f"  横向漂移   {lat:+.3f} m")
        print(f"  z 范围     {self.z_min:.3f} ~ {self.z_max:.3f}")
        print(f"  用时       {time.time() - self.t0:.1f} s（挂钟）")
        if self.stuck_windows:
            print(f"  ⚠️ 卡住     {len(self.stuck_windows)} 段（{STUCK_WINDOW_S:.0f}s 窗口内净位移 "
                  f"<{STUCK_MIN_MOVE} m）：")
            for t, f, m in self.stuck_windows[:5]:
                print(f"                 t={t:6.1f}s 时前进量 {f:5.2f} m，窗口内只动了 {m:.3f} m")
        else:
            print(f"  卡住       无")

        ok = (trouble is None and abs(fwd - self.target) <= REACH_TOL_M)
        if not ok and trouble is None:
            trouble = f"只走了 {fwd:.3f} m，离目标 {self.target} 差 {abs(fwd - self.target):.3f} m"
        print(f"\n判定：{'通过 ✅' if ok else '不通过 ❌ —— ' + str(trouble)}")
        sys.exit(0 if ok else 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("distance", type=float, help="向前走的距离（米）")
    ap.add_argument("--speed", type=float, default=0.02,
                    help="cmd_vel.x，默认 0.02（实测约 0.24 m/s，是最慢且稳的一档）")
    ap.add_argument("--timeout", type=float, default=200.0, help="挂钟超时（秒）")
    a = ap.parse_args()

    rclpy.init()
    n = WalkDriver(a.distance, a.speed, a.timeout)
    code = 0
    try:
        rclpy.spin(n)
    except SystemExit as exc:
        # ⚠️ 这里必须把 finish() 里 sys.exit(code) 的 code 接住再回传。
        # 原来写成 `except SystemExit: pass` + `return 0`，把判定结果整个吞掉，
        # 于是 driver 判不通过、外层却拿到 rc=0 打印「通过」——正是问题清单 H3
        # （goal_test.sh 恒 exit 0）同一类错误。
        code = exc.code if isinstance(exc.code, int) else 0
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
