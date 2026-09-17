#!/usr/bin/env python3
"""抢占探针：两个客户端抢同一个 ``NavigateToPose`` 服务器时，输的那个看到什么？

--------------------------------------------------------------------------------
它回答什么问题

契约规定低电量返航复用接口 02，也就是 G3 的返航点和 G2 的探索目标
**打同一个 action server**。Nav2 一次只收一个 goal，所以会抢占。

要决定 G2 怎么保护自己，必须先知道：**被顶掉的目标，客户端收到的是什么？**

* 如果被抢占表现为 ``CANCELED``，而导航失败表现为 ``ABORTED`` —— 那 G2 可以
  靠状态码区分，不需要额外接口。
* 如果两者都是 ``ABORTED`` —— 那 G2 **无法从 action 层分辨**，
  必须靠 G3 显式广播任务状态（见 docs/议题-接口02返航抢占.md）。

这个探针就是拿来把这件事**实测钉死**的，而不是靠猜。

用法（先起一个 ``fake_goal_server``）::

    ros2 run vision_explorer fake_goal_server --ros-args -p travel_time:=5.0
    ros2 run vision_explorer preemption_probe

输出形如::

    A 发出 -> ACCEPTED
    B 发出（模拟返航抢占）
    A 终态 = ABORTED        <- 关键结论
    B 终态 = SUCCEEDED
"""

from __future__ import annotations

import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

STATUS_NAME = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def _goal(x: float, y: float) -> NavigateToPose.Goal:
    g = NavigateToPose.Goal()
    g.pose.header.frame_id = "map"
    g.pose.pose.position.x = x
    g.pose.pose.position.y = y
    g.pose.pose.orientation.w = 1.0
    return g


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("preemption_probe")
        # 两个独立客户端，对应现实里的 G2 与 G3
        self.client_a = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.client_b = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def run(self) -> int:
        if not self.client_a.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("等不到 navigate_to_pose 服务器，先起 fake_goal_server")
            return 1

        # A 先发一个"探索目标"
        fut_a = self.client_a.send_goal_async(_goal(1.0, 0.0))
        rclpy.spin_until_future_complete(self, fut_a, timeout_sec=5.0)
        handle_a = fut_a.result()
        if handle_a is None or not handle_a.accepted:
            self.get_logger().error("目标 A 未被接受")
            return 1
        print(f"A 发出 -> {STATUS_NAME[handle_a.status]}", flush=True)

        result_a = handle_a.get_result_async()

        # 稍等一下，让 A 真的进入执行态
        time.sleep(0.5)

        # B 发返航点 —— 这就是抢占
        print("B 发出（模拟返航抢占）", flush=True)
        fut_b = self.client_b.send_goal_async(_goal(-1.0, 0.0))
        rclpy.spin_until_future_complete(self, fut_b, timeout_sec=5.0)
        handle_b = fut_b.result()
        if handle_b is None or not handle_b.accepted:
            self.get_logger().error("目标 B 未被接受")
            return 1

        result_b = handle_b.get_result_async()

        rclpy.spin_until_future_complete(self, result_a, timeout_sec=20.0)
        rclpy.spin_until_future_complete(self, result_b, timeout_sec=20.0)

        a_status = result_a.result().status if result_a.result() else GoalStatus.STATUS_UNKNOWN
        b_status = result_b.result().status if result_b.result() else GoalStatus.STATUS_UNKNOWN

        print("", flush=True)
        print(f"  A 终态 = {STATUS_NAME[a_status]}", flush=True)
        print(f"  B 终态 = {STATUS_NAME[b_status]}", flush=True)
        print("", flush=True)

        if a_status == GoalStatus.STATUS_CANCELED:
            print(
                "结论：被抢占表现为 CANCELED —— 与导航失败(ABORTED)可区分。",
                flush=True,
            )
        elif a_status == GoalStatus.STATUS_ABORTED:
            print(
                "结论：被抢占表现为 ABORTED —— **与导航失败无法区分**。\n"
                "      G2 不能靠 action 状态码判断自己是不是被抢占了，\n"
                "      必须靠 G3 显式广播任务状态。",
                flush=True,
            )
        else:
            print(f"结论：被抢占表现为 {STATUS_NAME[a_status]}（非预期，需进一步分析）", flush=True)

        print(
            "\n⚠️ 边界说明：本次结论是在 fake_goal_server 上得出的，"
            "而它的抢占行为（abort 旧目标）是**我们对 Nav2 的建模**。\n"
            "   这验证了「被 abort 的客户端看到 ABORTED」这条链路，"
            "但**不能替代对真 Nav2 的验证**。\n"
            "   等 G1 的 Nav2 起来后，把这个探针对着真服务器再跑一次即可（命令不变）。\n"
            "   无论结果是 ABORTED 还是 CANCELED，只要「导航失败」也是 ABORTED，"
            "G2 就无法靠 action 层区分 —— 这时必须走 G3 广播任务状态的方案。",
            flush=True,
        )

        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Probe()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    rc = 1
    try:
        rc = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
