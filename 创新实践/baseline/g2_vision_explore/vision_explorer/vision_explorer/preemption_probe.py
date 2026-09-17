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

--------------------------------------------------------------------------------
⚠️ 这个探针能证明什么、不能证明什么（2026-09-17 经审查后补充）

**不能：** 对着 ``fake_goal_server`` 跑时，结论是被**假服务器的实现**决定的 ——
它只有 ``abort()`` 这一条终止在飞目标的路径，所以「A 看到 ABORTED」是构造使然。
**不要把它读成「对 Nav2 的实测」。**

**能：** 验证「一个 goal 被服务器 abort 时，客户端确实看到 ABORTED」这条链路，
并给出一个**命令行可复跑的探针** —— 等 G1 的 Nav2 起来后原地复跑，
那一次的结果才是对 Nav2 的实测。

**真正支撑设计决策的证据是定义层面的**（可靠、不依赖本探针）：
``NavigateToPose`` 的 result 是 ``std_msgs/Empty``，没有任何 error_code。
只要真 Nav2 的「被抢占」与「导航失败」共用同一个终态，G2 就无法从 action 层分辨。

用法（先起一个 ``fake_goal_server``）::

    ros2 run vision_explorer fake_goal_server --ros-args -p travel_time:=5.0
    ros2 run vision_explorer preemption_probe

对真 Nav2 复跑时，把上面第一条换成 Nav2 的 bringup 即可，命令其余不变。
"""

from __future__ import annotations

import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

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

        # ------------------------------------------------------------------
        # ⚠️ 关于这条「结论」的证明力 —— 2026-09-17 经审查后重写
        #
        # 必须说清楚：**这个探针本身不能证明 Nav2 的行为。**
        #
        # 拿它对着 fake_goal_server 跑时，结论是被**假服务器的实现**决定的：
        # 假服务器里唯一能让在飞目标终止的路径就是 `victim.abort()`
        # （能让客户端看到 CANCELED 的只有"客户端自己取消"）。
        # 所以 A 必然、也只能看到 ABORTED —— 这是**重言式**，不是发现。
        #
        # 它真正验证的是另一件事，且这件事有价值：
        #   **「一个 goal 被服务器 abort 时，客户端确实看到 ABORTED」**这条链路。
        # 有了这条，才能做下面的推理。
        # ------------------------------------------------------------------
        if a_status == GoalStatus.STATUS_CANCELED:
            print(
                "A 被终止时看到 CANCELED。\n"
                "  → 若真 Nav2 也如此，且「导航失败」是 ABORTED，则两者**可区分**。",
                flush=True,
            )
        elif a_status == GoalStatus.STATUS_ABORTED:
            print(
                "A 被终止时看到 ABORTED。\n"
                "  → 结合 `ros2 interface show nav2_msgs/action/NavigateToPose`\n"
                "    查到的「result 是 std_msgs/Empty、无 error_code」，\n"
                "    说明**若真 Nav2 也走 abort 路径**，客户端就无法区分\n"
                "    「被抢占」与「导航失败」。",
                flush=True,
            )
        else:
            print(f"A 被终止时看到 {STATUS_NAME[a_status]}（非预期，需进一步分析）", flush=True)

        print(
            "\n⚠️ **这条结论的边界，别当成对真 Nav2 的实测：**\n"
            "   本次是在 fake_goal_server 上跑的，而它的抢占行为（abort 旧目标）\n"
            "   是**我们对 Nav2 的建模**，不是 Nav2 本身。\n"
            "   而假服务器只有 abort 这一条终止路径，所以「看到 ABORTED」是\n"
            "   构造使然，不是观察结果。\n"
            "\n"
            "   真正支撑设计决策的是**定义层面的证据**（可靠）：\n"
            "   `NavigateToPose` 的 result 是 `std_msgs/Empty`，没有任何 error_code。\n"
            "   只要真 Nav2 的「被抢占」与「导航失败」共用同一个终态，\n"
            "   G2 就无法从 action 层分辨 —— 这一个推理不依赖本探针。\n"
            "\n"
            "   等 G1 的 Nav2 起来后，**把这个探针对着真服务器再跑一次**（命令不变），\n"
            "   那一次的结果才是对 Nav2 的实测。",
            flush=True,
        )

        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Probe()
    # 注意：这里**不要**再构造 MultiThreadedExecutor 而不 spin ——
    # 那只是死代码，会让人误以为探针跑在多线程下。
    # 本探针的并发发生在**服务端**（fake_goal_server / 真 Nav2），
    # 客户端这边用 `spin_until_future_complete` 的单线程驱动就够了。
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
