#!/usr/bin/env python3
"""假的 ``NavigateToPose`` action server —— 用来在没有 G1 / 没有 Nav2 / 没有机器狗的情况下调 G2。

项目文档里写得很直接：

> `baseline/README.md`：「**G2 的探索节点对着一个假的目标接收器就能调**」

缺了它，G2 的状态机、黑名单、超时判据、震荡抑制这些逻辑在前几周**根本测不到** ——
而这些恰恰是最容易写错、又最难在真机上复现的部分。

--------------------------------------------------------------------------------
它模拟什么

* **正常到达** / **失败** / **卡住不返回**，三种终态
* **抢占** —— 这是重点。低电量返航走的是同一个 action server，
  G3 的返航点会把 G2 在飞的目标顶掉。这个场景真机上只有电量真低时才出现，
  平时复现不了，只能靠假服务器造
* **取消** —— G2 收到停手信号后应当主动 cancel，这里要能响应

⚠️ 关于「抢占」的实现（踩过坑，记在这里）

rclpy 的 ``ActionServer`` 用 ``ReentrantCallbackGroup`` 时，**多个 goal 会并发执行** ——
新目标到达并不会自动终止旧目标。第一版就是这么写的，结果两个目标各跑各的、
**双双返回 SUCCEEDED**，被抢占方完全不知道发生了什么。

真实 Nav2 不是这样：它会停掉当前的 behavior tree，旧目标被终止。
所以这里必须**显式实现抢占**：新目标到达时主动 abort 掉在飞的那个，
并置位一个停止事件让它的执行循环退出。

这个坑值得记下来 —— 用假服务器测出来的结论，只有在假服务器行为**忠实**时才可信。

用法::

    ros2 run vision_explorer fake_goal_server --ros-args \\
        -p outcome:=success -p travel_time:=3.0

参数：

===========================  ==================================================
``outcome``                  ``success`` | ``abort`` | ``hang``；目标终态
``travel_time``              模拟“走过去”的耗时（秒）
``fail_every_n``             每 N 个目标失败一个（0 = 关闭），用来测重试与黑名单
===========================  ==================================================
"""

from __future__ import annotations

import threading
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class FakeGoalServer(Node):
    def __init__(self) -> None:
        super().__init__("fake_goal_server")

        self.declare_parameter("outcome", "success")
        self.declare_parameter("travel_time", 2.0)
        self.declare_parameter("fail_every_n", 0)

        self._goal_seq = 0
        self._preempt_count = 0

        # 当前在飞的目标。用锁保护：_on_goal 在 executor 线程里跑，
        # _execute 在另一个线程里跑，两者会同时碰它。
        self._lock = threading.Lock()
        self._active_handle: ServerGoalHandle | None = None
        self._active_seq: int | None = None

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            # Reentrant：新目标的 goal_callback 必须在旧目标 execute 阻塞期间
            # 也能跑起来 —— 否则抢占根本递不进来，就测不到那个场景了。
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f"假目标服务器就绪：outcome={self.get_parameter('outcome').value} "
            f"travel_time={self.get_parameter('travel_time').value}s"
        )

    # ------------------------------------------------------------------
    def _on_goal(self, goal_request) -> GoalResponse:
        p = goal_request.pose.pose.position

        with self._lock:
            self._goal_seq += 1
            my_seq = self._goal_seq
            victim = self._active_handle
            victim_seq = self._active_seq

        if victim is not None and victim.is_active:
            # ★ 模拟 Nav2 的抢占：停掉在飞的目标，而不是让它继续跑。
            self._preempt_count += 1
            self.get_logger().warn(
                f"⚠️ 抢占：目标 #{victim_seq} 被 #{my_seq} 顶掉 "
                f"（累计 {self._preempt_count} 次）"
            )
            try:
                victim.abort()
                # abort() 会让 victim.is_active 变 False，
                # 它的 _execute 循环据此自行退出 —— 不需要额外的停止信号。
            except Exception as exc:  # pragma: no cover - 防御性
                self.get_logger().warn(f"终止旧目标时出错（可忽略）：{exc}")

        self.get_logger().info(f"收到目标 #{my_seq}: ({p.x:.2f}, {p.y:.2f})")
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info("收到取消请求")
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        with self._lock:
            my_seq = self._goal_seq
            self._active_handle = goal_handle
            self._active_seq = my_seq

        travel = float(self.get_parameter("travel_time").value)
        fail_every = int(self.get_parameter("fail_every_n").value)
        outcome = str(self.get_parameter("outcome").value)

        # 用循环而不是 sleep：要能在“行进”期间响应取消与抢占。
        #
        # 判据用 **goal_handle.is_active**，不要用共享的 threading.Event。
        # 踩过的坑：第一版用一个共享 event，结果新目标的 execute 启动时会
        # clear() 掉它，把发给**旧目标**的停止信号一并抹掉 —— 旧目标于是继续
        # 跑到终点，再去 succeed 一个已被 abort 的句柄，抛
        # "invalid transition from state ABORTED with event SUCCEED"。
        # is_active 是每个句柄自己的状态（abort/succeed/canceled 后都会变 False），
        # 天然按目标隔离，不会串味。
        deadline = time.monotonic() + travel
        while time.monotonic() < deadline:
            if not goal_handle.is_active or goal_handle.is_cancel_requested:
                break
            time.sleep(0.05)

        with self._lock:
            if self._active_seq == my_seq:
                self._active_handle = None
                self._active_seq = None

        # 已被抢占或取消终止 —— 状态由对方设置，这里不能再去 succeed/abort
        if not goal_handle.is_active:
            self.get_logger().info(f"目标 #{my_seq} 被抢占终止")
            return NavigateToPose.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info(f"目标 #{my_seq} 已取消")
            return NavigateToPose.Result()

        if outcome == "hang":
            # 既不成功也不失败 —— 用来测 G2 的超时判据
            self.get_logger().warn(f"目标 #{my_seq} 进入挂起状态（模拟无进展）")
            while rclpy.ok() and goal_handle.is_active and not goal_handle.is_cancel_requested:
                time.sleep(0.1)
            if goal_handle.is_active:
                goal_handle.canceled()
            return NavigateToPose.Result()

        if fail_every > 0 and my_seq % fail_every == 0:
            outcome = "abort"

        if outcome == "abort":
            goal_handle.abort()
            self.get_logger().info(f"目标 #{my_seq} -> 失败(ABORTED)")
        else:
            goal_handle.succeed()
            self.get_logger().info(f"目标 #{my_seq} -> 到达(SUCCEEDED)")

        return NavigateToPose.Result()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeGoalServer()
    # 必须多线程：单线程 executor 下 execute 阻塞时 goal_callback 递不进来，
    # 「抢占」这个场景就永远测不到。
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
