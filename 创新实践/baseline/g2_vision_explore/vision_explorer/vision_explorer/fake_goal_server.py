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

        # ⚠️ 编号必须在 _execute 里分配，不能在 _on_goal 里分配。
        #
        # 踩过的坑：第一版在 _on_goal 里分配 my_seq，但那个局部变量传不到
        # _execute（goal_callback 收到的是 goal REQUEST，拿不到 goal_id），
        # 于是 _execute 里重读了**全局计数器** —— 只要 accept 与 execute 之间
        # 又来了一个目标，_execute 就会读到别人的编号，随后收尾时按编号
        # 把**新目标**的台账抹掉，抢占从此静默失效（_preempt_count 不动、
        # 日志一个字不打）。
        #
        # 修法：编号只在 _execute 里分配（执行顺序，唯一）；台账清理改用
        # **句柄身份比较**（`is`），编号再也不参与正确性，只用于日志。
        self._exec_seq = 0
        self._accept_count = 0      # 仅用于日志
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
            self._accept_count += 1
            my_accept = self._accept_count
            victim = self._active_handle
            victim_seq = self._active_seq

        if victim is not None and victim.is_active:
            # ★ 模拟 Nav2 的抢占：停掉在飞的目标，而不是让它继续跑。
            with self._lock:
                self._preempt_count += 1
                n = self._preempt_count
            self.get_logger().warn(
                f"⚠️ 抢占：目标 #{victim_seq} 被新目标顶掉（累计 {n} 次）"
            )
            # check-then-act 不在锁内：两个 goal_callback 可并发，
            # victim 可能已被另一个 callback 或它自己终结。
            # 这里只能靠 is_active 判断 + 兜住异常；重复 abort 会被吞掉并记日志。
            try:
                victim.abort()
                # abort() 会让 victim.is_active 变 False，
                # 它的 _execute 循环据此自行退出 —— 不需要额外的停止信号。
            except Exception as exc:  # pragma: no cover - 防御性
                self.get_logger().warn(f"终止旧目标时出错（很可能已被并发终结）：{exc}")

        self.get_logger().info(
            f"收到目标（accept #{my_accept}）: ({p.x:.2f}, {p.y:.2f})"
        )
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info("收到取消请求")
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        # 编号在这里分配（执行顺序，唯一）。见 __init__ 里的说明：
        # 不能在 _on_goal 里分配，那个编号传不过来。
        with self._lock:
            self._exec_seq += 1
            my_seq = self._exec_seq
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
            # 用**句柄身份**比较，不用编号 —— 编号在并发下可能撞车，
            # 而身份比较不可能误判。这就是「旧目标收尾把新目标台账抹掉」
            # 那个 bug 的根治办法。
            if self._active_handle is goal_handle:
                self._active_handle = None
                self._active_seq = None

        # 已被抢占或取消终止 —— 状态由对方设置，这里不能再去 succeed/abort
        if not goal_handle.is_active:
            self.get_logger().info(f"目标 #{my_seq} 被抢占终止")
            return NavigateToPose.Result()
        if goal_handle.is_cancel_requested:
            self._terminate(goal_handle, "canceled", my_seq, "已取消")
            return NavigateToPose.Result()

        if outcome == "hang":
            # 既不成功也不失败 —— 用来测 G2 的超时判据
            self.get_logger().warn(f"目标 #{my_seq} 进入挂起状态（模拟无进展）")
            # ⚠️ 这个循环只能靠取消/抢占/shutdown 退出。它**会一直占着
            # 一个 executor 线程** —— 这是 hang 语义的应有之义，但要意识到
            # 它是「线程占用」而不是「阻塞一个目标」：多个 hang 目标叠加
            # 会耗尽线程池，届时连 goal_callback 都排不上，抢占彻底不可能。
            while rclpy.ok() and goal_handle.is_active and not goal_handle.is_cancel_requested:
                time.sleep(0.1)
            self._terminate(goal_handle, "canceled", my_seq, "挂起后被终止")
            return NavigateToPose.Result()

        if fail_every > 0 and my_seq % fail_every == 0:
            outcome = "abort"

        if outcome == "abort":
            self._terminate(goal_handle, "abort", my_seq, "失败(ABORTED)")
        else:
            self._terminate(goal_handle, "succeed", my_seq, "到达(SUCCEEDED)")

        return NavigateToPose.Result()

    # ------------------------------------------------------------------
    def _terminate(self, goal_handle, action: str, seq: int, label: str) -> None:
        """统一的落终态入口，兜住竞态。

        ⚠️ 为什么必须兜：`if not goal_handle.is_active` 这类检查与真正落终态之间
        **不是原子区间**（中间还有参数读取、取模、日志等）。抢占完全可能插进这个窗口，
        于是 succeed() 落在一个已被 abort 的句柄上，抛出
        "invalid transition from state ABORTED with event SUCCEED" 且无人接管、
        直接冒到 executor。

        `is_active` 是典型的 check-then-use，**关不掉这个窗口**，
        只能把落终态包起来、把异常降级成一条日志。
        """
        try:
            getattr(goal_handle, action)()
        except Exception as exc:  # pragma: no cover - 竞态，难以稳定复现
            self.get_logger().warn(
                f"目标 #{seq} 落终态({action})失败，很可能刚被抢占（属正常竞态）：{exc}"
            )
            return
        self.get_logger().info(f"目标 #{seq} -> {label}")


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
