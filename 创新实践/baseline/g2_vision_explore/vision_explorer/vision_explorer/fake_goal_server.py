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
``hang_max_s``               ``hang`` 的最长持续秒数，0 = 不限（默认）
===========================  ==================================================

⚠️ 跑 ``outcome:=hang`` 时建议设 ``hang_max_s``：挂起会**一直占住一个
executor 线程**，多个挂起叠加会耗尽线程池，届时连 goal_callback 都排不上，
抢占彻底失效 —— 那之后得出的测试结论不再可信。超过 3 个并发挂起时会打 ERROR 日志。
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
        self.declare_parameter("hang_max_s", 0.0)
        """``outcome:=hang`` 时的最长挂起秒数。0（默认）= 不设上限。

        ⚠️ 挂起会一直占住一个 executor 线程，多个挂起叠加会耗尽线程池、
        让抢占失效（测试结果就不再可信）。跑 `outcome:=hang` 的实验时建议设一个上限。
        """

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

        # 正在被抢占的句柄（按 id 认领），防止两个并发的 goal_callback
        # 对同一个 victim 各 abort 一次。见 _on_goal 的说明。
        self._preempting: set[int] = set()
        # 当前处于 hang 的目标数。hang 会**一直占住一个 executor 线程**，
        # 所以要有计数并在超阈值时告警。
        self._active_hangs = 0

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

        # ⚠️ 抢占的「检查 + 认领」必须在**同一个锁临界区**里完成。
        #
        # 原先的写法是 check-then-act：在锁内读 victim，在锁外判断 is_active
        # 再 abort()。但 ReentrantCallbackGroup 下两个 goal_callback 可以并发，
        # 于是两者可能同时判定「victim 还活着」并各调用一次 abort()。
        # 重复 abort 会抛异常，然后被 except 吞掉并记成「终止旧目标时出错（可忽略）」
        # —— **日志措辞把一个真实的竞态伪装成了噪音**。
        #
        # 现在用一个「正在抢占」的集合做原子认领：只有第一个看到它的
        # callback 会去 abort，第二个直接跳过。
        # 注意 abort() 本身仍然放在锁**外**调用 —— 避免在持自己的锁时
        # 进入 rclpy 的 per-handle 锁（那是锁序反转的经典来源）。
        with self._lock:
            self._accept_count += 1
            my_accept = self._accept_count
            victim = self._active_handle
            victim_seq = self._active_seq

            claimed = (
                victim is not None
                and victim.is_active
                and id(victim) not in self._preempting
            )
            if claimed:
                self._preempting.add(id(victim))
                self._preempt_count += 1
            n = self._preempt_count

        if claimed:
            # ★ 模拟 Nav2 的抢占：停掉在飞的目标，而不是让它继续跑。
            self.get_logger().warn(
                f"⚠️ 抢占：目标 #{victim_seq} 被新目标顶掉（累计 {n} 次）"
            )
            try:
                victim.abort()
                # abort() 会让 victim.is_active 变 False，
                # 它的 _execute 循环据此自行退出 —— 不需要额外的停止信号。
            except Exception as exc:  # pragma: no cover - 防御性
                self.get_logger().warn(f"终止旧目标时出错：{exc}")
            finally:
                with self._lock:
                    self._preempting.discard(id(victim))

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


        # ⚠️ 从这里到函数结束整段包在 try/finally 里，**只在目标真正收尾时才**
        # 把 ``_active_handle`` 摘掉（审核 P1-⑥）。
        #
        # 原先摘除写在这个位置 —— 行进循环刚结束、**进 hang 分支之前**。
        # 后果：整个挂起期间 ``_active_handle`` 都是 None，于是 ``_on_goal`` 里
        # ``victim = self._active_handle`` 取到 None → ``claimed = False``
        # → **不 abort、不计数、不告警**。
        #
        # 具体危害：用 ``-p outcome:=hang`` 同时测「G2 超时」和「返航抢占」时，
        # **抢占根本不会发生**，被抢占的客户端看到的是 ``SUCCEEDED``
        # 而不是真 Nav2 会给的 ``ABORTED`` —— 与这个假服务器自己立的规矩
        # （「用假服务器测出来的结论，只有在假服务器行为忠实时才可信」）直接冲突。
        #
        # 语义上 hang 中的目标**仍然是活跃的**，摘除本来就该等它收尾。
        try:
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
                max_hang = float(self.get_parameter("hang_max_s").value)

                with self._lock:
                    self._active_hangs += 1
                    n_hangs = self._active_hangs
                if n_hangs >= 3:
                    # ⚠️ hang 会**一直占住一个 executor 线程**（这是 hang 语义的应有之义），
                    # 但多个 hang 叠加会耗尽线程池 —— 届时连 goal_callback 都排不上，
                    # 抢占彻底不可能，这个测试工具就失去意义了。所以到这里要吼一声。
                    self.get_logger().error(
                        f"已有 {n_hangs} 个目标处于挂起状态，每个都占着一个 executor 线程。"
                        f"线程池耗尽后 goal_callback 将排不上队，抢占会失效 —— "
                        f"这时的测试结果不可信。可用 -p hang_max_s:=<秒> 给挂起加个上限。"
                    )

                try:
                    deadline = (time.monotonic() + max_hang) if max_hang > 0 else None
                    while rclpy.ok() and goal_handle.is_active and not goal_handle.is_cancel_requested:
                        if deadline is not None and time.monotonic() >= deadline:
                            self.get_logger().warn(
                                f"目标 #{my_seq} 挂起超过 hang_max_s={max_hang}s，自动收尾"
                            )
                            break
                        time.sleep(0.1)
                finally:
                    with self._lock:
                        self._active_hangs -= 1

                self._terminate(goal_handle, "canceled", my_seq, "挂起后被终止")
                return NavigateToPose.Result()

            if fail_every > 0 and my_seq % fail_every == 0:
                outcome = "abort"

            if outcome == "abort":
                self._terminate(goal_handle, "abort", my_seq, "失败(ABORTED)")
            else:
                self._terminate(goal_handle, "succeed", my_seq, "到达(SUCCEEDED)")

            return NavigateToPose.Result()
        finally:
            # ⚠️ 摘除**只能在这里**做 —— 见上面 try 的说明。
            with self._lock:
                if self._active_handle is goal_handle:
                    self._active_handle = None
                    self._active_seq = None

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
