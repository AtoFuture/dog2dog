"""探索状态机 —— 纯逻辑，不依赖 ROS2。

拆出来的理由：状态机的分支（尤其是「被叫停」「被抢占」这些）是最容易写错、
也最难在真机上复现的部分。做成纯逻辑之后，可以在没有 ROS2、没有仿真、
没有机器狗的机器上把每个分支都跑一遍。

ROS2 节点（``vision_explorer/explorer_node.py``）只做翻译：
把 action 的回调与话题消息转成下面这些 ``on_*`` 调用，
再把 ``next_command()`` 的返回变成真正的 action 调用。

--------------------------------------------------------------------------------
为什么需要 PASSIVE 状态

低电量返航走的是**同一个 action server**（契约规定「复用这条通路，不另开接口」）。
Nav2 一次只接受一个 goal，所以 G3 发返航点会**抢占** G2 在飞的目标。

危险在于：`NavigateToPose` 的 result 是 ``std_msgs/Empty``，没有任何 error_code，
客户端只能拿到 SUCCEEDED / ABORTED / CANCELED —— **分不清「导航失败」和「被抢占」**。
（已实测确认，见 docs/议题-接口02返航抢占.md。）

所以不能靠 action 的返回值去猜。正确做法是**由 G3 显式广播任务状态**，
G2 据此转入 PASSIVE 并主动让出。

⚠️ 这带来一条对 G3 的**时序要求**：**先广播状态，再发返航点**。
    否则 G2 会在「还没收到状态」的窗口里继续发目标，把返航点抢占掉。

--------------------------------------------------------------------------------
2026-09-17 经对抗性审查后重写的四处

1. **SEND_GOAL 现在是边沿触发**（原先号称边沿触发，实际连调三次返回三次
   ``SEND_GOAL``）。加了 ``Phase.SENDING`` 中间态：发出指令后必须由
   ``on_goal_sent()`` / ``on_goal_result()`` 确认，期间再调用返回 ``NONE``。

2. **重复广播任务状态不再重置取消标志**。原先 ``on_mission_state`` 每次调用都清
   ``_cancel_issued``，于是 G3 以帧率重复广播 ``"returning"`` 时，
   ``next_command()`` 每拍都返回 ``CANCEL_GOAL``。

3. **加了 ``on_nav_timeout()``**。原先 NAVIGATING 没有超时、没有自恢复 ——
   一次回执丢失就会永久静默（既不发新目标也出不来）。超时的**判据**留在节点里
   （需要时钟），状态机只提供这个入口，保持可测。

4. **``DONE`` 不再被 ``next_command()`` 无声改写成 ``PASSIVE``**。
   终止态就该是终止态；要复活必须显式调 ``revive()``。
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field


class Phase(enum.Enum):
    """探索节点的阶段。"""

    IDLE = "IDLE"
    """可以选下一个目标点。"""

    SENDING = "SENDING"
    """已经给出 SEND_GOAL 指令，等节点回报「发出去了」。

    这个中间态是**边沿触发**的关键：没有它，反复调用 ``next_command()``
    会反复要求发目标（真实场景里 Node 可能因为日志/可视化读两次）。
    """

    NAVIGATING = "NAVIGATING"
    """有目标在飞，等结果。"""

    PASSIVE = "PASSIVE"
    """任务状态不是「探索」—— 让出导航权，不发新目标。"""

    DONE = "DONE"
    """没有可探索的 frontier 了，探索完成。**终止态**，只能由 ``revive()`` 复活。"""


class Command(enum.Enum):
    """状态机要求节点执行的动作。"""

    NONE = "NONE"
    SEND_GOAL = "SEND_GOAL"
    CANCEL_GOAL = "CANCEL_GOAL"


MISSION_EXPLORING = "exploring"
"""任务状态取值里表示「可以探索」的那个。其余值一律视为需要停手。

这样设计是刻意的：**未知的状态值一律当作「停手」**。
将来 G3 加了新状态（比如 ``paused``、``charging``），
G2 不需要跟着改就能正确让出 —— 而反过来（未知状态当作可探索）会在
G3 引入新状态时静默地让 G2 继续抢道，那正是我们要避免的。
"""


@dataclass
class ExplorerStateMachine:
    """探索状态机。

    用法（节点的事件循环里）::

        sm = ExplorerStateMachine()
        ...
        cmd = sm.next_command()
        if cmd is Command.SEND_GOAL:
            result = selector.select(grid, robot_xy, now)   # 返回结构化结果
            if result.status is SelectStatus.GOAL:
                send_goal(result.goal)     # 真正的 action 调用
                sm.on_goal_sent()
            elif result.status is SelectStatus.NO_FRONTIER:
                sm.on_exhausted()          # 只有这一种才算探索完成
            else:
                sm.on_nav_timeout()        # 暂时选不出点，回 IDLE 稍后再试

        elif cmd is Command.CANCEL_GOAL:
            cancel_goal()
            sm.on_goal_cancelled()

    回调里::

        sm.on_goal_result(success=True/False)   # action 终态
        sm.on_mission_state("returning")        # G3 的 /mission_state
        sm.on_nav_timeout()                     # 节点自己的超时判据触发了
        sm.revive()                             # 地图更新后又有了新 frontier
    """

    phase: Phase = Phase.IDLE
    mission_state: str = MISSION_EXPLORING

    # 内部：是否已经为当前这次 PASSIVE 发出过 cancel 指令，
    # 避免每个 tick 都重复 cancel。
    _cancel_issued: bool = field(default=False, repr=False)

    # 内部：异常事件计数（静默吞掉的事件不再静默）
    _anomalies: list[str] = field(default_factory=list, repr=False)

    # ------------------------------------------------------------------
    # 事件
    # ------------------------------------------------------------------
    def on_goal_sent(self) -> bool:
        """目标已发出。返回 ``False`` 表示当前状态不该发目标（异常）。"""
        if self.phase is not Phase.SENDING:
            self._anomalies.append(
                f"on_goal_sent 在 {self.phase.value} 下被调用（预期 SENDING）"
            )
            return False
        self.phase = Phase.NAVIGATING
        return True

    def send_failed(self) -> None:
        """SEND_GOAL 指令发出去了，但动作调用本身失败了（未进入 SENDING 之外的状态）。

        这是给节点用的：``send_goal_async`` 返回失败时调用，
        让状态机回到 IDLE，而不是卡在 SENDING。
        """
        if self.phase is Phase.SENDING:
            self.phase = Phase.IDLE

    def on_goal_result(self, success: bool) -> None:
        """在飞目标到达终态。

        ``success=False`` 涵盖：被拒绝、超时、导航失败、**以及被抢占**。

        ⚠️ 这里**刻意不区分**「失败」与「被抢占」—— 因为 action 层给不出这个信息。
        被抢占的处理不靠这里，靠 ``on_mission_state``：
        只要任务状态不是「探索」，下一个 tick 就会转入 PASSIVE 而不是重发目标。

        ``success`` 参数保留在签名里是给调用方和日志用的 ——
        成功与失败**都**回 IDLE，拉黑是 GoalSelector 的职责，不在这里重复。
        """
        if self.phase not in (Phase.NAVIGATING, Phase.SENDING):
            self._anomalies.append(
                f"on_goal_result 在 {self.phase.value} 下被调用（预期 NAVIGATING/SENDING）"
            )
            return
        self.phase = Phase.IDLE
        self._cancel_issued = False

    def on_goal_cancelled(self) -> None:
        """自己发出的 cancel 已生效。"""
        if self.phase is not Phase.NAVIGATING:
            self._anomalies.append(
                f"on_goal_cancelled 在 {self.phase.value} 下被调用（预期 NAVIGATING）"
            )
            return
        self.phase = Phase.IDLE
        self._cancel_issued = False

    def on_nav_timeout(self) -> bool:
        """节点判定「目标卡住了 / 回执丢了」，主动收尾。

        ⚠️ **这是防死锁的关键入口。** 没有它，NAVIGATING 只能靠 action 回调出来；
        一次回调丢失（消息丢了、节点崩了、被抢占后没人通知）就会**永久静默**：
        既不发新目标，也没有任何日志。

        返回 ``True`` 表示确实从 NAVIGATING 收尾了。

        **超时判据本身不在状态机里** —— 那需要时钟，放在节点里（见 ``docs/框架规划.md``
        §4.3⑤：目标发出后 N 秒内到目标的距离没有明显缩短即判卡住）。
        """
        if self.phase is not Phase.NAVIGATING:
            return False
        self.phase = Phase.IDLE
        self._cancel_issued = False
        return True

    def on_mission_state(self, state: str) -> None:
        """收到 G3 广播的任务状态。"""
        if state == self.mission_state:
            # ⚠️ 重复广播**什么都不做**。
            # 原先每次调用都清 _cancel_issued，于是 G3 以帧率重复广播
            # "returning" 时，每个 tick 都会重新发一次 cancel。
            return

        self.mission_state = state
        self._cancel_issued = False

        if state == MISSION_EXPLORING:
            # 恢复探索
            if self.phase is Phase.PASSIVE:
                self.phase = Phase.IDLE

    def on_exhausted(self) -> None:
        """没有可探索的 frontier 了 —— **只有这一种情况才算探索完成**。

        ⚠️ 调用方必须区分「真的没有 frontier」与「暂时选不出候选点」：
        后者（候选全被黑名单滤掉、超出测地搜索半径、机器人位姿还没定）
        应当调 ``on_nav_timeout()`` 而不是这个 ——
        否则会把一个瞬时的选点失败当成探索结束，**永久停机**。
        """
        if self.phase in (Phase.IDLE, Phase.PASSIVE):
            self.phase = Phase.DONE

    def revive(self) -> bool:
        """把 DONE 拉回 IDLE（地图更新后又出现了新 frontier 时用）。

        ``DONE`` 是终止态，**不会**被 ``next_command()`` 或 ``on_mission_state``
        自动改写 —— 复活必须是显式动作，这样「终止」才有意义。
        """
        if self.phase is Phase.DONE:
            self.phase = Phase.IDLE
            return True
        return False

    # ------------------------------------------------------------------
    # 决策
    # ------------------------------------------------------------------
    @property
    def should_explore(self) -> bool:
        """任务状态是否允许探索。

        **未知状态一律返回 False** —— 见 ``MISSION_EXPLORING`` 的说明。
        """
        return self.mission_state == MISSION_EXPLORING

    @property
    def anomalies(self) -> list[str]:
        """记录到的不该发生的事件（给节点打日志用）。"""
        return list(self._anomalies)

    def next_command(self) -> Command:
        """返回节点当前该执行的指令。

        ⚠️ **这个方法有副作用**（会推进状态），不是纯查询。
        想在不推进状态的前提下看当前阶段，读 ``phase`` 字段。
        """
        if not self.should_explore:
            # 需要停手
            if self.phase in (Phase.NAVIGATING, Phase.SENDING):
                if not self._cancel_issued:
                    self._cancel_issued = True
                    return Command.CANCEL_GOAL
                return Command.NONE
            # 没有在飞目标：进入 PASSIVE 待命。
            # 注意**不碰 DONE** —— 终止态不该被无声改写。
            if self.phase is not Phase.DONE:
                self.phase = Phase.PASSIVE
            return Command.NONE

        # 允许探索
        if self.phase is Phase.PASSIVE:
            self.phase = Phase.IDLE
        if self.phase is Phase.IDLE:
            self.phase = Phase.SENDING     # ★ 边沿触发
            return Command.SEND_GOAL
        # SENDING / NAVIGATING / DONE：什么都不做
        return Command.NONE

    # ------------------------------------------------------------------
    def snapshot(self) -> dict:
        """给日志/调试用的状态快照。"""
        return {
            "phase": self.phase.value,
            "mission_state": self.mission_state,
            "should_explore": self.should_explore,
            "anomalies": len(self._anomalies),
        }
