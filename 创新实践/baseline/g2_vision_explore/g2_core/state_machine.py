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
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field


class Phase(enum.Enum):
    """探索节点的阶段。"""

    IDLE = "IDLE"
    """没有在飞的目标，可以选下一个。"""

    NAVIGATING = "NAVIGATING"
    """有目标在飞，等结果。"""

    PASSIVE = "PASSIVE"
    """任务状态不是「探索」—— 让出导航权，不发新目标。"""

    DONE = "DONE"
    """没有可探索的 frontier 了，探索完成。终止态。"""


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
            goal = selector.select(grid, robot_xy, now)
            if goal is None:
                sm.on_exhausted()
            else:
                send_goal(goal)      # 真正的 action 调用
                sm.on_goal_sent()

        elif cmd is Command.CANCEL_GOAL:
            cancel_goal()
            sm.on_goal_cancelled()

    回调里::

        sm.on_goal_result(success=True/False)   # action 终态
        sm.on_mission_state("returning")        # G3 的 /mission_state
    """

    phase: Phase = Phase.IDLE
    mission_state: str = MISSION_EXPLORING

    # 内部：是否已经为当前这次 PASSIVE 发出过 cancel 指令，
    # 避免每个 tick 都重复 cancel。
    _cancel_issued: bool = field(default=False, repr=False)

    # ------------------------------------------------------------------
    # 事件
    # ------------------------------------------------------------------
    def on_goal_sent(self) -> None:
        """目标已发出。"""
        if self.phase not in (Phase.IDLE, Phase.NAVIGATING):
            return
        self.phase = Phase.NAVIGATING

    def on_goal_result(self, success: bool) -> None:
        """在飞目标到达终态。

        ``success=False`` 涵盖：被拒绝、超时、导航失败、**以及被抢占**。

        ⚠️ 这里**刻意不区分**「失败」与「被抢占」—— 因为 action 层给不出这个信息。
        被抢占的处理不靠这里，靠 ``on_mission_state``：
        只要任务状态不是「探索」，下一个 tick 就会转入 PASSIVE 而不是重发目标。
        """
        if self.phase is not Phase.NAVIGATING:
            return
        # 成功与失败**都**回到 IDLE，不在这里分流 ——
        # 失败目标点的拉黑是 GoalSelector 的职责（它有黑名单和 TTL），
        # 状态机重复承担一遍只会让两处逻辑打架。
        # ``success`` 参数保留在签名里，是给调用方和日志用的。
        self.phase = Phase.IDLE
        self._cancel_issued = False

    def on_goal_cancelled(self) -> None:
        """自己发出的 cancel 已生效。"""
        if self.phase is Phase.NAVIGATING:
            self.phase = Phase.IDLE
        self._cancel_issued = False

    def on_mission_state(self, state: str) -> None:
        """收到 G3 广播的任务状态。"""
        self.mission_state = state
        if state != MISSION_EXPLORING:
            # 不是探索态 -> 让出。注意这里**不直接跳到 PASSIVE**，
            # 而是留给 next_command() 处理：若还有在飞目标，先 cancel。
            self._cancel_issued = False
        else:
            # 恢复探索：从 PASSIVE 回到 IDLE
            if self.phase is Phase.PASSIVE:
                self.phase = Phase.IDLE
            self._cancel_issued = False

    def on_exhausted(self) -> None:
        """没有可探索的 frontier 了。"""
        if self.phase is Phase.IDLE:
            self.phase = Phase.DONE

    # ------------------------------------------------------------------
    # 决策
    # ------------------------------------------------------------------
    @property
    def should_explore(self) -> bool:
        """任务状态是否允许探索。

        **未知状态一律返回 False** —— 见 ``MISSION_EXPLORING`` 的说明。
        """
        return self.mission_state == MISSION_EXPLORING

    def next_command(self) -> Command:
        """返回节点当前该执行的指令（边沿触发，可反复调用）。"""
        if not self.should_explore:
            # 需要停手
            if self.phase is Phase.NAVIGATING:
                if not self._cancel_issued:
                    self._cancel_issued = True
                    return Command.CANCEL_GOAL
                return Command.NONE
            # 没有在飞目标：直接进入 PASSIVE 待命
            self.phase = Phase.PASSIVE
            return Command.NONE

        # 允许探索
        if self.phase is Phase.PASSIVE:
            self.phase = Phase.IDLE
        if self.phase is Phase.IDLE:
            return Command.SEND_GOAL
        return Command.NONE

    # ------------------------------------------------------------------
    def snapshot(self) -> dict:
        """给日志/调试用的状态快照。"""
        return {
            "phase": self.phase.value,
            "mission_state": self.mission_state,
            "should_explore": self.should_explore,
        }
