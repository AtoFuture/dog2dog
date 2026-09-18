"""交付物 2 接线的测试：``explorer_node`` 里两个纯函数。

**为什么单独测这两个函数**：节点本体测不了（要 ROS 环境、要地图、要 action
服务器），但它们承载的是本项目里**最容易静默出错**的一环 ——

``DONE`` 是**终止态**，一旦误推进去，探索就永久停机，而且**没有任何日志**。
而 ``select()`` 有五种返回状态，其中**四种都不是「探完了」**：

    GOAL              选到了            → 发点
    NO_FRONTIER       一格 frontier 都没有 → **这才是真探完了**
    SMALL_FRONTIERS   有格子但每簇都太小  → 稍后再试
    NO_CANDIDATE      候选被黑名单/半径滤空 → 稍后再试
    NO_ROBOT_POSE     定不了机器人在哪    → 稍后再试

这五种的区分是 2026-09-17 对抗性审查补的（``SelectStatus`` 的 docstring 记了
原来的 bug：四种情形都返回 ``None``，调用方一律当「探完了」，
于是「地图还没加载完」会让探索永久停机）。

所以这里**逐个状态钉死**，尤其是那三种必须「稍后再试」的。

不需要 ROS：``explorer_node`` 被 import 时只依赖 rclpy 的消息类型定义，
而下面测的两个函数是纯逻辑 + ``GridMap``。
"""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("nav_msgs", reason="需要 ROS2 环境（容器内 colcon build 之后）")

from g2_core.explorer import Goal, GoalSelection, SelectStatus  # noqa: E402
from g2_core.grid import FREE, UNKNOWN, GridMap  # noqa: E402
from g2_core.state_machine import Command, ExplorerStateMachine, Phase  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402

from vision_explorer.explorer_node import (  # noqa: E402
    occupancy_to_grid,
    select_and_report,
)


# ----------------------------------------------------------------------
# 桩：把 select() 的返回值直接钉死
# ----------------------------------------------------------------------
class _StubSelector:
    """只回一个预设的 ``GoalSelection``，不真跑选点。

    这样测的是**节点的分支**，不是 ``GoalSelector`` 的内部逻辑
    （那个已经由 ``test_exploration.py`` 覆盖）。
    """

    def __init__(self, selection: GoalSelection):
        self._selection = selection
        self.calls: list[tuple] = []

    def select(self, grid, robot_xy, now):
        self.calls.append((grid, robot_xy, now))
        return self._selection


def _goal(x: float, y: float) -> Goal:
    """``Goal`` 有 7 个必填字段，测试里只关心 (x, y)，其余给合法占位值。"""
    return Goal(x=x, y=y, yaw=0.0, cluster_label=0, gain=1.0,
                geodesic_dist_m=1.0, unknown_fraction=0.5)


def _grid(size=20, fill=FREE):
    return GridMap(np.full((size, size), fill, dtype=np.int8), 0.05)


def _sm_in_sending() -> ExplorerStateMachine:
    """造一个处于 ``SENDING`` 的状态机 —— 也就是「刚决定要选点」那一刻。

    模拟节点真实路径：``next_command()`` 给出 ``SEND_GOAL`` 时状态机进 ``SENDING``，
    然后才算选点结果。
    """
    sm = ExplorerStateMachine()
    assert sm.next_command() is Command.SEND_GOAL
    assert sm.phase is Phase.SENDING, "前提：给出 SEND_GOAL 后应进入 SENDING"
    return sm


# ----------------------------------------------------------------------
# occupancy_to_grid
# ----------------------------------------------------------------------
def test_occupancy_converts_value_domain_and_geometry():
    """-1/0/100 原样进 GridMap，resolution 与 origin 都要对上。"""
    msg = OccupancyGrid()
    msg.info.width, msg.info.height = 4, 3
    msg.info.resolution = 0.05
    msg.info.origin.position.x = 1.5
    msg.info.origin.position.y = -2.0
    msg.data = [0, 0, 100, -1,
                0, 100, -1, 0,
                100, -1, 0, 0]

    g = occupancy_to_grid(msg)

    assert g.width == 4 and g.height == 3
    assert g.resolution == pytest.approx(0.05)
    assert g.origin == pytest.approx((1.5, -2.0))
    # ⚠️ data 是 row-major 的一维数组，reshape 顺序写反了不会报错，
    # 只会让整张地图转置 —— 所以这里逐格核对。
    assert g.data[0].tolist() == [0, 0, 100, -1]
    assert g.data[2].tolist() == [100, -1, 0, 0]


def test_occupancy_rejects_size_mismatch():
    """``data`` 长度与 ``info`` 对不上时必须**抛**，不能凑合。

    凑合的后果是 reshape 出别的形状、或直接抛在更深的地方 ——
    两者都比在这里拒绝难查。
    """
    msg = OccupancyGrid()
    msg.info.width, msg.info.height = 4, 3
    msg.info.resolution = 0.05
    msg.data = [0] * 11          # 应为 12

    with pytest.raises(ValueError, match="对不上"):
        occupancy_to_grid(msg)


# ----------------------------------------------------------------------
# select_and_report —— 核心：五种状态各自的后果
# ----------------------------------------------------------------------
def test_goal_is_returned_and_machine_leaves_sending():
    sm = _sm_in_sending()
    goal = _goal(1.5, -0.5)
    sel = _StubSelector(GoalSelection(status=SelectStatus.GOAL, goal=goal))

    out = select_and_report(sel, sm, _grid(), (0.0, 0.0), 0.0)

    assert out is goal, "选到的目标必须原样返回给调用方去发"
    assert sm.phase is Phase.NAVIGATING, "回报 on_goal_sent() 后应进入 NAVIGATING"


def test_no_frontier_is_the_only_status_that_finishes():
    """⭐ ``NO_FRONTIER`` 是**唯一**该推进 ``DONE`` 的状态。"""
    sm = _sm_in_sending()
    sel = _StubSelector(GoalSelection(status=SelectStatus.NO_FRONTIER))

    out = select_and_report(sel, sm, _grid(), (0.0, 0.0), 0.0)

    assert out is None
    assert sm.phase is Phase.DONE, "一格 frontier 都没有 = 真探完了"


@pytest.mark.parametrize("status", [
    SelectStatus.SMALL_FRONTIERS,
    SelectStatus.NO_CANDIDATE,
    SelectStatus.NO_ROBOT_POSE,
])
def test_retryable_statuses_must_not_finish(status):
    """⭐⭐ 回归：这三种都**不是**「探完了」，不许推进 ``DONE``。

    这条测试的由来就是那个 bug：``select()`` 原先四种情形都返回 ``None``，
    调用方一律当「探索完成」推进**不可逆的 DONE**。于是

        * 地图还没加载完         → 永久停机
        * 唯一候选刚进了 30 秒黑名单 → 永久停机
        * 定不了机器人位姿        → 永久停机

    而 ``DONE`` 没有任何日志，表现就是节点从此沉默。

    ⚠️ 所以这里显式列举，不用 ``status is not GOAL`` 之类的简写 ——
    那样将来新增状态值时会被自动归进「探完了」。
    """
    sm = _sm_in_sending()
    sel = _StubSelector(GoalSelection(status=status))

    out = select_and_report(sel, sm, _grid(), (0.0, 0.0), 0.0)

    assert out is None
    assert sm.phase is Phase.IDLE, (
        f"{status.value} 应当回到 IDLE 稍后再试，实得 {sm.phase.value}。"
        f"checkpoint：on_select_failed() 的 exhausted 参数是不是传错了？"
    )


def test_missing_map_does_not_finish_either():
    """⭐ 地图还没到时同理 —— 这是最常见的那种误停机。"""
    sm = _sm_in_sending()
    sel = _StubSelector(GoalSelection(status=SelectStatus.GOAL, goal=_goal(1.0, 1.0)))

    out = select_and_report(sel, sm, None, (0.0, 0.0), 0.0)

    assert out is None
    assert sm.phase is Phase.IDLE, "没地图只是稍后再试，不是探完了"
    assert not sel.calls, "没地图时不该去调 select()"


def test_machine_can_carry_on_after_a_retryable_status():
    """稍后再试之后，下一轮必须**还能再发出目标** —— 否则等于停机。"""
    sm = _sm_in_sending()
    select_and_report(_StubSelector(GoalSelection(status=SelectStatus.SMALL_FRONTIERS)),
                      sm, _grid(), (0.0, 0.0), 0.0)
    assert sm.phase is Phase.IDLE

    assert sm.next_command() is Command.SEND_GOAL, (
        "回到 IDLE 后必须能再次要求发点；返回 NONE 就说明卡死了"
    )
