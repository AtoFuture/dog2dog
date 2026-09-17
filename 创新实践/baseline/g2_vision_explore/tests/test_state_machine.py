"""探索状态机的测试。

这些分支（尤其是「被叫停」和「未知状态」）在真机上极难复现 ——
总不能为了测一次返航就去把电量耗到阈值。做成纯逻辑就是为了在这里全跑一遍。
"""

from __future__ import annotations

import pytest

from g2_core.state_machine import (
    MISSION_EXPLORING,
    Command,
    ExplorerStateMachine,
    Phase,
)


@pytest.fixture()
def sm():
    return ExplorerStateMachine()


# ----------------------------------------------------------------------
# 正常流程
# ----------------------------------------------------------------------
def test_starts_idle_and_wants_to_send(sm):
    assert sm.phase is Phase.IDLE
    assert sm.next_command() is Command.SEND_GOAL


def test_send_then_wait(sm):
    sm.next_command()
    sm.on_goal_sent()

    assert sm.phase is Phase.NAVIGATING
    assert sm.next_command() is Command.NONE, "有目标在飞时不应再发"


def test_success_returns_to_idle(sm):
    sm.on_goal_sent()
    sm.on_goal_result(success=True)

    assert sm.phase is Phase.IDLE
    assert sm.next_command() is Command.SEND_GOAL


def test_failure_also_returns_to_idle(sm):
    """失败也回 IDLE —— 拉黑是 GoalSelector 的职责，不在状态机里重复。"""
    sm.on_goal_sent()
    sm.on_goal_result(success=False)

    assert sm.phase is Phase.IDLE
    assert sm.next_command() is Command.SEND_GOAL


def test_exhausted_goes_done_and_stays(sm):
    sm.on_exhausted()

    assert sm.phase is Phase.DONE
    assert sm.next_command() is Command.NONE
    # 终止态：反复调用也不应再发目标
    assert sm.next_command() is Command.NONE


# ----------------------------------------------------------------------
# 被叫停 —— 本模块存在的核心理由
# ----------------------------------------------------------------------
def test_mission_state_returning_cancels_the_inflight_goal(sm):
    """G3 宣布返航时，G2 必须先把在飞目标让出来。

    否则两边会抢同一个 action server，机器人回不了家。
    """
    sm.on_goal_sent()
    assert sm.phase is Phase.NAVIGATING

    sm.on_mission_state("returning")

    assert sm.next_command() is Command.CANCEL_GOAL, "应主动让出，而不是等被抢占"


def test_cancel_is_issued_only_once(sm):
    """cancel 是边沿触发 —— 每个 tick 重复 cancel 会刷屏且无意义。"""
    sm.on_goal_sent()
    sm.on_mission_state("returning")

    assert sm.next_command() is Command.CANCEL_GOAL
    assert sm.next_command() is Command.NONE
    assert sm.next_command() is Command.NONE


def test_no_new_goal_after_cancel_until_cancel_confirms(sm):
    """cancel 还没生效前不能又发新目标 —— 那等于自己把取消抵消掉。"""
    sm.on_goal_sent()
    sm.on_mission_state("returning")
    sm.next_command()  # CANCEL_GOAL

    for _ in range(5):
        assert sm.next_command() is Command.NONE


def test_enters_passive_after_cancel(sm):
    sm.on_goal_sent()
    sm.on_mission_state("returning")
    sm.next_command()  # CANCEL_GOAL
    sm.on_goal_cancelled()

    assert sm.phase is Phase.IDLE  # 先回 IDLE
    sm.next_command()              # 但因为任务态不是探索，会进 PASSIVE
    assert sm.phase is Phase.PASSIVE


def test_passive_when_no_goal_in_flight(sm):
    """空闲时收到返航信号，直接进 PASSIVE。"""
    sm.on_mission_state("returning")
    sm.next_command()

    assert sm.phase is Phase.PASSIVE


def test_passive_sends_nothing(sm):
    sm.on_mission_state("returning")
    for _ in range(5):
        assert sm.next_command() is Command.NONE
    assert sm.phase is Phase.PASSIVE


def test_recovers_when_exploration_resumes(sm):
    sm.on_mission_state("returning")
    sm.next_command()
    assert sm.phase is Phase.PASSIVE

    sm.on_mission_state(MISSION_EXPLORING)
    assert sm.phase is Phase.IDLE
    assert sm.next_command() is Command.SEND_GOAL


def test_estop_also_stops(sm):
    """急停比低电量更危险 —— 同样必须停手。"""
    sm.on_goal_sent()
    sm.on_mission_state("estop")

    assert sm.next_command() is Command.CANCEL_GOAL


# ----------------------------------------------------------------------
# 安全属性：未知状态一律停手
# ----------------------------------------------------------------------
@pytest.mark.parametrize("state", ["paused", "charging", "returning", "estop", "", "banana"])
def test_unknown_mission_state_means_stop(sm, state):
    """**未知状态必须当作「停手」。**

    这是刻意选的方向：将来 G3 加了新状态，G2 不需要跟着改就能正确让出。
    反过来（未知当作可探索）会在 G3 引入新状态的当天，静默地让 G2 继续抢道——
    而那正是这个状态机要防的事故。
    """
    assert not ExplorerStateMachine(mission_state=state).should_explore
    assert ExplorerStateMachine(mission_state=state).next_command() is Command.NONE


def test_exploring_is_the_only_go_state(sm):
    assert ExplorerStateMachine(mission_state=MISSION_EXPLORING).should_explore
    assert not ExplorerStateMachine(mission_state="Exploring").should_explore, "大小写敏感"


# ----------------------------------------------------------------------
# 回归：没有状态信号时会怎样
# ----------------------------------------------------------------------
def test_without_mission_signal_g2_keeps_resending(sm):
    """**记录这条接口缺口的具体后果** —— 不是期望行为，是反证。

    如果 G3 不发任务状态、或者「先发返航点、后广播状态」，
    G2 收到的是 ABORTED（被抢占与导航失败在 action 层无法区分），
    于是它会心安理得地继续发下一个目标 —— 把返航点抢占掉。

    这个测试钉住现状，说明为什么必须补那条接口。
    """
    sm.on_goal_sent()          # G2 发探索目标
    sm.on_goal_result(success=False)   # 被 G3 的返航点抢占（但看起来就是"失败"）

    assert sm.phase is Phase.IDLE
    assert sm.next_command() is Command.SEND_GOAL, (
        "没有任务状态信号时，G2 会重发目标并抢占返航点 —— 这正是要修的缺口"
    )


def test_snapshot_is_serialisable(sm):
    snap = sm.snapshot()
    assert set(snap) == {"phase", "mission_state", "should_explore"}
    assert snap["phase"] == "IDLE"
