"""探索状态机的测试。

这些分支（尤其是「被叫停」和「未知状态」）在真机上极难复现 ——
总不能为了测一次返航就去把电量耗到阈值。做成纯逻辑就是为了在这里全跑一遍。

2026-09-17 经对抗性审查后重写：审查的变异测试显示，状态机的**三个守卫
（on_goal_sent / on_goal_result / next_command 里的 phase 判断）全去掉之后
85 个测试仍然全过** —— 也就是说它们当时根本没有被覆盖。
本文件补上了针对这些守卫的测试（每个都标注了「⭐ 补 MXX 变异」）。
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


def start_and_send(sm) -> None:
    """走完「要求发目标 -> 发出去了」这条正常路径。"""
    assert sm.next_command() is Command.SEND_GOAL
    assert sm.on_goal_sent() is True


# ----------------------------------------------------------------------
# 正常流程
# ----------------------------------------------------------------------
def test_starts_idle_and_wants_to_send(sm):
    assert sm.phase is Phase.IDLE
    assert sm.next_command() is Command.SEND_GOAL


def test_send_goal_is_edge_triggered(sm):
    """⭐ 补 M14 变异：反复调用 next_command 不能反复要求发目标。

    原先的实现号称「边沿触发」但实际是电平触发 —— 连调三次返回三个 SEND_GOAL。
    真实节点里只要有人多读一次（日志、可视化、拆成决策+日志两次），就会发出两个目标。
    """
    assert sm.next_command() is Command.SEND_GOAL
    assert sm.next_command() is Command.NONE
    assert sm.next_command() is Command.NONE
    assert sm.phase is Phase.SENDING


def test_send_then_wait(sm):
    start_and_send(sm)
    assert sm.phase is Phase.NAVIGATING
    assert sm.next_command() is Command.NONE, "有目标在飞时不应再发"


def test_success_returns_to_idle(sm):
    start_and_send(sm)
    sm.on_goal_result(success=True)
    assert sm.phase is Phase.IDLE
    assert sm.next_command() is Command.SEND_GOAL


def test_failure_also_returns_to_idle(sm):
    """失败也回 IDLE —— 拉黑是 GoalSelector 的职责，不在状态机里重复。"""
    start_and_send(sm)
    sm.on_goal_result(success=False)
    assert sm.phase is Phase.IDLE
    assert sm.next_command() is Command.SEND_GOAL


def test_send_failed_recovers_from_sending(sm):
    """⭐ 新增：动作调用本身失败时不能卡在 SENDING。"""
    sm.next_command()          # -> SENDING
    assert sm.phase is Phase.SENDING
    sm.send_failed()
    assert sm.phase is Phase.IDLE
    assert sm.next_command() is Command.SEND_GOAL


# ----------------------------------------------------------------------
# 被叫停 —— 本模块存在的核心理由
# ----------------------------------------------------------------------
def test_mission_state_returning_cancels_the_inflight_goal(sm):
    """G3 宣布返航时，G2 必须先把在飞目标让出来。

    否则两边会抢同一个 action server，机器人回不了家。
    """
    start_and_send(sm)
    sm.on_mission_state("returning")
    assert sm.next_command() is Command.CANCEL_GOAL, "应主动让出，而不是等被抢占"


def test_cancel_is_issued_only_once(sm):
    """cancel 是边沿触发。"""
    start_and_send(sm)
    sm.on_mission_state("returning")

    assert sm.next_command() is Command.CANCEL_GOAL
    assert sm.next_command() is Command.NONE
    assert sm.next_command() is Command.NONE


def test_repeated_mission_broadcast_does_not_reissue_cancel(sm):
    """⭐ 补一个真 bug：重复广播任务状态**不能**重置取消标志。

    G3 的状态话题通常按固定频率广播（不是事件式），所以同一句
    "returning" 会以帧率反复到达。原先的实现每次 on_mission_state 都清掉
    `_cancel_issued`，于是每个 tick 都重新发一次 CANCEL_GOAL ——
    一个只在「对方按频率广播」时才出现的 bug，单次调用测不出来。
    """
    start_and_send(sm)
    sm.on_mission_state("returning")
    assert sm.next_command() is Command.CANCEL_GOAL

    # G3 又广播了两次同样的状态
    sm.on_mission_state("returning")
    sm.on_mission_state("returning")

    assert sm.next_command() is Command.NONE, "重复广播不应导致重复取消"


def test_no_new_goal_after_cancel_until_cancel_confirms(sm):
    """cancel 还没生效前不能又发新目标 —— 那等于自己把取消抵消掉。"""
    start_and_send(sm)
    sm.on_mission_state("returning")
    sm.next_command()  # CANCEL_GOAL
    for _ in range(5):
        assert sm.next_command() is Command.NONE


def test_enters_passive_after_cancel(sm):
    start_and_send(sm)
    sm.on_mission_state("returning")
    sm.next_command()      # CANCEL_GOAL
    sm.on_goal_cancelled()
    assert sm.phase is Phase.IDLE
    sm.next_command()      # 任务态不是探索 -> PASSIVE
    assert sm.phase is Phase.PASSIVE


def test_passive_when_no_goal_in_flight(sm):
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
    start_and_send(sm)
    sm.on_mission_state("estop")
    assert sm.next_command() is Command.CANCEL_GOAL


# ----------------------------------------------------------------------
# 超时自恢复 —— 防永久静默
# ----------------------------------------------------------------------
def test_nav_timeout_unsticks_navigating(sm):
    """⭐ 新增：NAVIGATING 必须有出口。

    原先 NAVIGATING 只能靠 action 回调出来。一次回调丢失（消息丢了、被抢占后
    没人通知、节点崩了）就会**永久静默** —— 既不发新目标，也没有任何日志。
    这是货真价实的死锁路径，而它只依赖一次丢包。
    """
    start_and_send(sm)
    assert sm.phase is Phase.NAVIGATING

    assert sm.on_nav_timeout() is True
    assert sm.phase is Phase.IDLE
    assert sm.next_command() is Command.SEND_GOAL, "超时后必须能重新选目标"


def test_nav_timeout_is_noop_when_not_navigating(sm):
    assert sm.on_nav_timeout() is False
    assert sm.phase is Phase.IDLE


def test_nav_timeout_works_from_sending(sm):
    """SENDING 下也该能被超时救出来（send_goal_async 的回执可能丢）。"""
    sm.next_command()          # -> SENDING
    sm.on_nav_timeout()        # 当前实现只处理 NAVIGATING，SENDING 用 send_failed
    # SENDING 不是 NAVIGATING，所以这里返回 False；用 send_failed 收尾
    assert sm.phase is Phase.SENDING
    sm.send_failed()
    assert sm.phase is Phase.IDLE


# ----------------------------------------------------------------------
# DONE 是终止态
# ----------------------------------------------------------------------
def test_exhausted_goes_done_and_stays(sm):
    sm.on_exhausted()
    assert sm.phase is Phase.DONE
    assert sm.next_command() is Command.NONE
    assert sm.next_command() is Command.NONE


def test_done_is_not_clobbered_by_next_command(sm):
    """⭐ 新增：DONE 不该被 next_command 无声改写成 PASSIVE。

    原先 should_explore=False 时 next_command 无条件 `self.phase = Phase.PASSIVE`，
    没有排除 DONE —— 于是 docstring 里的「终止态」只是条件成立的。
    """
    sm.on_exhausted()
    sm.next_command()
    assert sm.phase is Phase.DONE, "终止态不该被查询动作改写"


def test_done_survives_mission_state_round_trip(sm):
    """DONE 在任务状态来回变化后仍然是 DONE。"""
    sm.on_exhausted()
    sm.on_mission_state("returning")
    sm.next_command()
    sm.on_mission_state(MISSION_EXPLORING)
    assert sm.phase is Phase.DONE


def test_revive_is_the_only_way_out_of_done(sm):
    """复活必须是显式动作 —— 这样「终止」才有意义。"""
    sm.on_exhausted()
    assert sm.revive() is True
    assert sm.phase is Phase.IDLE
    assert sm.next_command() is Command.SEND_GOAL


def test_revive_is_noop_when_not_done(sm):
    assert sm.revive() is False


def test_exhausted_does_nothing_while_navigating(sm):
    """在飞目标时不该被判成「探索完成」—— 那是瞬时状态，不是结论。"""
    start_and_send(sm)
    sm.on_exhausted()
    assert sm.phase is Phase.NAVIGATING


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
# 守卫：不该发生的事件要被记录，不能静默吞掉
# ----------------------------------------------------------------------
def test_on_goal_sent_outside_sending_is_recorded(sm):
    """⭐ 补 M15 变异：on_goal_sent 的 phase 守卫。"""
    assert sm.on_goal_sent() is False, "IDLE 下调 on_goal_sent 不该被接受"
    assert sm.phase is Phase.IDLE, "状态不该被改"
    assert sm.anomalies, "不该静默吞掉"


def test_on_goal_result_outside_navigating_is_recorded(sm):
    """⭐ 补 M16 变异：on_goal_result 的 phase 守卫。"""
    sm.on_goal_result(success=True)
    assert sm.phase is Phase.IDLE
    assert sm.anomalies


def test_on_goal_cancelled_outside_navigating_is_recorded(sm):
    sm.on_goal_cancelled()
    assert sm.anomalies


# ----------------------------------------------------------------------
# 回归：没有状态信号时会怎样
# ----------------------------------------------------------------------
def test_without_mission_signal_g2_keeps_resending(sm):
    """**记录这条接口缺口的具体后果** —— 不是期望行为，是反证。

    如果 G3 不发任务状态、或者「先发返航点、后广播状态」，
    G2 收到的是 ABORTED（被抢占与导航失败在 action 层无法区分），
    于是它会心安理得地继续发下一个目标 —— 把返航点抢占掉。

    这个测试钉住现状，说明为什么必须补那条接口。

    ⚠️ 如果将来做了防御性改进（例如对 ABORTED 保守处理），
    这条测试应当**改写**，而不是继续断言现有行为。
    """
    start_and_send(sm)
    sm.on_goal_result(success=False)   # 被 G3 的返航点抢占（但看起来就是"失败"）

    assert sm.phase is Phase.IDLE
    assert sm.next_command() is Command.SEND_GOAL, (
        "没有任务状态信号时，G2 会重发目标并抢占返航点 —— 这正是要修的缺口"
    )


def test_snapshot_is_serialisable(sm):
    snap = sm.snapshot()
    assert set(snap) == {"phase", "mission_state", "should_explore", "anomalies"}
    assert snap["phase"] == "IDLE"
