"""倒地**时间判据**的测试。

这一层的存在理由：单帧倾角判据实测不成立（跪、蹲、弯腰的躯干同样不水平，
一个四肢着地跪趴的人测出 84°，比最轻的真倒地还高）。
所以测试的重点不是「倾角算得准不准」，而是
**「什么情况不该报」** —— 那才是单帧判据做不到的部分。
"""

from __future__ import annotations

import pytest

from g2_core.fall_tracker import (
    HORIZONTAL_MIN_DEG,
    PERSIST_S,
    TRANSITION_MAX_S,
    UPRIGHT_MAX_DEG,
    FallTracker,
    TorsoObservation,
)


def feed(tracker, track_id, tilts, dt=0.2, t0=0.0, height=None):
    """按固定间隔喂一串倾角，返回产生的事件列表。"""
    events = []
    for i, tilt in enumerate(tilts):
        ev = tracker.update(track_id, TorsoObservation(
            stamp=t0 + i * dt, tilt_deg=tilt, height_m=height))
        if ev is not None:
            events.append(ev)
    return events


def fall_sequence(n_upright=5, n_lying=20, dt=0.2, upright=5.0, lying=85.0):
    """先直立一段时间，然后在**一帧之内**翻到水平并保持。"""
    return [upright] * n_upright + [lying] * n_lying, dt


# ----------------------------------------------------------------------
# 该报的
# ----------------------------------------------------------------------
def test_upright_then_lying_produces_one_event():
    tracker = FallTracker()
    seq, dt = fall_sequence()
    events = feed(tracker, 1, seq, dt=dt)

    assert len(events) == 1, "一次倒地只应产生一条事件（边沿触发）"


def test_onset_stamp_is_when_the_person_went_horizontal():
    """上报的应当是**事件起始时刻**，不是确认时刻。

    确认时刻取决于 PERSIST_S 这个内部参数，泄漏到接口上会让
    「什么时候摔的」这个评测口径跟着实现细节变。
    """
    tracker = FallTracker()
    seq, dt = fall_sequence(n_upright=5)
    events = feed(tracker, 1, seq, dt=dt)

    expected_onset = 5 * dt          # 第 6 帧转为水平
    assert events[0].onset_stamp == pytest.approx(expected_onset)
    assert events[0].detected_at > events[0].onset_stamp, "确认必然晚于起始"


def test_event_is_not_repeated_while_the_person_keeps_lying():
    """躺着 100 帧也还是一条。逐帧重发会把「1 次倒地」统计成「100 次」。"""
    tracker = FallTracker()
    seq, dt = fall_sequence(n_lying=100)
    events = feed(tracker, 1, seq, dt=dt)

    assert len(events) == 1


# ----------------------------------------------------------------------
# 不该报的 —— 这才是这层的价值
# ----------------------------------------------------------------------
def test_person_already_lying_when_first_seen_is_not_reported():
    """⚠️ 判据的固有边界，不是 bug。

    没有「直立」这个前状态，就无从判断他是摔倒的还是本来就在那儿躺着。
    真机上该由「首次进入视野时给更长的观察期」缓解，而不是猜。
    """
    tracker = FallTracker()
    events = feed(tracker, 1, [85.0] * 50, dt=0.2)

    assert events == []


def test_standing_still_is_not_reported():
    tracker = FallTracker()
    events = feed(tracker, 1, [3.0] * 50, dt=0.2)

    assert events == []


def test_bending_over_and_standing_back_up_is_not_reported():
    """弯腰捡东西：水平只维持了 0.6 s，不到 PERSIST_S。"""
    tracker = FallTracker()
    seq = [5.0] * 5 + [80.0] * 3 + [5.0] * 20
    events = feed(tracker, 1, seq, dt=0.2)

    assert events == [], "短暂弯腰不该报倒地"


def test_slowly_lying_down_is_not_reported():
    """慢慢躺下（自己休息）：倾角**渐变**，越过 30°~60° 这一段花的时间太长。

    这正是「时间上下文」要挡的东西 —— 单帧判据完全挡不住，
    因为它只看某一帧斜不斜，根本不知道人是「摔下来的」还是「躺下去的」。

    构造：5° -> 85° 线性过渡 15 秒。迟滞区挡不住它（迟滞只防止抖动重置），
    真正拦下它的是 `TRANSITION_MAX_S` —— 跨越 30°->60° 用了 (30/80)*15 ≈ 5.6 s。
    """
    tracker = FallTracker()
    n = 75                                       # 15 s @ dt=0.2
    ramp = [5.0 + (85.0 - 5.0) * i / (n - 1) for i in range(n)]
    seq = [5.0] * 10 + ramp + [85.0] * 20
    events = feed(tracker, 1, seq, dt=0.2)

    assert events == [], "慢慢躺下不是摔倒"


def test_a_fast_transition_over_the_same_range_is_reported():
    """对照：同样的角度范围，但**一帧内**翻过去 —— 那才是摔倒。"""
    tracker = FallTracker()
    seq = [5.0] * 10 + [85.0] * 20
    events = feed(tracker, 1, seq, dt=0.2)

    assert len(events) == 1

    # 说清分界线在哪：慢的那条跨越迟滞区花了 5.6 s，远超过 TRANSITION_MAX_S
    assert (HORIZONTAL_MIN_DEG - UPRIGHT_MAX_DEG) / 80.0 * 15.0 > TRANSITION_MAX_S


def test_kneeling_person_who_never_stood_up_is_not_reported():
    """一直跪着不动 —— 没见过他直立过，判不了。"""
    tracker = FallTracker()
    events = feed(tracker, 1, [80.0] * 50, dt=0.2)

    assert events == []


# ----------------------------------------------------------------------
# 高度门 —— 当前默认关闭，测试把两种行为都钉住
# ----------------------------------------------------------------------
def test_height_gate_is_off_by_default_so_kneeling_still_slips_through():
    """诚实记录当前的能力边界。

    一个先站着、再跪下并保持的人，**在高度门关闭时会误报**。
    这是已知缺口 —— 单靠倾角 + 时间分不开跪和躺。
    """
    tracker = FallTracker()
    seq = [5.0] * 5 + [80.0] * 20
    events = feed(tracker, 1, seq, dt=0.2, height=0.6)

    assert len(events) == 1, "当前默认确实分不开跪和躺 —— 见模块 docstring"


def test_height_gate_rejects_kneeling_when_enabled():
    """打开高度门后，跪姿（离地 0.6 m）被拦下，躺姿（0.2 m）放行。"""
    seq = [5.0] * 5 + [80.0] * 20

    kneeling = FallTracker(max_height_m=0.35)
    assert feed(kneeling, 1, seq, dt=0.2, height=0.6) == []

    lying = FallTracker(max_height_m=0.35)
    assert len(feed(lying, 2, seq, dt=0.2, height=0.2)) == 1


def test_missing_height_does_not_block_a_fall_when_gate_is_on():
    """高度没测出来时不能因此把倒地吞掉 —— 宁可误报也不能漏报。"""
    tracker = FallTracker(max_height_m=0.35)
    seq = [5.0] * 5 + [80.0] * 20
    events = feed(tracker, 1, seq, dt=0.2, height=None)

    assert len(events) == 1


# ----------------------------------------------------------------------
# 状态机的细节
# ----------------------------------------------------------------------
def test_hysteresis_band_does_not_reset_the_lying_state():
    """倾角在 30~60° 的空档里抖动，不应把已确认的水平段重置掉。

    没有迟滞的话，一个人躺在 59/61° 附近抖动会反复重置计时。
    """
    tracker = FallTracker()
    mid = (UPRIGHT_MAX_DEG + HORIZONTAL_MIN_DEG) / 2
    seq = [5.0] * 5 + [85.0] * 5 + [mid] * 3 + [85.0] * 20
    events = feed(tracker, 1, seq, dt=0.2)

    assert len(events) == 1
    assert events[0].onset_stamp == pytest.approx(5 * 0.2), "起始时刻不该被抖动推迟"


def test_missing_observation_does_not_advance_the_state():
    """测不出来 ≠ 不水平。把 None 当「不水平」会让漏检悄悄变成「没摔倒」。"""
    tracker = FallTracker()

    feed(tracker, 1, [5.0] * 5, dt=0.2)
    # 中间一段完全没观测
    for i in range(10):
        assert tracker.update(1, TorsoObservation(stamp=1.0 + i * 0.2,
                                                  tilt_deg=None)) is None
    # 再出现时仍然是水平 -> 应当照常触发
    events = feed(tracker, 1, [85.0] * 20, dt=0.2, t0=3.0)

    assert len(events) == 1


def test_two_tracks_are_tracked_independently():
    tracker = FallTracker()
    seq = [5.0] * 5 + [80.0] * 20

    events_a = feed(tracker, 1, seq, dt=0.2)
    events_b = feed(tracker, 2, seq, dt=0.2)

    assert len(events_a) == 1 and len(events_b) == 1
    assert events_a[0].track_id == 1 and events_b[0].track_id == 2


def test_standing_up_then_falling_again_after_cooldown_reports_twice():
    """起来、隔一会儿再摔 —— 是两次事件。

    冷却是为了挡住「躺着翻身」，但不能把真实的第二次摔倒也挡掉。
    """
    tracker = FallTracker()
    seq = ([5.0] * 5 + [80.0] * 20      # 第一次摔
           + [5.0] * 60                  # 站起来一段时间（> 冷却）
           + [80.0] * 20)                # 第二次摔
    events = feed(tracker, 1, seq, dt=0.2)

    assert len(events) == 2


def test_rolling_over_right_after_a_fall_does_not_refire():
    """躺着翻身：刚从水平起来一点又躺回去，不该再报一次。"""
    tracker = FallTracker()
    seq = [5.0] * 5 + [85.0] * 15 + [25.0] * 2 + [85.0] * 15
    events = feed(tracker, 1, seq, dt=0.2)

    assert len(events) == 1, f"冷却期内不该重复触发（得到 {len(events)} 次）"


def test_timestamp_going_backwards_raises():
    """状态机靠时间差判断，回退的时间戳会给出没有意义的结果，必须报错。"""
    tracker = FallTracker()
    tracker.update(1, TorsoObservation(stamp=10.0, tilt_deg=5.0))

    with pytest.raises(ValueError):
        tracker.update(1, TorsoObservation(stamp=9.0, tilt_deg=5.0))


def test_forget_drops_the_track():
    tracker = FallTracker()
    tracker.update(1, TorsoObservation(stamp=0.0, tilt_deg=5.0))
    assert tracker.n_tracks == 1

    tracker.forget(1)

    assert tracker.n_tracks == 0
    tracker.forget(1)          # 幂等


def test_prune_drops_stale_tracks_but_keeps_fresh_ones():
    """目标离开画面后跟踪器会分配新 id，旧 id 的状态不清理就是内存泄漏。"""
    tracker = FallTracker()
    tracker.update(1, TorsoObservation(stamp=0.0, tilt_deg=5.0))
    tracker.update(2, TorsoObservation(stamp=100.0, tilt_deg=5.0))

    n = tracker.prune(now=101.0, max_age_s=30.0)

    assert n == 1, "只该清掉 30 秒没出现的那个"
    assert tracker.n_tracks == 1


def test_prune_on_empty_tracker_is_a_no_op():
    assert FallTracker().prune(now=0.0) == 0


# ----------------------------------------------------------------------
# 「已躺平」的上界（审核 P1-④）
# ----------------------------------------------------------------------
def test_swapped_keypoints_do_not_report_a_fall():
    """⚠️ 回归：倾角 >120° 不算躺平。

    把肩髋两组关键点调换，倾角会变成 ``180° - θ`` —— 一个站立的人（真值 ~0°）
    于是算出 ~180°。若没有上界，`tilt >= 60` 一路放行，**站着的人被报成倒地**。

    这条规则 ``anomaly.FALLEN_MAX_DEG`` 早就写了，但节点侧的倒地路径不走那条函数
    （自己算倾角喂给本状态机），于是它一度**没传过来**。

    ⚠️ 而且 ``check_reprojection`` 抓不到这种错：肩宽/髋宽/躯干长在两组点
    调换下**全都不变**。
    """
    tracker = FallTracker()
    seq = [5.0] * 5 + [175.0] * 30      # 站立 -> 「弄反」后的 175°

    assert feed(tracker, 1, seq, dt=0.2) == [], "肩髋弄反不该被报成倒地"


def test_out_of_range_does_not_break_an_ongoing_lying_run():
    """超出上界应当当「没有信息」，而不是把正在保持的躺平段打断。

    躺姿本来就可能因为关键点抖动瞬间跳出量程。若把这种帧当成「不躺平」，
    `lying_since` 会被反复重置，倒地在真实数据上就永远确认不了。
    """
    tracker = FallTracker()
    seq = [5.0] * 5 + [85.0] * 5 + [170.0] * 2 + [85.0] * 20

    events = feed(tracker, 1, seq, dt=0.2)

    assert len(events) == 1
    assert events[0].onset_stamp == pytest.approx(5 * 0.2), "起始时刻不该被越界帧推迟"


def test_horizontal_max_is_the_same_value_as_the_anomaly_rule():
    """两处上界必须同值 —— 它们表达的是同一条规则。"""
    from g2_core.anomaly import FALLEN_MAX_DEG
    from g2_core.fall_tracker import HORIZONTAL_MAX_DEG

    assert HORIZONTAL_MAX_DEG == FALLEN_MAX_DEG
