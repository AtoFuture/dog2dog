"""接口 03 异常类字段的测试。

**为什么单独测消息构造**：倒地判据本身（fall_tracker）已经测过了，
但这个函数承载的是**契约**里最容易出错、也最难在联调时发现的几条：

  * `onset_stamp` 是「事件起始时刻」而不是发送时刻 —— 搞错的话
    G3 统计倒地**次数**会把一次算成零次（见 baseline/README.md）
  * 两个 `torso_*` 是**原始观测量**，不是判定结果
  * 异常类**不允许 id=0**

这些如果只在真机上靠「看消息对不对」来验证，代价太高；而它们出错时
**不会报任何错**，只会让 G3 的指标悄悄偏掉 —— 正合本项目一路在猎杀的那类问题。

需要 ``vision_interfaces`` 已编译（容器内 ``colcon build``）。
"""

from __future__ import annotations

import math

import pytest

from builtin_interfaces.msg import Time

from g2_core.fall_tracker import FallEvent
from vision_detector.detector_node import build_fall_detection

pytest.importorskip("vision_interfaces", reason="需要先 colcon build vision_interfaces")


def _Stamp():
    """真实的 builtin_interfaces/Time。

    ⚠️ 不能用替身：ROS 消息字段有**类型断言**，替身会在赋值时就抛，
    而错误信息是 "must be a sub message of type 'Time'" ——
    和契约本身没关系，很容易被误读成「消息定义有问题」。
    """
    return Time()


def _event(onset=1234.75, track_id=7, tilt=88.0, height=0.21):
    return FallEvent(
        onset_stamp=onset, track_id=track_id, tilt_deg=tilt,
        height_m=height, detected_at=onset + 2.0,
    )


def test_message_carries_the_contract_fields():
    msg = build_fall_detection(_event(), _Stamp(), (1.5, -0.5, 0.2), 0.91, True)

    assert msg.class_name == "person_fallen"
    assert msg.source == "pose"
    assert msg.id == 7
    assert msg.confidence == pytest.approx(0.91)
    assert msg.has_snapshot is True
    assert msg.pose.pose.orientation.w == 1.0, "契约：orientation 不使用，恒为单位四元数"
    assert (msg.pose.pose.position.x, msg.pose.pose.position.y,
            msg.pose.pose.position.z) == pytest.approx((1.5, -0.5, 0.2))


def test_onset_stamp_is_the_event_time_not_the_send_time():
    """⚠️ 这条最容易搞错，而且搞错不会报错，只会让倒地次数统计变成零。"""
    msg = build_fall_detection(_event(onset=1234.75), _Stamp(), (0, 0, 0), 0.9, False)

    assert msg.onset_stamp.sec == 1234
    assert msg.onset_stamp.nanosec == 750_000_000


def test_onset_stamp_rounds_up_instead_of_producing_invalid_nanoseconds():
    """浮点误差可能把小数部分凑到 1e9，那是**非法**的 nanosec，DDS 会拒收。"""
    msg = build_fall_detection(_event(onset=100.99999999999), _Stamp(), (0, 0, 0), 0.9, False)

    assert 0 <= msg.onset_stamp.nanosec < 1_000_000_000
    assert msg.onset_stamp.sec == 101


def test_raw_observations_are_carried_through():
    """原始观测量必须原样发出 —— G3 靠它在录好的 bag 上重扫阈值。"""
    msg = build_fall_detection(_event(tilt=87.4, height=0.23), _Stamp(), (0, 0, 0), 0.9, False)

    assert msg.torso_tilt_deg == pytest.approx(87.4)
    assert msg.torso_height_m == pytest.approx(0.23)


def test_missing_height_becomes_nan_not_zero():
    """高度没测出来要发 NaN。发 0 会被 G3 读成「离地 0 米」——
    那正好是「躺在地上」的值，等于凭空造出一个最像倒地的数。"""
    msg = build_fall_detection(_event(height=None), _Stamp(), (0, 0, 0), 0.9, False)

    assert math.isnan(msg.torso_height_m)


def test_anomaly_class_refuses_id_zero():
    """契约：「异常类不允许 id=0」—— 时间判据依赖轨迹连续性。

    这条必须在**产生消息的地方**拦住，而不是写在文档里靠人记得。
    """
    with pytest.raises(ValueError, match="id=0"):
        build_fall_detection(_event(track_id=0), _Stamp(), (0, 0, 0), 0.9, False)
