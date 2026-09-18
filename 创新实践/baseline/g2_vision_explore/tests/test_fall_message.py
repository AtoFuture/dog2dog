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
from geometry_msgs.msg import Pose

from g2_core.detector import Detection2D
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


# ----------------------------------------------------------------------
# 节点侧：普通检测消息的字段与运行统计
# ----------------------------------------------------------------------
class _Capture:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _P:
    def __init__(self, v):
        self.value = v


class _Log:
    def __init__(self):
        self.warns = []
        self.infos = []

    def warn(self, m, **k):
        self.warns.append(m)

    def info(self, m, **k):
        self.infos.append(m)

    def error(self, m, **k):
        pass

    def debug(self, m, **k):
        pass


def _fake_node(snapshot=False):
    """绕过 ``Node.__init__`` 造一个够用的节点，只测纯逻辑方法。

    （`Node` 是 Cython 类，直接 `__new__` 后 `get_parameter` 没法用，
    所以用子类把要用的几个方法覆写掉。）
    """
    from vision_detector.detector_node import VisionDetector

    class _N(VisionDetector):
        def __init__(self):
            self._det_pub = _Capture()
            self._log = _Log()
            self._snap = snapshot

        _DEFAULTS = {"publish_snapshot": False, "fall_min_keypoint_conf": 0.5}

        def get_parameter(self, name):
            if name == "publish_snapshot":
                return _P(self._snap)
            if name in self._DEFAULTS:
                return _P(self._DEFAULTS[name])
            raise KeyError(f"假节点没有这个参数：{name}（用到就得在 _DEFAULTS 里补上）")

        def get_logger(self):
            return self._log

        def _publish_snapshot(self, *a, **k):
            raise AssertionError("本用例不该走到抓拍")

    return _N()


def test_normal_detection_sends_nan_not_zero_for_torso_fields():
    """⚠️ 回归：普通检测消息必须显式填 NaN，不能靠消息类型的默认值。

    ROS 消息里 float32 的默认是 **0.0**，而契约要求非异常类填 NaN。
    原先 `_publish_detection` 一个都没碰这三个字段，于是每条 `source="coco"`
    的消息都带着 `torso_height_m = 0.0` —— **那正好是「躺在地上」的值**，
    等于给 G3 凭空造出一批数量占绝对多数、又最像倒地的样本，
    而两条消息唯一的区别只在 `source` 上，不报任何错。
    """
    node = _fake_node()
    det = Detection2D(class_name="person", class_id=0, confidence=0.9,
                      bbox_xyxy=(0.0, 0.0, 10.0, 10.0), track_id=3)

    node._publish_detection(det, Pose(), Time())
    msg = node._det_pub.msgs[0]

    assert msg.source == "coco"
    assert math.isnan(msg.torso_tilt_deg), "普通检测的倾角必须是 NaN，不是 0"
    assert math.isnan(msg.torso_height_m), "普通检测的高度必须是 NaN，不是 0"
    assert msg.onset_stamp.sec == 0 and msg.onset_stamp.nanosec == 0, \
        "契约：非异常类的 onset_stamp 填 0"


# ----------------------------------------------------------------------
# 运行统计
# ----------------------------------------------------------------------
def _stats_node(**overrides):
    node = _fake_node()
    node._fall_enabled = True
    base = {"frames": 100, "dets": 50, "proj_fail": 0, "dedup": 0, "tf_fail": 0,
            "published": 50, "fall_seen": 0, "fall_no_keypoints": 0,
            "fall_no_track": 0, "fall_unusable": 0, "fall_events": 0,
            "fall_tf_fail": 0, "fall_stamp_regress": 0}
    base.update(overrides)
    node._stat = base

    class _T:
        n_tracks = 0
    node._fall_tracker = _T()
    return node


def test_mostly_unusable_warning_fires_when_over_half():
    """⚠️ 回归：这条告警原先的条件恒为假，是**死代码**。

    原条件 `fall_unusable > fall_seen` 永远不成立 —— `fall_unusable` 只在
    `fall_seen += 1` **之后**才可能自增，故恒有 `unusable <= seen`。
    后果：倒地链路最主流的失效模式（深度取到背景 → 躯干不可用，实测占 74%）
    恰恰没有任何自动提示。
    """
    node = _stats_node(fall_seen=100, fall_unusable=74)
    node._report_stats()
    assert any("躯干大多不可用" in w for w in node._log.warns)

    ok = _stats_node(fall_seen=100, fall_unusable=20)
    ok._report_stats()
    assert not any("躯干大多不可用" in w for w in ok._log.warns)


def test_fall_stats_line_prints_even_when_tf_fails_every_frame():
    """⚠️ 回归：TF 全失败时 `fall_seen` 恒为 0，原先整行都不打印。

    运维看到的是「什么都没发生」，和「场景里没人」长得一模一样 ——
    而这两件事的处理方式完全不同。
    """
    node = _stats_node(fall_tf_fail=30)
    node._report_stats()

    assert any("[倒地]" in i for i in node._log.infos), "TF 全失败时也必须打印统计行"
    assert any("TF" in w for w in node._log.warns)


def test_stamp_regression_is_counted_and_warned():
    node = _stats_node(fall_stamp_regress=25)
    node._report_stats()
    assert any("时间戳回退" in w for w in node._log.warns)


def _tf_identity():
    class _V:
        def __init__(self, **kw):
            self.__dict__.update(kw)
    return _V(
        child_frame_id="camera_color_optical_frame",
        transform=_V(
            rotation=_V(x=0.0, y=0.0, z=0.0, w=1.0),
            translation=_V(x=0.0, y=0.0, z=0.0),
        ),
    )


def _person_det(track_id=5):
    """一个躯干尺度落在人体区间内的假检测（用 fx=615、深度 2 m 反推的像素）。"""
    import numpy as np
    kp = np.zeros((17, 3))
    for i, (u, v) in ((5, (280.0, 200.0)), (6, (360.0, 200.0)),
                      (11, (290.0, 320.0)), (12, (350.0, 320.0))):
        kp[i] = (u, v, 0.9)
    return Detection2D(class_name="person", class_id=0, confidence=0.9,
                       bbox_xyxy=(270.0, 190.0, 370.0, 330.0),
                       track_id=track_id, keypoints=kp)


class _RaisingTracker:
    """模拟 `FallTracker.update()` 在时间戳回退时抛 ValueError。"""

    n_tracks = 0

    def update(self, track_id, obs):
        raise ValueError("轨迹 5 的时间戳回退了")

    def prune(self, now, max_age_s=30.0):
        return 0


def test_stamp_regression_does_not_escape_the_callback():
    """⚠️ 回归：`FallTracker.update()` 抛的 ValueError 必须在这一层被接住。

    不接的后果不是「静默错」，而是**更糟**：异常穿过 `_on_images`（检测循环
    那段没有 try），而 rclpy 的 executor 会在 **spin 线程**上重新抛出回调异常
    —— 倒地和普通检测**一起停**，进程带 traceback 退出。

    触发条件是现实的：`ApproximateTimeSynchronizer` 不保证回调的时间戳单调
    （两路 BEST_EFFORT 图像乱序、rosbag --loop、/clock 跳变都会造成）。

    正确行为：丢掉这一帧的**判据**输入并计数，但保留这一帧的普通检测。
    """
    import numpy as np
    from g2_core.projector import CameraIntrinsics

    node = _fake_node()
    node._fall_enabled = True
    node._stat = {k: 0 for k in ("fall_seen", "fall_no_keypoints", "fall_no_track",
                                 "fall_unusable", "fall_events", "fall_tf_fail",
                                 "fall_stamp_regress")}
    node._fall_tracker = _RaisingTracker()
    node._intrinsics = CameraIntrinsics(fx=615.0, fy=615.0, cx=320.0, cy=240.0)

    depth = np.full((480, 640), 2.0, dtype=np.float32)
    node._assess_fall(_person_det(), depth, _tf_identity(), Time())   # 不应抛

    assert node._stat["fall_stamp_regress"] == 1
    assert node._stat["fall_events"] == 0
