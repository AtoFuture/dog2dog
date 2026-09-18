"""倒地判据的测试。

本文件的核心使命：**确保 v1 那个反向判据不会复活**。
v1 写的是 ``angle(T, 重力方向) > 60``（T 由髋指向肩），
站立时 T 朝上、重力朝下、夹角 180 度 —— 站着的人被判倒地。
下面的 ``test_standing_is_not_fallen_*`` 就是钉死这一点的回归测试。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from g2_core.anomaly import (
    FALLEN_MAX_DEG,
    FALLEN_MIN_DEG,
    assess_fall_from_keypoints_3d,
    bbox_aspect_is_fallen,
    check_reprojection,
    keypoints_to_torso_3d,
    midpoints_from_keypoints_2d,
    torso_tilt_from_vertical,
)

UP = np.array([0.0, 0.0, 1.0])
HIGH_CONF = (0.9, 0.9, 0.9, 0.9)


# ----------------------------------------------------------------------
# 判据本体的方向性 —— 最重要的回归测试
# ----------------------------------------------------------------------
def test_standing_torso_tilt_is_near_zero():
    """站立：肩在髋上方 -> 与竖直向上夹角 ≈ 0 度。"""
    hip = np.array([0.0, 0.0, 0.9])
    shoulder = np.array([0.0, 0.0, 1.5])  # 比髋高 0.6 m

    tilt = math.degrees(torso_tilt_from_vertical(shoulder, hip, up=UP))

    assert tilt == pytest.approx(0.0, abs=1e-6)


def test_fallen_torso_tilt_is_near_90():
    """倒地：躯干水平 -> 与竖直向上夹角 ≈ 90 度。"""
    hip = np.array([0.0, 0.0, 0.2])
    shoulder = np.array([0.6, 0.0, 0.2])  # 与髋同高，水平偏移

    tilt = math.degrees(torso_tilt_from_vertical(shoulder, hip, up=UP))

    assert tilt == pytest.approx(90.0, abs=1e-6)


def test_v1_bug_would_have_inverted_this():
    """显式对照 v1 的错误写法，证明两者结论相反。

    v1: angle(T, 重力) 其中 T = shoulder - hip、重力 = (0,0,-1)
    站立时该值 ≈ 180 度，远大于 60 —— 会把站立判成倒地。
    """
    hip = np.array([0.0, 0.0, 0.9])
    shoulder = np.array([0.0, 0.0, 1.5])

    torso = shoulder - hip
    torso = torso / np.linalg.norm(torso)
    v1_angle = math.degrees(math.acos(float(np.clip(np.dot(torso, np.array([0, 0, -1.0])), -1, 1))))

    # v1 的判据会得到 ~180 度，并且满足 "> 60" -> 误判为倒地
    assert v1_angle == pytest.approx(180.0, abs=1e-6)
    assert v1_angle > 60.0

    # 修正后的判据得到 ~0 度 -> 不倒地
    assert math.degrees(torso_tilt_from_vertical(shoulder, hip, up=UP)) < FALLEN_MIN_DEG


# ----------------------------------------------------------------------
# 完整评估
# ----------------------------------------------------------------------
def test_standing_person_is_not_fallen():
    a = assess_fall_from_keypoints_3d(
        shoulder_left_3d=np.array([-0.2, 0.0, 1.5]),
        shoulder_right_3d=np.array([0.2, 0.0, 1.5]),
        hip_left_3d=np.array([-0.15, 0.0, 0.9]),
        hip_right_3d=np.array([0.15, 0.0, 0.9]),
        keypoint_confs=HIGH_CONF,
    )
    assert a.is_fallen is False
    assert a.method == "pose_3d"
    assert a.tilt_deg == pytest.approx(0.0, abs=1e-6)


def test_fallen_person_is_fallen():
    a = assess_fall_from_keypoints_3d(
        shoulder_left_3d=np.array([0.4, -0.2, 0.2]),
        shoulder_right_3d=np.array([0.4, 0.2, 0.2]),
        hip_left_3d=np.array([-0.4, -0.15, 0.2]),
        hip_right_3d=np.array([-0.4, 0.15, 0.2]),
        keypoint_confs=HIGH_CONF,
    )
    assert a.is_fallen is True
    assert a.tilt_deg == pytest.approx(90.0, abs=1e-6)


def test_low_keypoint_confidence_is_discarded_not_judged():
    """关键点看不清时应当丢弃，而不是硬给一个结论。"""
    a = assess_fall_from_keypoints_3d(
        shoulder_left_3d=np.array([0.4, -0.2, 0.2]),
        shoulder_right_3d=np.array([0.4, 0.2, 0.2]),
        hip_left_3d=np.array([-0.4, -0.15, 0.2]),
        hip_right_3d=np.array([-0.4, 0.15, 0.2]),
        keypoint_confs=(0.9, 0.9, 0.2, 0.9),  # 左髋不可靠
    )
    assert a.method == "insufficient"
    assert a.is_fallen is False
    assert a.tilt_deg is None


def test_inverted_keypoints_are_not_reported_as_fallen():
    """肩髋被弄反（tilt > 120 度）不应报倒地，那多半是关键点出错。"""
    a = assess_fall_from_keypoints_3d(
        shoulder_left_3d=np.array([-0.2, 0.0, 0.9]),   # 肩在下面
        shoulder_right_3d=np.array([0.2, 0.0, 0.9]),
        hip_left_3d=np.array([-0.15, 0.0, 1.5]),       # 髋在上面
        hip_right_3d=np.array([0.15, 0.0, 1.5]),
        keypoint_confs=HIGH_CONF,
    )
    assert a.is_fallen is False
    assert a.tilt_deg is not None and a.tilt_deg > FALLEN_MAX_DEG


def test_dispersion_gate_rejects_scattered_points():
    """四个关键点离散得离谱 -> 反投影有问题，应丢弃。"""
    a = assess_fall_from_keypoints_3d(
        shoulder_left_3d=np.array([-0.2, 0.0, 1.5]),
        shoulder_right_3d=np.array([0.2, 0.0, 1.5]),
        hip_left_3d=np.array([-5.0, 0.0, 0.9]),   # 明显跑飞
        hip_right_3d=np.array([5.0, 0.0, 0.9]),
        keypoint_confs=HIGH_CONF,
    )
    assert a.method == "insufficient"


# ----------------------------------------------------------------------
# 边界
# ----------------------------------------------------------------------
def test_exactly_zero_torso_raises():
    with pytest.raises(ValueError):
        torso_tilt_from_vertical(np.array([1.0, 1.0, 1.0]), np.array([1.0, 1.0, 1.0]))


# ----------------------------------------------------------------------
# 方法 A：长宽比粗筛
# ----------------------------------------------------------------------
def test_bbox_ratio_standing_vs_lying():
    standing = (0.0, 0.0, 40.0, 100.0)   # 高瘦
    lying = (0.0, 0.0, 100.0, 35.0)      # 扁宽

    assert bbox_aspect_is_fallen(standing)[0] is False
    assert bbox_aspect_is_fallen(lying)[0] is True


def test_pitch_normalisation_moves_toward_the_unpitched_baseline():
    """俯角归一化必须把比例往「无俯角基准」**拉**，而不是推远。

    ⚠️ 本测试原先断言的是反方向（`fixed_ratio > raw_ratio`），
    把实现里那处方向错误钉成了期望行为，2026-09-17 修正。

    数值实测（针孔模型，D=5m、人高 1.7m、相机 0.35m、f=500）：

        下俯角   h/w原始   ×cos     ÷cos
          0°     3.778    3.778    3.778   ← 基准
         30°     5.735    4.967    6.623

    关键物理事实：相机下俯时，远处竖直目标的**像高会变大**（透视拉长），
    所以观测到的 h/w 随俯角单调**增大**，归一化应当**乘** cos。
    除以 cos 会把它推得更远 —— 让倒地的人显得更像站着，漏检更多。
    """
    BASE_RATIO = 3.778        # θ=0 时的比例（数值实测定标）
    RAW_AT_30DEG = 5.735      # 下俯 30° 时观测到的比例

    # 用 w=100、h=573.5 构造出 raw ratio 5.735 的框
    w = 100.0
    bbox = (0.0, 0.0, w, RAW_AT_30DEG * w)

    _, raw_ratio = bbox_aspect_is_fallen(bbox, camera_pitch_rad=0.0)
    _, fixed_ratio = bbox_aspect_is_fallen(bbox, camera_pitch_rad=math.radians(30))

    assert raw_ratio == pytest.approx(RAW_AT_30DEG, rel=1e-6)

    # 核心断言：归一化后**离基准更近**，而不是更远
    assert abs(fixed_ratio - BASE_RATIO) < abs(raw_ratio - BASE_RATIO), (
        f"归一化把比例从 {raw_ratio:.3f} 变成 {fixed_ratio:.3f}，"
        f"而基准是 {BASE_RATIO:.3f} —— 方向反了"
    )
    assert fixed_ratio < raw_ratio, "下俯 30° 的修正应当减小比例"


def test_midpoints_from_keypoints_2d():
    kp = np.zeros((17, 3))
    kp[5] = [10.0, 20.0, 0.9]    # 左肩
    kp[6] = [30.0, 20.0, 0.8]    # 右肩
    kp[11] = [12.0, 60.0, 0.7]   # 左髋
    kp[12] = [28.0, 60.0, 0.6]   # 右髋

    shoulder_mid, hip_mid, confs = midpoints_from_keypoints_2d(kp)

    assert shoulder_mid == pytest.approx([20.0, 20.0])
    assert hip_mid == pytest.approx([20.0, 60.0])
    assert confs == pytest.approx((0.9, 0.8, 0.7, 0.6))


def test_midpoints_rejects_wrong_shape():
    with pytest.raises(ValueError):
        midpoints_from_keypoints_2d(np.zeros((17, 2)))


# ----------------------------------------------------------------------
# 人体尺度门（2026-09-18 加的，替代不生效的离散度门）
# ----------------------------------------------------------------------
# frame 185916 的**真实测量值**（坐在床上的人，被 pose 模型把右肩点放到了
# 身体轮廓之外，深度取到了背景）。原先的离散度门把它放行了，
# 于是算出一个 88° 的倾角 —— 而人是坐着的。
REAL_BAD_SHOULDER_LEFT = np.array([0.10, 0.26, 1.64])
REAL_BAD_SHOULDER_RIGHT = np.array([0.53, 0.24, 3.03])
REAL_BAD_HIP_LEFT = np.array([0.25, 0.53, 1.51])
REAL_BAD_HIP_RIGHT = np.array([0.39, 0.49, 1.66])


def test_body_proportions_accepts_a_normal_adult():
    assert check_reprojection(
        np.array([-0.2, 0.0, 1.5]), np.array([0.2, 0.0, 1.5]),
        np.array([-0.15, 0.0, 0.9]), np.array([0.15, 0.0, 0.9]),
    ) is None


def test_reprojection_rejects_a_shoulder_out_on_the_background():
    """肩点落到背景 —— 这是真实数据里发生过的失败（frame 185916）。

    ⚠️ 2026-09-18 之后**主质检换成了同侧深度差**，所以拦下它的理由变了：
    从「肩宽 1.45 m 超出人体尺度」变成「两肩深度差 1.39 m 超过 0.40 m」。
    后者更好 —— 它直接测那个失效模式，而且**不含焦距**。
    """
    bad = check_reprojection(
        REAL_BAD_SHOULDER_LEFT, REAL_BAD_SHOULDER_RIGHT,
        REAL_BAD_HIP_LEFT, REAL_BAD_HIP_RIGHT,
    )
    assert bad is not None
    assert "深度差" in bad, f"应当由深度差门拦下，实际理由：{bad}"


def test_reprojection_rejects_a_gross_scale_error():
    """绝对尺度只做**粗错兜底** —— 拦「量纲整体错了」，不拦精度。

    深度编码搞混（毫米当米）会让所有距离差 1000 倍，这一道专门兜住它。
    """
    # 构造：**只有肩宽**超标（髋宽正常、深度差正常），
    # 这样它只能被肩宽那一条界限拦下 —— 否则测试会靠别的界限「蒙混通过」。
    bad = check_reprojection(
        np.array([-30.0, 0.0, 1.5]), np.array([30.0, 0.0, 1.5]),     # 肩宽 60 m
        np.array([-0.15, 0.0, 0.9]), np.array([0.15, 0.0, 0.9]),     # 髋宽 0.30 m ✓
    )
    assert bad is not None
    assert "肩宽" in bad and "粗错兜底" in bad


def test_real_failure_frame_is_now_discarded_not_judged():
    """回归：这一帧必须被丢弃，不能给出倾角结论。

    修复前它返回 method="pose_3d"、tilt_deg≈88°（判为倒地），
    而真值是「人坐在床上」。
    """
    a = assess_fall_from_keypoints_3d(
        shoulder_left_3d=REAL_BAD_SHOULDER_LEFT,
        shoulder_right_3d=REAL_BAD_SHOULDER_RIGHT,
        hip_left_3d=REAL_BAD_HIP_LEFT,
        hip_right_3d=REAL_BAD_HIP_RIGHT,
        keypoint_confs=HIGH_CONF,
    )
    assert a.method == "insufficient"
    assert a.tilt_deg is None
    assert a.is_fallen is False


def test_old_dispersion_gate_alone_would_have_missed_it():
    """说明为什么非加人体尺度门不可：这一帧的离散度其实很小。"""
    pts = np.stack([REAL_BAD_SHOULDER_LEFT, REAL_BAD_SHOULDER_RIGHT,
                    REAL_BAD_HIP_LEFT, REAL_BAD_HIP_RIGHT])
    dispersion = float(np.linalg.norm(
        np.percentile(pts, 75, axis=0) - np.percentile(pts, 25, axis=0)))

    assert dispersion < 1.0, "旧门限放行它，所以旧门不够用"


def test_body_proportions_rejects_a_too_short_torso():
    """肩髋挨在一起（关键点塌缩）也要拦。"""
    bad = check_reprojection(
        np.array([-0.2, 0.0, 1.5]), np.array([0.2, 0.0, 1.5]),
        np.array([-0.15, 0.0, 1.45]), np.array([0.15, 0.0, 1.45]),
    )
    assert bad is not None and "躯干长" in bad


# ----------------------------------------------------------------------
# 关键点 -> 三维躯干（链路里真正调用的那一层）
# ----------------------------------------------------------------------
class _K:
    """够用的 CameraIntrinsics 替身。"""
    fx = fy = 615.0
    cx, cy = 320.0, 240.0


def _kp(shoulder_y=200.0, hip_y=320.0, conf=0.9):
    kp = np.zeros((17, 3))
    kp[5] = (280.0, shoulder_y, conf)
    kp[6] = (360.0, shoulder_y, conf)
    kp[11] = (290.0, hip_y, conf)
    kp[12] = (350.0, hip_y, conf)
    return kp


def _depth(value=2.0, shape=(480, 640)):
    return np.full(shape, value, dtype=np.float32)


#: 相机光学系里的「上」。x 右 / y 下 / z 前，所以「上」是 -y。
#: 注意**不是** (0,0,1) —— 那是「前方」。写错会让站着的人算出 90°。
UP_CAMERA = np.array([0.0, -1.0, 0.0])


def test_keypoints_to_torso_3d_happy_path():
    torso, why = keypoints_to_torso_3d(_kp(), _depth(), _K())

    assert why == ""
    assert torso is not None
    # 肩在图像上方(y 小) -> 相机系 y 更小 -> 躯干朝上，倾角接近 0
    assert abs(torso.tilt_deg(up=UP_CAMERA)) < 5.0
    assert torso.shoulder_mid[2] == pytest.approx(2.0)


def test_tilt_deg_requires_an_explicit_up():
    """``up`` 必填是刻意的。

    相机系里「上」是 (0,-1,0) 而不是 (0,0,1)；给个 (0,0,1) 的默认值会让
    站着的人算出 90° —— 看着像个正经角度，实际是坐标系搞错了。
    """
    torso, _ = keypoints_to_torso_3d(_kp(), _depth(), _K())

    with pytest.raises(TypeError):
        torso.tilt_deg()                       # 不给就是不给

    assert torso.tilt_deg(up=np.array([0.0, 0.0, 1.0])) == pytest.approx(90.0), \
        "用错坐标系会得到 90° —— 这正是为什么要强制写明"


def test_keypoints_to_torso_3d_reports_why_on_low_confidence():
    torso, why = keypoints_to_torso_3d(_kp(conf=0.2), _depth(), _K())

    assert torso is None
    assert "置信度" in why, "失败原因要能直接进日志定位"


def test_keypoints_to_torso_3d_reports_why_on_missing_depth():
    d = _depth()
    d[:, :] = np.nan

    torso, why = keypoints_to_torso_3d(_kp(), d, _K())

    assert torso is None
    assert "深度" in why


def test_keypoints_to_torso_3d_catches_the_real_shoulder_failure():
    """回归：右肩落在身体轮廓外、深度取到背景 —— 必须被深度差门拦下。

    真实数据里的失败（frame 185916）：被遮挡的右肩点被 pose 模型放到了
    身体轮廓**之外**，深度取到了身后那堵墙 —— 两肩深度 1.64 / 3.03，差 1.39 m。

    ⚠️ 构造上要注意一件事：``sample_depth_near`` 用的是 **35x35 窗口**，
    所以关键点离身体**太近**时窗口会「够到」身体、把失效掩盖掉
    （本用例第一版就是这样，关键点只偏离 10 行，p5 仍取到人身上）。
    真实失效里关键点偏离得多得多，这里按同样的量级构造。
    """
    d = _depth(value=1.6)
    d[:, 320:] = 3.0            # 画面右半边（含右肩像素 x=360）整片是背景
    kp = _kp()
    assert kp[6][0] > 320, "右肩必须落在背景那一侧，否则用例无效"
    assert kp[6][0] - 320 > 17, "且要离身体边界超过一个窗口半径，否则 p5 会够到身体"

    torso, why = keypoints_to_torso_3d(kp, d, _K())

    assert torso is None
    assert "深度差" in why, f"应当由深度差门拦下，实际理由：{why}"
