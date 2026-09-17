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


def test_bbox_ratio_pitch_normalisation_makes_standing_harder_to_judge():
    """俯角会让站立的人看起来变扁 —— 归一化后应恢复成不倒地。

    这正说明长宽比方法为什么只能当粗筛：换个机位结论就变。
    """
    bbox = (0.0, 0.0, 40.0, 22.0)  # 俯视 55 度下站立的人，投影被压扁

    raw_fallen, raw_ratio = bbox_aspect_is_fallen(bbox, camera_pitch_rad=0.0)
    fixed_fallen, fixed_ratio = bbox_aspect_is_fallen(bbox, camera_pitch_rad=math.radians(55))

    assert raw_fallen is True, "不做归一化会误判成倒地"
    assert fixed_fallen is False, "归一化后应恢复为站立"
    assert fixed_ratio > raw_ratio


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
