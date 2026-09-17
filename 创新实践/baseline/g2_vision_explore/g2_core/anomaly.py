"""异常识别：人员倒地。

零训练成本路线，但有两个必须说清的前提（见 docs/框架规划.md §4.4）：

**前提一：预训练检测器在躺姿上的召回是存疑的。**
方法 A 和方法 B 都建立在「COCO 预训练的 YOLO 能稳定检出躺着的人」之上。
而 COCO 缺少躺姿样本，已有文献报告 YOLO 会把蹲/躺的人预测成 ``dog``，
且这类检测错误是端到端倒地检测误报的主要来源。
**上真机前必须先用真实躺姿图片实测 recall。**

**前提二：倒地判据必须在三维、重力对齐的坐标系里算。**
用二维图像关键点做不到「不受视角影响」：
  * Go2 头部相机几乎必然下俯，站立的人躯干在图像里也不再竖直；
  * **沿光轴躺倒的人（脚朝相机）在图像里几乎是竖直的，会被判成站立** ——
    这是方向性漏检，不是精度问题。

--------------------------------------------------------------------------------
关于 v1 那个反向的判据（留档，避免再犯）::

    v1 写：  T = S - H   （S=肩中点, H=髋中点）
             倒地 if angle(T, 重力方向) > 60°

    T 由髋指向肩。站立时肩在髋**上方**，所以 T 朝**上**；
    重力方向朝**下**。两者夹角 = 180°，满足 `> 60°` ——
    **站立的人被全部判成倒地，判据完全反了。**

    正确写法见 torso_tilt_from_vertical()：与**竖直向上**方向比，
    站立约 0°、倒地约 90°。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# COCO 17 关键点索引
LEFT_SHOULDER, RIGHT_SHOULDER = 5, 6
LEFT_HIP, RIGHT_HIP = 11, 12

# 判定阈值
FALLEN_MIN_DEG = 60.0
FALLEN_MAX_DEG = 120.0
"""躯干倾角落在 [60, 120] 度判为倒地。

上限 120 度是为了挡掉「肩髋关键点被弄反」造成的假阳性 ——
真正倒立的人（>120 度）在侦查场景里不构成有效威胁目标，
更可能是检测/关键点出错。

**这是起步值，需要用真实数据标定。**
"""


@dataclass
class FallAssessment:
    """一次倒地判定的完整信息。

    保留中间量（角度、覆盖率）而不是只返回 bool —— 调阈值时不必重跑检测，
    也便于把「为什么判倒地」记录下来做复盘。
    """

    is_fallen: bool
    tilt_deg: float | None
    """躯干与竖直向上方向的夹角（度）。站立约 0，倒地约 90。"""
    method: str
    """``"pose_3d"`` / ``"bbox_ratio"`` / ``"insufficient"``。"""
    confidence: float
    """判据的可信度 0~1。pose 方法下由关键点置信度与三维离散度决定。"""
    reason: str


# ----------------------------------------------------------------------
# 方法 A：检测框长宽比（快速粗筛）
# ----------------------------------------------------------------------
def bbox_aspect_is_fallen(
    bbox_xyxy: tuple[float, float, float, float],
    camera_pitch_rad: float = 0.0,
    stand_ratio: float = 1.2,
    fallen_ratio: float = 0.6,
) -> tuple[bool, float]:
    """按检测框长宽比粗判倒地。

    🔴 **本方法已实测失败，不要用它做判定。**
    低机位下它的最优工作点是「召回 43% / 误报 0%」——丢掉一半以上的真实倒地；
    扫描全部阈值与俯仰角都**找不到**同时满足「检出率 >85%、误检率 <10%」的工作点。
    详见 ``docs/倒地判据实测-方法A失败.md``。
    保留此函数只为**日志/可视化参考**与复现当时的评估。

    关于俯仰角归一化（2026-09-17 修正）：

    原先的注释说「相机下俯时远处站立的人的投影高度被压缩约 cos(θ)」——
    **这个物理描述是错的**。实际上透视会把竖直方向**拉长**：
    远处竖直目标的像高随下俯角**增大**，而横向宽度基本不变，
    所以观测到的 h/w 随下俯角单调增大，归一化应当**乘** cos(θ)。

    但要注意：乘 cos(θ) **只是方向对，不是精确还原**。
    数值实测（针孔模型，详见函数体注释）显示 55° 下它只把 16.99 拉回 9.75，
    离无俯角基准 3.78 仍差很远。真实的俯仰依赖不是 1/cos 这种简单形式，
    这个近似忽略了目标自身的仰角 —— 这也是它**只能当粗筛、最终被弃用**的原因之一。

    Returns
    -------
    (是否疑似倒地, 归一化后的高宽比)
    """
    x1, y1, x2, y2 = bbox_xyxy
    w = abs(x2 - x1)
    h = abs(y2 - y1)
    if w < 1e-6:
        return False, 0.0

    ratio = h / w
    cos_p = math.cos(camera_pitch_rad)
    if cos_p > 1e-3:
        # ⚠️ 这里原先是 `ratio /= cos_p`，**方向是反的**，2026-09-17 修正。
        #
        # 相机下俯时，远处竖直目标的**像高会变大**（不是像直觉以为的变小 ——
        # 透视把竖直方向拉长了），而横向宽度基本不变，
        # 所以观测到的 h/w 随下俯角单调**增大**。
        # 要还原到「无俯角」的基准，应当**乘** cos，而不是除。
        # 除以 cos 会把它推得更远，让倒地的人显得更像站着 → 漏检更多。
        #
        # 数值实测（针孔模型，D=5m、人高 1.7m、相机 0.35m、f=500）：
        #
        #     下俯角   h/w原始   ×cos     ÷cos
        #       0°     3.778    3.778    3.778   ← 基准
        #      10°     4.040    3.979    4.102
        #      30°     5.735    4.967    6.623
        #      55°    16.991    9.746   29.623   ← 除以 cos 严重放大
        #
        # ⚠️ 注意 `×cos` **只是方向对，不是精确还原** ——
        # 55° 下它只把 16.99 拉回 9.75，离基准 3.78 仍差很远。
        # 真实的俯仰依赖不是 1/cos 这种简单形式。
        #
        # 好在这不影响本项目：方法 A 已经实测失败并**降级为不参与判定**
        # （见 docs/倒地判据实测-方法A失败.md），这里只是把一个方向性错误修正掉，
        # 不声称修正后它就可用了。
        ratio *= cos_p

    return ratio < fallen_ratio, ratio


# ----------------------------------------------------------------------
# 方法 B：三维躯干倾角（判据本体）
# ----------------------------------------------------------------------
def torso_tilt_from_vertical(
    shoulder_mid_3d: np.ndarray,
    hip_mid_3d: np.ndarray,
    up: np.ndarray | None = None,
) -> float:
    """躯干与**竖直向上**方向的夹角（弧度）。

    这是修正后的判据。与「重力方向」（向下）比是错的 —— 那会让站立的人
    得到 180 度。

    Parameters
    ----------
    shoulder_mid_3d, hip_mid_3d : np.ndarray
        ``map`` 系（**重力对齐**）下的肩中点与髋中点。不是像素坐标。
    up : np.ndarray | None
        ``map`` 系的「上」方向，默认 ``(0, 0, 1)``。

    Returns
    -------
    float
        弧度。站立 ≈ 0，倒地 ≈ pi/2。
    """
    up = np.array([0.0, 0.0, 1.0]) if up is None else np.asarray(up, dtype=np.float64)
    up = up / np.linalg.norm(up)

    torso = np.asarray(shoulder_mid_3d, dtype=np.float64) - np.asarray(hip_mid_3d, dtype=np.float64)
    norm = np.linalg.norm(torso)
    if norm < 1e-6:
        raise ValueError("肩中点与髋中点重合，无法确定躯干方向")

    cos_a = float(np.clip(np.dot(torso / norm, up), -1.0, 1.0))
    return math.acos(cos_a)


def assess_fall_from_keypoints_3d(
    shoulder_left_3d: np.ndarray,
    shoulder_right_3d: np.ndarray,
    hip_left_3d: np.ndarray,
    hip_right_3d: np.ndarray,
    keypoint_confs: tuple[float, float, float, float],
    min_keypoint_conf: float = 0.5,
    max_3d_dispersion_m: float = 1.0,
    up: np.ndarray | None = None,
) -> FallAssessment:
    """用**三维**关键点判定倒地。

    四个关键点（左/右肩、左/右髋）都应当是已反投影到 ``map`` 系的坐标，
    而不是像素坐标。反投影用 ``projector.project_depth_bbox`` 或
    ``project_lidar_cluster``。

    关键点置信度低时（人背对、被遮挡）肩髋点不可靠，此时**丢弃**而不是硬判。

    ``max_3d_dispersion_m`` 是四个关键点三维坐标的离散度上限（米）。
    取值要**大于人体本身的肩髋跨度**，否则每个正常人都过不了这道门 ——
    成人肩宽约 0.4 m、肩到髋约 0.5 m，四点包围盒对角约 0.7 m，
    所以 1.0 是合理下限。若设成 0.6，站姿的人会被判为「反投影不可信」而全部丢弃。

    Returns
    -------
    FallAssessment
    """
    confs = list(keypoint_confs)
    if min(confs) < min_keypoint_conf:
        return FallAssessment(
            is_fallen=False,
            tilt_deg=None,
            method="insufficient",
            confidence=0.0,
            reason=f"关键点置信度过低（min={min(confs):.2f} < {min_keypoint_conf}），丢弃",
        )

    pts = np.stack([shoulder_left_3d, shoulder_right_3d, hip_left_3d, hip_right_3d])
    if not np.isfinite(pts).all():
        return FallAssessment(
            is_fallen=False, tilt_deg=None, method="insufficient",
            confidence=0.0, reason="关键点三维坐标含 NaN",
        )

    # 三维离散度：四个关键点应当落在人体尺度内。过大说明反投影有问题
    dispersion = float(np.linalg.norm(np.percentile(pts, 75, axis=0) - np.percentile(pts, 25, axis=0)))
    if dispersion > max_3d_dispersion_m:
        return FallAssessment(
            is_fallen=False,
            tilt_deg=None,
            method="insufficient",
            confidence=0.0,
            reason=f"三维离散度过大（{dispersion:.2f} m），反投影不可信",
        )

    s_mid = (np.asarray(shoulder_left_3d) + np.asarray(shoulder_right_3d)) / 2.0
    h_mid = (np.asarray(hip_left_3d) + np.asarray(hip_right_3d)) / 2.0

    tilt = torso_tilt_from_vertical(s_mid, h_mid, up=up)
    tilt_deg = math.degrees(tilt)

    is_fallen = FALLEN_MIN_DEG <= tilt_deg <= FALLEN_MAX_DEG

    # 可信度：关键点置信度均值 * 离散度惩罚
    conf = float(np.mean(confs)) * max(0.0, 1.0 - dispersion / max_3d_dispersion_m)

    if is_fallen:
        reason = f"躯干倾角 {tilt_deg:.1f}°，落在 [{FALLEN_MIN_DEG}, {FALLEN_MAX_DEG}] 区间"
    elif tilt_deg < FALLEN_MIN_DEG:
        reason = f"躯干倾角 {tilt_deg:.1f}°，接近竖直（站立）"
    else:
        reason = (
            f"躯干倾角 {tilt_deg:.1f}° 超过 {FALLEN_MAX_DEG}°，"
            "可能是肩髋关键点被弄反，不作为倒地"
        )

    return FallAssessment(
        is_fallen=is_fallen,
        tilt_deg=tilt_deg,
        method="pose_3d",
        confidence=conf,
        reason=reason,
    )


def midpoints_from_keypoints_2d(
    keypoints: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, tuple[float, float, float, float]]:
    """从 COCO 17 关键点里取出肩中点、髋中点与四个置信度。

    Parameters
    ----------
    keypoints : np.ndarray
        形状 ``(17, 3)``，每行 ``(x, y, confidence)``
        （ultralytics 的 ``keypoints.data``）。

    Returns
    -------
    (肩中点像素坐标, 髋中点像素坐标, (左右肩左右髋的置信度))
    """
    kp = np.asarray(keypoints, dtype=np.float64)
    if kp.shape != (17, 3):
        raise ValueError(f"keypoints 形状应为 (17, 3)，得到 {kp.shape}")

    sl, sr = kp[LEFT_SHOULDER], kp[RIGHT_SHOULDER]
    hl, hr = kp[LEFT_HIP], kp[RIGHT_HIP]

    shoulder_mid = (sl[:2] + sr[:2]) / 2.0
    hip_mid = (hl[:2] + hr[:2]) / 2.0
    confs = (float(sl[2]), float(sr[2]), float(hl[2]), float(hr[2]))

    return shoulder_mid, hip_mid, confs


def estimate_ground_range_for_pitch(pitch_rad: float, camera_height_m: float) -> float:
    """见 ``projector.magnitude_of_ground_range`` —— 此处在 anomaly 语境下重导出，
    方便评估「近场判据是否覆盖得到倒地目标」。"""
    if pitch_rad <= 1e-9:
        return float("inf")
    return camera_height_m / math.tan(pitch_rad)
