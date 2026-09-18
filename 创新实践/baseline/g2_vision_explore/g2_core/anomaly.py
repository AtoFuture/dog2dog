"""异常识别：人员倒地。

================================================================================
🔴 判决（2026-09-18 实测）：**方法 B 作为「瞬时倾角判据」不成立，不要拿它做倒地判定。**

在 57 帧真实数据（含 15 条已倒地轨迹）上实测，结论是**正负类严重重叠**：

    已倒地  倾角 [75, 83, 86, 87, 88, 90, 94, 97]°
    正常活动 倾角 [31, 37, 38, 51, 55, 60, 62, 88]°   ← 注意这个 88

最致命的一条是 185819 帧：人**四肢着地跪趴着**，测出 84°。
内参、深度、关键点**全都没错**（三维肩宽 0.53 m，在人体尺度内）——
躯干确实是水平的。但它不是倒地。

**「躯干不水平」和「人倒在地上了」不是一回事。**
跪、蹲、弯腰捡东西、爬行，几何上与倒地几乎无法区分。
这不是测量精度问题，**换更好的深度相机、更好的模型、更多的数据都解决不了** ——
它是判据本身缺少**时间上下文**。

排查过程中已排除的混淆因素（详见 docs/倒地判据实测-方法B.md）：
  * **内参**：数据集未提供标定。把焦距从 350 扫到 700 逐个找最优工作点，
    **每一个**焦距下负类最大值都超过正类最小值，重叠始终存在。
  * **深度精度**：深度图本身是好的（视差量化 ~600 层，与理论值吻合）。
  * **关键点**：pose 模型在躺姿上可用（检出 12/15，关键点置信度 0.94~0.98）。

**下一步该做什么**：把判据从「瞬时倾角」改成「**时间上的状态转移**」——
跟踪一个人的三维躯干朝向序列，倒地 = 从竖直转为水平**并保持住**。
跪/蹲/弯腰是短暂的或可逆的，倒地不是。这正是 state_machine 里
``anomalies`` 序列该承担的事。
================================================================================

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


# 成年人体的三维尺度（米）。用于**反投影质检**，见 check_body_proportions()。
SHOULDER_WIDTH_M = (0.25, 0.65)
HIP_WIDTH_M = (0.10, 0.55)
TORSO_LENGTH_M = (0.25, 0.95)


def check_body_proportions(
    shoulder_left_3d: np.ndarray,
    shoulder_right_3d: np.ndarray,
    hip_left_3d: np.ndarray,
    hip_right_3d: np.ndarray,
    shoulder_width_m: tuple[float, float] = SHOULDER_WIDTH_M,
    hip_width_m: tuple[float, float] = HIP_WIDTH_M,
    torso_length_m: tuple[float, float] = TORSO_LENGTH_M,
) -> str | None:
    """检查四个关键点在三维里是否构成一个**人体尺度**的躯干。

    返回 ``None`` 表示合格，否则返回不合格的原因字符串。

    ---------------------------------------------------------------------------
    为什么需要这道门（2026-09-18，实测驱动）

    原先只有 ``max_3d_dispersion_m`` 一道门，**实测证明它挡不住真正的错误**。
    真实数据里出现过这样的反投影（frame 185916，坐姿）：

        左肩深度 1.64 m，右肩深度 3.03 m   —— 同一个人的两个肩差 1.39 m

    成因：pose 模型把**被遮挡的远侧关键点放到了身体轮廓之外**，
    逐像素取深度于是取到了背景。四个点的离散度只有 0.5 m，
    小于默认门限 1.0 m，**被放行了**，最后算出一个 88° 的倾角
    （真人坐着，躯干接近竖直）。

    而肩宽是个**自带真值的量**：成年人肩宽就是 0.40 m 上下，
    跟姿态、视角、远近都无关。所以「三维肩宽必须是 0.40 m」
    是一道**用解剖学常数做的、不需要标注的**质检。

    实测效果（57 帧真实数据；真值 0.40 m，离散度越小越好）::

        取深度方式          肩宽 中位 / 四分位距
        5x5 中位（原状）      0.45 / 0.86     ← 一半样本差近 1 米
        窗口近端 p5          0.24 / 0.13     ← 离散度降 7 倍，但偏小 40%

    ⚠️ 注意近端取深度会把距离**系统性压小**（0.24 < 0.40），
    于是本门的**下限**会误杀一部分真样本。这个偏置与门限是**互相拉扯**的，
    本次没有找到两头都好的参数组合 —— 详见 docs/倒地判据实测-方法B.md §4。

    ---------------------------------------------------------------------------
    ⚠️ 这道门是**绝对尺度**的检查，因此**依赖准确的内参**。
    焦距估错会让所有三维距离整体缩放，门就可能误杀或放过。
    内参应当来自 ``CameraInfo``，不要用近似值。
    本门也**不能**替代时间上下文 —— 它只保证「量出来的是个人」，
    不保证「躯干水平 = 倒地」，见 docs/倒地判据实测-方法B.md。
    """
    sl, sr = np.asarray(shoulder_left_3d), np.asarray(shoulder_right_3d)
    hl, hr = np.asarray(hip_left_3d), np.asarray(hip_right_3d)

    sw = float(np.linalg.norm(sl - sr))
    hw = float(np.linalg.norm(hl - hr))
    tl = float(np.linalg.norm((sl + sr) / 2.0 - (hl + hr) / 2.0))

    for name, val, (lo, hi) in (
        ("肩宽", sw, shoulder_width_m),
        ("髋宽", hw, hip_width_m),
        ("躯干长", tl, torso_length_m),
    ):
        if not (lo <= val <= hi):
            return (
                f"{name} {val:.2f} m 超出人体尺度 [{lo}, {hi}] —— "
                "关键点多半落到了身体轮廓之外（被遮挡的远侧关节），"
                "深度取到了背景"
            )
    return None


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

    # ① 人体尺度门：肩宽/髋宽/躯干长必须是成年人的尺度。
    #    这是**主质检** —— 它直接用量出来的绝对尺寸对解剖学常数，
    #    实测能抓住「关键点落到轮廓外、深度取到背景」这类错误，而 ② 抓不住。
    bad = check_body_proportions(shoulder_left_3d, shoulder_right_3d,
                                 hip_left_3d, hip_right_3d)
    if bad is not None:
        return FallAssessment(
            is_fallen=False, tilt_deg=None, method="insufficient",
            confidence=0.0, reason=f"反投影超出人体尺度：{bad}",
        )

    # ② 离散度门：兜住 ① 漏掉的形态（例如四点各自都对但整体散开）
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
