"""异常识别：人员倒地。

================================================================================
🔴 判决（2026-09-18 实测）：**方法 B 作为「瞬时倾角判据」不成立，不要拿它做倒地判定。**

在 57 帧真实数据（含 15 条已倒地轨迹）上实测，结论是**正负类严重重叠**：

    已倒地  倾角 [75, 86, 87, 88, 90, 94]°
    正常活动 倾角 [31, 37, 42, 51, 55, 60, 63, 88]°   ← 注意这个 88

（这组数字以 ``tools/validate_method_b.py`` 的**工具输出**为准；
2026-09-18 审核发现本文件里曾写着一组对不上的旧数字。）

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

from .projector import sample_depth_near

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
    fallen_ratio: float = 0.6,
) -> tuple[bool, float]:
    """按检测框长宽比粗判倒地。

    🔴 **本方法已实测失败，不要用它做判定。**
    低机位下它的最优工作点是「召回 43% / 误报 0%」——丢掉一半以上的真实倒地；
    扫描全部阈值与俯仰角都**找不到**同时满足「检出率 >85%、误检率 <10%」的工作点。
    详见 ``docs/倒地判据实测-方法A失败.md``。
    保留此函数只为**日志/可视化参考**与复现当时的评估。

    ⚠️ 2026-09-18 审核发现签名里曾有一个 ``stand_ratio`` 参数**从未被使用**
    （函数体只做 ``ratio < fallen_ratio`` 的二值判断）—— 一个静默失效的旋钮：
    调用方传了它、以为设了「站立/倒地」的分界，实际毫无作用且不报错。
    已删除。

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


# ----------------------------------------------------------------------
# 反投影质检的两道门
# ----------------------------------------------------------------------
MAX_PAIR_DEPTH_DIFF_M = 0.40
"""同侧两个关键点的**深度差**上限（米）。**这是主质检。**

物理依据：一个人的左肩与右肩（或左髋与右髋）在深度方向上的距离，
不管什么姿态都不可能超过 0.4 m 量级 —— 肩宽本身才 0.4 m，
躺下时最坏也是这个量级。所以「深度差 > 0.4 m」几乎只有一个解释：
**其中一个关键点落到了身体轮廓之外，取到了背景。**

实测（57 帧真实数据，56 条样本）::

    质检方式                        留存率   拦住那个真实的失败样本
    绝对尺度门 [0.25, 0.65]（原状）   36%     拦下
    本门 |Δz| <= 0.40                **84%**  拦下
    本门 + 绝对粗错兜底               80%     拦下（两道都拦）

那个失败样本是 frame 185916：pose 模型把被遮挡的右肩点放到了身体轮廓外，
**两肩深度 1.64 m / 3.03 m，差 1.39 m** —— 远超本门。

⚠️ 本门**不含焦距**，因此对**内参误差免疫**。绝对尺度门做不到这一点 ——
理由要说准确（2026-09-18 审核指出原表述有误）：

    反投影是 ``X=(u-cx)·z/fx, Y=(v-cy)·z/fy, Z=z`` —— **z 来自深度图，与焦距无关**。
    所以焦距估错时三维距离**不是均匀缩放**：深度分量原样不动，只有横向两项按 1/f 变。
    对肩宽这种「两个关键点深度相近」的量，横向项占主导，于是它**近似**按 1/f 缩放
    —— 足以让以米为单位的绝对界限整体失准，但不是严格的等比。
"""

SHOULDER_WIDTH_M = (0.08, 1.20)
HIP_WIDTH_M = (0.05, 1.00)
TORSO_LENGTH_M = (0.15, 1.40)
"""**粗错兜底**用的绝对尺度界限（米）。故意放得很宽。

⚠️ 这几个界限**不是**用来保证测量精度的 —— 实测证明这个测量达不到那个精度：

    界限                       留存率
    [0.25, 0.65]（成人肩宽±48%）  36%
    [0.15, 0.90]                 73%
    [0.08, 1.20]（本值）          ~80%

而且**不管换哪个深度估计器都一样**（p5/p15/p25/p50 的留存率都只有 27~38%）：
肩宽的实测中位数随分位数单调变化（p5 → 0.24，p50 → 0.38，真值 0.40），
所以收窄界限等于「从一个有偏分布里截取一段」，而不是在筛掉坏样本。

它们只用来兜住「量纲整体错了」这一类粗错（例如深度编码搞混造成的 1000 倍偏差），
以及作为深度差门的冗余保险。**真正的质检靠上面的深度差门。**
"""


def check_reprojection(
    shoulder_left_3d: np.ndarray,
    shoulder_right_3d: np.ndarray,
    hip_left_3d: np.ndarray,
    hip_right_3d: np.ndarray,
    max_pair_depth_diff_m: float = MAX_PAIR_DEPTH_DIFF_M,
    shoulder_width_m: tuple[float, float] = SHOULDER_WIDTH_M,
    hip_width_m: tuple[float, float] = HIP_WIDTH_M,
    torso_length_m: tuple[float, float] = TORSO_LENGTH_M,
) -> str | None:
    """检查四个关键点的反投影是否可信。返回 ``None`` 表示合格，否则返回原因。

    **两道门，主次分明**（2026-09-18 实测后重构，原实现只有第 ② 道）：

    ① **同侧深度差**（主）—— 见 ``MAX_PAIR_DEPTH_DIFF_M``。
       直接测那个真正的失效模式：「其中一个关键点落到了身体轮廓之外」。
       不含焦距，对内参误差免疫。

    ② **绝对尺度兜底**（次）—— 见 ``SHOULDER_WIDTH_M`` 等。
       只拦「量纲整体错了」这一类粗错，界限放得很宽。

    ---------------------------------------------------------------------------
    为什么改成这样（实测驱动，详见 docs/倒地判据实测-方法B.md §4）

    原先只有第 ② 道，界限是 [0.25, 0.65]（成人肩宽 ±48%）。方向是对的 ——
    肩宽、髋宽、躯干长是**自带真值的量**，不需要标注就能当质检基准。
    但实测发现它**留存率只有 36%**，而这不是「拦掉了坏的」，是
    「从一个有偏分布里截取了一段」：

        肩宽实测中位数（同一份数据，只换深度分位数）
            p5 → 0.24   p25 → 0.30   p50 → 0.38   真值 0.40

    也就是说估计器的偏置是分位数的单调函数，而**没有任何一个分位数
    能在窄界限下拿到好留存率**（27~38%）。继续调参数是死路。

    换成第 ① 道之后留存率 36% → 84%，而那个真实的失败样本照旧拦得住。
    """
    sl, sr = np.asarray(shoulder_left_3d), np.asarray(shoulder_right_3d)
    hl, hr = np.asarray(hip_left_3d), np.asarray(hip_right_3d)

    # ① 同侧深度差 —— 主质检
    for name, a, b in (("两肩", sl, sr), ("两髋", hl, hr)):
        dz = abs(float(a[2]) - float(b[2]))
        if not np.isfinite(dz) or dz > max_pair_depth_diff_m:
            return (
                f"{name}的深度差 {dz:.2f} m 超过 {max_pair_depth_diff_m} m —— "
                "其中一个关键点多半落到了身体轮廓之外（被遮挡的远侧关节），"
                "深度取到了背景"
            )

    # ② 绝对尺度兜底 —— 只拦量纲级的粗错
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
                f"{name} {val:.2f} m 超出粗错兜底区间 [{lo}, {hi}] —— "
                "这不是测量精度问题，是尺度整体错了（检查内参 / 深度编码）"
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
    bad = check_reprojection(shoulder_left_3d, shoulder_right_3d,
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


@dataclass
class Torso3D:
    """一个人四个躯干关键点的**相机系**三维坐标。

    只存四个点而不是直接给倾角 —— 因为转成 map 系（拿重力方向和离地高度）
    是调用方的事，而这个类不该知道 TF 的存在。
    """

    shoulder_left: np.ndarray
    shoulder_right: np.ndarray
    hip_left: np.ndarray
    hip_right: np.ndarray
    keypoint_confs: tuple[float, float, float, float]

    @property
    def shoulder_mid(self) -> np.ndarray:
        return (self.shoulder_left + self.shoulder_right) / 2.0

    @property
    def hip_mid(self) -> np.ndarray:
        return (self.hip_left + self.hip_right) / 2.0

    def tilt_deg(self, up: np.ndarray) -> float:
        """躯干与**竖直向上**方向的夹角（度）—— 站立 ≈ 0，倒地 ≈ 90。

        ⚠️ 不是「与重力方向的夹角」。后者会让站立的人得到 180°，
        正是本模块开头留档的 v1 那个反向判据的错误。

        ⚠️ ``up`` 是**必填**，故意不给默认值。

        本类存的点一律在**相机光学系**（x 右 / y 下 / z 前）里，
        而在那个坐标系里「上」是 ``(0, -1, 0)``，**不是** ``(0, 0, 1)``
        （``(0,0,1)`` 是「前方」）。

        给个 ``(0,0,1)`` 的默认值会让「站着的人」算出 90° —— 看起来像个
        正经的角度，实际是坐标系搞错了。这类错误在本项目里已经出现过几次
        （TF 四元数、俯仰归一化方向），所以这里强制调用方写明是哪个系。
        """
        return math.degrees(torso_tilt_from_vertical(self.shoulder_mid, self.hip_mid, up=up))


#: 四个躯干关键点在 COCO 17 里的下标。顺序与 Torso3D 的字段一致。
TORSO_KEYPOINT_IDS = (5, 6, 11, 12)


def keypoints_to_torso_3d(
    keypoints: np.ndarray,
    depth_m: np.ndarray,
    intrinsics,
    min_keypoint_conf: float = 0.5,
) -> tuple[Torso3D | None, str]:
    """二维关键点 + 深度图 -> ``Torso3D``。返回 ``(结果, 失败原因)``。

    失败时结果是 ``None``，原因是一句能直接进日志的话（**为什么要带原因**：
    「没检出人」「关键点置信度低」「深度取不到」「反投影超出人体尺度」
    在日志里长得一模一样，都是「没有倒地结论」，不区分就没法定位）。

    三道检查，缺一不可：

    1. **关键点置信度** —— 人背对相机、被遮挡时肩髋点不可靠。
    2. **深度可取样** —— 用 ``sample_depth_near``（窗口近端分位数），
       不是逐像素：关键点可能落在身体轮廓之外，逐像素会取到背景。
    3. **反投影质检** —— 见 ``check_reprojection``。主质检是**同侧深度差**
       （直接测「关键点落到轮廓外」这个失效模式），绝对尺度只做粗错兜底。

    Parameters
    ----------
    keypoints : np.ndarray
        ``(17, 3)``，每行 ``(x, y, confidence)``（ultralytics 的 ``keypoints.data``）。
    depth_m : np.ndarray
        **米制**深度图，无效值为 NaN（见 ``projector.depth_to_meters``）。
    intrinsics : projector.CameraIntrinsics
        真实标定值，不要用近似 —— 人体尺度门是绝对尺度的检查，焦距错了它就失准。
    """
    kp = np.asarray(keypoints, dtype=np.float64)
    if kp.shape != (17, 3):
        return None, f"关键点形状应为 (17, 3)，得到 {kp.shape}"

    confs = tuple(float(kp[i][2]) for i in TORSO_KEYPOINT_IDS)
    if min(confs) < min_keypoint_conf:
        return None, f"关键点置信度过低（min={min(confs):.2f}）"

    pts = []
    for i in TORSO_KEYPOINT_IDS:
        z = sample_depth_near(depth_m, kp[i][0], kp[i][1])
        if not np.isfinite(z) or z <= 0:
            return None, f"关键点 {i} 处取不到有效深度"
        # 反投影到相机光学系（x 右 / y 下 / z 前）
        pts.append(np.array([
            (kp[i][0] - intrinsics.cx) * z / intrinsics.fx,
            (kp[i][1] - intrinsics.cy) * z / intrinsics.fy,
            z,
        ]))

    torso = Torso3D(pts[0], pts[1], pts[2], pts[3], confs)

    bad = check_reprojection(torso.shoulder_left, torso.shoulder_right,
                                 torso.hip_left, torso.hip_right)
    if bad is not None:
        return None, f"反投影超出人体尺度：{bad}"

    return torso, ""


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
