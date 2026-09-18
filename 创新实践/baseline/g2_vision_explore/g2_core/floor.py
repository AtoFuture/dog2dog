"""地面平面拟合，**以及「拟合出来的到底是不是地面」的校验**。

为什么校验必须在这里、而不是留给调用方（2026-09-18，真实 RGB-D 素材实测）：

RANSAC 的工作方式是「内点最多的平面」，它**没有**任何机制保证那个平面是地面。
在 ``1378_rgb_depth16.mkv`` 上每 5 帧取一帧扫 276 帧，**约 8%（23 帧）**
RANSAC 锁到了天花板/墙 —— 相机到平面的距离量出来是 **-3.3 m**
（平面跑到相机上方三米多）。而原先的代码没有任何检查会把它们挡下。

> ⚠️ 归因更正：最初记的是「帧 1032 因床占据视野而锁错平面，量出 -0.21 m」。
> **那条不成立** —— 帧 1032 的平面复算下来是全好的（全图中位高度 +0.55，
> 与其余各帧一致）；那个 -0.21 m 来自关键点/取深度那一环，不是平面。
> 真失败发生在其它的帧上。详见 ``docs/真实RGB-D素材实测-1378.md`` §4。

🔴 **但这 8% 是「俯视机位」下的数字，别当普适。**

1378 素材里 Kinect 架在 **2.44 m 俯视**，地面占了大半画面，
所以「地面是主平面」这个前提**碰巧成立**，上面那个 8% 只是它的偶发失效。

**真机是前视低机位**（离地 35 cm、俯仰 0°，见 ``docs/设备清单.md``），
画面里几乎全是墙和货架、地面只占最下面一条 —— 前提本身不成立。
仿真里复现了：算出的相机离地 **-6.79 m**（平面跑到相机上方），
只是**碰巧**被 :attr:`FloorFitStatus.NOT_AT_BOTTOM` 挡下（不让下方比 48.3%）。

⚠️ 也就是说上面的校验是**兜底，不是解法**：它挡得住错平面，
但挡下之后**这一帧就没有结果了**。换机位后可能从 8% 变成常态。
详见 ``docs/真实RGB-D素材实测-1378.md`` §4.7。

所以本模块只暴露一个入口 :func:`fit_floor_plane`，它**只在平面通过了全部校验时**
才返回 ``OK``；不合格时返回带原因的 :class:`FloorFit`，且 ``normal`` 为 ``None``，
调用方拿不到一个「能用的」错平面。

> 📌 校验阈值是**被实测改下来的**，不是拍的 —— 见 :func:`fit_floor_plane`
> 的 docstring。初版凭直觉写的 0.95 在多拒 28% 的帧。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

__all__ = ["FloorFit", "FloorFitStatus", "fit_floor_plane"]


class FloorFitStatus(str, Enum):
    """拟合结果的状态。除了 ``OK`` 都表示**这一帧不能用来量离地高度**。"""

    OK = "ok"
    TOO_FEW_POINTS = "too_few_points"
    """有效深度点太少，谈不上拟合。"""
    NO_PLANE = "no_plane"
    """RANSAC 没能解出任何平面（三点共线等退化情形）。"""
    NOT_AT_BOTTOM = "not_at_bottom"
    """平面不在场景底部 —— 有太多点落在它下方，说明锁到了别的平面。"""
    CAMERA_HEIGHT_IMPLAUSIBLE = "camera_height_implausible"
    """相机到平面的距离不像是「相机架在地面上方」该有的值。"""


_REASON_ZH = {
    FloorFitStatus.OK: "通过校验",
    FloorFitStatus.TOO_FEW_POINTS: "有效深度点不足",
    FloorFitStatus.NO_PLANE: "RANSAC 未解出平面",
    FloorFitStatus.NOT_AT_BOTTOM: "平面不在场景底部（下方有点）",
    FloorFitStatus.CAMERA_HEIGHT_IMPLAUSIBLE: "相机离地高度不合理",
}


@dataclass(frozen=True, eq=False)
class FloorFit:
    """一次地面拟合的结果，**无论成功与否都带上诊断量**。

    ``eq=False``：``normal`` 是 ndarray，默认生成的 ``__eq__`` 会在比较时
    抛「truth value of an array is ambiguous」。

    诊断量单独留着，是为了失败时能打印「差多少」而不是只说一句失败 ——
    阈值该不该调，靠的就是这些数。
    """

    status: FloorFitStatus

    # 判定时实际用的阈值。**故意不设默认值。**
    #
    # ⚠️ 它们曾经是带默认值的字段，而那是**影子配置**：``fit_floor_plane``
    # 每次都把算好的值显式传进来，字段上那份永远不生效 ——
    # 变异测试实测：把 ``min_camera_height_m`` 的字段默认改掉，全部测试照过，
    # 因为函数签名上还有另一份默认值。
    # 谁看到字段上的默认值去改它，都会以为改了行为而其实没有。
    # 设成必填，这种误解就不可能发生。
    min_above_ratio: float
    min_camera_height_m: float
    max_camera_height_m: float

    normal: np.ndarray | None = None
    """相机光学系下的「上」方向单位向量（``y`` 分量为负，因为光学系 ``+y`` 朝下）。

    只在 ``status is OK`` 时非 ``None``。
    """
    offset: float = float("nan")
    """平面方程 ``normal · p + offset = 0`` 里的 ``offset``。"""
    camera_height_m: float = float("nan")
    """**有符号**的相机离地高度。

    数值上等于 ``offset``（相机在原点，代入平面方程即得），但**故意不取绝对值**：
    平面跑到相机上方时它是负的，而 ``abs()`` 会把这个信号抹掉、
    把它变成一个看起来很正常的正数。
    """
    inlier_ratio: float = 0.0
    """内点占比。真实地面上通常只有 10~20%（地板只占画面一部分）。"""
    above_ratio: float = 0.0
    """**不在平面下方**的点占比 —— 即 ``normal · p + offset > -thr`` 的比例。

    用「不低于 -thr」而不是「> 0」：内点本身散布在平面两侧（``|v| < thr``），
    其中约一半严格小于 0。实测同一批帧两种口径差了 **3~7 个百分点**
    （如帧 114：98.2% vs 92.3%），按 ``> 0`` 统计会平白多出一截假的「在下方」。
    """

    @property
    def ok(self) -> bool:
        return self.status is FloorFitStatus.OK

    @property
    def reason(self) -> str:
        """中文原因，可直接进日志。"""
        base = _REASON_ZH[self.status]
        if self.status is FloorFitStatus.NOT_AT_BOTTOM:
            return f"{base}：仅 {self.above_ratio:.1%} 的点不在下方（需 ≥{self.min_above_ratio:.0%}）"
        if self.status is FloorFitStatus.CAMERA_HEIGHT_IMPLAUSIBLE:
            return (f"{base}：{self.camera_height_m:+.2f} m"
                    f"（需 {self.min_camera_height_m:.1f}~{self.max_camera_height_m:.1f} m）")
        return base


def fit_floor_plane(
    points: np.ndarray,
    *,
    thr: float = 0.02,
    iters: int = 400,
    step: int = 1,
    seed: int = 0,
    min_points: int = 100,
    min_above_ratio: float = 0.50,
    min_camera_height_m: float = 0.2,
    max_camera_height_m: float = 3.0,
) -> FloorFit:
    """从**已反投影**的三维点里拟合地面，并校验它确实是地面。

    收 ``points`` ``(N, 3)`` 而不是 ``depth_mm`` + 内参，是因为内参取决于相机与
    分辨率（本仓库同时存在 640x480 的 RealSense 与 320x240 的 Kinect v1），
    放在这里会变成一组要跟着素材改的全局量。反投影是调用方的事。

    ``points`` 里的无效点（NaN / 非有限值）会被丢掉。

    校验两条，都是「不报错但结果全错」的兜底：

    * **平面在场景底部**：``min_above_ratio`` 比例的点不在它下方。
    * **相机高度合理**：``normal · p + offset`` 定号后，相机到平面的**有符号**
      距离落在 ``[min_camera_height_m, max_camera_height_m]``。

    ---------------------------------------------------------------------------
    ⚠️ ``min_above_ratio`` 的默认值 **0.50 是被实测改下来的**，不是拍的

    初版按「≥95% 的点在平面上方」写死 0.95。在 ``1378_rgb_depth16.mkv`` 上
    每 5 帧取一帧扫了 **276 帧**，分布是：

        相机高度在 [1,3] m 内  253 帧 → 不让下方比 0.543 ~ 0.996
        相机高度在 [1,3] m 外   23 帧 → 不让下方比 0.106 ~ 0.193

    也就是说 **[0.193, 0.543] 是个空档**，而 0.95 落在「正确那一堆的中间」——
    拒帧率从 8.3%（只用相机高度）涨到 **36.2%**，多拒的 28% 里包含
    实测**物理量正确**的帧：帧 229「坐床」被拒但髋高 +0.65 m（= 床面高度），
    帧 1263「站立」被拒但髋高 +0.91/+0.91 m（站立正确值）。

    根因是这个比例**测的是拟合精度而不是「是不是地面」**：远处地板上
    零点几度的角度误差就能累积出超过 ``thr`` 的偏差，把点算到平面下方去。
    所以它只适合当**灾难性错误**的兜底（平面跑到场景中间那种），
    阈值取宽；精度问题交给相机高度那条。

    0.50 在实测空档内，且语义上仍讲得通：**至少一半的点不在平面下方**。
    在本序列上与「只用相机高度」拒掉的帧**完全相同**（8.3%），零额外误杀。

    默认下限 **0.2 m**。2026-09-18 实测确认了部署平台的真实高度：
    **Go2 的相机装在离地约 35 cm 处** —— 官方 URDF 的 `front_camera_joint`
    相对 `base` 是 `z = +0.043 m`，加上机器人自报的 `body_height = 0.309 m`。

    原先的 **1.0 m** 是按手持/固定安装的 Kinect 素材（约 2.4 m）定的。
    按那个值上真机，**每一帧都会被判不合格**，而且**不报错** ——
    只表现为「永远没有离地高度」，正是本项目反复踩的那类失效。

    0.2 m 有三重依据：

    1. 低于 Go2 的 0.35 m，并给蹲伏姿态/低装留出余量；
    2. 本仓库另外两份素材仍在区间内（`corner1_low` ≈ 1.2 m、`1378` ≈ 2.4 m）；
    3. 实测到的**灾难性错拟合给的是负值**（相机高 −3.3 m，平面跑到相机上方），
       下限只需要挡住那一类 —— 它是**灾难性错误的兜底**，不该卡得紧。

    > 📌 与 `min_above_ratio` 同一条原则：这条校验测的是「是不是离谱」，
    > 不是「准不准」。精度问题不归它管。
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]
    if step > 1:
        pts = pts[::step]

    if len(pts) < min_points:
        return FloorFit(FloorFitStatus.TOO_FEW_POINTS,
                        min_above_ratio=min_above_ratio,
                        min_camera_height_m=min_camera_height_m,
                        max_camera_height_m=max_camera_height_m)

    n = len(pts)
    rng = np.random.default_rng(seed)
    best_count, best_normal, best_offset = 0, None, None
    for _ in range(iters):
        i = rng.choice(n, 3, replace=False)
        p0, p1, p2 = pts[i]
        nv = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(nv)
        if norm < 1e-9:                       # 三点共线，退化
            continue
        nv = nv / norm
        d = -float(nv @ p0)
        count = int((np.abs(pts @ nv + d) < thr).sum())
        if count > best_count:
            best_count, best_normal, best_offset = count, nv, d

    if best_normal is None:
        return FloorFit(FloorFitStatus.NO_PLANE,
                        min_above_ratio=min_above_ratio,
                        min_camera_height_m=min_camera_height_m,
                        max_camera_height_m=max_camera_height_m)

    # 定号：光学系 +y 朝下，所以朝上的法向其 y 分量必为负。
    # offset 必须跟着一起翻，否则平面方程和法向对不上。
    if best_normal[1] > 0:
        best_normal, best_offset = -best_normal, -best_offset

    values = pts @ best_normal + best_offset        # 相机处取值 = best_offset
    above_ratio = float((values > -thr).mean())
    cam_h = float(best_offset)

    common = dict(
        offset=float(best_offset),
        camera_height_m=cam_h,
        inlier_ratio=best_count / n,
        above_ratio=above_ratio,
        min_above_ratio=min_above_ratio,
        min_camera_height_m=min_camera_height_m,
        max_camera_height_m=max_camera_height_m,
    )

    if above_ratio < min_above_ratio:
        return FloorFit(FloorFitStatus.NOT_AT_BOTTOM, **common)
    if not (min_camera_height_m <= cam_h <= max_camera_height_m):
        return FloorFit(FloorFitStatus.CAMERA_HEIGHT_IMPLAUSIBLE, **common)

    return FloorFit(FloorFitStatus.OK, normal=best_normal, **common)
