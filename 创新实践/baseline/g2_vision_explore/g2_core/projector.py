"""三维定位：把二维检测框解算成相机系 / map 系的三维坐标。

三条路线（见 docs/框架规划.md §4.2）：

* ``depth``         —— 深度相机。首选，精度最好。
* ``ground_plane``  —— 单目 + 地面假设。**有硬性距离上限**（约 h/tanθ，通常只有几米），
                       只在深度不可用时作降级。
* ``lidar_cluster`` —— 点云簇关联。有激光雷达时的标准做法。

本模块只做**相机系**的几何解算，不碰 TF —— 坐标系变换由调用方用 tf2 完成。
这样几何部分可以脱离 ROS2 独立单元测试。

关于「深度与像素必须同源」（v2 修正的核心问题）::

    错误做法：在检测框内取「25 分位深度」d，再用框中心的 (u, v) 反投影。
    问题：d 和 (u, v) 来自**不同的像素**。fx≈600、d=5 m、两者相距 40 px 时，
    仅横向就有 40 * 5 / 600 ≈ 0.33 m 的系统偏差。

    本模块的做法：把区域内**所有有效像素整体反投影**，对 X/Y/Z **分别取中位数**。
    这样的估计点未必对应某个真实像素，但它是稳健的、且不受「深度取自哪个像素」影响。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


class DepthEncodingError(ValueError):
    """深度图编码不受支持。"""


@dataclass
class CameraIntrinsics:
    """针孔相机内参。

    注意：**输入图像必须已去畸变**。Go2 头部相机是 120 度广角，
    理想针孔模型在画面边缘误差不可忽略。
    """

    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_camera_info(cls, k: list[float] | np.ndarray) -> "CameraIntrinsics":
        """从 ``sensor_msgs/CameraInfo.K``（长度 9 的行主序数组）构造。"""
        k = np.asarray(k).ravel()
        if k.size != 9:
            raise ValueError(f"CameraInfo.K 应为 9 个元素，得到 {k.size}")
        return cls(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]))


@dataclass
class ProjectionResult:
    """一次三维解算的结果。"""

    point: np.ndarray
    """相机系下的 ``(x, y, z)``，单位米。光学系：x 右 / y 下 / z 前。"""

    valid_pixels: int
    """参与解算的有效深度像素数。"""

    dispersion: float
    """三维点的离散度（各轴 IQR 的模），米。用于判断这次解算可不可信。"""

    spread_per_axis: np.ndarray
    """逐轴 IQR，形状 ``(3,)``，米。是 ``dispersion`` 的构成分量。

    单列出来是为了换算协方差 —— 从「模」反推单轴需要假设各向同性，
    而深度方向的离散通常**远大于**横向（同一个物体表面在深度上铺得开），
    那个假设明显不成立。
    """

    @property
    def apparent_extent(self) -> np.ndarray:
        """该目标像素簇的**表观尺寸**（各轴 IQR，米）。**不是测量不确定度。**

        🔴 **不要把它当协方差填进消息**（2026-09-17 修正）。

        这里原先有个叫 ``covariance`` 的属性，把 ``spread_per_axis / 1.349``
        当成位置协方差。那是错的：``spread_per_axis`` 衡量的是
        **物体自身的空间展布**，不是测量误差。

        举个能看出问题的例子：一辆 4 m 长的车，深度方向 IQR 可能有 1.5 m，
        换算出的 sigma_z ≈ 1.1 m —— 而真实的测量误差可能是 5 cm 量级。
        填进 ``Detection3D.pose.covariance`` 会让 G3 的数据关联与加权完全失真。

        更微妙的是：原先的 docstring 把它描述成「不含外参/TF/同步误差，
        所以是个**下界**」—— 对**测量误差**而言确实是下界，
        但对**这个数值想表达的东西**而言恰好相反：它是**上界**，
        因为里面混进了大量与测量无关的物体尺寸。

        这个量本身有用（大而分散的像素簇确实说明"这一团不太像单一物体表面"），
        所以保留，但改成诚实的名字，并且**不再**拿它填协方差。

        G2 目前**无法**给出有意义的测量协方差 —— 见
        ``docs/框架规划.md`` 接口 03 的说明：``pose.covariance`` 一律置零
        （按 ROS 惯例，全零表示「协方差未知」），而不是填一个唬人的数。
        """
        return np.asarray(self.spread_per_axis, dtype=np.float64)


# ----------------------------------------------------------------------
# 深度图编码
# ----------------------------------------------------------------------
def depth_to_meters(depth: np.ndarray, encoding: str) -> np.ndarray:
    """把任意编码的深度图统一成「米」为单位的 float32 数组，无效值为 NaN。

    这是**必须做**的一步（v1 遗漏、且是经典坑）：

    * ``16UC1`` —— 单位**毫米**，0 表示无效。整数，**没有 NaN 可以判定**。
    * ``32FC1`` —— 单位**米**，NaN 表示无效。

    搞错的后果是**深度差 1000 倍**。而且症状很隐蔽：近距离目标会变成几百米、
    远距离会变成几千米，被量程过滤掉之后表现为「深度全是无效值」，
    很容易被误诊成相机坏了或话题接错。

    Parameters
    ----------
    depth : np.ndarray
        原始深度图。
    encoding : str
        ``sensor_msgs/Image.encoding``，支持 ``16UC1`` / ``32FC1``
        （带 ``mono`` 前缀的变体也接受）。

    Returns
    -------
    np.ndarray
        ``float32``，单位米，无效值（0 / NaN / inf / 负数）为 ``NaN``。
    """
    enc = (encoding or "").strip().lower()

    if enc in ("16uc1", "mono16"):
        meters = depth.astype(np.float32) / 1000.0
        invalid = depth == 0
    elif enc in ("32fc1", "32fc"):
        meters = depth.astype(np.float32)
        invalid = np.zeros_like(meters, dtype=bool)
    else:
        raise DepthEncodingError(
            f"不支持的深度编码 {encoding!r}；支持 16UC1（毫米）/ 32FC1（米）"
        )

    meters = np.where(invalid, np.nan, meters)
    # 负值、inf 一律视为无效
    meters = np.where(np.isfinite(meters) & (meters > 0), meters, np.nan)
    return meters


# ----------------------------------------------------------------------
# 路线 A：深度相机
# ----------------------------------------------------------------------
def project_depth_bbox(
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    bbox_xyxy: tuple[float, float, float, float],
    mask: np.ndarray | None = None,
    min_valid_pixels: int = 10,
    max_dispersion_m: float = 0.5,
    anchor: str = "center",
) -> ProjectionResult | None:
    """用深度图把一个检测框解算成相机系三维点。

    Parameters
    ----------
    depth_m : np.ndarray
        **已转成米**的深度图（先过 ``depth_to_meters``）。
    intrinsics : CameraIntrinsics
    bbox_xyxy : tuple
        ``(x1, y1, x2, y2)`` 像素坐标。
    mask : np.ndarray | None
        可选的实例分割掩膜（bool，与原图同形状）。**强烈建议提供** ——
        掩膜内的深度采样严格优于框内的统计启发式，且对遮挡更鲁棒。
        （``yolo26*-seg.pt`` 是 COCO 预训练权重，零训练成本。）
    min_valid_pixels : int
        有效深度像素少于此值则丢弃该检测（目标被严重遮挡时深度不可信）。
    max_dispersion_m : float
        三维点离散度上限。超过说明这一团像素不属于同一个物体表面
        （例如框住了人和他背后的墙），结果不可信。
    anchor : str
        ``"center"`` —— 3D 包围盒的中心。
        ``"bottom"`` —— 3D 包围盒**底面**中心。

        倒地检测推荐 ``bottom``：人与地面的接触点比身体中心更稳定，
        且人躺下时身体中心会飘。
        **注意**：这里的「底」按相机的 -y 方向（光学系中 y 向下，故 -y 为上）判定，
        对明显俯仰的相机是近似。要精确应在 map 系（重力对齐）里判定 ——
        调用方拿到相机系点后自行处理更合适。

    Returns
    -------
    ProjectionResult 或 ``None``（有效像素不足 / 离散度过大 / 框内无深度）。
    """
    h, w = depth_m.shape
    x1, y1, x2, y2 = (int(round(v)) for v in bbox_xyxy)
    x1, x2 = max(0, min(x1, x2)), min(w, max(x1, x2))
    y1, y2 = max(0, min(y1, y2)), min(h, max(y1, y2))
    if x2 - x1 < 1 or y2 - y1 < 1:
        return None

    sub = depth_m[y1:y2, x1:x2]
    # 深度 <= 0 一律无效。0 在 16UC1(毫米) 里表示「无回波」，
    # 在米制里也可能因未走 depth_to_meters 而残留 —— 不排除的话会解算出
    # 一个落在相机原点上的「目标」，看起来像正常结果，实则完全错误。
    valid = np.isfinite(sub) & (sub > 0)
    if mask is not None:
        # ⚠️ 掩膜形状必须与深度图一致，否则**静默采错像素**。
        #
        # 掩膜来自 ultralytics，它的 `masks.data` 在某些配置下（rect /
        # retina_masks 关闭 + 非正方形输入 letterbox 补边）**不是原图尺寸**，
        # 而是补边后的推理尺寸。此时 `mask[y1:y2, x1:x2]` 用原图坐标去切，
        # 要么切成完全不相干的区域（不报错），要么在极端宽高比下抛
        # broadcast 错误。
        #
        # 本仓库实测（ultralytics 8.4.154、640×480 与 1024×1024 输入）掩膜尺寸是对的，
        # 但那是**当前版本的行为**，不是契约。这里断言一次，把潜在的静默错误
        # 变成一条明确的报错。
        if mask.shape != depth_m.shape:
            raise ValueError(
                f"分割掩膜形状 {mask.shape} 与深度图 {depth_m.shape} 不一致。\n"
                f"  掩膜必须与原图同尺寸；若它来自 ultralytics 的 masks.data，\n"
                f"  说明该版本的输出尺寸与输入不同（letterbox / retina_masks 设置相关），\n"
                f"  直接在 DetectorConfig 里开 retina_masks=True，或先把掩膜 resize 回原图。\n"
                f"  继续跑下去会用原图坐标切到错误的像素，而且不报错。"
            )
        valid &= mask[y1:y2, x1:x2].astype(bool)

    if valid.sum() < min_valid_pixels:
        return None

    rows, cols = np.nonzero(valid)
    z = sub[rows, cols].astype(np.float64)

    # 整块像素一起反投影，而不是「深度取分位数 + 坐标取框中心」
    u = cols + x1
    v = rows + y1
    x = (u - intrinsics.cx) * z / intrinsics.fx
    y = (v - intrinsics.cy) * z / intrinsics.fy

    pts = np.stack([x, y, z], axis=1)  # (N, 3)

    # 逐轴取中位数：稳健，且不受「深度取自哪个像素」影响
    center = np.median(pts, axis=0)

    # 离散度：两个指标取较大者，因为它们的敏感方向不同。
    #
    # (a) IQR（各轴取模）—— 对离群点不敏感，衡量整体的三维铺开程度，
    #     用来发现「框住了人 + 他背后一整面墙」这类大面积混合。
    # (b) Z 方向的高分位偏差 —— IQR 对**少数**离群完全无感
    #     （5% 的远处像素不影响四分位数），但正是「框里混进一条远处的
    #     背景边」这种情形会让中位数悄悄倒向错误的一侧。
    #     取 98 分位而不是最大值，是为了不被个别噪声像素触发。
    q75, q25 = np.percentile(pts, [75, 25], axis=0)
    iqr_spread = float(np.linalg.norm(q75 - q25))

    p98_z_dev = float(np.percentile(np.abs(z - np.median(z)), 98))

    dispersion = max(iqr_spread, p98_z_dev)

    if dispersion > max_dispersion_m:
        return None

    if anchor == "bottom":
        # 光学系里 y 向下，故「上」是 -y；底面即 y 最大的那一层
        # 取 90 分位代表底面，避免个别离群点把底面拉偏
        y_bottom = np.percentile(pts[:, 1], 90)
        bottom_pts = pts[pts[:, 1] >= y_bottom]
        if bottom_pts.size:
            pb = np.median(bottom_pts, axis=0)
            # 底面中心：x、z 用整体中位数，y 用底面
            center = np.array([pb[0], y_bottom, pb[2]], dtype=np.float64)
    elif anchor != "center":
        raise ValueError(f"anchor 只能是 'center' 或 'bottom'，得到 {anchor!r}")

    return ProjectionResult(
        point=center.astype(np.float64),
        valid_pixels=int(valid.sum()),
        dispersion=dispersion,
        spread_per_axis=(q75 - q25).astype(np.float64),
    )


# ----------------------------------------------------------------------
# 路线 B：单目 + 地面假设
# ----------------------------------------------------------------------
def project_ground_plane(
    u: float,
    v: float,
    intrinsics: CameraIntrinsics,
    camera_height_m: float,
    pitch_rad: float,
    max_range_m: float = 10.0,
) -> np.ndarray | None:
    """单目 + 地面假设：像素 -> 与地平面的交点（相机系）。

    ⚠️ **有硬性距离上限**。图像中心（光轴）对应的地面距离是 ``h / tan(θ)``：

        h = 0.35 m（Go2 相机高度）时
            θ = 10 度 -> 约 2.0 m
            θ =  5 度 -> 约 4.0 m
            θ =  3 度 -> 约 6.7 m，但可用像素挤在地平线附近，角分辨率急剧恶化

    也就是说这条路线**只能覆盖近场**。侦查场景是要看远方的，
    所以在决定用它之前先确认这个距离够不够。

    推导（相机系，光学系 x 右 / y 下 / z 前）：地平面在相机系中的方程为
        cos(θ) * Y + sin(θ) * Z = h
    光线 ``t * ((u-cx)/fx, (v-cy)/fy, 1)`` 代入解出 t。

    Parameters
    ----------
    pitch_rad : float
        相机**向下**的俯仰角（弧度）。地平线在图像中的位置由它决定。
    max_range_m : float
        返回点到相机的**欧氏距离**上限（米）。光线越接近与地面平行，
        交点是越远 —— 没有这个上限会返回反方向或无穷远的假坐标。

        ⚠️ 这里校验的是 ``|p|`` 而**不是**沿光轴的深度 ``t``（2026-09-17 修正）。
        原先校验的是 ``t``，但 ``t`` 与真实距离差一个
        ``sqrt(1 + dx² + dy²)``：对 120° 广角（fx≈185@640 宽）这个因子最大约 2.4，
        也就是说参数名叫「距离上限」、实际却可能返回**它 2 倍多**远的点。
        既然名字写的是距离，就按距离校验。

    Returns
    -------
    np.ndarray 或 ``None``（射线朝上 / 超出距离上限）。
    """
    dx = (u - intrinsics.cx) / intrinsics.fx
    dy = (v - intrinsics.cy) / intrinsics.fy
    dz = 1.0

    cos_t, sin_t = math.cos(pitch_rad), math.sin(pitch_rad)
    denom = cos_t * dy + sin_t * dz

    # 射线与地面平行或朝上 -> 无交点
    if denom <= 1e-9:
        return None

    t = camera_height_m / denom
    if not math.isfinite(t) or t <= 0:
        return None

    point = np.array([t * dx, t * dy, t * dz], dtype=np.float64)

    # 按**欧氏距离**校验上限，而不是按沿光轴深度 t —— 见 docstring。
    if float(np.linalg.norm(point)) > max_range_m:
        return None

    return point


# ----------------------------------------------------------------------
# 路线 C：点云簇关联
# ----------------------------------------------------------------------
def project_lidar_cluster(
    points_camera: np.ndarray,
    intrinsics: CameraIntrinsics,
    bbox_xyxy: tuple[float, float, float, float],
    min_points: int = 3,
) -> ProjectionResult | None:
    """点云簇关联：把落在检测框内的三维点取中位数。

    有激光雷达时的**标准做法**，比单目地面假设鲁棒得多，且不依赖深度相机。
    ``points_camera`` 应当是已经变换到相机系的点云（变换由调用方做，
    或先在 map 系里筛完再变换）。

    Parameters
    ----------
    points_camera : np.ndarray
        形状 ``(N, 3)``，相机系（光学系）下的点。
    bbox_xyxy : tuple
        检测框。用来把点投影到图像平面后筛选。
    min_points : int
        框内点数少于此值则丢弃。

    Returns
    -------
    ProjectionResult 或 ``None``。
    """
    if points_camera.size == 0:
        return None
    pts = np.asarray(points_camera, dtype=np.float64)
    pts = pts[np.isfinite(pts).all(axis=1)]
    if pts.size == 0:
        return None

    z = pts[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = np.where(z > 0, pts[:, 0] / z * intrinsics.fx + intrinsics.cx, -1.0)
        v = np.where(z > 0, pts[:, 1] / z * intrinsics.fy + intrinsics.cy, -1.0)

    x1, y1, x2, y2 = bbox_xyxy
    inside = (z > 0) & (u >= min(x1, x2)) & (u <= max(x1, x2)) & (v >= min(y1, y2)) & (v <= max(y1, y2))
    sel = pts[inside]
    if sel.shape[0] < min_points:
        return None

    center = np.median(sel, axis=0)
    q75, q25 = np.percentile(sel, [75, 25], axis=0)
    return ProjectionResult(
        point=center,
        valid_pixels=int(sel.shape[0]),
        dispersion=float(np.linalg.norm(q75 - q25)),
        spread_per_axis=(q75 - q25).astype(np.float64),
    )


def optical_axis_ground_distance(pitch_rad: float, camera_height_m: float) -> float:
    """**光轴与地面的交点**到相机的水平距离（米）= ``h / tan(theta)``。

    ⚠️ 这个函数原先叫 ``magnitude_of_ground_range``，docstring 说它是
    「理论最远可视地面距离」—— **两个都不对**（2026-09-17 修正）：

    * 它不是「最远可视距离」。真正的上限取决于地平线落在画面内还是画面外：
      - 下俯角大于竖直半视场角时，地平线在画面外，上限就是**画面最下缘**
        那条射线打到地面的位置；
      - 下俯角小于竖直半视场角时，**地平线本身就在画面里**，
        理论上可以看无限远，实际受限于角分辨率与标定误差。
    * 它也不叫「range」—— 它是**沿光轴**那条射线落点的水平距离。

    这个值仍然有用：它是路线 B 的一个自然参考点（画面中心能看到多远），
    也是「这条路线的覆盖够不够」的粗估起点。但**不要**把它当成硬上限，
    更不要拿它去校验别的量 —— 校验距离请用
    ``project_ground_plane(..., max_range_m=...)``，那里是按真实欧氏距离算的。
    """
    if pitch_rad <= 1e-9:
        return float("inf")
    return camera_height_m / math.tan(pitch_rad)
