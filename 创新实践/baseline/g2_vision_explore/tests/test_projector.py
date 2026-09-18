"""三维定位的测试。

重点：
* **深度编码 16UC1(毫米) vs 32FC1(米) 必须归一化到同一个结果** ——
  搞错就是 1000 倍的误差，且症状隐蔽（表现为「深度全是无效值」）。
* **「深度与像素同源」** —— 这是 v2 修掉的问题，不能退回
  「取分位数深度 + 用框中心坐标」的写法。
* 地面假设路线的**距离上限**必须生效。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from g2_core.projector import (
    CameraIntrinsics,
    DepthEncodingError,
    depth_to_meters,
    pitch_axis_angle,
    quaternion_multiply,
    quaternion_to_matrix,
    rotate_translate,
    sample_depth_near,
    optical_axis_ground_distance,
    project_depth_bbox,
    project_ground_plane,
    project_lidar_cluster,
)

K = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0)


# ----------------------------------------------------------------------
# 深度编码
# ----------------------------------------------------------------------
def test_16uc1_and_32fc1_agree():
    """同一场景的两种编码必须得到相同的米制深度。

    这是防「差 1000 倍」的核心测试。
    """
    mm = np.array([[5000, 3000], [0, 1500]], dtype=np.uint16)   # 毫米, 0=无效
    m = np.array([[5.0, 3.0], [np.nan, 1.5]], dtype=np.float32)  # 米

    a = depth_to_meters(mm, "16UC1")
    b = depth_to_meters(m, "32FC1")

    assert np.allclose(a, b, equal_nan=True)


def test_16uc1_zero_is_invalid_not_zero_meters():
    """16UC1 的 0 表示无效，不能当成 0 米（那会在相机原点上）。"""
    mm = np.array([[0, 2000]], dtype=np.uint16)
    out = depth_to_meters(mm, "16UC1")

    assert np.isnan(out[0, 0])
    assert out[0, 1] == pytest.approx(2.0)


def test_inf_and_negative_are_invalid():
    m = np.array([[1.0, np.inf, -3.0, np.nan]], dtype=np.float32)
    out = depth_to_meters(m, "32FC1")

    assert out[0, 0] == pytest.approx(1.0)
    assert np.isnan(out[0, 1])
    assert np.isnan(out[0, 2])
    assert np.isnan(out[0, 3])


def test_unsupported_encoding_raises():
    with pytest.raises(DepthEncodingError):
        depth_to_meters(np.zeros((2, 2), dtype=np.float32), "rgb8")


# ----------------------------------------------------------------------
# 路线 A：深度反投影
# ----------------------------------------------------------------------
def _flat_depth(shape=(480, 640), z=5.0):
    """构造一张处处为 z 的深度图（米）。"""
    return np.full(shape, z, dtype=np.float32)


def test_backprojection_of_bbox_center_pixel():
    """光轴上的点应当解算到相机正前方。"""
    depth = _flat_depth()
    # 以主点 (320, 240) 为中心的框
    res = project_depth_bbox(depth, K, (310, 230, 331, 251))
    assert res is not None
    assert res.point == pytest.approx([0.0, 0.0, 5.0], abs=0.05)


def test_backprojection_offset_matches_geometry():
    """偏离主点的框，解算结果要符合手算。"""
    depth = _flat_depth()
    # 框中心在 (370, 290)：X = 50*5/500 = 0.5, Y = 50*5/500 = 0.5
    res = project_depth_bbox(depth, K, (350, 270, 391, 311))
    assert res is not None
    assert res.point[0] == pytest.approx(0.5, abs=0.05)
    assert res.point[1] == pytest.approx(0.5, abs=0.05)
    assert res.point[2] == pytest.approx(5.0, abs=0.01)


def test_depth_median_matches_bbox_median():
    """v2 修正的核心：深度必须与其像素同源，逐像素反投影后取中位数。

    用一个横向梯度来锚定这个语义：返回的 Z 应当恰好是框内深度的中位数。
    若退回 v1 的写法（取一个分位数深度 + 用框中心像素），
    Z 会等于分位数本身，且 X 会与 Z 来自不同的像素。
    """
    depth = np.full((480, 640), 2.0, dtype=np.float32)
    # 框内深度从左到右由 2.0 缓升到 2.4（梯度要平缓，否则会触发离散度门限）
    depth[230:251, 310:331] = np.linspace(2.0, 2.4, 21)[None, :]

    res = project_depth_bbox(depth, K, (310, 230, 331, 251))
    assert res is not None

    expected_z = float(np.median(depth[230:251, 310:331]))
    assert res.point[2] == pytest.approx(expected_z, abs=1e-3)

    # X 必须与 Z 同源：用期望的 Z 反推 X，应当一致
    u, v = np.meshgrid(np.arange(310, 331), np.arange(230, 251))
    z = depth[230:251, 310:331]
    expected_x = float(np.median((u - K.cx) * z / K.fx))
    assert res.point[0] == pytest.approx(expected_x, abs=1e-3)


def test_bimodal_depth_inside_bbox_is_rejected():
    """框里混进两个深度面（人 + 背后的墙）→ 结果不可信，应丢弃。

    少量远处像素不会改变中位数（那正是中位数的优点），
    但会让「这个三维点代表谁」变得不可判定，所以宁可不报。
    """
    depth = np.full((480, 640), 2.0, dtype=np.float32)
    depth[230:251, 330:331] = 20.0  # 一列很远的背景，约占框内 5%

    assert project_depth_bbox(depth, K, (310, 230, 331, 251), max_dispersion_m=1.0) is None


def test_invalid_pixels_are_excluded():
    depth = _flat_depth()
    depth[230:251, 310:331] = 0.0  # 框内全为无效

    assert project_depth_bbox(depth, K, (310, 230, 331, 251)) is None


def test_min_valid_pixels_gate():
    depth = _flat_depth()
    depth[230:251, 310:331] = np.nan
    depth[240, 320] = 5.0  # 只留 1 个有效像素

    assert project_depth_bbox(depth, K, (310, 230, 331, 251), min_valid_pixels=10) is None
    assert project_depth_bbox(depth, K, (310, 230, 331, 251), min_valid_pixels=1) is not None


def test_dispersion_gate_rejects_mixed_depth():
    """框内混了远近两团像素（例如人 + 背后的墙）应被判为不可信。"""
    depth = _flat_depth(z=2.0)
    depth[230:251, 330:331] = 20.0  # 一列很远的背景

    res = project_depth_bbox(depth, K, (310, 230, 331, 251), max_dispersion_m=1.0)
    assert res is None, "离散度超限时应当丢弃而不是给出一个看似正常的坐标"


def test_mask_restricts_sampling():
    """给分割掩膜时，只采样掩膜内的像素。"""
    depth = _flat_depth(z=2.0)
    depth[230:251, 310:331] = 8.0

    mask = np.zeros((480, 640), dtype=bool)
    mask[230:251, 310:315] = True  # 只框住左边一小条（深度 8）

    res = project_depth_bbox(depth, K, (310, 230, 331, 251), mask=mask)
    assert res is not None
    assert res.point[2] == pytest.approx(8.0, abs=0.01)
    assert res.valid_pixels == 21 * 5


def test_bottom_anchor_is_strictly_lower_than_center():
    """bottom 锚点必须**严格**比 center 靠下（光学系 y 向下即更大）。

    ⚠️ 这条测试原先用的是**平深度图** —— 那时所有三维点的 y 完全相同，
    bottom 与 center 恒等，断言 `>=` 空洞地成立。变异测试证实：
    把 bottom 分支改成等价于 center 的空操作，测试照样通过（M21 存活）。

    现在构造一个**沿竖直方向递增的深度梯度**：远处（图像下方）的点更深，
    于是 y = (v-cy)*d/fy 的分布右偏，90 分位严格大于中位数 ——
    这条断言才有判别力。
    """
    depth = np.full((480, 640), 2.0, dtype=np.float32)
    # 框内深度自上而下由 2.0 缓升到 2.5（梯度要平缓，否则触发离散度门限）
    depth[230:251, 310:331] = np.linspace(2.0, 2.5, 21)[:, None]

    center = project_depth_bbox(depth, K, (310, 230, 331, 251), anchor="center")
    bottom = project_depth_bbox(depth, K, (310, 230, 331, 251), anchor="bottom")

    assert center is not None and bottom is not None
    assert bottom.point[1] > center.point[1] + 1e-3, (
        f"bottom 锚点({bottom.point[1]:.4f}) 必须严格低于 center({center.point[1]:.4f})；"
        f"相等说明 bottom 分支没有真正生效"
    )
    # x / z 取自底面点，但不应跳到另一个量级
    assert bottom.point[2] == pytest.approx(center.point[2], rel=0.5)


def test_bad_anchor_raises():
    depth = _flat_depth()
    with pytest.raises(ValueError):
        project_depth_bbox(depth, K, (310, 230, 331, 251), anchor="middle")


# ----------------------------------------------------------------------
# 路线 B：地面假设
# ----------------------------------------------------------------------
def test_ground_plane_centre_pixel_geometry():
    """光轴像素与地面的交点，几何关系要闭合。

    相机系下的点 ``(X, Y, Z)`` 中 ``Z`` 是**沿光轴**的距离，
    而「最远可视地面距离」是**水平**距离。两者差一个 ``cos(theta)``：

        沿光轴  t = h / sin(theta)
        水平    t * cos(theta) = h / tan(theta)   <- 这就是路线 B 的覆盖上限
    """
    h, pitch = 0.35, math.radians(5.0)
    pt = project_ground_plane(K.cx, K.cy, K, h, pitch, max_range_m=100.0)

    assert pt is not None
    assert pt[2] == pytest.approx(h / math.sin(pitch), rel=1e-3)
    assert pt[2] * math.cos(pitch) == pytest.approx(
        optical_axis_ground_distance(pitch, h), rel=1e-3
    )


def test_ground_plane_rejects_rays_pointing_up():
    """地平线以上的像素不应产生坐标（否则会得到无穷远或反方向的假点）。"""
    h, pitch = 0.35, math.radians(5.0)
    # 画面很上方，射线朝上
    assert project_ground_plane(K.cx, K.cy - 300, K, h, pitch) is None


def test_ground_plane_enforces_max_range():
    h, pitch = 0.35, math.radians(5.0)
    theoretical = optical_axis_ground_distance(pitch, h)  # 约 4.0 m

    assert project_ground_plane(K.cx, K.cy - 10, K, h, pitch, max_range_m=theoretical * 0.5) is None


def test_ground_range_grows_as_pitch_flattens():
    """下俯角越小上限越远 —— 这是评估路线 B 是否可用的关键数字。"""
    assert optical_axis_ground_distance(math.radians(10), 0.35) == pytest.approx(2.0, abs=0.1)
    assert optical_axis_ground_distance(math.radians(5), 0.35) == pytest.approx(4.0, abs=0.2)
    assert optical_axis_ground_distance(math.radians(3), 0.35) == pytest.approx(6.7, abs=0.4)


# ----------------------------------------------------------------------
# 路线 C：点云簇
# ----------------------------------------------------------------------
def test_lidar_cluster_takes_median_of_points_inside_bbox():
    pts = np.array([
        [0.0, 0.0, 5.0],
        [0.1, 0.1, 5.0],
        [-0.1, -0.1, 5.0],
        [0.0, 0.0, 5.0],
    ])
    # 这些点都投影到主点附近
    res = project_lidar_cluster(pts, K, (300, 220, 340, 260))

    assert res is not None
    assert res.point[2] == pytest.approx(5.0)
    assert res.valid_pixels == 4


def test_lidar_cluster_ignores_points_behind_camera():
    pts = np.array([[0.0, 0.0, -5.0]])  # z<0 在相机背后
    assert project_lidar_cluster(pts, K, (0, 0, 640, 480)) is None


def test_lidar_cluster_respects_min_points():
    pts = np.array([[0.0, 0.0, 5.0]])
    assert project_lidar_cluster(pts, K, (300, 220, 340, 260), min_points=3) is None


def test_camera_intrinsics_from_k():
    k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
    intr = CameraIntrinsics.from_camera_info(k)

    assert (intr.fx, intr.fy, intr.cx, intr.cy) == (500.0, 500.0, 320.0, 240.0)


def test_camera_intrinsics_rejects_bad_k():
    with pytest.raises(ValueError):
        CameraIntrinsics.from_camera_info([1.0, 2.0, 3.0])


def test_ground_plane_max_range_uses_euclidean_distance():
    """`max_range_m` 必须按**欧氏距离**校验，不是沿光轴深度。

    ⚠️ 回归测试：原先校验的是 ``t``（沿光轴深度），而 ``t`` 与真实距离
    差一个 ``sqrt(1 + dx² + dy²)``。对 120° 广角这个因子最大约 2.4 ——
    也就是说参数名叫「距离上限」，实际却可能返回它 2 倍多远的点。

    构造一个**远离光轴**的像素：它的 ``t`` 很小（射线很快打到地面），
    但真实距离并不小。用一个小上限，按距离算应当被拦下。
    """
    h, pitch = 0.35, math.radians(5.0)
    # 画面右下角：dx、dy 都很大
    u, v = K.cx + 300, K.cy + 200

    p = project_ground_plane(u, v, K, h, pitch, max_range_m=1e9)
    assert p is not None, "先确认这个像素确实能落到地面上"
    real_dist = float(np.linalg.norm(p))
    t_axis = float(p[2])

    assert real_dist > t_axis, "离轴越远，欧氏距离与沿轴深度的差越大"

    # 上限卡在两者之间：按距离算应当拒绝，按沿轴深度算会放行
    limit = (real_dist + t_axis) / 2
    assert project_ground_plane(u, v, K, h, pitch, max_range_m=limit) is None, (
        "上限用的应当是欧氏距离；这里返回了点说明还在按沿轴深度校验"
    )


# ----------------------------------------------------------------------
# sample_depth_near —— 关键点取深度必须取「近端」
# ----------------------------------------------------------------------
def _scene_with_person():
    """3x3 米背景在 3.0 m，中间一块 1.5 m 的人（占窗口约 1/4）。"""
    z = np.full((40, 40), 3.0, dtype=np.float32)
    z[14:26, 14:26] = 1.5
    return z


def test_sample_depth_near_picks_the_person_over_the_background():
    """窗口里背景占多数时，中位数会给出背景 —— 近端分位数不会。"""
    z = _scene_with_person()

    near = sample_depth_near(z, 20, 20, radius=9, percentile=5.0)

    assert near == pytest.approx(1.5, abs=0.01)
    # 对照：中位数被背景拖走
    patch = z[11:30, 11:30]
    assert float(np.median(patch)) == pytest.approx(3.0)


def test_sample_depth_near_nan_is_excluded_from_the_percentile():
    """无效值（NaN）不参与分位数 —— 这是它比 np.percentile 裸调用强的地方。"""
    z = _scene_with_person()
    z[14:26, 14:26] = np.nan      # 人那块无效

    # 剩下的全是 3.0 的背景，于是返回 3.0 而不是 NaN
    assert sample_depth_near(z, 20, 20, radius=9) == pytest.approx(3.0)


def test_sample_depth_near_silently_falls_back_to_background():
    """⚠️ 已知危险行为：人的深度若整体无效，本函数**静默**返回背景深度。

    它没有能力判断「取到的这一层是不是人」。所以它**不能单独使用** ——
    必须搭配 anomaly.check_body_proportions()：那里的肩宽门就是为
    这种情况准备的（取到背景 -> 两个肩深度差一大截 -> 肩宽远超 0.65 m -> 丢弃）。
    """
    z = _scene_with_person()
    z[14:26, 14:26] = np.nan

    est = sample_depth_near(z, 20, 20, radius=9)

    assert est == pytest.approx(3.0), "取到的是背景，不是人"
    assert est > 2.0, "这个值本身看不出问题 —— 必须靠人体尺度门兜住"


def test_sample_depth_near_returns_nan_outside_the_image():
    z = _scene_with_person()
    assert math.isnan(sample_depth_near(z, -5, 20))
    assert math.isnan(sample_depth_near(z, 20, 999))


def test_sample_depth_near_respects_min_valid():
    """有效像素太少时宁可返回 nan，也不要凭两三个点下结论。"""
    z = np.full((40, 40), np.nan, dtype=np.float32)
    z[20, 20] = 1.5

    assert math.isnan(sample_depth_near(z, 20, 20, radius=9, min_valid=10))
    assert sample_depth_near(z, 20, 20, radius=9, min_valid=1) == pytest.approx(1.5)


def test_sample_depth_near_is_biased_near_on_a_sloped_surface():
    """已知代价：窗口跨越斜面时低分位数取到近端边缘，深度系统性偏近。

    这不是 bug，是这套方法的固有偏置 —— 所以它必须搭配
    anomaly.check_body_proportions 那种**区间**质检用，不能单独用。
    """
    z = np.tile(np.linspace(2.0, 4.0, 40, dtype=np.float32), (40, 1))

    est = sample_depth_near(z, 20, 20, radius=9, percentile=5.0)

    assert est < z[20, 20], "低分位数应当偏近"
    assert z[20, 20] - est > 0.3, "偏置量级应当是可见的（这里接近窗口的近端边缘）"


# ----------------------------------------------------------------------
# 四元数 -> 旋转矩阵（倒地链路用它把关键点从相机系转到 map 系）
# ----------------------------------------------------------------------
def test_quaternion_to_matrix_identity():
    assert quaternion_to_matrix(0.0, 0.0, 0.0, 1.0) == pytest.approx(np.eye(3))


def test_quaternion_to_matrix_180_about_z():
    R = quaternion_to_matrix(0.0, 0.0, 1.0, 0.0)
    assert R == pytest.approx(np.diag([-1.0, -1.0, 1.0]))


def test_quaternion_to_matrix_matches_the_replay_tf():
    """回归：``replay_images`` 里那个光学系 -> map 系的四元数。

    之前手写成 (0.707,-0.707,0,0)，把光学「前」映射到了 map 的 **-z（朝下）**，
    三维点整体转了 90° 而**不报任何错**。这里把正确的映射钉死。
    """
    R = quaternion_to_matrix(0.5, -0.5, 0.5, -0.5)

    # 光学 z(前) -> map +x ；光学 x(右) -> map -y ；光学 y(下) -> map -z
    assert R @ np.array([0.0, 0.0, 1.0]) == pytest.approx([1.0, 0.0, 0.0])
    assert R @ np.array([1.0, 0.0, 0.0]) == pytest.approx([0.0, -1.0, 0.0])
    assert R @ np.array([0.0, 1.0, 0.0]) == pytest.approx([0.0, 0.0, -1.0])
    assert np.linalg.det(R) == pytest.approx(1.0), "必须是纯旋转，不能带镜像或缩放"


def test_rotate_translate_applies_rotation_then_translation():
    p = np.array([1.0, 0.0, 0.0])
    # 绕 z 转 90°：x -> y
    out = rotate_translate(p, (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)),
                           (10.0, 20.0, 30.0))

    assert out == pytest.approx([10.0, 21.0, 30.0])


def test_rotate_translate_keeps_distances():
    """纯旋转 + 平移不改变两点间距离 —— 人体尺度质检依赖这一点。"""
    a = np.array([0.3, -0.2, 2.0])
    b = np.array([-0.1, 0.4, 2.1])
    quat = (0.2, -0.3, 0.5, 0.787)      # 未归一化！下面的断言会暴露它
    trans = (1.0, 2.0, 3.0)

    d_in = float(np.linalg.norm(a - b))
    d_out = float(np.linalg.norm(rotate_translate(a, quat, trans)
                                 - rotate_translate(b, quat, trans)))

    # 非归一化四元数会带来整体缩放，距离对不上 —— 这里正是要证明函数**不替调用方**
    # 归一化。TF 给的四元数一定是归一化的，正常路径不会踩到。
    assert d_out != pytest.approx(d_in, rel=1e-6), "非归一化四元数应当能看出缩放"

    n = math.sqrt(sum(c * c for c in quat))
    quat_n = tuple(c / n for c in quat)
    d_ok = float(np.linalg.norm(rotate_translate(a, quat_n, trans)
                                - rotate_translate(b, quat_n, trans)))
    assert d_ok == pytest.approx(d_in, rel=1e-6)


def test_quaternion_multiply_order_matches_matrix_order():
    """``a ⊗ b`` 对应 ``R(a) @ R(b)`` —— 顺序反了会得到一个看着合理的错姿态。"""
    a = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))   # 绕 z 90°
    b = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))   # 绕 x 90°

    ab = quaternion_to_matrix(*quaternion_multiply(a, b))

    assert ab == pytest.approx(quaternion_to_matrix(*a) @ quaternion_to_matrix(*b))


def test_pitched_replay_tf_puts_world_up_where_the_physics_says():
    """回放器加俯仰后，map 的「上」在相机系里应当落在 (0,-cosθ,-sinθ)。

    这是倒地判据的**重力来源**：俯仰角填错，所有倾角会整体平移。
    """
    q0 = (0.5, -0.5, 0.5, -0.5)                       # 光学 -> map（相机水平）
    theta = math.radians(20.0)

    q = quaternion_multiply(q0, pitch_axis_angle((1.0, 0.0, 0.0), -theta))
    R = quaternion_to_matrix(*q)

    # 反解：map 的 +z 在相机系里的方向 = R^T @ (0,0,1)
    up_in_camera = R.T @ np.array([0.0, 0.0, 1.0])

    assert up_in_camera == pytest.approx(
        [0.0, -math.cos(theta), -math.sin(theta)], abs=1e-9)
    assert np.linalg.det(R) == pytest.approx(1.0)
