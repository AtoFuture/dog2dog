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
    magnitude_of_ground_range,
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


def test_bottom_anchor_is_lower_than_center():
    """bottom 锚点应当比 center 更靠下（光学系 y 向下即更大）。"""
    depth = _flat_depth()
    center = project_depth_bbox(depth, K, (310, 230, 331, 251), anchor="center")
    bottom = project_depth_bbox(depth, K, (310, 230, 331, 251), anchor="bottom")

    assert center is not None and bottom is not None
    assert bottom.point[1] >= center.point[1]


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
        magnitude_of_ground_range(pitch, h), rel=1e-3
    )


def test_ground_plane_rejects_rays_pointing_up():
    """地平线以上的像素不应产生坐标（否则会得到无穷远或反方向的假点）。"""
    h, pitch = 0.35, math.radians(5.0)
    # 画面很上方，射线朝上
    assert project_ground_plane(K.cx, K.cy - 300, K, h, pitch) is None


def test_ground_plane_enforces_max_range():
    h, pitch = 0.35, math.radians(5.0)
    theoretical = magnitude_of_ground_range(pitch, h)  # 约 4.0 m

    assert project_ground_plane(K.cx, K.cy - 10, K, h, pitch, max_range_m=theoretical * 0.5) is None


def test_ground_range_grows_as_pitch_flattens():
    """下俯角越小上限越远 —— 这是评估路线 B 是否可用的关键数字。"""
    assert magnitude_of_ground_range(math.radians(10), 0.35) == pytest.approx(2.0, abs=0.1)
    assert magnitude_of_ground_range(math.radians(5), 0.35) == pytest.approx(4.0, abs=0.2)
    assert magnitude_of_ground_range(math.radians(3), 0.35) == pytest.approx(6.7, abs=0.4)


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
