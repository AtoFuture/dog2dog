"""``floor`` 模块的测试。

这个模块存在的理由就是「RANSAC 只找内点最多的平面，不保证那是地面」，
所以测试的重点不是「能不能拟合出平面」（那很平常），而是
**「拟合错了的时候会不会被抓出来」**。

合成点云足够：地面是不是「在相机下方 1.5 m 的一个平面」这件事，
和真实深度图无关。
"""

from __future__ import annotations

import numpy as np
import pytest

from g2_core.floor import FloorFitStatus, fit_floor_plane


def _plane_cloud(y: float, n: int = 4000, noise: float = 0.005, seed: int = 0):
    """造一片水平面上的点云。相机在原点，光学系 ``+y`` 朝下。

    ``y > 0`` 在相机下方（像地面），``y < 0`` 在相机上方（像天花板）。
    """
    rng = np.random.default_rng(seed)
    x = rng.uniform(-2, 2, n)
    z = rng.uniform(0.5, 4.0, n)
    return np.stack([x, np.full(n, y) + rng.normal(0, noise, n), z], axis=-1)


def _plane_plus_volume_below(plane_y: float, n_plane: int, n_below: int, seed: int = 0):
    """一片平面，外加**散布在它下方的体**（不是另一个平面）。

    这样 RANSAC 只会选中那片平面（体里凑不出第二个平面），
    但下方占多数 —— 用来单独触发「平面不在场景底部」这一条，
    同时让相机高度保持在合理范围内，把两条校验隔离开。
    """
    rng = np.random.default_rng(seed)
    plane = _plane_cloud(plane_y, n_plane, seed=seed)
    below = np.stack([rng.uniform(-2, 2, n_below),
                      rng.uniform(plane_y + 0.1, plane_y + 1.5, n_below),
                      rng.uniform(0.5, 4.0, n_below)], axis=-1)
    return np.vstack([plane, below])


# ----------------------------------------------------------------------
# 正常情形
# ----------------------------------------------------------------------
def test_floor_below_camera_is_accepted():
    fit = fit_floor_plane(_plane_cloud(1.5))

    assert fit.status is FloorFitStatus.OK
    assert fit.ok
    assert fit.normal is not None
    assert fit.camera_height_m == pytest.approx(1.5, abs=0.02)
    assert fit.above_ratio > 0.99


def test_normal_points_up():
    """光学系 ``+y`` 朝下，所以「上」的法向其 y 分量必须为负。

    定号错了会让所有离地高度整体变号 —— 站立的人变成地下 1 m。
    """
    fit = fit_floor_plane(_plane_cloud(1.5))

    assert fit.normal[1] < 0
    # 平面方程自洽：真值平面上的点代进去应当接近 0。
    # 容差取 1 cm —— 平面由三个**含噪声**的采样点确定（噪声 5 mm），
    # 卡得比噪声还紧只会测到随机数。
    assert abs(fit.normal @ np.array([0.0, 1.5, 2.0]) + fit.offset) < 0.01


def test_nan_points_are_dropped():
    """深度图里无效点是 NaN，不能把拟合带崩。"""
    pts = np.vstack([_plane_cloud(1.5, 2000),
                     np.full((500, 3), np.nan)])

    fit = fit_floor_plane(pts)

    assert fit.status is FloorFitStatus.OK
    assert fit.camera_height_m == pytest.approx(1.5, abs=0.02)


# ----------------------------------------------------------------------
# 拟合错了 —— 这些才是本模块存在的理由
# ----------------------------------------------------------------------
def test_ceiling_is_rejected_and_height_keeps_its_sign():
    """⚠️ 核心回归：平面在相机**上方**时必须被拒，且高度保持**负号**。

    这条钉的是原先那个 ``abs(d)``。天花板在相机上方 1.5 m，
    取绝对值之后会变成 ``+1.5`` —— 一个看起来完全正常的「相机离地 1.5 m」，
    于是天花板被当成地面用，量出来的离地高度整体偏掉 3 m，**不报任何错**。
    """
    fit = fit_floor_plane(_plane_cloud(-1.5))

    assert fit.status is FloorFitStatus.CAMERA_HEIGHT_IMPLAUSIBLE
    assert not fit.ok
    assert fit.normal is None, "不合格时不许把平面交出去"
    assert fit.camera_height_m == pytest.approx(-1.5, abs=0.02)


def test_plane_above_the_scene_bottom_is_rejected():
    """大片点落在平面**下方** = 锁到了别的平面（床、桌面），不是地面。

    相机高度特意留在合理范围内，好把这一条与「相机高度」那条**隔离开** ——
    否则测试通过了也分不清是哪条校验起的作用。
    """
    pts = _plane_plus_volume_below(plane_y=1.5, n_plane=1500, n_below=2500)
    fit = fit_floor_plane(pts)

    assert fit.status is FloorFitStatus.NOT_AT_BOTTOM, (
        f"实得 {fit.status.value}，相机高度 {fit.camera_height_m:.2f}")
    assert fit.normal is None
    assert fit.above_ratio < 0.5, f"实得 {fit.above_ratio:.2%}"
    # 相机高度是合理的 —— 证明拒它的确实是「不在底部」这条
    assert 0.2 <= fit.camera_height_m <= 3.0


def test_default_above_ratio_is_not_the_naive_high_value():
    """⚠️ 回归：``min_above_ratio`` 的默认值不许调回 0.95。

    0.95 是初版凭直觉写的，在 1378 素材上实测**多拒 28% 的帧**，
    其中包含物理量正确的帧（帧 229 坐床、帧 1263 站立）。
    实测空档是 [0.193, 0.543]，默认值必须落在这个空档里。
    详见 ``fit_floor_plane`` 的 docstring。
    """
    import inspect

    from g2_core import floor as floor_mod

    default = inspect.signature(floor_mod.fit_floor_plane).parameters["min_above_ratio"].default
    assert 0.193 < default < 0.543, (
        f"min_above_ratio 默认值 {default} 落在实测的「正确帧」区间里了"
    )


def test_default_camera_height_covers_the_deployment_platform():
    """⭐ 回归：默认下限必须覆盖**部署平台** —— Go2 的车载相机离地约 0.35 m。

    这条测试的由来（2026-09-18）。**它原先的方向是反的**：老版本叫
    ``test_camera_height_below_the_lower_bound_is_rejected``，
    断言「0.4 m 会被默认拒绝，真机上得记得显式调小」——
    把「默认值不适配部署平台」当成了**使用者的责任**记在测试里。

    实测确认之后，这不再是使用者的责任，是**默认值错了**：

        官方 URDF  front_camera_joint 相对 base  z = +0.043 m
        机器人自报  /sportmodestate 的 body_height   = 0.309 m
        ────────────────────────────────────────────────────
        车载相机离地 ≈ 0.35 m

    而默认下限是 1.0 m（按手持/固定安装的 Kinect 素材定的）。
    按那个默认值上真机，**每一帧都会被判不合格，而且不报错** ——
    只表现为「永远没有离地高度」，正是本项目反复踩的那类失效。

    ⚠️ 用**全默认参数**调用。生产代码走的就是默认值，
    「默认值坏了」受影响的是现场而不是测试。
    """
    pts = _plane_cloud(0.35)          # ← Go2 实测的车载相机高度

    fit = fit_floor_plane(pts)

    assert fit.status is FloorFitStatus.OK, (
        f"0.35 m（Go2 车载相机的实测高度）被默认参数拒了：{fit.reason}。"
        f"checkpoint：min_camera_height_m 是不是被调回 1.0 了？"
    )
    assert fit.camera_height_m == pytest.approx(0.35, abs=0.02)


def test_camera_height_below_the_lower_bound_is_still_rejected():
    """下限仍要挡得住**离谱**的低值 —— 放宽不等于取消。

    放宽默认值是为了覆盖部署平台，不是把这道校验废掉：
    实测到的**灾难性错拟合给的是负值**（相机高 −3.3 m，平面跑到相机上方），
    而 5 cm 这种值任何车载/手持相机都不可能给出。
    """
    pts = _plane_cloud(0.05)

    assert fit_floor_plane(pts).status is FloorFitStatus.CAMERA_HEIGHT_IMPLAUSIBLE


def test_too_few_points():
    fit = fit_floor_plane(np.zeros((10, 3)))

    assert fit.status is FloorFitStatus.TOO_FEW_POINTS
    assert fit.normal is None


def test_collinear_points_yield_no_plane():
    """三点共线一直是 RANSAC 的退化情形，不能让它在 ``np.cross`` 之后静默出结果。"""
    t = np.linspace(0, 1, 500)
    pts = np.stack([t, t, t], axis=-1) * 3.0

    fit = fit_floor_plane(pts, iters=50)

    assert fit.status is FloorFitStatus.NO_PLANE
    assert fit.normal is None


# ----------------------------------------------------------------------
# 诊断量
# ----------------------------------------------------------------------
def test_reason_text_carries_the_numbers():
    """失败原因要能直接进日志 —— 只说「失败」没法判断阈值该不该调。"""
    bad_bottom = fit_floor_plane(
        _plane_plus_volume_below(plane_y=1.5, n_plane=1500, n_below=2500))
    assert "不在场景底部" in bad_bottom.reason
    assert "%" in bad_bottom.reason

    bad_height = fit_floor_plane(_plane_cloud(-1.5))
    assert "相机离地高度" in bad_height.reason
    assert "m" in bad_height.reason

    assert fit_floor_plane(_plane_cloud(1.5)).reason == "通过校验"


def test_failures_still_report_diagnostics():
    """失败也要带上诊断量，否则没法回答「差多少」。"""
    fit = fit_floor_plane(_plane_cloud(-1.5))

    assert np.isfinite(fit.inlier_ratio)
    assert fit.inlier_ratio > 0.9
    assert np.isfinite(fit.above_ratio)


def test_step_subsampling_does_not_change_the_answer():
    pts = _plane_cloud(1.5, n=8000)

    a = fit_floor_plane(pts)
    b = fit_floor_plane(pts, step=5)

    assert a.status is b.status is FloorFitStatus.OK
    assert a.camera_height_m == pytest.approx(b.camera_height_m, abs=0.02)
