"""ROS/算法适配层：把 PointCloud2 中的 L1 点云定位到二维检测框。

本模块不发布 ROS 消息，也不查询 map TF。它只负责：

* 校验显式的相机内参和 LiDAR→camera 外参；
* 用 ``sensor_msgs_py.point_cloud2`` 读取 XYZ，保留无效点给后续过滤；
* 将点云变换到相机光学系，并复用 ``g2_core.project_lidar_cluster``。

map←camera 的 TF、时间同步和 Detection3D 发布仍由 ``detector_node`` 负责，
这样 LiDAR backend 可以在没有 ROS graph 的合成测试中独立验证。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from g2_core.projector import (
    CameraIntrinsics,
    ProjectionResult,
    project_lidar_cluster,
    select_lidar_points_in_bbox,
    transform_points,
)


class CalibrationError(ValueError):
    """LiDAR-camera 标定缺失、非法或仍是占位值。"""


@dataclass(frozen=True)
class LidarCalibration:
    """显式的 LiDAR-camera 标定，不包含任何 Go2 默认猜测值。

    ``translation_xyz`` 和 ``quaternion_xyzw`` 表示
    ``p_camera = T_camera_lidar * p_lidar``。
    """

    intrinsics: CameraIntrinsics
    image_width: int
    image_height: int
    lidar_frame: str
    camera_frame: str
    translation_xyz: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]

    def validate(self) -> None:
        vals = (
            self.intrinsics.fx, self.intrinsics.fy,
            self.intrinsics.cx, self.intrinsics.cy,
            *self.translation_xyz, *self.quaternion_xyzw,
        )
        if not all(math.isfinite(float(v)) for v in vals):
            raise CalibrationError("LiDAR-camera calibration 含 NaN/Inf")
        if self.intrinsics.fx <= 0.0 or self.intrinsics.fy <= 0.0:
            raise CalibrationError("fx/fy 必须为正数")
        if int(self.image_width) <= 0 or int(self.image_height) <= 0:
            raise CalibrationError("image_width/image_height 必须为正数")
        if not self.lidar_frame.strip() or not self.camera_frame.strip():
            raise CalibrationError("lidar_frame/camera_frame 不能为空")
        q_norm = float(np.linalg.norm(self.quaternion_xyzw))
        if not math.isclose(q_norm, 1.0, rel_tol=1e-3, abs_tol=1e-3):
            raise CalibrationError(
                f"LiDAR-camera quaternion 必须已归一化，范数为 {q_norm:.6f}"
            )


@dataclass(frozen=True)
class LidarBackendConfig:
    """LiDAR bbox 关联参数。"""

    bbox_shrink_ratio: float = 0.2
    min_points: int = 3
    depth_cluster_bin_m: float = 0.25
    min_depth_m: float = 0.1
    max_depth_m: float = 100.0

    def validate(self) -> None:
        if not (math.isfinite(self.bbox_shrink_ratio)
                and 0.0 <= self.bbox_shrink_ratio < 1.0):
            raise CalibrationError("bbox_shrink_ratio 必须在 [0, 1) 内")
        if int(self.min_points) < 1:
            raise CalibrationError("lidar_min_points 必须至少为 1")
        if not (math.isfinite(self.depth_cluster_bin_m)
                and self.depth_cluster_bin_m > 0.0):
            raise CalibrationError("lidar_depth_cluster_bin_m 必须为正数")
        if not (math.isfinite(self.min_depth_m)
                and math.isfinite(self.max_depth_m)
                and 0.0 < self.min_depth_m < self.max_depth_m):
            raise CalibrationError("LiDAR 有效深度范围非法")


@dataclass(frozen=True)
class PointCloudSample:
    """已从 ROS PointCloud2 解出的、带时间和 frame 的点云样本。"""

    stamp_sec: float
    frame_id: str
    points_lidar: np.ndarray


@dataclass(frozen=True)
class LidarLocalizationResult:
    """LiDAR backend 的可诊断结果。"""

    projection: ProjectionResult | None
    reason: str


def stamp_to_seconds(stamp) -> float:
    """把 ROS builtin_interfaces/Time 转成秒；零时间戳保持为 0。"""
    return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1e-9


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """从 PointCloud2 读取 ``(x, y, z)``，保留 NaN/Inf 供算法统计过滤。

    ``sensor_msgs_py`` 负责 PointField offset、endianness 和 point_step，
    不自行按字节猜布局。
    """
    field_names = {field.name for field in msg.fields}
    missing = {"x", "y", "z"} - field_names
    if missing:
        raise ValueError(f"PointCloud2 缺少 XYZ 字段：{sorted(missing)}")

    raw = point_cloud2.read_points(
        msg,
        field_names=("x", "y", "z"),
        skip_nans=False,
    )
    if isinstance(raw, np.ndarray):
        if raw.dtype.names:
            xyz = np.column_stack((raw["x"], raw["y"], raw["z"]))
        else:
            xyz = np.asarray(raw)
    else:
        xyz = np.asarray(list(raw), dtype=np.float64)

    if xyz.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    return xyz


def pointcloud2_to_sample(msg: PointCloud2) -> PointCloudSample:
    """读取 PointCloud2 并保留其时间戳和 frame_id。"""
    return PointCloudSample(
        stamp_sec=stamp_to_seconds(msg.header.stamp),
        frame_id=str(msg.header.frame_id),
        points_lidar=pointcloud2_to_xyz(msg),
    )


class LidarLocalizationBackend:
    """可插拔 LiDAR localization strategy。

    返回 reason 而不是抛异常，方便节点分别统计 no-points、cluster rejected
    和 frame mismatch。map TF 与时间门控不属于该纯 backend。
    """

    name = "lidar"

    def __init__(self, calibration: LidarCalibration, config: LidarBackendConfig):
        calibration.validate()
        config.validate()
        self.calibration = calibration
        self.config = config

    def localize(
        self,
        detection,
        sample: PointCloudSample,
        image_shape: tuple[int, int],
    ) -> LidarLocalizationResult:
        if tuple(image_shape) != (self.calibration.image_height, self.calibration.image_width):
            return LidarLocalizationResult(None, "image_size_mismatch")
        if sample.frame_id != self.calibration.lidar_frame:
            return LidarLocalizationResult(None, "frame_mismatch")

        points_lidar = np.asarray(sample.points_lidar, dtype=np.float64)
        points_lidar = points_lidar[np.isfinite(points_lidar).all(axis=1)]
        if points_lidar.size == 0:
            return LidarLocalizationResult(None, "no_points_in_bbox")

        points_camera = transform_points(
            points_lidar,
            self.calibration.quaternion_xyzw,
            self.calibration.translation_xyz,
        )
        candidates = select_lidar_points_in_bbox(
            points_camera,
            self.calibration.intrinsics,
            detection.bbox_xyxy,
            bbox_shrink_ratio=self.config.bbox_shrink_ratio,
            min_depth_m=self.config.min_depth_m,
            max_depth_m=self.config.max_depth_m,
        )
        if candidates.shape[0] == 0:
            return LidarLocalizationResult(None, "no_points_in_bbox")

        projection = project_lidar_cluster(
            points_camera,
            self.calibration.intrinsics,
            detection.bbox_xyxy,
            min_points=self.config.min_points,
            bbox_shrink_ratio=self.config.bbox_shrink_ratio,
            depth_cluster_bin_m=self.config.depth_cluster_bin_m,
            min_depth_m=self.config.min_depth_m,
            max_depth_m=self.config.max_depth_m,
        )
        if projection is None:
            return LidarLocalizationResult(None, "cluster_rejected")
        return LidarLocalizationResult(projection, "ok")


def _tuple_of(value, size: int, name: str) -> tuple[float, ...]:
    try:
        vals = tuple(float(v) for v in value)
    except (TypeError, ValueError) as exc:
        raise CalibrationError(f"{name} 必须是 {size} 个数值") from exc
    if len(vals) != size:
        raise CalibrationError(f"{name} 必须是 {size} 个元素，得到 {len(vals)}")
    return vals


def calibration_from_parameters(values: Mapping[str, object]) -> LidarCalibration:
    """从 ROS 参数值构造并校验 calibration；零值占位会被拒绝。"""
    calibration = LidarCalibration(
        intrinsics=CameraIntrinsics(
            fx=float(values["lidar_fx"]),
            fy=float(values["lidar_fy"]),
            cx=float(values["lidar_cx"]),
            cy=float(values["lidar_cy"]),
        ),
        image_width=int(values["lidar_image_width"]),
        image_height=int(values["lidar_image_height"]),
        lidar_frame=str(values["lidar_frame"]),
        camera_frame=str(values["lidar_camera_frame"]),
        translation_xyz=_tuple_of(values["lidar_translation_xyz"], 3, "lidar_translation_xyz"),
        quaternion_xyzw=_tuple_of(values["lidar_quaternion_xyzw"], 4, "lidar_quaternion_xyzw"),
    )
    calibration.validate()
    return calibration


def backend_config_from_parameters(values: Mapping[str, object]) -> LidarBackendConfig:
    """从 ROS 参数构造 LiDAR 算法配置。"""
    config = LidarBackendConfig(
        bbox_shrink_ratio=float(values["lidar_bbox_shrink_ratio"]),
        min_points=int(values["lidar_min_points"]),
        depth_cluster_bin_m=float(values["lidar_depth_cluster_bin_m"]),
        min_depth_m=float(values["lidar_min_depth_m"]),
        max_depth_m=float(values["lidar_max_depth_m"]),
    )
    config.validate()
    return config
