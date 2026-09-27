"""L1 LiDAR localization 的确定性合成测试。

这些测试只证明软件几何、PointCloud2 解码、标定校验和时间门控；不代表
``/utlidar/cloud`` 或 Go2 真机的 frame、内参、外参和时钟已经验证。
"""

from __future__ import annotations

from collections import deque

import numpy as np
import pytest

from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2

from g2_core.detector import Detection2D
from g2_core.projector import (
    CameraIntrinsics,
    quaternion_to_matrix,
    transform_points,
)
from vision_detector.detector_node import TrackedFrame2D, VisionDetector
from vision_detector.lidar_backend import (
    CalibrationError,
    LidarBackendConfig,
    LidarCalibration,
    LidarLocalizationBackend,
    PointCloudSample,
    calibration_from_parameters,
    pointcloud2_to_sample,
    pointcloud2_to_xyz,
)


K = CameraIntrinsics(fx=100.0, fy=100.0, cx=320.0, cy=240.0)
IMAGE_SHAPE = (480, 640)
BBOX = (320.0, 220.0, 355.0, 260.0)


def _detection(bbox=BBOX):
    return Detection2D(
        class_name="person",
        class_id=0,
        confidence=0.9,
        bbox_xyxy=bbox,
        track_id=5,
    )


def _calibration(
    translation=(0.0, 0.0, 0.0),
    quaternion=(0.0, 0.0, 0.0, 1.0),
):
    return LidarCalibration(
        intrinsics=K,
        image_width=IMAGE_SHAPE[1],
        image_height=IMAGE_SHAPE[0],
        lidar_frame="utlidar_lidar",
        camera_frame="go2_camera_optical",
        translation_xyz=translation,
        quaternion_xyzw=quaternion,
    )


def _backend(calibration=None, **kwargs):
    return LidarLocalizationBackend(
        calibration or _calibration(),
        LidarBackendConfig(**kwargs),
    )


def _sample(points, stamp=10.0, frame_id="utlidar_lidar"):
    return PointCloudSample(
        stamp_sec=stamp,
        frame_id=frame_id,
        points_lidar=np.asarray(points, dtype=np.float64),
    )


def _target_cluster(center=(0.5, 0.0, 3.0), count=12, seed=7):
    rng = np.random.default_rng(seed)
    return np.asarray(center) + rng.normal(scale=(0.02, 0.02, 0.03), size=(count, 3))


def test_normal_foreground_cluster_recovers_camera_point():
    result = _backend().localize(
        _detection(), _sample(_target_cluster()), IMAGE_SHAPE
    )

    assert result.reason == "ok"
    assert result.projection is not None
    assert result.projection.point == pytest.approx([0.5, 0.0, 3.0], abs=0.08)
    assert result.projection.valid_pixels >= 3


def test_nearest_dense_cluster_wins_over_background_wall():
    foreground = _target_cluster(count=12, seed=8)
    # 同一像素区域内的远处墙：数量更多，但深度层明显更远。
    background = _target_cluster(center=(1.35, 0.0, 8.0), count=80, seed=9)
    result = _backend().localize(
        _detection(), _sample(np.vstack((foreground, background))), IMAGE_SHAPE
    )

    assert result.reason == "ok"
    assert result.projection is not None
    assert result.projection.point[2] == pytest.approx(3.0, abs=0.1)


def test_points_outside_bbox_are_reported_without_localization():
    result = _backend().localize(
        _detection(), _sample(_target_cluster(center=(3.0, 0.0, 3.0))), IMAGE_SHAPE
    )

    assert result.projection is None
    assert result.reason == "no_points_in_bbox"


def test_points_behind_camera_are_rejected():
    result = _backend().localize(
        _detection(), _sample(_target_cluster(center=(0.5, 0.0, -3.0))), IMAGE_SHAPE
    )

    assert result.projection is None
    assert result.reason == "no_points_in_bbox"


def test_nan_and_inf_are_rejected():
    invalid = np.array([
        [np.nan, 0.0, 3.0],
        [0.5, np.inf, 3.0],
        [0.5, 0.0, np.inf],
    ])
    result = _backend().localize(_detection(), _sample(invalid), IMAGE_SHAPE)

    assert result.projection is None
    assert result.reason == "no_points_in_bbox"


def test_rotation_and_translation_extrinsic_are_applied():
    angle = np.pi / 2.0
    quaternion = (0.0, 0.0, np.sin(angle / 2.0), np.cos(angle / 2.0))
    translation = (0.2, -0.1, 0.3)
    target_camera = _target_cluster(seed=12)
    rotation = quaternion_to_matrix(*quaternion)
    points_lidar = (rotation.T @ (target_camera - np.asarray(translation)).T).T

    result = _backend(_calibration(translation, quaternion)).localize(
        _detection(), _sample(points_lidar), IMAGE_SHAPE
    )

    assert result.reason == "ok"
    assert result.projection is not None
    assert result.projection.point == pytest.approx([0.5, 0.0, 3.0], abs=0.08)


def test_real_pointcloud2_message_decodes_and_reaches_projector():
    header = Header()
    header.frame_id = "utlidar_lidar"
    header.stamp.sec = 10
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    points = _target_cluster(count=8, seed=13).tolist()
    points.extend([[np.nan, 0.0, 3.0], [0.5, 0.0, np.inf]])
    msg = point_cloud2.create_cloud(header, fields, points)

    xyz = pointcloud2_to_xyz(msg)
    sample = pointcloud2_to_sample(msg)
    result = _backend().localize(_detection(), sample, IMAGE_SHAPE)

    assert xyz.shape == (10, 3)
    assert np.isnan(xyz[-2, 0])
    assert np.isinf(xyz[-1, 2])
    assert sample.stamp_sec == pytest.approx(10.0)
    assert result.reason == "ok"
    assert result.projection is not None


def test_frame_and_image_geometry_are_checked():
    wrong_frame = _backend().localize(
        _detection(), _sample(_target_cluster(), frame_id="other_lidar"), IMAGE_SHAPE
    )
    wrong_size = _backend().localize(
        _detection(), _sample(_target_cluster()), (720, 1280)
    )

    assert wrong_frame.reason == "frame_mismatch"
    assert wrong_size.reason == "image_size_mismatch"


def test_sync_slop_rejects_old_pointcloud():
    class _Param:
        def __init__(self, value):
            self.value = value

    class _Node:
        _lidar_clouds = deque([_sample(_target_cluster(), stamp=9.0)])
        _stat = {"unsynced": 0}

        def get_parameter(self, name):
            assert name == "lidar_sync_slop_s"
            return _Param(0.05)

        def _nearest_lidar_cloud(self, stamp):
            return VisionDetector._nearest_lidar_cloud(self, stamp)

        def _record_unsynced(self):
            self._stat["unsynced"] += 1
            self._stat["dropped_unsynced"] = self._stat.get("dropped_unsynced", 0) + 1

    stamp = Time()
    stamp.sec = 10
    node = _Node()
    assert VisionDetector._nearest_lidar_cloud(node, stamp) is None
    assert node._stat["unsynced"] == 1
    assert node._stat["dropped_unsynced"] == 1


def test_missing_calibration_is_rejected_not_defaulted():
    values = {
        "lidar_fx": 0.0,
        "lidar_fy": 0.0,
        "lidar_cx": 0.0,
        "lidar_cy": 0.0,
        "lidar_image_width": 0,
        "lidar_image_height": 0,
        "lidar_frame": "utlidar_lidar",
        "lidar_camera_frame": "",
        "lidar_translation_xyz": [0.0, 0.0, 0.0],
        "lidar_quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }

    with pytest.raises(CalibrationError):
        calibration_from_parameters(values)


def test_invalid_quaternion_transform_is_not_silently_normalized():
    with pytest.raises(CalibrationError):
        _calibration(quaternion=(0.0, 0.0, 0.0, 2.0)).validate()


def test_detector_lidar_path_publishes_only_after_all_gates_pass():
    class _Param:
        def __init__(self, value):
            self.value = value

    class _Node:
        _lidar_backend = _backend()
        _lidar_clouds = deque([_sample(_target_cluster(), stamp=10.0)])
        _stat = {
            "frames_with_2d_detection": 0,
            "unsynced": 0,
            "fusion_attempts": 0,
            "fusion_success": 0,
            "no_points_in_bbox": 0,
            "cluster_rejected": 0,
            "tf_failed": 0,
            "published": 0,
            "lidar_published": 0,
            "dedup": 0,
        }

        def __init__(self):
            self.published = []

        def get_parameter(self, name):
            assert name == "lidar_sync_slop_s"
            return _Param(0.05)

        def _nearest_lidar_cloud(self, stamp):
            return VisionDetector._nearest_lidar_cloud(self, stamp)

        def _record_unsynced(self):
            self._stat["unsynced"] += 1
            self._stat["dropped_unsynced"] = self._stat.get("dropped_unsynced", 0) + 1

        def _lookup_tf(self, camera_frame, stamp):
            assert camera_frame == "go2_camera_optical"
            return "map_tf"

        def _to_map(self, projection, tf):
            assert tf == "map_tf"
            return tuple(projection.point)

        def _should_publish(self, track_id, now):
            return True

        def _publish_detection(self, detection, pose, stamp):
            self.published.append((detection, pose, stamp))

    stamp = Time()
    stamp.sec = 10
    frame = TrackedFrame2D(
        detections=[_detection()],
        stamp=stamp,
        frame_id="go2_camera_optical",
        image_shape=IMAGE_SHAPE,
    )
    node = _Node()

    VisionDetector._process_lidar_detections(node, frame, now=10.0)

    assert len(node.published) == 1
    assert node._stat["fusion_attempts"] == 1
    assert node._stat["fusion_success"] == 1
    assert node._stat["lidar_published"] == 1


def test_detector_lidar_path_keeps_3d_unpublished_when_unsynced():
    class _Param:
        def __init__(self, value):
            self.value = value

    class _Node:
        _lidar_backend = _backend()
        _lidar_clouds = deque([_sample(_target_cluster(), stamp=8.0)])
        _stat = {
            "frames_with_2d_detection": 0,
            "unsynced": 0,
            "fusion_attempts": 0,
            "fusion_success": 0,
            "no_points_in_bbox": 0,
            "cluster_rejected": 0,
            "tf_failed": 0,
            "published": 0,
            "lidar_published": 0,
            "dedup": 0,
        }

        def get_parameter(self, name):
            assert name == "lidar_sync_slop_s"
            return _Param(0.05)

        def _nearest_lidar_cloud(self, stamp):
            return VisionDetector._nearest_lidar_cloud(self, stamp)

        def _record_unsynced(self):
            self._stat["unsynced"] += 1
            self._stat["dropped_unsynced"] = self._stat.get("dropped_unsynced", 0) + 1

    stamp = Time()
    stamp.sec = 10
    frame = TrackedFrame2D(
        detections=[_detection()],
        stamp=stamp,
        frame_id="go2_camera_optical",
        image_shape=IMAGE_SHAPE,
    )
    node = _Node()
    node.published = []

    VisionDetector._process_lidar_detections(node, frame, now=10.0)

    assert node._stat["unsynced"] == 1
    assert node._stat["dropped_unsynced"] == 1
    assert node._stat["fusion_success"] == 0
    assert node._stat["published"] == 0


def test_batch_transform_matches_expected_rigid_transform():
    points = np.array([[1.0, 0.0, 2.0]])
    transformed = transform_points(points, (0.0, 0.0, 0.0, 1.0), (0.5, -0.5, 1.0))
    assert transformed[0] == pytest.approx([1.5, -0.5, 3.0])
