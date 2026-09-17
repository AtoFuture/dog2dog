"""G2 视觉探索 —— 算法核心。

本包**不依赖 ROS2**，只依赖 numpy / OpenCV，因此可以脱离仿真环境独立单元测试。
ROS2 节点（``vision_detector`` / ``vision_explorer``）是这一层的薄封装：
负责订阅话题、调 tf2 做坐标变换、把结果打包成消息。

这样分层的原因见 docs/框架规划.md —— G2 的算法逻辑不该被「ROS2 装没装」
「G1 的接口好了没」阻塞。
"""

from .anomaly import (
    FallAssessment,
    assess_fall_from_keypoints_3d,
    bbox_aspect_is_fallen,
    midpoints_from_keypoints_2d,
    torso_tilt_from_vertical,
)
from .explorer import ExplorerParams, Goal, GoalSelector
from .frontier import FrontierCluster, detect_frontiers
from .geodesic import geodesic_distance, geodesic_nearest_cell
from .grid import FREE, OCCUPIED, UNKNOWN, GridMap
from .projector import (
    CameraIntrinsics,
    DepthEncodingError,
    ProjectionResult,
    depth_to_meters,
    magnitude_of_ground_range,
    project_depth_bbox,
    project_ground_plane,
    project_lidar_cluster,
)

__all__ = [
    # grid
    "GridMap", "FREE", "OCCUPIED", "UNKNOWN",
    # frontier
    "detect_frontiers", "FrontierCluster",
    # geodesic
    "geodesic_distance", "geodesic_nearest_cell",
    # explorer
    "GoalSelector", "ExplorerParams", "Goal",
    # projector
    "CameraIntrinsics", "ProjectionResult", "DepthEncodingError",
    "depth_to_meters", "project_depth_bbox", "project_ground_plane",
    "project_lidar_cluster", "magnitude_of_ground_range",
    # anomaly
    "FallAssessment", "torso_tilt_from_vertical", "assess_fall_from_keypoints_3d",
    "bbox_aspect_is_fallen", "midpoints_from_keypoints_2d",
]
