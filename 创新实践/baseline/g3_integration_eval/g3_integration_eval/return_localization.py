"""Pure 2-D localization checks used to verify that RETURN reached home.

The navigation stack may report success after its map frame has moved.  This
module keeps the frame arithmetic and the independent home-scan comparison out
of the ROS node so neither result depends on Gazebo ground truth.
"""

from dataclasses import dataclass
import math
from typing import Iterable, Optional

import numpy as np


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class LocalizationSample:
    map_pose: Pose2
    odom_pose: Pose2
    map_to_odom: Pose2
    linear_speed: float
    angular_speed: float
    stamp_ns: int


@dataclass(frozen=True)
class HomeReference:
    map_pose: Pose2
    odom_pose: Pose2
    map_to_odom: Pose2
    scan_points: np.ndarray
    stamp_ns: int


@dataclass(frozen=True)
class IntegrityResult:
    healthy: bool
    reason: str
    map_odom_step_m: float
    map_odom_step_yaw: float
    stationary_drift_m: float
    stationary_drift_yaw: float


@dataclass(frozen=True)
class ScanMatch:
    valid: bool
    current_in_home: Pose2
    rmse_m: float
    inlier_ratio: float
    inlier_count: int
    iterations: int
    reason: str


DEFAULT_RETURN_LOCALIZATION = {
    "enabled": False,
    "stale_s": 1.0,
    "jump_translation_m": 0.35,
    "jump_yaw_rad": 0.5,
    "stationary_linear_mps": 0.03,
    "stationary_angular_radps": 0.08,
    "stationary_drift_m": 0.08,
    "stationary_drift_yaw_rad": 0.12,
    "position_tolerance_m": 0.35,
    "yaw_tolerance_rad": 0.45,
    "stable_s": 3.0,
    "verification_timeout_s": 8.0,
    "max_corrections": 2,
    "scan_match_required": True,
    "scan_match_rmse_m": 0.18,
    "scan_match_min_inlier_ratio": 0.35,
    "scan_match_max_correspondence_m": 0.75,
    "scan_match_min_points": 40,
    "scan_match_max_points": 360,
    "scan_match_max_iterations": 15,
    "scan_match_convergence_m": 0.002,
    "max_scan_correction_m": 1.8,
    "max_scan_correction_yaw_rad": 1.2,
}


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def compose_pose(parent_from_middle: Pose2, middle_from_child: Pose2) -> Pose2:
    c = math.cos(parent_from_middle.yaw)
    s = math.sin(parent_from_middle.yaw)
    return Pose2(
        parent_from_middle.x + c * middle_from_child.x - s * middle_from_child.y,
        parent_from_middle.y + s * middle_from_child.x + c * middle_from_child.y,
        wrap_angle(parent_from_middle.yaw + middle_from_child.yaw),
    )


def inverse_pose(parent_from_child: Pose2) -> Pose2:
    c = math.cos(parent_from_child.yaw)
    s = math.sin(parent_from_child.yaw)
    return Pose2(
        -c * parent_from_child.x - s * parent_from_child.y,
        s * parent_from_child.x - c * parent_from_child.y,
        wrap_angle(-parent_from_child.yaw),
    )


def relative_pose(parent_from_first: Pose2, parent_from_second: Pose2) -> Pose2:
    """Return the pose of ``second`` expressed in ``first`` coordinates."""

    return compose_pose(inverse_pose(parent_from_first), parent_from_second)


def pose_error(first: Pose2, second: Pose2) -> tuple[float, float]:
    delta = relative_pose(first, second)
    return math.hypot(delta.x, delta.y), abs(delta.yaw)


def pose_dict(pose: Pose2) -> dict:
    return {"x": pose.x, "y": pose.y, "yaw": pose.yaw}


def validate_return_localization_config(value) -> dict:
    if not isinstance(value, dict):
        raise ValueError("return localization config must be an object")
    unknown = set(value) - set(DEFAULT_RETURN_LOCALIZATION)
    if unknown:
        raise ValueError("unknown return localization fields: " + ", ".join(sorted(unknown)))
    config = dict(DEFAULT_RETURN_LOCALIZATION)
    config.update(value)
    for key in ("enabled", "scan_match_required"):
        if type(config[key]) is not bool:
            raise ValueError(f"{key} must be boolean")
    for key in ("max_corrections", "scan_match_min_points", "scan_match_max_points", "scan_match_max_iterations"):
        if type(config[key]) is not int or config[key] < (0 if key == "max_corrections" else 1):
            raise ValueError(f"{key} must be an integer in range")
    for key, item in config.items():
        if key in ("enabled", "scan_match_required", "max_corrections", "scan_match_min_points",
                   "scan_match_max_points", "scan_match_max_iterations"):
            continue
        if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item) or item <= 0:
            raise ValueError(f"{key} must be finite and positive")
    if config["stable_s"] >= config["verification_timeout_s"]:
        raise ValueError("verification timeout must exceed the stable interval")
    if not 0 < config["scan_match_min_inlier_ratio"] <= 1:
        raise ValueError("scan_match_min_inlier_ratio must be in (0, 1]")
    if config["scan_match_min_points"] > config["scan_match_max_points"]:
        raise ValueError("scan match minimum points exceeds maximum points")
    return config


class LocalizationIntegrity:
    """Detect sudden frame jumps and map motion while odometry is stationary."""

    def __init__(self, config: dict):
        self.config = validate_return_localization_config(config)
        self._previous_map_to_odom: Optional[Pose2] = None
        self._stationary_anchor: Optional[Pose2] = None

    def reset(self) -> None:
        self._previous_map_to_odom = None
        self._stationary_anchor = None

    def observe(self, sample: LocalizationSample) -> IntegrityResult:
        cfg = self.config
        step_m = step_yaw = 0.0
        reason = "healthy"
        if self._previous_map_to_odom is not None:
            step_m, step_yaw = pose_error(self._previous_map_to_odom, sample.map_to_odom)
            if step_m > cfg["jump_translation_m"] or step_yaw > cfg["jump_yaw_rad"]:
                reason = "map_to_odom_jump"
        self._previous_map_to_odom = sample.map_to_odom

        stationary = (
            abs(sample.linear_speed) <= cfg["stationary_linear_mps"]
            and abs(sample.angular_speed) <= cfg["stationary_angular_radps"]
        )
        drift_m = drift_yaw = 0.0
        if stationary:
            if self._stationary_anchor is None:
                self._stationary_anchor = sample.map_to_odom
            drift_m, drift_yaw = pose_error(self._stationary_anchor, sample.map_to_odom)
            if (drift_m > cfg["stationary_drift_m"] or
                    drift_yaw > cfg["stationary_drift_yaw_rad"]):
                if reason == "healthy":
                    reason = "stationary_map_drift"
                # Report one discontinuity, then require a fresh stable window
                # around the new localization solution.
                self._stationary_anchor = sample.map_to_odom
        else:
            self._stationary_anchor = None
        return IntegrityResult(
            reason == "healthy", reason, step_m, step_yaw, drift_m, drift_yaw
        )


def scan_to_points(
    ranges: Iterable[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    base_from_sensor: Pose2,
    max_points: int,
) -> np.ndarray:
    """Convert finite LaserScan returns into a bounded point cloud in base."""

    values = np.asarray(tuple(ranges), dtype=float)
    if values.ndim != 1 or values.size == 0:
        return np.empty((0, 2), dtype=float)
    indexes = np.arange(values.size, dtype=float)
    mask = np.isfinite(values) & (values >= range_min) & (values <= range_max)
    values = values[mask]
    angles = angle_min + indexes[mask] * angle_increment
    points = np.column_stack((values * np.cos(angles), values * np.sin(angles)))
    c, s = math.cos(base_from_sensor.yaw), math.sin(base_from_sensor.yaw)
    rotation = np.asarray(((c, -s), (s, c)))
    points = points @ rotation.T + np.asarray((base_from_sensor.x, base_from_sensor.y))
    if len(points) > max_points:
        selection = np.linspace(0, len(points) - 1, max_points, dtype=int)
        points = points[selection]
    return np.asarray(points, dtype=float)


def _transform_points(points: np.ndarray, pose: Pose2) -> np.ndarray:
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    rotation = np.asarray(((c, -s), (s, c)))
    return points @ rotation.T + np.asarray((pose.x, pose.y))


def _nearest(reference: np.ndarray, query: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    indexes = np.empty(len(query), dtype=int)
    squared = np.empty(len(query), dtype=float)
    for start in range(0, len(query), 128):
        part = query[start:start + 128]
        distances = np.sum((part[:, None, :] - reference[None, :, :]) ** 2, axis=2)
        local = np.argmin(distances, axis=1)
        indexes[start:start + len(part)] = local
        squared[start:start + len(part)] = distances[np.arange(len(part)), local]
    return indexes, squared


def _best_fit(source: np.ndarray, target: np.ndarray) -> Pose2:
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    left, _, right = np.linalg.svd(covariance)
    rotation = right.T @ left.T
    if np.linalg.det(rotation) < 0:
        right[-1, :] *= -1
        rotation = right.T @ left.T
    translation = target_center - rotation @ source_center
    return Pose2(float(translation[0]), float(translation[1]),
                 math.atan2(rotation[1, 0], rotation[0, 0]))


def match_home_scan(
    reference: np.ndarray,
    current: np.ndarray,
    initial_current_in_home: Pose2,
    config: dict,
) -> ScanMatch:
    """Use bounded point-to-point ICP to estimate current base in home base."""

    cfg = validate_return_localization_config(config)
    reference = np.asarray(reference, dtype=float)
    current = np.asarray(current, dtype=float)
    minimum = cfg["scan_match_min_points"]
    if (reference.ndim != 2 or current.ndim != 2 or reference.shape[1:] != (2,)
            or current.shape[1:] != (2,) or len(reference) < minimum or len(current) < minimum
            or not np.isfinite(reference).all() or not np.isfinite(current).all()):
        return ScanMatch(False, initial_current_in_home, math.inf, 0.0, 0, 0,
                         "insufficient_scan_points")

    pose = initial_current_in_home
    max_distance_sq = cfg["scan_match_max_correspondence_m"] ** 2
    iterations = 0
    for iterations in range(1, cfg["scan_match_max_iterations"] + 1):
        transformed = _transform_points(current, pose)
        nearest, squared = _nearest(reference, transformed)
        mask = squared <= max_distance_sq
        if int(mask.sum()) < minimum:
            return ScanMatch(False, pose, math.inf, float(mask.mean()), int(mask.sum()),
                             iterations, "insufficient_correspondences")
        delta = _best_fit(transformed[mask], reference[nearest[mask]])
        pose = compose_pose(delta, pose)
        if math.hypot(delta.x, delta.y) <= cfg["scan_match_convergence_m"] and abs(delta.yaw) <= 0.002:
            break

    transformed = _transform_points(current, pose)
    _, squared = _nearest(reference, transformed)
    mask = squared <= max_distance_sq
    count = int(mask.sum())
    ratio = float(mask.mean())
    rmse = math.sqrt(float(squared[mask].mean())) if count else math.inf
    if count < minimum:
        reason = "insufficient_correspondences"
    elif ratio < cfg["scan_match_min_inlier_ratio"]:
        reason = "low_inlier_ratio"
    elif rmse > cfg["scan_match_rmse_m"]:
        reason = "high_rmse"
    else:
        reason = "matched"
    return ScanMatch(reason == "matched", pose, rmse, ratio, count, iterations, reason)


def dynamic_home_target(current_map_to_odom: Pose2, home_odom_pose: Pose2) -> Pose2:
    """Express the recorded odometry home in the current map frame."""

    return compose_pose(current_map_to_odom, home_odom_pose)


def scan_corrected_home_target(current_map_pose: Pose2, match: ScanMatch) -> Pose2:
    """Turn a home-scan relative pose into a corrected target in current map."""

    return compose_pose(current_map_pose, inverse_pose(match.current_in_home))
