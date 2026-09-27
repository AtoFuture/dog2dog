"""棋盘格相机内参标定准备工具。

该工具只输出 camera intrinsics。它不会猜测 Go2 camera frame、LiDAR frame 或
camera-LiDAR 外参；输出结果明确标记为 ``intrinsics_only``，不能直接作为完整
LiDAR localization calibration 使用。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def calibration_api_capabilities() -> dict[str, bool]:
    """Report APIs available in the current OpenCV installation."""
    aruco = getattr(cv2, "aruco", None)
    return {
        "chessboard_findChessboardCorners": hasattr(cv2, "findChessboardCorners"),
        "chessboard_findChessboardCornersSB": hasattr(cv2, "findChessboardCornersSB"),
        "charuco_module": aruco is not None,
        "charuco_board_api": bool(aruco is not None and hasattr(aruco, "CharucoBoard")),
    }


def _object_points(pattern_size: tuple[int, int], square_size_m: float) -> np.ndarray:
    cols, rows = pattern_size
    points = np.zeros((rows * cols, 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    points[:, :2] *= float(square_size_m)
    return points


def _find_corners(gray: np.ndarray, pattern_size: tuple[int, int]) -> tuple[bool, np.ndarray | None]:
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, pattern_size, None)
        if found:
            return True, np.asarray(corners, dtype=np.float32)
    found, corners = cv2.findChessboardCorners(
        gray,
        pattern_size,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not found:
        return False, None
    refined = cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
    )
    return True, np.asarray(refined, dtype=np.float32)


def _reprojection_error(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    rvecs: list[np.ndarray],
    tvecs: list[np.ndarray],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> float:
    total = 0.0
    count = 0
    for obj, observed, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, distortion)
        observed_xy = np.asarray(observed, dtype=np.float64).reshape(-1, 2)
        projected_xy = np.asarray(projected, dtype=np.float64).reshape(-1, 2)
        residual = observed_xy - projected_xy
        total += float(np.sum(residual * residual))
        count += int(observed_xy.shape[0])
    return float(np.sqrt(total / count)) if count else float("nan")


def calibrate_from_images(
    input_dir: Path,
    pattern_size: tuple[int, int] = (9, 6),
    square_size_m: float = 0.025,
) -> dict[str, Any]:
    if pattern_size[0] <= 0 or pattern_size[1] <= 0:
        raise ValueError("pattern_size must be positive")
    if square_size_m <= 0.0:
        raise ValueError("square_size_m must be positive")

    image_paths = sorted(
        path for path in input_dir.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    ) if input_dir.exists() else []
    if not image_paths:
        raise FileNotFoundError(f"no calibration images found in {input_dir}")

    object_template = _object_points(pattern_size, square_size_m)
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    used_images: list[str] = []
    image_size: tuple[int, int] | None = None
    rejected_images: list[str] = []

    for path in image_paths:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            rejected_images.append(f"{path.name}: unreadable")
            continue
        current_size = (int(image.shape[1]), int(image.shape[0]))
        if image_size is None:
            image_size = current_size
        elif image_size != current_size:
            rejected_images.append(f"{path.name}: image size differs from {image_size}")
            continue
        found, corners = _find_corners(image, pattern_size)
        if not found or corners is None:
            rejected_images.append(f"{path.name}: chessboard not found")
            continue
        object_points.append(object_template.copy())
        image_points.append(corners)
        used_images.append(str(path))

    if image_size is None:
        raise RuntimeError("all calibration images were unreadable")
    if len(object_points) < 3:
        raise RuntimeError(
            f"only {len(object_points)} valid chessboard views; at least 3 are required"
        )

    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )
    reprojection = _reprojection_error(
        object_points, image_points, rvecs, tvecs, camera_matrix, distortion
    )
    return {
        "calibration_type": "camera_intrinsics_only",
        "calibration_status": "intrinsics_only",
        "opencv_api": calibration_api_capabilities(),
        "pattern": {
            "type": "chessboard",
            "columns": int(pattern_size[0]),
            "rows": int(pattern_size[1]),
            "square_size_m": float(square_size_m),
        },
        "image_count": len(used_images),
        "images": used_images,
        "rejected_images": rejected_images,
        "image_width": image_size[0],
        "image_height": image_size[1],
        "fx": float(camera_matrix[0, 0]),
        "fy": float(camera_matrix[1, 1]),
        "cx": float(camera_matrix[0, 2]),
        "cy": float(camera_matrix[1, 2]),
        "distortion": np.asarray(distortion).reshape(-1).astype(float).tolist(),
        "rms_reprojection_error_px": float(rms),
        "reprojection_error_px": float(reprojection),
        # Directly named fields make later conversion to detector YAML explicit.
        "lidar_fx": float(camera_matrix[0, 0]),
        "lidar_fy": float(camera_matrix[1, 1]),
        "lidar_cx": float(camera_matrix[0, 2]),
        "lidar_cy": float(camera_matrix[1, 2]),
        "lidar_image_width": image_size[0],
        "lidar_image_height": image_size[1],
        "lidar_frame": "",
        "lidar_camera_frame": "",
        "lidar_translation_xyz": None,
        "lidar_quaternion_xyzw": None,
        "camera_lidar_extrinsic": None,
        "note": "Intrinsics only. Do not use as complete Go2 LiDAR calibration until measured frames and extrinsic are added.",
    }


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate camera intrinsics from chessboard images")
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--columns", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-size-m", type=float, default=0.025)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    result = calibrate_from_images(
        args.input_dir,
        pattern_size=(args.columns, args.rows),
        square_size_m=args.square_size_m,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"intrinsics-only calibration written to {args.output}")
    print(f"valid views={result['image_count']} reprojection_error_px={result['reprojection_error_px']:.4f}")
    print("camera-LiDAR extrinsic remains unset; this is not a complete Go2 calibration")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
