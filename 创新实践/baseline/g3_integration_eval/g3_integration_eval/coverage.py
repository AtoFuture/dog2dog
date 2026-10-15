"""Map coverage evaluation helpers for G3."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml


PIXEL_FREE = "free"
PIXEL_OCCUPIED = "occupied"
PIXEL_UNKNOWN = "unknown"

DEFAULT_FREE_THRESH = 0.196
DEFAULT_OCCUPIED_THRESH = 0.65


@dataclass(frozen=True)
class MapData:
    width: int
    height: int
    data: bytes
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    yaml_path: Path
    free_thresh: float = DEFAULT_FREE_THRESH
    occupied_thresh: float = DEFAULT_OCCUPIED_THRESH
    negate: bool = False


@dataclass(frozen=True)
class CoverageResult:
    reference_free_cells: int
    observed_reference_free_cells: int
    observed_as_free: int
    observed_as_occupied: int
    outside_slam_canvas: int
    coverage: float


def _load_pgm(path: Path):
    with path.open("rb") as file:
        tokens = []

        while len(tokens) < 4:
            line = file.readline()

            if not line:
                raise RuntimeError("unexpected end of PGM header")

            line = line.split(b"#", 1)[0]

            if line.strip():
                tokens.extend(line.split())

        magic = tokens[0]
        width = int(tokens[1])
        height = int(tokens[2])
        maxval = int(tokens[3])

        if magic != b"P5":
            raise ValueError(
                f"only binary P5 PGM is supported: {path}"
            )

        if maxval != 255:
            raise ValueError(
                f"unsupported PGM maxval {maxval}: {path}"
            )

        data = file.read(width * height)

        if len(data) != width * height:
            raise ValueError(
                f"PGM data size mismatch: {path}"
            )

    return width, height, data


def load_map(yaml_path) -> MapData:
    yaml_path = Path(yaml_path).expanduser().resolve()

    if not yaml_path.is_file():
        raise FileNotFoundError(
            f"map YAML does not exist: {yaml_path}"
        )

    config = yaml.safe_load(
        yaml_path.read_text(encoding="utf-8")
    )

    image_name = config.get("image")
    if not image_name:
        raise ValueError(
            f"map YAML has no image field: {yaml_path}"
        )

    image_path = Path(image_name)

    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path

    image_path = image_path.resolve()

    if not image_path.is_file():
        raise FileNotFoundError(
            f"map image does not exist: {image_path}"
        )

    width, height, data = _load_pgm(image_path)

    origin = config.get("origin")

    if not isinstance(origin, list) or len(origin) < 3:
        raise ValueError(
            f"invalid map origin in {yaml_path}"
        )

    return MapData(
        width=width,
        height=height,
        data=data,
        resolution=float(config["resolution"]),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        origin_yaw=float(origin[2]),
        yaml_path=yaml_path,
        free_thresh=float(config.get("free_thresh", DEFAULT_FREE_THRESH)),
        occupied_thresh=float(config.get("occupied_thresh", DEFAULT_OCCUPIED_THRESH)),
        negate=bool(config.get("negate", False)),
    )


def _classify_pixel(
    pixel: int,
    *,
    free_thresh: float,
    occupied_thresh: float,
    negate: bool,
) -> str:
    """Classify a PGM pixel as free/occupied/unknown per ROS map YAML semantics.

    occupancy probability = 1 - pixel/255 (or pixel/255 if negate).
    > occupied_thresh → occupied; < free_thresh → free; else → unknown.
    """
    if negate:
        occ_prob = pixel / 255.0
    else:
        occ_prob = 1.0 - pixel / 255.0
    if occ_prob > occupied_thresh:
        return PIXEL_OCCUPIED
    if occ_prob < free_thresh:
        return PIXEL_FREE
    return PIXEL_UNKNOWN


def calculate_coverage(
    reference_map_path,
    slam_map_path,
) -> CoverageResult:
    reference = load_map(reference_map_path)
    slam = load_map(slam_map_path)

    if not math.isclose(
        reference.resolution,
        slam.resolution,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise ValueError(
            "reference and SLAM map resolutions differ"
        )

    if (
        abs(reference.origin_yaw) > 1e-9
        or abs(slam.origin_yaw) > 1e-9
    ):
        raise ValueError(
            "rotated map origins are not supported"
        )

    resolution = reference.resolution

    reference_free = 0
    observed_reference_free = 0
    observed_as_free = 0
    observed_as_occupied = 0
    outside_slam_canvas = 0

    for reference_row in range(reference.height):
        for reference_col in range(reference.width):
            reference_pixel = reference.data[
                reference_row * reference.width
                + reference_col
            ]

            if _classify_pixel(
                reference_pixel,
                free_thresh=reference.free_thresh,
                occupied_thresh=reference.occupied_thresh,
                negate=reference.negate,
            ) != PIXEL_FREE:
                continue

            reference_free += 1

            reference_grid_y = (
                reference.height - 1 - reference_row
            )

            world_x = (
                reference.origin_x
                + (reference_col + 0.5) * resolution
            )

            world_y = (
                reference.origin_y
                + (reference_grid_y + 0.5) * resolution
            )

            slam_col = math.floor(
                (world_x - slam.origin_x) / resolution
            )

            slam_grid_y = math.floor(
                (world_y - slam.origin_y) / resolution
            )

            if (
                slam_col < 0
                or slam_col >= slam.width
                or slam_grid_y < 0
                or slam_grid_y >= slam.height
            ):
                outside_slam_canvas += 1
                continue

            slam_row = (
                slam.height - 1 - slam_grid_y
            )

            slam_pixel = slam.data[
                slam_row * slam.width + slam_col
            ]

            slam_class = _classify_pixel(
                slam_pixel,
                free_thresh=slam.free_thresh,
                occupied_thresh=slam.occupied_thresh,
                negate=slam.negate,
            )

            if slam_class == PIXEL_UNKNOWN:
                continue

            observed_reference_free += 1

            if slam_class == PIXEL_FREE:
                observed_as_free += 1
            elif slam_class == PIXEL_OCCUPIED:
                observed_as_occupied += 1

    if reference_free == 0:
        raise ValueError(
            "reference map contains no free cells"
        )

    coverage = (
        observed_reference_free / reference_free
    )

    return CoverageResult(
        reference_free_cells=reference_free,
        observed_reference_free_cells=(
            observed_reference_free
        ),
        observed_as_free=observed_as_free,
        observed_as_occupied=observed_as_occupied,
        outside_slam_canvas=outside_slam_canvas,
        coverage=coverage,
    )
