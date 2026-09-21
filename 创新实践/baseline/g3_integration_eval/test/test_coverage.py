from pathlib import Path

import pytest
import yaml

from g3_integration_eval.coverage import calculate_coverage


def write_map(
    root: Path,
    name: str,
    width: int,
    height: int,
    pixels,
    resolution=0.05,
    origin=(0.0, 0.0, 0.0),
):
    pgm_path = root / f"{name}.pgm"
    yaml_path = root / f"{name}.yaml"

    with pgm_path.open("wb") as file:
        file.write(b"P5\n")
        file.write(f"{width} {height}\n".encode())
        file.write(b"255\n")
        file.write(bytes(pixels))

    yaml_path.write_text(
        yaml.safe_dump(
            {
                "image": pgm_path.name,
                "mode": "trinary",
                "resolution": resolution,
                "origin": list(origin),
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
            }
        ),
        encoding="utf-8",
    )

    return yaml_path


def test_identical_maps_have_full_coverage(tmp_path):
    reference = write_map(
        tmp_path,
        "reference",
        2,
        2,
        [254, 254, 254, 254],
    )

    slam = write_map(
        tmp_path,
        "slam",
        2,
        2,
        [254, 254, 254, 254],
    )

    result = calculate_coverage(reference, slam)

    assert result.reference_free_cells == 4
    assert result.observed_reference_free_cells == 4
    assert result.coverage == pytest.approx(1.0)


def test_unknown_cells_reduce_coverage(tmp_path):
    reference = write_map(
        tmp_path,
        "reference",
        2,
        2,
        [254, 254, 254, 254],
    )

    slam = write_map(
        tmp_path,
        "slam",
        2,
        2,
        [254, 205, 254, 205],
    )

    result = calculate_coverage(reference, slam)

    assert result.reference_free_cells == 4
    assert result.observed_reference_free_cells == 2
    assert result.coverage == pytest.approx(0.5)


def test_observed_occupied_cell_counts_as_observed(tmp_path):
    reference = write_map(
        tmp_path,
        "reference",
        2,
        2,
        [254, 254, 254, 254],
    )

    slam = write_map(
        tmp_path,
        "slam",
        2,
        2,
        [254, 0, 205, 205],
    )

    result = calculate_coverage(reference, slam)

    assert result.observed_reference_free_cells == 2
    assert result.observed_as_free == 1
    assert result.observed_as_occupied == 1
    assert result.coverage == pytest.approx(0.5)


def test_world_coordinate_alignment(tmp_path):
    reference = write_map(
        tmp_path,
        "reference",
        1,
        1,
        [254],
        resolution=1.0,
        origin=(0.0, 0.0, 0.0),
    )

    slam = write_map(
        tmp_path,
        "slam",
        3,
        3,
        [
            205, 205, 205,
            205, 254, 205,
            205, 205, 205,
        ],
        resolution=1.0,
        origin=(-1.0, -1.0, 0.0),
    )

    result = calculate_coverage(reference, slam)

    assert result.reference_free_cells == 1
    assert result.observed_reference_free_cells == 1
    assert result.outside_slam_canvas == 0
    assert result.coverage == pytest.approx(1.0)


def test_resolution_mismatch_is_rejected(tmp_path):
    reference = write_map(
        tmp_path,
        "reference",
        1,
        1,
        [254],
        resolution=0.05,
    )

    slam = write_map(
        tmp_path,
        "slam",
        1,
        1,
        [254],
        resolution=0.10,
    )

    with pytest.raises(ValueError, match="resolutions differ"):
        calculate_coverage(reference, slam)


@pytest.mark.parametrize(
    "reference_yaw,slam_yaw",
    [
        (0.1, 0.0),
        (0.0, 0.1),
    ],
)
def test_rotated_map_is_rejected(
    tmp_path,
    reference_yaw,
    slam_yaw,
):
    reference = write_map(
        tmp_path,
        "reference",
        1,
        1,
        [254],
        origin=(0.0, 0.0, reference_yaw),
    )

    slam = write_map(
        tmp_path,
        "slam",
        1,
        1,
        [254],
        origin=(0.0, 0.0, slam_yaw),
    )

    with pytest.raises(ValueError, match="rotated"):
        calculate_coverage(reference, slam)


def test_missing_yaml_is_rejected(tmp_path):
    with pytest.raises(FileNotFoundError):
        calculate_coverage(
            tmp_path / "missing.yaml",
            tmp_path / "also_missing.yaml",
        )


def test_missing_image_is_rejected(tmp_path):
    yaml_path = tmp_path / "broken.yaml"

    yaml_path.write_text(
        yaml.safe_dump(
            {
                "image": "missing.pgm",
                "resolution": 0.05,
                "origin": [0.0, 0.0, 0.0],
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(FileNotFoundError):
        calculate_coverage(yaml_path, yaml_path)


def test_reference_without_free_cells_is_rejected(tmp_path):
    reference = write_map(
        tmp_path,
        "reference",
        2,
        2,
        [0, 0, 205, 205],
    )

    slam = write_map(
        tmp_path,
        "slam",
        2,
        2,
        [254, 254, 254, 254],
    )

    with pytest.raises(
        ValueError,
        match="contains no free cells",
    ):
        calculate_coverage(reference, slam)
