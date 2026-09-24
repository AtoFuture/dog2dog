import pytest

from g3_integration_eval.battery_adapter import unitree_soc_to_percentage


@pytest.mark.parametrize(
    ("soc", "expected"),
    [(0, 0.0), (1, 0.01), (20, 0.20), (50, 0.50), (100, 1.0)],
)
def test_unitree_soc_is_converted_to_ros_percentage(soc, expected):
    assert unitree_soc_to_percentage(soc) == pytest.approx(expected)


@pytest.mark.parametrize(
    "soc",
    [None, "20", True, float("nan"), float("inf"), -1, 101],
)
def test_invalid_unitree_soc_is_rejected(soc):
    with pytest.raises(ValueError):
        unitree_soc_to_percentage(soc)
