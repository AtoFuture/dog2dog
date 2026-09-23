import math

import pytest

from g3_integration_eval.metrics import (
    availability,
    battery_statistics,
    duration_seconds,
    is_valid_battery_percentage,
)


def test_duration_seconds():
    assert duration_seconds(1_000_000_000, 10_500_000_000) == pytest.approx(9.5)


def test_zero_duration():
    assert duration_seconds(123, 123) == 0.0


@pytest.mark.parametrize(
    "start_ns,end_ns",
    [(-1, 10), (10, -1), (20, 10)],
)
def test_invalid_duration_rejected(start_ns, end_ns):
    with pytest.raises(ValueError):
        duration_seconds(start_ns, end_ns)


@pytest.mark.parametrize("value", [0.0, 0.2, 0.8, 1.0])
def test_valid_battery_percentage(value):
    assert is_valid_battery_percentage(value)


@pytest.mark.parametrize(
    "value",
    [-0.01, 1.01, math.nan, math.inf, -math.inf, None, "invalid"],
)
def test_invalid_battery_percentage(value):
    assert not is_valid_battery_percentage(value)


def test_battery_statistics():
    result = battery_statistics([0.8, 0.7, 0.6])

    assert result.sample_count == 3
    assert result.minimum == pytest.approx(0.6)
    assert result.maximum == pytest.approx(0.8)
    assert result.average == pytest.approx(0.7)


def test_battery_statistics_ignores_invalid_values():
    result = battery_statistics([0.8, math.nan, -0.1, 1.1, 0.6])

    assert result.sample_count == 2
    assert result.minimum == pytest.approx(0.6)
    assert result.maximum == pytest.approx(0.8)
    assert result.average == pytest.approx(0.7)


def test_empty_battery_statistics():
    result = battery_statistics([])

    assert result.sample_count == 0
    assert result.minimum is None
    assert result.maximum is None
    assert result.average is None


def test_available_metric():
    assert availability(9.46) == {
        "available": True,
        "value": 9.46,
    }


def test_unavailable_metric_has_reason():
    assert availability(None, "ground truth not provided") == {
        "available": False,
        "value": None,
        "reason": "ground truth not provided",
    }
