"""Pure metric helpers for G3 offline experiment evaluation."""

import math
from dataclasses import dataclass
from typing import Iterable, Optional


NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True)
class BatteryStatistics:
    sample_count: int
    minimum: Optional[float]
    maximum: Optional[float]
    average: Optional[float]


def duration_seconds(start_ns: int, end_ns: int) -> float:
    if start_ns < 0 or end_ns < 0:
        raise ValueError("timestamps must be non-negative")

    if end_ns < start_ns:
        raise ValueError("end timestamp must not precede start timestamp")

    return (end_ns - start_ns) / NANOSECONDS_PER_SECOND


def is_valid_battery_percentage(value) -> bool:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return False

    return math.isfinite(numeric) and 0.0 <= numeric <= 1.0


def battery_statistics(values: Iterable[float]) -> BatteryStatistics:
    valid = [
        float(value)
        for value in values
        if is_valid_battery_percentage(value)
    ]

    if not valid:
        return BatteryStatistics(0, None, None, None)

    return BatteryStatistics(
        sample_count=len(valid),
        minimum=min(valid),
        maximum=max(valid),
        average=sum(valid) / len(valid),
    )


def availability(value, reason=None) -> dict:
    if value is None:
        result = {
            "available": False,
            "value": None,
        }
        if reason:
            result["reason"] = reason
        return result

    return {
        "available": True,
        "value": value,
    }
