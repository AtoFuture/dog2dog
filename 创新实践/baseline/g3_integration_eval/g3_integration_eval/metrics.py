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


NAV_STATUS_SUCCEEDED = 4
NAV_STATUS_CANCELED = 5
NAV_STATUS_ABORTED = 6
NAV_TERMINAL_STATUSES = {
    NAV_STATUS_SUCCEEDED,
    NAV_STATUS_CANCELED,
    NAV_STATUS_ABORTED,
}


@dataclass(frozen=True)
class NavigationStatistics:
    observed_goal_count: int
    succeeded: int
    aborted: int
    canceled: int
    incomplete: int
    conflict: int
    terminal_goal_count: int
    success_rate: Optional[float]
    observed_goal_ids: tuple
    conflicting_goal_ids: tuple


def navigation_statistics(observed_goal_ids, statuses_by_goal) -> NavigationStatistics:
    """Summarize NavigateToPose outcomes for goals active in this bag.

    ``GoalStatusArray`` may contain historical goals that predate the bag.  A goal
    is therefore considered part of the current observation window only when its
    UUID appears on the NavigateToPose feedback topic in the bag.

    Repeated status snapshots are de-duplicated by goal UUID.  A goal with no
    observed terminal state is ``incomplete``.  If the same UUID contains more
    than one distinct terminal state, it is marked ``conflict`` and excluded from
    the success-rate denominator.
    """
    observed = tuple(sorted(set(observed_goal_ids)))

    succeeded = 0
    aborted = 0
    canceled = 0
    incomplete = 0
    conflicts = []

    for goal_id in observed:
        terminal_states = {
            int(status)
            for status in statuses_by_goal.get(goal_id, ())
            if int(status) in NAV_TERMINAL_STATUSES
        }

        if not terminal_states:
            incomplete += 1
            continue

        if len(terminal_states) > 1:
            conflicts.append(goal_id)
            continue

        terminal_state = next(iter(terminal_states))

        if terminal_state == NAV_STATUS_SUCCEEDED:
            succeeded += 1
        elif terminal_state == NAV_STATUS_ABORTED:
            aborted += 1
        elif terminal_state == NAV_STATUS_CANCELED:
            canceled += 1

    terminal_goal_count = succeeded + aborted + canceled
    success_rate = (
        succeeded / terminal_goal_count
        if terminal_goal_count
        else None
    )

    return NavigationStatistics(
        observed_goal_count=len(observed),
        succeeded=succeeded,
        aborted=aborted,
        canceled=canceled,
        incomplete=incomplete,
        conflict=len(conflicts),
        terminal_goal_count=terminal_goal_count,
        success_rate=success_rate,
        observed_goal_ids=observed,
        conflicting_goal_ids=tuple(sorted(conflicts)),
    )


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
