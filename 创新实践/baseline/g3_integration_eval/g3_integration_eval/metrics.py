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


RETURN_FAILURE_STATUSES = {
    "aborted",
    "canceled",
    "rejected",
    "server_unavailable",
    "send_error",
    "goal_response_error",
    "result_error",
    "unknown",
}
RETURN_TERMINAL_STATUSES = RETURN_FAILURE_STATUSES | {"succeeded"}


@dataclass(frozen=True)
class ReturnStatistics:
    event_count: int
    attempt_count: int
    preview: int
    succeeded: int
    aborted: int
    canceled: int
    rejected: int
    server_unavailable: int
    error: int
    unknown: int
    incomplete: int
    conflict: int
    terminal_attempt_count: int
    success_rate: Optional[float]
    goal_ids: tuple
    conflicting_attempt_ids: tuple


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


def return_statistics(events: Iterable[dict]) -> ReturnStatistics:
    """Summarize explicit G3 RETURN events grouped by local attempt ID.

    Preview-only attempts remain visible but do not enter the success-rate
    denominator. Once transmission is enabled, server, rejection, callback, and
    action-result failures are final failed attempts. Duplicate bag messages are
    de-duplicated, while contradictory final states are reported as conflicts.
    """

    valid_events = [event for event in events if isinstance(event, dict)]
    statuses_by_attempt = {}
    goal_ids = set()

    for index, event in enumerate(valid_events):
        attempt_id = event.get("attempt_id")
        goal_id = event.get("goal_id")
        status = event.get("status")

        if not isinstance(attempt_id, str) or not attempt_id:
            if isinstance(goal_id, str) and goal_id:
                attempt_id = f"goal:{goal_id}"
            else:
                attempt_id = f"unidentified:{index}"

        if isinstance(status, str) and status:
            statuses_by_attempt.setdefault(attempt_id, set()).add(status)
        else:
            statuses_by_attempt.setdefault(attempt_id, set())

        if isinstance(goal_id, str) and goal_id:
            goal_ids.add(goal_id)

    counts = {
        "preview": 0,
        "succeeded": 0,
        "aborted": 0,
        "canceled": 0,
        "rejected": 0,
        "server_unavailable": 0,
        "error": 0,
        "unknown": 0,
        "incomplete": 0,
    }
    conflicts = []

    for attempt_id, statuses in statuses_by_attempt.items():
        terminal = statuses & RETURN_TERMINAL_STATUSES

        if len(terminal) > 1:
            conflicts.append(attempt_id)
            continue

        if terminal:
            status = next(iter(terminal))
            if status in {"send_error", "goal_response_error", "result_error"}:
                counts["error"] += 1
            else:
                counts[status] += 1
        elif "preview" in statuses:
            counts["preview"] += 1
        else:
            counts["incomplete"] += 1

    terminal_attempt_count = (
        counts["succeeded"]
        + counts["aborted"]
        + counts["canceled"]
        + counts["rejected"]
        + counts["server_unavailable"]
        + counts["error"]
        + counts["unknown"]
    )
    success_rate = (
        counts["succeeded"] / terminal_attempt_count
        if terminal_attempt_count
        else None
    )

    return ReturnStatistics(
        event_count=len(valid_events),
        attempt_count=len(statuses_by_attempt),
        preview=counts["preview"],
        succeeded=counts["succeeded"],
        aborted=counts["aborted"],
        canceled=counts["canceled"],
        rejected=counts["rejected"],
        server_unavailable=counts["server_unavailable"],
        error=counts["error"],
        unknown=counts["unknown"],
        incomplete=counts["incomplete"],
        conflict=len(conflicts),
        terminal_attempt_count=terminal_attempt_count,
        success_rate=success_rate,
        goal_ids=tuple(sorted(goal_ids)),
        conflicting_attempt_ids=tuple(sorted(conflicts)),
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
