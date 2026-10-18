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
    preempted: int
    terminal_goal_count: int
    success_rate: Optional[float]
    observed_goal_ids: tuple
    conflicting_goal_ids: tuple
    preempted_goal_ids: tuple


RETURN_FAILURE_STATUSES = {
    "aborted",
    "canceled",
    "rejected",
    "server_unavailable",
    "send_error",
    "goal_response_error",
    "result_error",
    "localization_rejected",
    "localization_failed",
    "unknown",
}
RETURN_TERMINAL_STATUSES = RETURN_FAILURE_STATUSES | {"succeeded"}
RETURN_START_STATUSES = {"waiting_for_server"}


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


@dataclass(frozen=True)
class ReturnAttemptDuration:
    attempt_id: str
    status: Optional[str]
    available: bool
    duration_seconds: Optional[float]
    reason: Optional[str]


@dataclass(frozen=True)
class ReturnTimingStatistics:
    attempts: tuple
    successful_attempt_count: int
    measured_successful_attempt_count: int
    unavailable_successful_attempt_count: int
    average_seconds: Optional[float]
    minimum_seconds: Optional[float]
    maximum_seconds: Optional[float]
    available: bool
    reason: Optional[str]


def _return_attempt_id(event: dict, index: int) -> str:
    attempt_id = event.get("attempt_id")
    goal_id = event.get("goal_id")
    if isinstance(attempt_id, str) and attempt_id:
        return attempt_id
    if isinstance(goal_id, str) and goal_id:
        return f"goal:{goal_id}"
    return f"unidentified:{index}"


def _event_timestamp_ns(event: dict) -> Optional[int]:
    timestamp = event.get("timestamp")
    if not isinstance(timestamp, dict):
        return None
    try:
        sec = int(timestamp["sec"])
        nanosec = int(timestamp["nanosec"])
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    if sec < 0 or not 0 <= nanosec < NANOSECONDS_PER_SECOND:
        return None
    return sec * NANOSECONDS_PER_SECOND + nanosec


def return_timing_statistics(events: Iterable[dict]) -> ReturnTimingStatistics:
    """Measure RETURN attempts from their explicit start to terminal event.

    The client emits ``waiting_for_server`` before transmitting or failing a
    request. Its event timestamp is the ROS clock timestamp, so missing or
    contradictory events are reported as unavailable instead of inferred.
    """
    grouped = {}
    for index, event in enumerate(events):
        if not isinstance(event, dict):
            continue
        attempt_id = _return_attempt_id(event, index)
        grouped.setdefault(attempt_id, []).append(event)

    attempt_results = []
    successful_durations = []
    successful_attempt_count = 0

    for attempt_id, attempt_events in grouped.items():
        statuses = {
            event.get("status")
            for event in attempt_events
            if isinstance(event.get("status"), str)
            and event.get("status")
        }
        terminal_statuses = statuses & RETURN_TERMINAL_STATUSES

        if len(terminal_statuses) > 1:
            attempt_results.append(ReturnAttemptDuration(
                attempt_id,
                "conflict",
                False,
                None,
                "conflicting_terminal_events",
            ))
            continue

        terminal_status = (
            next(iter(terminal_statuses))
            if terminal_statuses
            else None
        )
        if terminal_status == "succeeded":
            successful_attempt_count += 1

        if not terminal_status and "preview" in statuses:
            attempt_results.append(ReturnAttemptDuration(
                attempt_id,
                "preview",
                False,
                None,
                "preview_attempt_not_transmitted",
            ))
            continue

        start_events = [
            event for event in attempt_events
            if event.get("status") in RETURN_START_STATUSES
        ]
        if not start_events:
            reason = (
                "start_event_missing"
                if terminal_status
                else "start_and_terminal_events_missing"
            )
            attempt_results.append(ReturnAttemptDuration(
                attempt_id, terminal_status, False, None, reason
            ))
            continue

        if terminal_status is None:
            attempt_results.append(ReturnAttemptDuration(
                attempt_id,
                "incomplete",
                False,
                None,
                "terminal_event_missing",
            ))
            continue

        start_times = [
            stamp for stamp in
            (_event_timestamp_ns(event) for event in start_events)
            if stamp is not None
        ]
        terminal_events = [
            event for event in attempt_events
            if event.get("status") == terminal_status
        ]
        terminal_times = [
            stamp for stamp in
            (_event_timestamp_ns(event) for event in terminal_events)
            if stamp is not None
        ]
        if not start_times:
            attempt_results.append(ReturnAttemptDuration(
                attempt_id,
                terminal_status,
                False,
                None,
                "start_timestamp_missing_or_invalid",
            ))
            continue
        if not terminal_times:
            attempt_results.append(ReturnAttemptDuration(
                attempt_id,
                terminal_status,
                False,
                None,
                "terminal_timestamp_missing_or_invalid",
            ))
            continue

        start_ns = min(start_times)
        terminal_ns = min(terminal_times)
        if terminal_ns < start_ns:
            attempt_results.append(ReturnAttemptDuration(
                attempt_id,
                terminal_status,
                False,
                None,
                "terminal_timestamp_precedes_start",
            ))
            continue

        elapsed = duration_seconds(start_ns, terminal_ns)
        attempt_results.append(ReturnAttemptDuration(
            attempt_id, terminal_status, True, elapsed, None
        ))
        if terminal_status == "succeeded":
            successful_durations.append(elapsed)

    if successful_durations:
        return ReturnTimingStatistics(
            attempts=tuple(attempt_results),
            successful_attempt_count=successful_attempt_count,
            measured_successful_attempt_count=len(successful_durations),
            unavailable_successful_attempt_count=(
                successful_attempt_count - len(successful_durations)
            ),
            average_seconds=sum(successful_durations) / len(successful_durations),
            minimum_seconds=min(successful_durations),
            maximum_seconds=max(successful_durations),
            available=True,
            reason=None,
        )

    if not grouped:
        reason = "no_return_events"
    elif successful_attempt_count:
        reason = "successful_attempt_durations_unavailable"
    else:
        reason = "no_successful_return_attempt_observed"

    return ReturnTimingStatistics(
        attempts=tuple(attempt_results),
        successful_attempt_count=successful_attempt_count,
        measured_successful_attempt_count=0,
        unavailable_successful_attempt_count=successful_attempt_count,
        average_seconds=None,
        minimum_seconds=None,
        maximum_seconds=None,
        available=False,
        reason=reason,
    )


def state_statistics(samples: Iterable[tuple]) -> dict:
    """Summarize state messages and observed transitions in bag timestamp order.

    Each sample is ``(state_string, bag_timestamp_ns)``. A state's dwell time
    is measured only when a later state transition bounds the interval; the
    final state in a bag is therefore reported as an incomplete interval.
    """
    valid_samples = []
    invalid_messages = 0
    for state, timestamp_ns in samples:
        if not isinstance(state, str) or not state.strip():
            invalid_messages += 1
            continue
        try:
            timestamp_ns = int(timestamp_ns)
        except (TypeError, ValueError, OverflowError):
            timestamp_ns = None
        if timestamp_ns is not None and timestamp_ns < 0:
            timestamp_ns = None
        valid_samples.append((state.strip(), timestamp_ns))

    counts = {}
    for state, _timestamp_ns in valid_samples:
        counts[state] = counts.get(state, 0) + 1

    transitions = []
    duration_samples = []
    previous_state = None
    state_entry_ns = None
    for state, timestamp_ns in valid_samples:
        if previous_state is None:
            previous_state = state
            state_entry_ns = timestamp_ns
            continue
        if state == previous_state:
            continue

        transitions.append({
            "from": previous_state,
            "to": state,
            "timestamp_ns": timestamp_ns,
        })
        if state_entry_ns is not None and timestamp_ns is not None:
            try:
                elapsed = duration_seconds(state_entry_ns, timestamp_ns)
            except ValueError:
                elapsed = None
            if elapsed is not None:
                duration_samples.append({
                    "state": previous_state,
                    "start_timestamp_ns": state_entry_ns,
                    "end_timestamp_ns": timestamp_ns,
                    "duration_seconds": elapsed,
                })
        previous_state = state
        state_entry_ns = timestamp_ns

    duration_values = {}
    for sample in duration_samples:
        duration_values.setdefault(sample["state"], []).append(
            sample["duration_seconds"]
        )
    duration_summary = {
        state: {
            "completed_intervals": len(values),
            "average_seconds": sum(values) / len(values),
            "minimum_seconds": min(values),
            "maximum_seconds": max(values),
        }
        for state, values in sorted(duration_values.items())
    }

    available = bool(valid_samples)
    return {
        "available": available,
        "message_count": len(valid_samples) + invalid_messages,
        "invalid_messages": invalid_messages,
        "state_counts": dict(sorted(counts.items())),
        "transition_count": len(transitions),
        "transitions": transitions,
        "duration_basis": "rosbag storage timestamps; only transition-bounded intervals",
        "duration_samples": duration_samples,
        "duration_summary": duration_summary,
        "final_state_interval": (
            {
                "state": previous_state,
                "start_timestamp_ns": state_entry_ns,
                "duration_seconds": None,
                "reason": "no_later_transition_in_bag",
            }
            if previous_state is not None
            else None
        ),
        **({} if available else {"reason": "no_state_messages"}),
    }


def navigation_statistics(observed_goal_ids, statuses_by_goal, return_goal_ids=()) -> NavigationStatistics:
    """Summarize NavigateToPose outcomes for goals active in this bag.

    ``GoalStatusArray`` may contain historical goals that predate the bag.  A goal
    is therefore considered part of the current observation window only when its
    UUID appears on the NavigateToPose feedback topic in the bag.

    Repeated status snapshots are de-duplicated by goal UUID.  A goal with no
    observed terminal state is ``incomplete``.  If the same UUID contains more
    than one distinct terminal state, it is marked ``conflict`` and excluded from
    the success-rate denominator.

    Goals whose UUIDs appear in ``return_goal_ids`` are RETURN navigation goals
    (or goals preempted by RETURN).  They are excluded from the exploration
    success-rate denominator and counted as ``preempted`` instead of
    ``aborted``/``canceled``.
    """
    observed = tuple(sorted(set(observed_goal_ids)))
    return_ids = set(return_goal_ids)

    succeeded = 0
    aborted = 0
    canceled = 0
    incomplete = 0
    conflicts = []
    preempted_ids = []

    for goal_id in observed:
        terminal_states = {
            int(status)
            for status in statuses_by_goal.get(goal_id, ())
            if int(status) in NAV_TERMINAL_STATUSES
        }

        if not terminal_states:
            if goal_id in return_ids:
                preempted_ids.append(goal_id)
                continue
            incomplete += 1
            continue

        if len(terminal_states) > 1:
            conflicts.append(goal_id)
            continue

        terminal_state = next(iter(terminal_states))

        if goal_id in return_ids:
            preempted_ids.append(goal_id)
            continue

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
        preempted=len(preempted_ids),
        terminal_goal_count=terminal_goal_count,
        success_rate=success_rate,
        observed_goal_ids=observed,
        conflicting_goal_ids=tuple(sorted(conflicts)),
        preempted_goal_ids=tuple(sorted(preempted_ids)),
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
            if status in {"send_error", "goal_response_error", "result_error", "localization_failed"}:
                counts["error"] += 1
            elif status == "localization_rejected":
                counts["rejected"] += 1
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


@dataclass(frozen=True)
class DetectionMetrics:
    ground_truth_count: int
    detection_count: int
    true_positives: int
    false_positives: int
    false_negatives: int
    detection_rate: Optional[float]
    false_positive_rate: Optional[float]


def detection_metrics(
    ground_truth: list,
    detections: list,
    match_radius_m: float = 0.5,
    match_time_s: float = 1.0,
) -> DetectionMetrics:
    """Compute detection rate and false-positive rate.

    Each item is a dict with keys: ``t_sec``, ``class``, ``x``, ``y``, ``z``.
    A ground-truth detection is a true positive if an actual detection of the
    same class exists within ``match_radius_m`` (Euclidean in 3-D) and
    ``match_time_s`` (absolute time difference).  Greedy nearest-first matching.
    """
    gt_count = len(ground_truth)
    det_count = len(detections)

    if gt_count == 0 and det_count == 0:
        return DetectionMetrics(0, 0, 0, 0, 0, None, None)

    matched_detections = set()
    true_positives = 0

    for gt in ground_truth:
        best_idx = -1
        best_dist = float("inf")
        for i, det in enumerate(detections):
            if i in matched_detections:
                continue
            if det["class"] != gt["class"]:
                continue
            dt = abs(det["t_sec"] - gt["t_sec"])
            if dt > match_time_s:
                continue
            dx = det["x"] - gt["x"]
            dy = det["y"] - gt["y"]
            dz = det["z"] - gt["z"]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist > match_radius_m:
                continue
            if dist < best_dist:
                best_dist = dist
                best_idx = i
        if best_idx >= 0:
            matched_detections.add(best_idx)
            true_positives += 1

    false_negatives = gt_count - true_positives
    false_positives = det_count - true_positives

    detection_rate = (
        true_positives / gt_count if gt_count > 0 else None
    )
    false_positive_rate = (
        false_positives / det_count if det_count > 0 else None
    )

    return DetectionMetrics(
        ground_truth_count=gt_count,
        detection_count=det_count,
        true_positives=true_positives,
        false_positives=false_positives,
        false_negatives=false_negatives,
        detection_rate=detection_rate,
        false_positive_rate=false_positive_rate,
    )
