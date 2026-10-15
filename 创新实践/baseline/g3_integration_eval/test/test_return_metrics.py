import pytest

from g3_integration_eval.metrics import (
    return_statistics,
    return_timing_statistics,
    state_statistics,
)


def event(attempt_id, status, goal_id=None):
    return {
        "attempt_id": attempt_id,
        "status": status,
        "goal_id": goal_id,
    }


def timed_event(attempt_id, status, timestamp_seconds):
    sec = int(timestamp_seconds)
    nanosec = int((timestamp_seconds - sec) * 1_000_000_000)
    return {
        "attempt_id": attempt_id,
        "status": status,
        "timestamp": {"sec": sec, "nanosec": nanosec},
    }


def test_no_return_events_has_no_success_rate():
    result = return_statistics([])

    assert result.attempt_count == 0
    assert result.terminal_attempt_count == 0
    assert result.success_rate is None


def test_preview_is_not_counted_as_a_transmitted_outcome():
    result = return_statistics([event("preview-1", "preview")])

    assert result.preview == 1
    assert result.terminal_attempt_count == 0
    assert result.success_rate is None


def test_return_rate_includes_all_enabled_terminal_failures():
    events = [
        event("a", "accepted", "goal-a"),
        event("a", "succeeded", "goal-a"),
        event("b", "rejected", "goal-b"),
        event("c", "server_unavailable"),
        event("d", "aborted", "goal-d"),
    ]

    result = return_statistics(events)

    assert result.succeeded == 1
    assert result.rejected == 1
    assert result.server_unavailable == 1
    assert result.aborted == 1
    assert result.terminal_attempt_count == 4
    assert result.success_rate == pytest.approx(0.25)
    assert result.goal_ids == ("goal-a", "goal-b", "goal-d")


def test_duplicate_events_do_not_duplicate_an_attempt():
    result = return_statistics(
        [
            event("a", "accepted", "goal-a"),
            event("a", "succeeded", "goal-a"),
            event("a", "succeeded", "goal-a"),
        ]
    )

    assert result.attempt_count == 1
    assert result.succeeded == 1
    assert result.success_rate == pytest.approx(1.0)


def test_accepted_without_result_is_incomplete():
    result = return_statistics([event("a", "accepted", "goal-a")])

    assert result.incomplete == 1
    assert result.terminal_attempt_count == 0
    assert result.success_rate is None


def test_conflicting_terminal_events_are_excluded():
    result = return_statistics(
        [
            event("a", "succeeded", "goal-a"),
            event("a", "aborted", "goal-a"),
        ]
    )

    assert result.conflict == 1
    assert result.conflicting_attempt_ids == ("a",)
    assert result.terminal_attempt_count == 0
    assert result.success_rate is None


@pytest.mark.parametrize(
    "status",
    ["send_error", "goal_response_error", "result_error"],
)
def test_callback_and_send_errors_are_failed_attempts(status):
    result = return_statistics([event("a", status)])

    assert result.error == 1
    assert result.terminal_attempt_count == 1
    assert result.success_rate == pytest.approx(0.0)


def test_succeeded_return_duration_uses_start_and_terminal_events():
    result = return_timing_statistics([
        timed_event("a", "waiting_for_server", 10.0),
        timed_event("a", "accepted", 10.5),
        timed_event("a", "succeeded", 14.25),
    ])

    assert result.available
    assert result.attempts[0].status == "succeeded"
    assert result.attempts[0].duration_seconds == pytest.approx(4.25)
    assert result.average_seconds == pytest.approx(4.25)
    assert result.minimum_seconds == pytest.approx(4.25)
    assert result.maximum_seconds == pytest.approx(4.25)


def test_aborted_return_duration_is_reported_but_not_in_success_summary():
    result = return_timing_statistics([
        timed_event("a", "waiting_for_server", 1.0),
        timed_event("a", "aborted", 4.0),
    ])

    assert result.attempts[0].duration_seconds == pytest.approx(3.0)
    assert not result.available
    assert result.successful_attempt_count == 0
    assert result.reason == "no_successful_return_attempt_observed"


def test_multiple_attempts_aggregate_only_successful_durations():
    result = return_timing_statistics([
        timed_event("success-a", "waiting_for_server", 10.0),
        timed_event("success-a", "succeeded", 12.0),
        timed_event("failed", "waiting_for_server", 20.0),
        timed_event("failed", "aborted", 25.0),
        timed_event("success-b", "waiting_for_server", 30.0),
        timed_event("success-b", "succeeded", 34.0),
    ])

    assert len(result.attempts) == 3
    assert result.successful_attempt_count == 2
    assert result.measured_successful_attempt_count == 2
    assert result.average_seconds == pytest.approx(3.0)
    assert result.minimum_seconds == pytest.approx(2.0)
    assert result.maximum_seconds == pytest.approx(4.0)


def test_terminal_event_without_start_is_unavailable():
    result = return_timing_statistics([
        timed_event("a", "succeeded", 12.0),
    ])

    assert not result.available
    assert result.attempts[0].reason == "start_event_missing"
    assert result.reason == "successful_attempt_durations_unavailable"


def test_start_without_terminal_is_unavailable():
    result = return_timing_statistics([
        timed_event("a", "waiting_for_server", 12.0),
    ])

    assert not result.available
    assert result.attempts[0].status == "incomplete"
    assert result.attempts[0].reason == "terminal_event_missing"


def test_no_return_events_is_unavailable():
    result = return_timing_statistics([])

    assert not result.available
    assert result.attempts == ()
    assert result.reason == "no_return_events"


def test_state_statistics_count_states_transitions_and_complete_durations():
    result = state_statistics([
        ("EXPLORE", 0),
        ("EXPLORE", 1_000_000_000),
        ("RETURN", 4_000_000_000),
        ("STANDBY", 7_000_000_000),
    ])

    assert result["state_counts"] == {
        "EXPLORE": 2,
        "RETURN": 1,
        "STANDBY": 1,
    }
    assert result["transition_count"] == 2
    assert result["transitions"] == [
        {"from": "EXPLORE", "to": "RETURN", "timestamp_ns": 4_000_000_000},
        {"from": "RETURN", "to": "STANDBY", "timestamp_ns": 7_000_000_000},
    ]
    assert result["duration_summary"]["EXPLORE"]["average_seconds"] == 4.0
    assert result["duration_summary"]["RETURN"]["average_seconds"] == 3.0
    assert result["final_state_interval"]["state"] == "STANDBY"
