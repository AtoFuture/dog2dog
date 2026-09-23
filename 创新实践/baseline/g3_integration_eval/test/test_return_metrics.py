import pytest

from g3_integration_eval.metrics import return_statistics


def event(attempt_id, status, goal_id=None):
    return {
        "attempt_id": attempt_id,
        "status": status,
        "goal_id": goal_id,
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
