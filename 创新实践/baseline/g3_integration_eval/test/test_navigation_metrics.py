import pytest

from g3_integration_eval.metrics import navigation_statistics


def test_historical_status_goals_are_excluded_without_feedback():
    result = navigation_statistics(
        {"active"},
        {
            "old-1": [6],
            "old-2": [6],
            "active": [2],
        },
    )

    assert result.observed_goal_count == 1
    assert result.aborted == 0
    assert result.incomplete == 1
    assert result.terminal_goal_count == 0
    assert result.success_rate is None


def test_active_executing_goal_is_incomplete():
    result = navigation_statistics(
        {"goal-a"},
        {"goal-a": [1, 2, 2]},
    )

    assert result.incomplete == 1
    assert result.succeeded == 0
    assert result.success_rate is None


def test_succeeded_goal_produces_full_success_rate():
    result = navigation_statistics(
        {"goal-a"},
        {"goal-a": [1, 2, 4, 4]},
    )

    assert result.succeeded == 1
    assert result.terminal_goal_count == 1
    assert result.success_rate == pytest.approx(1.0)


def test_success_rate_uses_only_terminal_observed_goals():
    result = navigation_statistics(
        {"success", "abort", "cancel", "running"},
        {
            "success": [2, 4],
            "abort": [2, 6],
            "cancel": [2, 3, 5],
            "running": [2],
            "historical": [4],
        },
    )

    assert result.observed_goal_count == 4
    assert result.succeeded == 1
    assert result.aborted == 1
    assert result.canceled == 1
    assert result.incomplete == 1
    assert result.terminal_goal_count == 3
    assert result.success_rate == pytest.approx(1 / 3)


def test_conflicting_terminal_states_are_excluded_from_rate():
    result = navigation_statistics(
        {"conflict", "success"},
        {
            "conflict": [2, 4, 6],
            "success": [2, 4],
        },
    )

    assert result.conflict == 1
    assert result.conflicting_goal_ids == ("conflict",)
    assert result.terminal_goal_count == 1
    assert result.success_rate == pytest.approx(1.0)


def test_observed_goal_without_status_is_incomplete():
    result = navigation_statistics(
        {"feedback-only"},
        {},
    )

    assert result.incomplete == 1
    assert result.success_rate is None


def test_duplicate_feedback_goal_ids_count_once():
    result = navigation_statistics(
        ["goal-a", "goal-a", "goal-a"],
        {"goal-a": [4]},
    )

    assert result.observed_goal_count == 1
    assert result.succeeded == 1
