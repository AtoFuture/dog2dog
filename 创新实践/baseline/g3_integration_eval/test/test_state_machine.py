import pytest

from g3_integration_eval.state_machine import (
    Event,
    InvalidTransition,
    State,
    battery_return_event,
    mission_state_value,
    normalize_battery_percentage,
    parse_event,
    transition,
    validate_low_battery_threshold,
)


@pytest.mark.parametrize(
    ("state", "expected"),
    [
        (State.STANDBY, "standby"),
        (State.EXPLORE, "exploring"),
        (State.RETURN, "returning"),
        (State.ESTOP, "estop"),
    ],
)
def test_mission_state_mapping_matches_g2_contract(state, expected):
    assert mission_state_value(state) == expected


def test_nominal_mission_cycle():
    state = State.STANDBY
    state = transition(state, Event.EXPLORE)
    assert state is State.EXPLORE
    state = transition(state, Event.RETURN)
    assert state is State.RETURN
    state = transition(state, Event.STANDBY)
    assert state is State.STANDBY


@pytest.mark.parametrize("state", list(State))
def test_estop_is_reachable_from_every_state(state):
    assert transition(state, Event.ESTOP) is State.ESTOP


def test_estop_requires_reset_before_normal_operation():
    with pytest.raises(InvalidTransition):
        transition(State.ESTOP, Event.EXPLORE)
    assert transition(State.ESTOP, Event.RESET) is State.STANDBY


def test_return_cannot_jump_directly_to_explore():
    with pytest.raises(InvalidTransition):
        transition(State.RETURN, Event.EXPLORE)


def test_command_parsing_is_whitespace_and_case_tolerant():
    assert parse_event("  ReTuRn ") is Event.RETURN


def test_unknown_command_is_rejected():
    with pytest.raises(InvalidTransition):
        parse_event("go_home_now")


@pytest.mark.parametrize(
    ("percentage", "expected"),
    [
        (0.19, Event.RETURN),
        (0.20, None),
        (0.21, None),
    ],
)
def test_low_battery_threshold_is_strictly_below(percentage, expected):
    assert battery_return_event(State.EXPLORE, percentage, 0.20) is expected


@pytest.mark.parametrize(
    ("percentage", "expected"),
    [
        (0.0, 0.0),
        (0.85, 0.85),
        (1.0, 1.0),
        (15.0, 0.15),
        (85.0, 0.85),
        (100.0, 1.0),
    ],
)
def test_battery_percentage_accepts_normalized_and_whole_percent_scales(
    percentage, expected
):
    assert normalize_battery_percentage(percentage) == pytest.approx(expected)


@pytest.mark.parametrize(
    ("percentage", "expected"),
    [(19.0, Event.RETURN), (20.0, None), (85.0, None)],
)
def test_whole_percent_battery_readings_use_normalized_threshold(
    percentage, expected
):
    assert battery_return_event(State.EXPLORE, percentage, 0.20) is expected


@pytest.mark.parametrize(
    "state",
    [State.STANDBY, State.RETURN, State.ESTOP],
)
def test_low_battery_only_triggers_while_exploring(state):
    assert battery_return_event(state, 0.01, 0.20) is None


def test_repeated_low_battery_readings_trigger_return_only_once():
    state = State.EXPLORE
    trigger_count = 0

    for percentage in (0.19, 0.18, 0.17):
        event = battery_return_event(state, percentage, 0.20)
        if event is not None:
            trigger_count += 1
            state = transition(state, event)

    assert state is State.RETURN
    assert trigger_count == 1


def test_estop_has_priority_over_low_battery_return():
    state = State.EXPLORE
    event = battery_return_event(state, 0.10, 0.20)
    assert event is Event.RETURN
    state = transition(state, event)

    state = transition(state, Event.ESTOP)
    assert state is State.ESTOP
    assert battery_return_event(state, 0.01, 0.20) is None


@pytest.mark.parametrize(
    "percentage", [float("nan"), float("inf"), -0.1, 100.1]
)
def test_unknown_or_invalid_battery_readings_are_ignored(percentage):
    assert battery_return_event(State.EXPLORE, percentage, 0.20) is None


@pytest.mark.parametrize("threshold", [float("nan"), -0.01, 1.01])
def test_invalid_battery_threshold_is_rejected(threshold):
    with pytest.raises(ValueError):
        validate_low_battery_threshold(threshold)
