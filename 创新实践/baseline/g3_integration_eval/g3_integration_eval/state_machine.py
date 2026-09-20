"""Pure, ROS-independent state and low-battery transition rules for G3."""

from enum import Enum
from math import isfinite
from typing import Optional


class State(str, Enum):
    """Top-level G3 operating states."""

    STANDBY = "STANDBY"
    EXPLORE = "EXPLORE"
    RETURN = "RETURN"
    ESTOP = "ESTOP"


class Event(str, Enum):
    """Commands accepted by the state machine."""

    STANDBY = "standby"
    EXPLORE = "explore"
    RETURN = "return"
    ESTOP = "estop"
    RESET = "reset"


class InvalidTransition(ValueError):
    """Raised when an event is unsafe or undefined for the current state."""


_TRANSITIONS = {
    (State.STANDBY, Event.STANDBY): State.STANDBY,
    (State.STANDBY, Event.EXPLORE): State.EXPLORE,
    (State.EXPLORE, Event.EXPLORE): State.EXPLORE,
    (State.EXPLORE, Event.RETURN): State.RETURN,
    (State.EXPLORE, Event.STANDBY): State.STANDBY,
    (State.RETURN, Event.RETURN): State.RETURN,
    (State.RETURN, Event.STANDBY): State.STANDBY,
    (State.ESTOP, Event.RESET): State.STANDBY,
}


def parse_event(raw_event: str) -> Event:
    """Normalize a command string and convert it to an Event."""

    try:
        return Event(raw_event.strip().lower())
    except ValueError as exc:
        valid = ", ".join(event.value for event in Event)
        raise InvalidTransition(
            f"unknown event {raw_event!r}; expected one of: {valid}"
        ) from exc


def transition(current: State, event: Event) -> State:
    """Return the next state without causing any robot or ROS side effects."""

    if event is Event.ESTOP:
        return State.ESTOP

    try:
        return _TRANSITIONS[(current, event)]
    except KeyError as exc:
        raise InvalidTransition(
            f"event {event.value!r} is not allowed while in {current.value}"
        ) from exc


def validate_low_battery_threshold(threshold: float) -> float:
    """Validate and return a normalized battery threshold in the [0, 1] range."""

    normalized = float(threshold)
    if not isfinite(normalized) or not 0.0 <= normalized <= 1.0:
        raise ValueError(
            "low_battery_threshold must be a finite value between 0.0 and 1.0"
        )
    return normalized


def battery_return_event(
    current: State,
    percentage: float,
    threshold: float = 0.20,
) -> Optional[Event]:
    """Request RETURN once a valid battery reading is below the threshold.

    Only EXPLORE may produce the event. Once RETURN or ESTOP has been entered,
    subsequent battery messages therefore cannot retrigger or override it.
    BatteryState uses NaN for an unknown percentage, so non-finite and
    out-of-range readings are deliberately ignored.
    """

    normalized_threshold = validate_low_battery_threshold(threshold)
    normalized_percentage = float(percentage)

    if current is not State.EXPLORE:
        return None
    if not isfinite(normalized_percentage):
        return None
    if not 0.0 <= normalized_percentage <= 1.0:
        return None
    if normalized_percentage < normalized_threshold:
        return Event.RETURN
    return None
