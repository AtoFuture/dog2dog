"""ROS-independent battery discharge model used by the simulator node."""

from math import isfinite


def _finite_number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    normalized = float(value)
    if not isfinite(normalized):
        raise ValueError(f"{name} must be finite")
    return normalized


def validate_percentage(value, name: str = "percentage") -> float:
    normalized = _finite_number(value, name)
    if not 0.0 <= normalized <= 1.0:
        raise ValueError(f"{name} must be in the range [0, 1]")
    return normalized


def validate_nonnegative(value, name: str) -> float:
    normalized = _finite_number(value, name)
    if normalized < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return normalized


def validate_positive(value, name: str) -> float:
    normalized = _finite_number(value, name)
    if normalized <= 0.0:
        raise ValueError(f"{name} must be positive")
    return normalized


def twist_is_moving(
    linear_x: float,
    linear_y: float,
    linear_z: float,
    angular_x: float,
    angular_y: float,
    angular_z: float,
    epsilon: float,
) -> bool:
    threshold = validate_nonnegative(epsilon, "motion_epsilon")
    components = (
        linear_x,
        linear_y,
        linear_z,
        angular_x,
        angular_y,
        angular_z,
    )
    return any(
        abs(_finite_number(value, "twist component")) > threshold
        for value in components
    )


def elapsed_seconds(previous_ns, current_ns: int) -> float:
    """Return safe elapsed time across initial/sim-clock discontinuities."""
    if isinstance(current_ns, bool) or not isinstance(current_ns, int):
        raise ValueError("current_ns must be an integer")
    if previous_ns is None or current_ns <= 0:
        return 0.0
    if isinstance(previous_ns, bool) or not isinstance(previous_ns, int):
        raise ValueError("previous_ns must be an integer or None")
    if previous_ns <= 0 or current_ns < previous_ns:
        return 0.0
    return (current_ns - previous_ns) / 1e9


class BatteryDischargeModel:
    """Decrease battery percentage using idle or moving drain rates."""

    def __init__(
        self,
        initial_percentage: float,
        idle_drain_per_sec: float,
        moving_drain_per_sec: float,
    ) -> None:
        self._percentage = validate_percentage(
            initial_percentage,
            "initial_percentage",
        )
        self._idle_drain_per_sec = validate_nonnegative(
            idle_drain_per_sec,
            "idle_drain_per_sec",
        )
        self._moving_drain_per_sec = validate_nonnegative(
            moving_drain_per_sec,
            "moving_drain_per_sec",
        )

    @property
    def percentage(self) -> float:
        return self._percentage

    def advance(self, elapsed_sec: float, *, moving: bool) -> float:
        elapsed = validate_nonnegative(elapsed_sec, "elapsed_sec")
        drain_rate = (
            self._moving_drain_per_sec
            if moving
            else self._idle_drain_per_sec
        )
        self._percentage = max(
            0.0,
            self._percentage - drain_rate * elapsed,
        )
        return self._percentage
