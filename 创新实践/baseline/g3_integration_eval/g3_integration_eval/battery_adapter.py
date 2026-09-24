"""Pure conversion helpers for Unitree battery telemetry."""

from math import isfinite


def unitree_soc_to_percentage(soc) -> float:
    """Convert Unitree BMS SOC (0..100) to BatteryState percentage (0..1)."""

    if isinstance(soc, bool) or not isinstance(soc, (int, float)):
        raise ValueError("Unitree battery SOC must be numeric")

    value = float(soc)
    if not isfinite(value) or not 0.0 <= value <= 100.0:
        raise ValueError("Unitree battery SOC must be in the range [0, 100]")
    return value / 100.0
