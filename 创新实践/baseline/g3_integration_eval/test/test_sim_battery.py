import pytest

from g3_integration_eval.sim_battery import (
    BatteryDischargeModel,
    elapsed_seconds,
    twist_is_moving,
)


def test_idle_and_moving_use_different_drain_rates():
    model = BatteryDischargeModel(0.8, 0.001, 0.01)

    assert model.advance(10.0, moving=False) == pytest.approx(0.79)
    assert model.advance(10.0, moving=True) == pytest.approx(0.69)


def test_percentage_is_clamped_at_zero():
    model = BatteryDischargeModel(0.1, 0.0, 0.2)

    assert model.advance(1.0, moving=True) == 0.0
    assert model.advance(10.0, moving=True) == 0.0


@pytest.mark.parametrize(
    "arguments",
    [
        (-0.1, 0.0, 0.1),
        (1.1, 0.0, 0.1),
        (0.8, -0.1, 0.1),
        (0.8, 0.0, float("nan")),
    ],
)
def test_invalid_model_parameters_are_rejected(arguments):
    with pytest.raises(ValueError):
        BatteryDischargeModel(*arguments)


def test_negative_elapsed_time_is_rejected():
    model = BatteryDischargeModel(0.8, 0.001, 0.01)

    with pytest.raises(ValueError):
        model.advance(-1.0, moving=False)


def test_twist_motion_threshold():
    assert not twist_is_moving(0.01, 0, 0, 0, 0, 0, 0.01)
    assert twist_is_moving(0.02, 0, 0, 0, 0, 0, 0.01)
    assert twist_is_moving(0, 0, 0, 0, 0, -0.02, 0.01)


def test_sim_clock_initial_jump_does_not_drain_battery():
    assert elapsed_seconds(None, 12_000_000_000_000) == 0.0
    assert elapsed_seconds(0, 12_000_000_000_000) == 0.0
    assert elapsed_seconds(12_000_000_000_000, 12_001_500_000_000) == 1.5


def test_sim_clock_backward_jump_does_not_drain_battery():
    assert elapsed_seconds(20_000_000_000, 10_000_000_000) == 0.0
