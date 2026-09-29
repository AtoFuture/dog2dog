import pytest

from g3_integration_eval.sim_battery import (
    BatteryDischargeModel,
    elapsed_seconds,
    successful_return_attempt_id,
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


def test_recharge_raises_battery_to_target_without_lowering_it():
    model = BatteryDischargeModel(0.2, 0.0, 0.1)

    model.advance(1.0, moving=True)
    assert model.recharge_to(0.8) == pytest.approx(0.8)
    assert model.percentage == pytest.approx(0.8)
    assert model.recharge_to(0.5) == pytest.approx(0.8)


@pytest.mark.parametrize("target", [-0.1, 1.1, float("nan")])
def test_invalid_recharge_target_is_rejected(target):
    model = BatteryDischargeModel(0.2, 0.0, 0.1)

    with pytest.raises(ValueError):
        model.recharge_to(target)


def test_succeeded_return_event_yields_attempt_id():
    payload = '{"attempt_id":"return-1","status":"succeeded"}'

    assert successful_return_attempt_id(payload) == "return-1"


@pytest.mark.parametrize("status", ["accepted", "aborted", "preview"])
def test_non_succeeded_return_event_is_ignored(status):
    payload = f'{{"attempt_id":"return-1","status":"{status}"}}'

    assert successful_return_attempt_id(payload) is None


@pytest.mark.parametrize(
    "payload",
    ["not json", "[]", '{"status":"succeeded"}'],
)
def test_invalid_return_event_is_rejected(payload):
    with pytest.raises(ValueError):
        successful_return_attempt_id(payload)
