import importlib
import sys
from types import ModuleType, SimpleNamespace

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


def test_node_clock_rollback_invalidates_old_motion_timestamp(monkeypatch):
    rclpy = ModuleType("rclpy")
    rclpy.init = lambda **_kwargs: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda: None
    rclpy_node = ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    rclpy_qos = ModuleType("rclpy.qos")
    rclpy_qos.qos_profile_sensor_data = object()

    geometry_msgs = ModuleType("geometry_msgs")
    geometry_msgs_msg = ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = type("Twist", (), {})
    sensor_msgs = ModuleType("sensor_msgs")
    sensor_msgs_msg = ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.BatteryState = type("BatteryState", (), {})
    std_msgs = ModuleType("std_msgs")
    std_msgs_msg = ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {})

    for name, module in (
        ("rclpy", rclpy),
        ("rclpy.node", rclpy_node),
        ("rclpy.qos", rclpy_qos),
        ("geometry_msgs", geometry_msgs),
        ("geometry_msgs.msg", geometry_msgs_msg),
        ("sensor_msgs", sensor_msgs),
        ("sensor_msgs.msg", sensor_msgs_msg),
        ("std_msgs", std_msgs),
        ("std_msgs.msg", std_msgs_msg),
    ):
        monkeypatch.setitem(sys.modules, name, module)

    module_name = "g3_integration_eval.sim_battery_node"
    sys.modules.pop(module_name, None)
    node_module = importlib.import_module(module_name)
    try:
        node = object.__new__(node_module.SimulatedBatteryNode)
        clock_ns = [10_000_000_000]

        def current_time():
            return SimpleNamespace(
                nanoseconds=clock_ns[0],
                to_msg=lambda: SimpleNamespace(),
            )

        node.get_clock = lambda: SimpleNamespace(now=current_time)
        node.get_logger = lambda: SimpleNamespace(info=lambda _message: None)
        node._model = BatteryDischargeModel(0.8, 0.001, 0.01)
        node._last_update_ns = 20_000_000_000
        node._last_motion_ns = 19_900_000_000
        node._last_log_bucket = 16
        node._motion_timeout_sec = 1.0
        published_motion_states = []
        node._publish = lambda _now, *, moving: published_motion_states.append(
            moving
        )

        node._on_timer()
        assert node._last_motion_ns is None
        assert published_motion_states == [False]

        clock_ns[0] = 10_500_000_000
        node._on_timer()

        assert published_motion_states == [False, False]
        assert node._model.percentage == pytest.approx(0.7995)
    finally:
        sys.modules.pop(module_name, None)


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
