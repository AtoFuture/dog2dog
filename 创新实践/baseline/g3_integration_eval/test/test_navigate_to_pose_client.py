from types import SimpleNamespace

import pytest

from g3_integration_eval.navigate_to_pose_client import NavigateToPoseClient


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


class FakeClock:
    def now(self):
        stamp = SimpleNamespace(sec=123, nanosec=456)
        return SimpleNamespace(to_msg=lambda: stamp)


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()

    def get_clock(self):
        return FakeClock()

    def get_logger(self):
        return self.logger


class FakeGoal:
    def __init__(self):
        self.pose = SimpleNamespace(
            header=SimpleNamespace(frame_id="", stamp=None),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
            ),
        )


class FakeFuture:
    def __init__(self, value=None, *, error=None, complete_immediately=True):
        self._value = value
        self._error = error
        self._complete_immediately = complete_immediately
        self.callback = None

    def result(self):
        if self._error is not None:
            raise self._error
        return self._value

    def add_done_callback(self, callback):
        self.callback = callback
        if self._complete_immediately:
            callback(self)


class FakeGoalHandle:
    def __init__(self, *, accepted, result_future, uuid_bytes=bytes(range(16))):
        self.accepted = accepted
        self.goal_id = SimpleNamespace(uuid=list(uuid_bytes))
        self._result_future = result_future

    def get_result_async(self):
        return self._result_future


class FakeActionClient:
    def __init__(self, *, server_available=True, response_future=None):
        self.server_available = server_available
        self.response_future = response_future
        self.wait_timeouts = []
        self.sent_goals = []

    def wait_for_server(self, *, timeout_sec):
        self.wait_timeouts.append(timeout_sec)
        return self.server_available

    def send_goal_async(self, goal):
        self.sent_goals.append(goal)
        return self.response_future


HOME = {"frame_id": "map", "x": 1.0, "y": -2.0, "yaw": 0.5}
GOAL_ID = bytes(range(16)).hex()


def make_client(*, enabled, action_client, events):
    return NavigateToPoseClient(
        FakeNode(),
        enable_navigation_transmission=enabled,
        server_timeout_sec=1.25,
        event_callback=events.append,
        action_client=action_client,
        goal_factory=FakeGoal,
        attempt_id_factory=lambda: "attempt-1",
    )


def test_transmission_disabled_is_preview_only():
    events = []
    action_client = FakeActionClient()
    client = make_client(
        enabled=False,
        action_client=action_client,
        events=events,
    )
    goal = client.build_goal(**HOME)

    attempt_id = client.request_return(goal, trigger="battery", home_target=HOME)

    assert attempt_id == "attempt-1"
    assert action_client.wait_timeouts == []
    assert action_client.sent_goals == []
    assert [event["status"] for event in events] == ["preview"]
    assert events[0]["goal_id"] is None
    assert events[0]["trigger"] == "battery"
    assert events[0]["home_target"] == HOME
    assert events[0]["timestamp"] == {"sec": 123, "nanosec": 456}


def test_server_unavailable_does_not_send_goal():
    events = []
    action_client = FakeActionClient(server_available=False)
    client = make_client(
        enabled=True,
        action_client=action_client,
        events=events,
    )

    client.request_return(FakeGoal(), trigger="command", home_target=HOME)

    assert action_client.wait_timeouts == [1.25]
    assert action_client.sent_goals == []
    assert [event["status"] for event in events] == [
        "waiting_for_server",
        "server_unavailable",
    ]


def test_goal_rejected_records_goal_uuid():
    events = []
    result_future = FakeFuture(SimpleNamespace(status=4))
    handle = FakeGoalHandle(accepted=False, result_future=result_future)
    action_client = FakeActionClient(response_future=FakeFuture(handle))
    client = make_client(
        enabled=True,
        action_client=action_client,
        events=events,
    )

    client.request_return(FakeGoal(), trigger="command", home_target=HOME)

    assert [event["status"] for event in events] == [
        "waiting_for_server",
        "goal_request_sent",
        "rejected",
    ]
    assert events[-1]["goal_id"] == GOAL_ID


def test_goal_accepted_records_uuid_before_result():
    events = []
    result_future = FakeFuture(complete_immediately=False)
    handle = FakeGoalHandle(accepted=True, result_future=result_future)
    action_client = FakeActionClient(response_future=FakeFuture(handle))
    client = make_client(
        enabled=True,
        action_client=action_client,
        events=events,
    )

    client.request_return(FakeGoal(), trigger="battery", home_target=HOME)

    assert events[-1]["status"] == "accepted"
    assert events[-1]["goal_id"] == GOAL_ID
    assert result_future.callback is not None


@pytest.mark.parametrize(
    ("result_status", "expected"),
    [(4, "succeeded"), (5, "canceled"), (6, "aborted")],
)
def test_terminal_result_is_mapped_and_keeps_goal_uuid(result_status, expected):
    events = []
    result = SimpleNamespace(status=result_status)
    handle = FakeGoalHandle(
        accepted=True,
        result_future=FakeFuture(result),
    )
    action_client = FakeActionClient(response_future=FakeFuture(handle))
    client = make_client(
        enabled=True,
        action_client=action_client,
        events=events,
    )

    client.request_return(FakeGoal(), trigger="battery", home_target=HOME)

    assert events[-1]["status"] == expected
    assert events[-1]["goal_id"] == GOAL_ID


def test_build_goal_sets_home_pose_and_stamp():
    client = make_client(
        enabled=False,
        action_client=FakeActionClient(),
        events=[],
    )

    goal = client.build_goal(**HOME)

    assert goal.pose.header.frame_id == "map"
    assert goal.pose.header.stamp.sec == 123
    assert goal.pose.pose.position.x == pytest.approx(1.0)
    assert goal.pose.pose.position.y == pytest.approx(-2.0)
    assert goal.pose.pose.orientation.z == pytest.approx(0.2474039593)
    assert goal.pose.pose.orientation.w == pytest.approx(0.9689124217)


@pytest.mark.parametrize("timeout", [0.0, -1.0, float("nan")])
def test_invalid_server_timeout_is_rejected(timeout):
    with pytest.raises(ValueError, match="server_timeout_sec"):
        NavigateToPoseClient(
            FakeNode(),
            server_timeout_sec=timeout,
            action_client=FakeActionClient(),
            goal_factory=FakeGoal,
        )
