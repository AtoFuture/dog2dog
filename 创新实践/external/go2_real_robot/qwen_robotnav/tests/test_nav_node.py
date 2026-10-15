"""ROS callback regressions; no API calls, ROS graph or motion publishers."""

import threading
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('cv_bridge')
from qwen_robotnav_ros import nav_node


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(nav_node.time, 'monotonic', lambda: 100.0)
    published = []
    warnings = []
    executor = ThreadPoolExecutor(max_workers=1)
    obj = SimpleNamespace(
        _mission_allowed=True, _gate_by_mission=True,
        lock=threading.Lock(),
        frame_history=deque([{'Front': object()}]),
        latest_frame=np.zeros((2, 2, 3), dtype=np.uint8),
        _last_frame_time=100.0,
        _last_frame_stamp=None,
        max_frame_age_sec=15.0,
        _executor=executor,
        _inference_future=None,
        _inference_frame=None,
        _inference_frame_time=None,
        _inference_frame_stamp=None,
        _instruction_revision=0,
        _inference_revision=0,
        publish_debug_image=False,
        _publish_waypoints=published.append,
        get_logger=lambda: SimpleNamespace(debug=lambda *_: None, info=lambda *_: None,
                                           warning=warnings.append, error=warnings.append),
        published=published,
        warnings=warnings,
    )
    yield obj
    executor.shutdown(wait=True, cancel_futures=True)


def test_slow_inference_does_not_block_callback_or_queue_duplicate_requests(node):
    started = threading.Event()
    release = threading.Event()

    def engine(frames):
        started.set()
        assert release.wait(2)
        return np.zeros((1, 3)), '{}'

    node.engine = engine
    try:
        nav_node.NavNode._inference_callback(node)
        assert started.wait(1)
        first = node._inference_future
        assert not first.done()
        nav_node.NavNode._inference_callback(node)
        assert node._inference_future is first
        nav_node.NavNode._poll_inference(node)
        assert node.published == []
    finally:
        release.set()
    first.result(timeout=1)
    nav_node.NavNode._poll_inference(node)
    assert len(node.published) == 1
    assert node._inference_future is None


def test_stale_camera_frame_never_starts_an_api_request(node):
    node._last_frame_time = 80
    node.engine = lambda *_: pytest.fail('stale frame reached engine')
    nav_node.NavNode._inference_callback(node)
    assert node._inference_future is None


def test_late_prediction_is_not_published(node):
    node._inference_future = Future()
    node._inference_future.set_result((np.zeros((1, 3)), '{}'))
    node._inference_frame_time = 80
    nav_node.NavNode._poll_inference(node)
    assert node.published == []
    assert node.warnings
    assert node._inference_future is None


def test_timeout_frees_worker_slot_and_is_reported(node):
    node._inference_future = Future()
    node._inference_future.set_exception(TimeoutError('total deadline'))
    nav_node.NavNode._poll_inference(node)
    assert node.published == []
    assert 'total deadline' in node.warnings[0]
    assert node._inference_future is None
    assert node._inference_frame is None


def test_new_instruction_invalidates_inflight_prediction(node):
    node._inference_future = Future()
    node._inference_future.set_result((np.zeros((1, 3)), '{}'))
    node._inference_frame_time = 100
    node._instruction_revision = 1
    nav_node.NavNode._poll_inference(node)
    assert node.published == []
    assert 'superseded instruction' in node.warnings[0]


def test_waypoints_keep_image_acquisition_timestamp_for_nav2():
    from builtin_interfaces.msg import Time

    messages=[]
    stamp=Time(sec=42,nanosec=123)
    obj=SimpleNamespace(_inference_frame_stamp=stamp,
                        waypoint_pub=SimpleNamespace(publish=messages.append),
                        get_clock=lambda:pytest.fail('Publication time replaced image time'))
    nav_node.NavNode._publish_waypoints(obj,np.zeros((1,3)))
    assert messages[0].header.stamp==stamp


def test_return_suspends_requests_and_invalidates_inflight_prediction(node):
    node._inference_future = Future()
    node._inference_future.set_result((np.zeros((1,3)), '{}'))
    node._inference_frame_time=100
    nav_node.NavNode._mission_callback(node,SimpleNamespace(data='returning'))
    nav_node.NavNode._poll_inference(node)
    assert node.published==[]
    node.engine=lambda *_:pytest.fail('RETURN started an unnecessary API request')
    nav_node.NavNode._inference_callback(node)
    assert node._inference_future is None
    nav_node.NavNode._mission_callback(node,SimpleNamespace(data='exploring'))
    assert node._mission_allowed
