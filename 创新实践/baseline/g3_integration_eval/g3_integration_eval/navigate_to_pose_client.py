"""Safety-gated NavigateToPose client used by the G3 RETURN flow."""

from math import cos, isfinite, sin
from typing import Callable, Optional
from uuid import uuid4

try:
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
except ImportError:  # Allows dependency-injected unit tests outside ROS 2.
    NavigateToPose = None
    ActionClient = None


NAV_STATUS_SUCCEEDED = 4
NAV_STATUS_CANCELED = 5
NAV_STATUS_ABORTED = 6

RESULT_STATUS_NAMES = {
    NAV_STATUS_SUCCEEDED: "succeeded",
    NAV_STATUS_CANCELED: "canceled",
    NAV_STATUS_ABORTED: "aborted",
}


class NavigationTransmissionDisabled(RuntimeError):
    """Retained for callers that explicitly require a transmission interlock."""


class NavigateToPoseClient:
    """Preview or asynchronously send one safety-gated RETURN goal.

    ``action_client`` and ``goal_factory`` are injectable so all branches can be
    tested without a running ROS graph or Nav2 action server.
    """

    def __init__(
        self,
        node,
        action_name: str = "/robot1/navigate_to_pose",
        *,
        enable_navigation_transmission: bool = False,
        server_timeout_sec: float = 2.0,
        event_callback: Optional[Callable[[dict], None]] = None,
        action_client=None,
        goal_factory=None,
        attempt_id_factory: Optional[Callable[[], str]] = None,
    ) -> None:
        timeout = float(server_timeout_sec)
        if not isfinite(timeout) or timeout <= 0.0:
            raise ValueError("server_timeout_sec must be finite and greater than zero")

        self._node = node
        self._action_name = action_name
        self._transmission_enabled = bool(enable_navigation_transmission)
        self._server_timeout_sec = timeout
        self._event_callback = event_callback or (lambda _event: None)
        self._attempt_id_factory = attempt_id_factory or (lambda: uuid4().hex)

        if goal_factory is None:
            if NavigateToPose is None:
                raise RuntimeError(
                    "nav2_msgs is required unless goal_factory is provided"
                )
            goal_factory = NavigateToPose.Goal
        self._goal_factory = goal_factory

        if action_client is None:
            if ActionClient is None or NavigateToPose is None:
                raise RuntimeError(
                    "rclpy and nav2_msgs are required without action_client"
                )
            action_client = ActionClient(node, NavigateToPose, action_name)
        self._client = action_client

    @property
    def action_name(self) -> str:
        return self._action_name

    @property
    def transmission_enabled(self) -> bool:
        return self._transmission_enabled

    def build_goal(
        self,
        *,
        frame_id: str,
        x: float,
        y: float,
        yaw: float,
    ):
        """Construct a stamped home goal locally without transmitting it."""

        goal = self._goal_factory()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = sin(yaw / 2.0)
        goal.pose.pose.orientation.w = cos(yaw / 2.0)
        return goal

    def preview_goal(self, goal) -> None:
        """Log a goal preview without waiting for a server or sending a request."""

        pose = goal.pose.pose
        self._node.get_logger().warning(
            "NAVIGATION TRANSMISSION DISABLED: preview only; "
            f"action={self._action_name}, frame={goal.pose.header.frame_id}, "
            f"x={pose.position.x:.3f}, y={pose.position.y:.3f}, "
            f"qz={pose.orientation.z:.3f}, qw={pose.orientation.w:.3f}"
        )

    def request_return(self, goal, *, trigger: str, home_target: dict) -> str:
        """Preview or begin transmitting a RETURN goal and return its attempt ID."""

        context = {
            "attempt_id": str(self._attempt_id_factory()),
            "trigger": str(trigger),
            "home_target": dict(home_target),
        }

        if not self._transmission_enabled:
            self.preview_goal(goal)
            self._emit(context, "preview")
            return context["attempt_id"]

        self._emit(context, "waiting_for_server")
        try:
            server_available = self._client.wait_for_server(
                timeout_sec=self._server_timeout_sec
            )
        except Exception as exc:  # ROS middleware failures must become bag events.
            self._emit(context, "server_unavailable", detail=str(exc))
            return context["attempt_id"]

        if not server_available:
            self._emit(context, "server_unavailable")
            return context["attempt_id"]

        try:
            response_future = self._client.send_goal_async(goal)
        except Exception as exc:
            self._emit(context, "send_error", detail=str(exc))
            return context["attempt_id"]

        self._emit(context, "goal_request_sent")
        response_future.add_done_callback(
            lambda future: self._on_goal_response(future, context)
        )
        return context["attempt_id"]

    def _on_goal_response(self, future, context: dict) -> None:
        try:
            goal_handle = future.result()
            goal_id = self._goal_id(goal_handle)
        except Exception as exc:
            self._emit(context, "goal_response_error", detail=str(exc))
            return

        if not goal_handle.accepted:
            self._emit(context, "rejected", goal_id=goal_id)
            return

        self._emit(context, "accepted", goal_id=goal_id)
        try:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda result: self._on_result(result, context, goal_id)
            )
        except Exception as exc:
            self._emit(context, "result_error", goal_id=goal_id, detail=str(exc))

    def _on_result(self, future, context: dict, goal_id: Optional[str]) -> None:
        try:
            result_response = future.result()
            status = RESULT_STATUS_NAMES.get(int(result_response.status), "unknown")
            self._emit(context, status, goal_id=goal_id)
        except Exception as exc:
            self._emit(context, "result_error", goal_id=goal_id, detail=str(exc))

    @staticmethod
    def _goal_id(goal_handle) -> Optional[str]:
        try:
            return bytes(goal_handle.goal_id.uuid).hex()
        except (AttributeError, TypeError, ValueError):
            return None

    def _emit(
        self,
        context: dict,
        status: str,
        *,
        goal_id: Optional[str] = None,
        detail: Optional[str] = None,
    ) -> None:
        stamp = self._node.get_clock().now().to_msg()
        event = {
            "schema_version": 1,
            "attempt_id": context["attempt_id"],
            "action_name": self._action_name,
            "goal_id": goal_id,
            "trigger": context["trigger"],
            "timestamp": {
                "sec": int(stamp.sec),
                "nanosec": int(stamp.nanosec),
            },
            "home_target": dict(context["home_target"]),
            "status": status,
        }
        if detail:
            event["detail"] = detail
        self._event_callback(event)


# Backward-compatible import name used by the previous scaffold.
NavigateToPoseClientSkeleton = NavigateToPoseClient
