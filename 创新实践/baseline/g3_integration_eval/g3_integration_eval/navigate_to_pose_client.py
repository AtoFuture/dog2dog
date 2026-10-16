"""Safety-gated NavigateToPose client used by the G3 RETURN flow."""

from math import cos, isfinite, sin
from time import monotonic
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
        monotonic_fn: Optional[Callable[[], float]] = None,
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
        self._monotonic = monotonic_fn or monotonic
        self._active_context = None

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

    @property
    def has_active_goal(self) -> bool:
        """Whether a RETURN attempt may still create or drive robot motion."""

        return self._active_context is not None

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

        if self._active_context is not None:
            raise RuntimeError("a RETURN navigation attempt is already active")

        context = {
            "attempt_id": str(self._attempt_id_factory()),
            "trigger": str(trigger),
            "home_target": dict(home_target),
            "goal": goal,
            "cancel_requested": False,
            "cancel_reason": None,
            "cancel_future": None,
            "goal_handle": None,
            "server_timer": None,
            "terminal": False,
        }

        if not self._transmission_enabled:
            self.preview_goal(goal)
            self._emit(context, "preview")
            return context["attempt_id"]

        self._active_context = context
        context["stage"] = "waiting_for_server"
        context["server_deadline"] = self._monotonic() + self._server_timeout_sec
        self._emit(context, "waiting_for_server")
        self._poll_server(context)
        return context["attempt_id"]

    def cancel_current_goal(self, *, reason: str = "estop") -> bool:
        """Cancel or invalidate the current RETURN attempt without blocking.

        A goal request can be canceled before Nav2 accepts it. In that case the
        context remains active and the eventual accepted goal is canceled as
        soon as its handle arrives.
        """

        context = self._active_context
        if context is None or context["terminal"]:
            return False
        if context["cancel_requested"]:
            return True

        context["cancel_requested"] = True
        context["cancel_reason"] = str(reason)

        if context["stage"] == "waiting_for_server":
            self._emit(
                context,
                "canceled",
                detail=f"canceled before goal transmission: {reason}",
            )
            self._finish_context(context)
        elif context["goal_handle"] is None:
            self._emit(
                context,
                "cancel_pending",
                detail=f"will cancel if Nav2 accepts the goal: {reason}",
            )
        else:
            self._request_cancel(context)
        return True

    def _poll_server(self, context: dict) -> None:
        if context["terminal"] or self._active_context is not context:
            return
        if context["cancel_requested"]:
            self._emit(
                context,
                "canceled",
                detail=(
                    "canceled before goal transmission: "
                    f"{context['cancel_reason'] or 'canceled'}"
                ),
            )
            self._finish_context(context)
            return

        try:
            server_available = self._client.server_is_ready()
        except Exception as exc:  # ROS middleware failures must become bag events.
            self._emit(context, "server_unavailable", detail=str(exc))
            self._finish_context(context)
            return

        if server_available:
            self._cancel_server_timer(context)
            self._send_goal(context)
            return

        if self._monotonic() >= context["server_deadline"]:
            self._emit(context, "server_unavailable")
            self._finish_context(context)
            return

        if context["server_timer"] is None:
            poll_period = min(0.1, self._server_timeout_sec)
            context["server_timer"] = self._node.create_timer(
                poll_period,
                lambda: self._poll_server(context),
            )

    def _send_goal(self, context: dict) -> None:
        if context["terminal"] or context["cancel_requested"]:
            return

        try:
            response_future = self._client.send_goal_async(context["goal"])
        except Exception as exc:
            self._emit(context, "send_error", detail=str(exc))
            self._finish_context(context)
            return

        context["stage"] = "goal_request_sent"
        self._emit(context, "goal_request_sent")
        response_future.add_done_callback(
            lambda future: self._on_goal_response(future, context)
        )

    def _on_goal_response(self, future, context: dict) -> None:
        try:
            goal_handle = future.result()
            goal_id = self._goal_id(goal_handle)
        except Exception as exc:
            self._emit(context, "goal_response_error", detail=str(exc))
            self._finish_context(context)
            return

        if not goal_handle.accepted:
            self._emit(context, "rejected", goal_id=goal_id)
            self._finish_context(context)
            return

        context["stage"] = "accepted"
        context["goal_handle"] = goal_handle
        context["goal_id"] = goal_id
        self._emit(context, "accepted", goal_id=goal_id)
        try:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda result: self._on_result(result, context, goal_id)
            )
        except Exception as exc:
            self._emit(context, "result_error", goal_id=goal_id, detail=str(exc))
            if not context["cancel_requested"]:
                self._finish_context(context)
                return

        if context["cancel_requested"] and not context["terminal"]:
            self._request_cancel(context)

    def _on_result(self, future, context: dict, goal_id: Optional[str]) -> None:
        try:
            result_response = future.result()
            status = RESULT_STATUS_NAMES.get(int(result_response.status), "unknown")
            # G3 may synchronously leave RETURN in the callback. Release the
            # terminal goal before notifying it, so it cannot cancel success.
            self._finish_context(context)
            self._emit(context, status, goal_id=goal_id)
        except Exception as exc:
            self._emit(context, "result_error", goal_id=goal_id, detail=str(exc))
        self._finish_context(context)

    def _request_cancel(self, context: dict) -> None:
        if context["terminal"] or context["cancel_future"] is not None:
            return

        goal_handle = context["goal_handle"]
        if goal_handle is None:
            return

        try:
            cancel_future = goal_handle.cancel_goal_async()
            context["cancel_future"] = cancel_future
            self._emit(
                context,
                "cancel_request_sent",
                goal_id=context.get("goal_id"),
                detail=context["cancel_reason"],
            )
            cancel_future.add_done_callback(
                lambda future: self._on_cancel_response(future, context)
            )
        except Exception as exc:
            self._emit(
                context,
                "cancel_error",
                goal_id=context.get("goal_id"),
                detail=str(exc),
            )

    def _on_cancel_response(self, future, context: dict) -> None:
        if context["terminal"]:
            return
        try:
            response = future.result()
            goals_canceling = getattr(response, "goals_canceling", ())
            status = "cancel_accepted" if goals_canceling else "cancel_rejected"
            self._emit(context, status, goal_id=context.get("goal_id"))
        except Exception as exc:
            self._emit(
                context,
                "cancel_error",
                goal_id=context.get("goal_id"),
                detail=str(exc),
            )

    def _cancel_server_timer(self, context: dict) -> None:
        timer = context.get("server_timer")
        if timer is None:
            return
        timer.cancel()
        try:
            self._node.destroy_timer(timer)
        except AttributeError:
            pass
        context["server_timer"] = None

    def _finish_context(self, context: dict) -> None:
        if context["terminal"]:
            return
        context["terminal"] = True
        context["stage"] = "terminal"
        self._cancel_server_timer(context)
        if self._active_context is context:
            self._active_context = None

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
