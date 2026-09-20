"""NavigateToPose client skeleton with transmission deliberately disabled."""

from math import cos, sin

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class NavigationTransmissionDisabled(RuntimeError):
    """Raised whenever code attempts to transmit a navigation goal."""


class NavigateToPoseClientSkeleton:
    """Build and inspect return goals without transmitting them.

    An ActionClient is created so interface names and types can be integrated now.
    This class intentionally never invokes the action client's transmit method.
    """

    def __init__(self, node, action_name: str = "/robot1/navigate_to_pose") -> None:
        self._node = node
        self._action_name = action_name
        self._client = ActionClient(node, NavigateToPose, action_name)

    @property
    def action_name(self) -> str:
        return self._action_name

    def build_goal(
        self,
        *,
        frame_id: str,
        x: float,
        y: float,
        yaw: float,
    ) -> NavigateToPose.Goal:
        """Construct a goal message locally; this performs no DDS transmission."""

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = sin(yaw / 2.0)
        goal.pose.pose.orientation.w = cos(yaw / 2.0)
        return goal

    def preview_goal(self, goal: NavigateToPose.Goal) -> None:
        """Log a goal preview without waiting for a server or sending a request."""

        pose = goal.pose.pose
        self._node.get_logger().warning(
            "NAVIGATION TRANSMISSION DISABLED: preview only; "
            f"action={self._action_name}, frame={goal.pose.header.frame_id}, "
            f"x={pose.position.x:.3f}, y={pose.position.y:.3f}, "
            f"qz={pose.orientation.z:.3f}, qw={pose.orientation.w:.3f}"
        )

    def send_goal(self, _goal: NavigateToPose.Goal) -> None:
        """Hard safety interlock for the initial scaffold."""

        raise NavigationTransmissionDisabled(
            "NavigateToPose transmission is intentionally disabled in this scaffold"
        )
