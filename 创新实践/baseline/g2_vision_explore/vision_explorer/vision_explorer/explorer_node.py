"""G2 探索节点：frontier 选点 → 发 ``NavigateToPose``。

这是**交付物 2 的接线**。在此之前 ``g2_core`` 里的 exploration 部分
（``grid`` / ``frontier`` / ``geodesic`` / ``explorer`` / ``state_machine``）
一直只是**库 + 单测**，没有任何 ROS2 节点把它们串成一个跑得起来的组件。
本文件补的就是这一块。

--------------------------------------------------------------------------------
职责边界（项目的硬规矩）

**只决定「去哪」，不碰「怎么走」。**

    G2（本节点）                 G1（Nav2）
    frontier / 信息增益  ──▶    路径规划 / 绕障 / costmap

所以这里只做三件事：订阅地图、选点、把点发出去。路径怎么走一概不管。

--------------------------------------------------------------------------------
两个已经拍板的设计，别自己发明（都在 docs/议题-*.md 里）

**① ``SENDING`` 态的出口 —— 走方案乙（已实现）**

``SENDING`` 混淆了「正在发出」和「刚决定要选点」两种情形，
所以有三种结局必须写全，且互斥：

    on_goal_sent()          目标落地了
    send_failed()           发不出去（action server 不在等）
    on_select_failed()      **没得发**（没选到点）

漏掉第三种，状态机就卡在 ``SENDING`` 出不来。
详见 ``docs/议题-SENDING态出口.md``。

**② 返航抢占 —— 订阅 ``/mission_state``（G2 的建议方案 B，待全队拍板）**

不能让 G2 靠 action 返回值去猜自己是不是被抢占了 ——
实测（在 ``fake_goal_server`` 上）证明被抢占时客户端拿到的状态码
与「正常完成」无法区分。所以由 G3 显式广播任务状态。
详见 ``docs/议题-接口02返航抢占.md``。

⚠️ **该接口状态是「待全队决定」**。本节点按建议实现了订阅，
但**没有这条话题时不会报错** —— 收不到就一直当「可探索」，
行为与接线前一致。所以拍了板也不会返工，只是多了个安全阀。

--------------------------------------------------------------------------------
运行

    ros2 run vision_explorer explorer_node --ros-args -p use_sim_time:=true

自测（不需要 G1 / Nav2 / 真机）：

    ros2 run vision_explorer fake_goal_server      # 假的 NavigateToPose 服务器
    ros2 run vision_explorer explorer_node
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

from g2_core.explorer import ExplorerParams, Goal, GoalSelector, SelectStatus
from g2_core.grid import GridMap
from g2_core.state_machine import (MISSION_EXPLORING, Command,
                                   ExplorerStateMachine, Phase)

# ---------------------------------------------------------------------------
# QoS
# ---------------------------------------------------------------------------
# ⚠️ ``/map`` 的 durability 是本项目反复强调的坑（见 docs/框架规划.md 的待确认项）：
# 节点晚启动时，**volatile 的 latched 地图收不到**，表现是「一直没有地图 →
# 一直不选点」，而且不报错。所以这里明确用 TRANSIENT_LOCAL。
#
# 与 G1 的实际发布端不一致时会出现「订阅上了但收不到」—— 那正是要显式写死的原因，
# 免得将来靠猜。
_MAP_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# 任务状态：同样 transient_local —— 晚启动的 G2 也要能立刻拿到「现在是停手态」，
# 否则它会在返航途中继续发探索目标。见 docs/议题-接口02返航抢占.md。
_MISSION_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def occupancy_to_grid(msg: OccupancyGrid) -> GridMap:
    """``nav_msgs/OccupancyGrid`` → :class:`GridMap`。

    抽成模块级函数是为了**可测** —— 节点方法测不了，而这个转换有两个
    静默出错的点：``data`` 是 row-major 的一维数组（要 reshape 成
    ``(height, width)``），``origin`` 是**左下角格的外角**而不是中心。

    Raises
    ------
    ValueError
        shape 与 ``info`` 对不上，或 resolution 非正（``GridMap`` 会校验）。
    """
    data = np.asarray(msg.data, dtype=np.int8)
    expected = msg.info.height * msg.info.width
    if data.size != expected:
        raise ValueError(
            f"data 长度 {data.size} 与 info 的 "
            f"{msg.info.height}x{msg.info.width}={expected} 对不上"
        )
    return GridMap(
        data.reshape(msg.info.height, msg.info.width),
        msg.info.resolution,
        origin=(msg.info.origin.position.x, msg.info.origin.position.y),
    )


def select_and_report(
    selector: GoalSelector,
    sm: ExplorerStateMachine,
    grid: GridMap | None,
    robot_xy: tuple[float, float],
    now: float,
) -> Goal | None:
    """选一个目标点，并把结果**如实**回报给状态机。返回要发的目标（没有则 ``None``）。

    ⚠️ **这个函数的全部价值在于 ``on_select_failed`` 的那个布尔参数。**

    状态机在 ``SENDING`` 态等一个回执，三种结局必须写全且互斥
    （``docs/议题-SENDING态出口.md``）：

        on_goal_sent()                目标发出去了
        send_failed()                 发不出去（action server 不在）
        on_select_failed(exhausted)   **没得发**，且只有 ``exhausted=True``
                                      才推进不可逆的 ``DONE``

    传错那一个布尔值就是**探索永久停机**：地图还没到、候选点刚进了黑名单、
    frontier 太小 —— 这些都会「暂时选不出点」，但它们**都不是探完了**。
    而 ``DONE`` 是终止态，没有任何日志，表现就是节点从此沉默。

    所以这里显式地写 ``status is SelectStatus.NO_FRONTIER``，
    而不是 ``status is not SelectStatus.GOAL``（后者会把将来新增的
    状态值全都误判成「探完了」）。
    """
    if grid is None:
        # 地图还没到 —— 稍后再试，**不是**探完了。
        sm.on_select_failed(False)
        return None

    result = selector.select(grid, robot_xy, now)
    if result.status is SelectStatus.GOAL:
        sm.on_goal_sent()
        return result.goal

    sm.on_select_failed(result.status is SelectStatus.NO_FRONTIER)
    return None


class VisionExplorer(Node):
    """frontier 探索节点。"""

    def __init__(self) -> None:
        super().__init__("vision_explorer")

        self._declare_params()
        p = self._p()

        self._sm = ExplorerStateMachine()
        # ⚠️ 选点参数目前用 ExplorerParams 的默认值。
        # 要调参就先把它做成 yaml + ``--params-file``（14 个字段，
        # 一个个 declare_parameter 不划算）。默认值是经过实测与审查的，先不动。
        self._selector = GoalSelector(ExplorerParams())

        self._grid: GridMap | None = None
        self._grid_stamp: float = -math.inf
        self._goal: Goal | None = None          # 在飞的那个目标
        self._goal_sent_at: float = -math.inf
        self._mission_state: str = MISSION_EXPLORING

        self._nav = ActionClient(self, NavigateToPose, p["nav_action"])
        self._goal_handle = None

        self.create_subscription(OccupancyGrid, p["map_topic"], self._on_map,
                                 _MAP_QOS)
        self.create_subscription(String, p["mission_state_topic"],
                                 self._on_mission_state, _MISSION_QOS)

        self.create_timer(p["tick_period_s"], self._tick)

        self.get_logger().info(
            f"探索节点已起：map={p['map_topic']} "
            f"mission={p['mission_state_topic']} nav={p['nav_action']} "
            f"tick={p['tick_period_s']}s"
        )
        if p["mission_state_topic"]:
            self.get_logger().info(
                "⚠️ /mission_state 是 G2 建议的接口（议题-接口02），"
                "全队尚未拍板；收不到该话题时按「可探索」处理。"
            )

    # ------------------------------------------------------------------
    # 参数
    # ------------------------------------------------------------------
    def _declare_params(self) -> None:
        self.declare_parameters("", [
            ("map_topic", "/map"),
            ("mission_state_topic", "/mission_state"),
            ("nav_action", "navigate_to_pose"),
            ("tick_period_s", 2.0),
            ("max_nav_timeout_s", 120.0),
            ("robot_frame", "base_link"),
            ("map_frame", "map"),
            # 选点本身的参数由 ExplorerParams 声明（阈值、权重、TTL…）
        ])

    def _p(self) -> dict:
        return {n: self.get_parameter(n).value for n in (
            "map_topic", "mission_state_topic", "nav_action", "tick_period_s",
            "max_nav_timeout_s", "robot_frame", "map_frame")}

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid) -> None:
        """``OccupancyGrid`` → :class:`GridMap`。

        ⚠️ **只消费 OccupancyGrid，不消费 costmap。** 两者取值域完全不同，
        把 costmap 当 OccupancyGrid 会得到一张语义全错的图，而且不报错。
        ``GridMap`` 的构造里有取值域校验，见 ``g2_core/grid.py``。
        """
        try:
            self._grid = occupancy_to_grid(msg)
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error(f"地图解析失败，丢弃这一帧：{exc}")
            return
        self._grid_stamp = self._now()
        # DONE 是终止态，不会自己出来 —— 但**新地图可能带来新 frontier**
        # （典型场景：探索到一半失联，G1 重新建图后又有可探区域了）。
        # ``revive()`` 的 docstring 说的就是这个用法。没有新 frontier 时
        # 下一轮 select() 会立刻返回 NO_FRONTIER 再次进 DONE，不会空转。
        self._sm.revive()

    def _on_mission_state(self, msg: String) -> None:
        """G3 的任务状态。

        ⚠️ **未知取值一律当作「停手」** ——
        将来 G3 加了 ``paused`` / ``charging`` 之类的新状态，
        G2 不需要跟着改就能正确让出。
        反过来（未知当可探索）会在 G3 引入新状态时**静默地让 G2 继续抢道**。
        """
        if msg.data != self._mission_state:
            self.get_logger().info(f"任务状态：{self._mission_state} → {msg.data}")
        self._mission_state = msg.data
        self._sm.on_mission_state(msg.data)

    # ------------------------------------------------------------------
    # 控制循环
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if not rclpy.ok():
            return
        now = self._now()
        self._prune(now)

        cmd = self._sm.next_command()
        if cmd is Command.SEND_GOAL:
            self._select_and_send(now)
        elif cmd is Command.CANCEL_GOAL:
            self._cancel_goal(now)
        # Command.NONE：什么都不做（绝大多数 tick 都是这个）

    def _select_and_send(self, now: float) -> None:
        goal = select_and_report(
            self._selector, self._sm, self._grid, self._robot_xy(), now)
        if goal is not None:
            self._send_goal(goal, now)
        else:
            self.get_logger().debug(f"本轮没发点（{self._sm.phase.value}）")

    def _send_goal(self, goal: Goal, now: float) -> None:
        if not self._nav.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("NavigateToPose 服务器不在，放弃这次发点")
            self._sm.send_failed()
            return

        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = self._p()["map_frame"]
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(goal.x)
        msg.pose.pose.position.y = float(goal.y)
        msg.pose.pose.position.z = 0.0
        # 朝向：只给一个合法的单位四元数。G2 **不规定终点朝向** ——
        # 怎么停是 G1 的事（契约边界）。
        msg.pose.pose.orientation.w = 1.0

        self._goal = goal
        self._goal_sent_at = now
        self._goal_handle = None
        fut = self._nav.send_goal_async(msg)
        fut.add_done_callback(self._on_goal_accepted)
        self.get_logger().info(
            f"发出探索目标 ({goal.x:.2f}, {goal.y:.2f})  "
            f"[{self._sm.phase.value}]"
        )

    def _on_goal_accepted(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn("目标被拒绝")
            self._goal = None
            self._sm.send_failed()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_done)

    def _on_goal_done(self, future) -> None:
        now = self._now()
        try:
            status = future.result().status
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error(f"取目标结果失败：{exc}")
            self._sm.on_goal_cancelled()
            return

        success = status == 4                          # STATUS_SUCCEEDED
        self.get_logger().info(f"探索目标结束：status={status}")
        if self._goal is not None:
            self._selector.on_result(self._goal, success, now)
            if success:
                self._selector.mark_visited(self._goal.x, self._goal.y, now)
        self._goal = None
        self._goal_handle = None
        self._sm.on_goal_result(success)

    def _cancel_goal(self, now: float) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        # ⚠️ 这里**立刻**回报 on_goal_cancelled()，不等 cancel 的回执。
        # 理由：状态机需要马上离开 NAVIGATING 才能响应抢占，
        # 而 cancel 的回执可能很晚甚至不来（action 层不保证）。
        self._goal = None
        self._goal_handle = None
        self._sm.on_goal_cancelled()
        self.get_logger().info("已取消在飞目标（让出导航权）")

    def _prune(self, now: float) -> None:
        """在飞目标超时 → 交给状态机处理。"""
        if self._sm.phase is not Phase.NAVIGATING:
            return
        p = self._p()
        if now - self._goal_sent_at > p["max_nav_timeout_s"]:
            self.get_logger().warn(
                f"目标超时（>{p['max_nav_timeout_s']}s），收回")
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self._goal = None
            self._goal_handle = None
            self._sm.on_nav_timeout()

    # ------------------------------------------------------------------
    # 杂项
    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _robot_xy(self) -> tuple[float, float]:
        """机器人在 ``map`` 系下的位置。

        TF 拿不到时退回 ``(0, 0)`` —— 这是**刻意的退化**：
        位姿不准只会让选点次优，而抛异常会让整个节点停摆。
        契约上 ``map → base_link`` 是 G1 的交付物（接口 01），
        拿不到时的责任不在 G2，但 G2 不该因此停摆。
        """
        p = self._p()
        try:
            from rclpy.time import Time
            from tf2_ros import Buffer, TransformListener
            if not hasattr(self, "_tf"):
                self._tf = Buffer()
                self._tf_listener = TransformListener(self._tf, self)
            tr = self._tf.lookup_transform(p["map_frame"], p["robot_frame"],
                                           Time())
            t = tr.transform.translation
            return float(t.x), float(t.y)
        except Exception:                              # noqa: BLE001
            return 0.0, 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
