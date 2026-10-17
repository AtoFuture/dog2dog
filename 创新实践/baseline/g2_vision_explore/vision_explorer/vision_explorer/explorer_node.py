"""G2 frontier 探索：消费在线 OccupancyGrid，选取自由空间目标，交给 Nav2 绕障。

只在地图和 map→base_link TF 可用时选点。require_mission_state 可要求 G3
显式允许探索；收到 RETURN 或交接 token 时取消自己的 action，等待真实终态后
再确认导航权释放。未知任务状态停止发点，返航取消不记为 frontier 失败。
"""

from __future__ import annotations

import math
import json
import copy
import signal
from functools import partial
import time
import uuid
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

from g2_core.explorer import ExplorerParams, Goal, GoalSelector, GoalSelection, SelectStatus
from g2_core.grid import GridMap
from g2_core.mission_progress import MissionProgress, ProgressLimits
from g2_core.observation import observation_candidates, tracked_observation_candidates
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
    if (msg.info.width <= 0 or msg.info.height <= 0
        or not math.isfinite(msg.info.resolution) or msg.info.resolution <= 0):
        raise ValueError('map dimensions and resolution must be positive and finite')
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


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Convert a planar yaw to a unit quaternion ``(x, y, z, w)``.

    Frontier goals use ``Goal.yaw`` to point the robot toward nearby unknown
    space when it arrives.  The navigation action still carries a full 3-D
    quaternion, so the planar rotation is represented about the Z axis.
    """
    half_yaw = float(yaw) / 2.0
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def select_and_report(
    selector: GoalSelector,
    sm: ExplorerStateMachine,
    grid: GridMap | None,
    robot_xy: tuple[float, float],
    now: float,
    *,
    on_status=None,
    confirm_exhaustion=True,
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
        if on_status is not None:
            on_status(None)
        return None

    result = selector.select(grid, robot_xy, now)
    if result.status is SelectStatus.GOAL:
        sm.on_goal_sent()
        if on_status is not None:
            on_status(SelectStatus.GOAL)
        return result.goal

    sm.on_select_failed(confirm_exhaustion and result.status is SelectStatus.NO_FRONTIER)
    if on_status is not None:
        on_status(result.status)
    return None


def padded_planning_map(msg: OccupancyGrid, padding_m: float) -> OccupancyGrid:
    """Preserve every observed cell; add only unknown cells outside map bounds."""
    grid=occupancy_to_grid(msg)
    if not math.isfinite(padding_m) or padding_m<=0:
        raise ValueError('planning map padding must be finite and positive')
    cells=math.ceil(padding_m/grid.resolution)
    data=np.full((grid.height+2*cells,grid.width+2*cells),-1,dtype=np.int8)
    data[cells:cells+grid.height,cells:cells+grid.width]=grid.data
    result=OccupancyGrid();result.header=copy.deepcopy(msg.header);result.info=copy.deepcopy(msg.info)
    result.info.width=data.shape[1];result.info.height=data.shape[0]
    q=msg.info.origin.orientation
    if (not all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w,msg.info.origin.position.x,msg.info.origin.position.y,msg.info.origin.position.z))
        or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01):
        raise ValueError('planning map requires a valid finite origin')
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    offset=cells*grid.resolution
    result.info.origin.position.x-=offset*(math.cos(yaw)-math.sin(yaw))
    result.info.origin.position.y-=offset*(math.sin(yaw)+math.cos(yaw))
    result.data=data.ravel().tolist()
    return result


class VisionExplorer(Node):
    """frontier 探索节点。"""

    def __init__(self) -> None:
        super().__init__("vision_explorer")

        self._declare_params()
        p = self._p()
        if not math.isfinite(p["brain_timeout_s"]) or not .1 <= p["brain_timeout_s"] <= 60:
            raise ValueError("brain_timeout_s must be .1..60")
        if not 1 <= p["brain_max_candidates"] <= 16:
            raise ValueError("brain_max_candidates must be 1..16")
        if not isinstance(p["mission_instruction"], str) or not p["mission_instruction"].strip():
            raise ValueError("mission_instruction must not be empty")
        if len(p["mission_instruction"]) > 2000:
            raise ValueError("mission_instruction must not exceed 2000 characters")

        if (p['active_observation'] or p['track_observation']) and (not p['use_mission_brain'] or not Path(p['observation_bt_path']).is_file()):
            raise ValueError('Active observation requires mission brain and a generated observation BT file')
        if p['max_bootstrap_scans'] > 0 and not Path(p['scan_bt_path']).is_file():
            raise ValueError('Bootstrap scans require a generated scan BT with a yaw-aware goal checker')
        self._progress_limits = ProgressLimits(**{key: p[key] for key in ProgressLimits.__dataclass_fields__})
        self._progress = None
        self._completion_pending = None
        self._completion_published_at = -math.inf
        self._map_frame_id = None
        self._selection_frame_id = None
        self._sm = ExplorerStateMachine()
        # 安全与增益参数沿用内核默认值，物理目标的最小距离由 ROS 参数指定。
        self._selector = GoalSelector(ExplorerParams(min_goal_distance_m=p['min_goal_distance_m'], require_path_clearance=p['require_path_clearance'],max_bootstrap_scans=p['max_bootstrap_scans'],goal_obstacle_clearance_m=p['goal_obstacle_clearance_m']))

        self._grid: GridMap | None = None
        self._grid_stamp: float = -math.inf
        self._goal: Goal | None = None          # 在飞的那个目标
        self._goal_sent_at: float = -math.inf
        self._mission_state: str = 'standby' if p['require_mission_state'] else MISSION_EXPLORING
        self._sm.on_mission_state(self._mission_state)
        self._attempt = None
        self._sequence = 0
        self._handoff_token = None
        self._stopped = False
        self._brain_pending = None
        self._recon_status=None;self._recon_status_at=-math.inf

        self._small_frontiers_count = 0
        self._no_candidate_count = 0
        self._no_pose_count = 0
        self._last_anomaly_count = 0
        self._last_select_status: SelectStatus | None = None
        self._frontier_threshold_adapted = False

        self._nav = ActionClient(self, NavigateToPose, p["nav_action"])
        self._goal_handle = None

        if p['planning_map_topic']==p['map_topic']:
            raise ValueError('Planning map output must differ from raw map input')
        self._planning_map_pub=(self.create_publisher(OccupancyGrid,p['planning_map_topic'],_MAP_QOS)
                                if p['planning_map_topic'] else None)
        self.create_subscription(OccupancyGrid, p["map_topic"], self._on_map,
                                 _MAP_QOS)
        self.create_subscription(String, p["mission_state_topic"],
                                 self._on_mission_state, _MISSION_QOS)

        self._candidates = self.create_publisher(String, '/exploration/candidates', 10)
        self.create_subscription(String,'/mission/recon_status',self._on_recon_status,10)
        self._context_pub = self.create_publisher(String, '/mission/context', 10)
        self.create_subscription(String, '/brain/decision', self._on_brain_decision, 10)
        self._events = self.create_publisher(String, '/exploration/events', 10)
        self._completion_pub = self.create_publisher(String, '/exploration/completion', _MISSION_QOS)
        self.create_subscription(String, '/g3/exploration_session', self._on_exploration_session, _MISSION_QOS)
        self._handoff_ack = self.create_publisher(String, '/navigation/handoff_ack', 10)
        self.create_subscription(String, '/navigation/handoff_request', self._on_handoff, 10)
        self._watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._tick_timer = self.create_timer(p['tick_period_s'], self._tick, clock=self._watchdog_clock)
        self._watchdog_timer = self.create_timer(0.1, self._watch_navigation, clock=self._watchdog_clock)

        self.get_logger().info(
            f"探索节点已起：map={p['map_topic']} "
            f"mission={p['mission_state_topic']} nav={p['nav_action']} "
            f"tick={p['tick_period_s']}s"
        )
        self.get_logger().info('任务状态订阅已启用；RETURN 使用终态确认交接')

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
            ("min_frontier_area_m2", 0.0),
            ("require_mission_state", False),
            ("map_timeout_s", 20.0),
            ("min_goal_distance_m", 0.5),
            ("require_path_clearance", False),
            ("goal_obstacle_clearance_m", 0.0),
            ("planning_map_topic", ""),
            ("planning_map_padding_m", 1.0),
            ("max_bootstrap_scans", 0),
            ("scan_bt_path", ""),
            ("use_mission_brain", False),
            ("brain_timeout_s", 15.0),
            ("brain_max_candidates", 6),
            ("mission_instruction", "Explore the unknown environment safely."),
            ("task_completion_enabled", False),
            *[(key, value) for key, value in vars(ProgressLimits()).items()],
            ("active_observation", False),
            ("track_observation", False),
            ("observation_bt_path", ""),
            # 选点本身的参数由 ExplorerParams 声明（阈值、权重、TTL…）
        ])

    def _p(self) -> dict:
        return {n: self.get_parameter(n).value for n in (
            "map_topic", "mission_state_topic", "nav_action", "tick_period_s",
            "max_nav_timeout_s", "robot_frame", "map_frame",
            "min_frontier_area_m2", "require_mission_state", "map_timeout_s", "min_goal_distance_m", "require_path_clearance", "max_bootstrap_scans", "scan_bt_path", "planning_map_topic", "planning_map_padding_m", "goal_obstacle_clearance_m",
            "use_mission_brain", "brain_timeout_s", "brain_max_candidates", "mission_instruction",
            "active_observation", "track_observation", "observation_bt_path", "task_completion_enabled",
            *ProgressLimits.__dataclass_fields__)}

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------
    def _adapt_frontier_threshold(self, resolution: float) -> None:
        p = self._p()
        configured = float(p["min_frontier_area_m2"])
        if configured > 0.0:
            self._selector.params.min_frontier_area_m2 = configured
            self.get_logger().info(
                f"frontier 阈值: min_frontier_area_m2={configured:.4f} m² "
                f"(用户配置)"
            )
            return
        min_door_width_m = 0.8
        auto_threshold = min_door_width_m * resolution
        self._selector.params.min_frontier_area_m2 = auto_threshold
        self.get_logger().info(
            f"frontier 阈值自适应: min_frontier_area_m2={auto_threshold:.4f} m² "
            f"(门宽 {min_door_width_m} m × 分辨率 {resolution:.3f} m/格)"
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        """``OccupancyGrid`` → :class:`GridMap`。

        ⚠️ **只消费 OccupancyGrid，不消费 costmap。** 两者取值域完全不同，
        把 costmap 当 OccupancyGrid 会得到一张语义全错的图，而且不报错。
        ``GridMap`` 的构造里有取值域校验，见 ``g2_core/grid.py``。
        """
        if msg.header.frame_id != self._p()['map_frame']:
            self.get_logger().warning('地图坐标系与 map_frame 不一致，丢弃')
            return
        try:
            grid = occupancy_to_grid(msg)
            planned=(padded_planning_map(msg,float(self._p()['planning_map_padding_m']))
                     if getattr(self,'_planning_map_pub',None) is not None else None)
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error(f"地图解析失败，丢弃这一帧：{exc}")
            return
        self._grid=grid
        if planned is not None:self._planning_map_pub.publish(planned)
        self._grid_stamp = time.monotonic()
        self._map_frame_id = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if self._progress is not None:
            self._progress.observe_area(float(np.count_nonzero(self._grid.data >= 0))*self._grid.resolution**2)
        if not self._frontier_threshold_adapted:
            self._adapt_frontier_threshold(self._grid.resolution)
            self._frontier_threshold_adapted = True
        # DONE 是终止态，不会自己出来 —— 但**新地图可能带来新 frontier**
        # （典型场景：探索到一半失联，G1 重新建图后又有可探区域了）。
        # ``revive()`` 的 docstring 说的就是这个用法。没有新 frontier 时
        # 下一轮 select() 会立刻返回 NO_FRONTIER 再次进 DONE，不会空转。
        if self._completion_pending is None:
            self._sm.revive()

    def _on_mission_state(self, msg: String) -> None:
        """G3 的任务状态。

        ⚠️ **未知取值一律当作「停手」** ——
        将来 G3 加了 ``paused`` / ``charging`` 之类的新状态，
        G2 不需要跟着改就能正确让出。
        反过来（未知当可探索）会在 G3 引入新状态时**静默地让 G2 继续抢道**。
        """
        if self._stopped:
            return
        if (getattr(self, '_completion_pending', None) is not None and msg.data == MISSION_EXPLORING):
            # A completion rejected during a safety pause must be retried on
            # resume; keep goal generation blocked while refreshing mission state.
            self._mission_state = msg.data
            self._progress.advance(time.monotonic(), active=True)
            return
        if self._stopped or (self._handoff_token is not None and msg.data == MISSION_EXPLORING):
            return
        if msg.data != self._mission_state:
            self.get_logger().info(f"任务状态：{self._mission_state} → {msg.data}")
        self._mission_state = msg.data
        if getattr(self, '_progress', None) is not None:
            self._progress.advance(time.monotonic(), active=msg.data == MISSION_EXPLORING)
        self._sm.on_mission_state(msg.data)
        if not self._sm.should_explore:
            self._discard_brain("mission:"+msg.data)
            self._cancel_goal(self._now(), reason="mission:"+msg.data)

    def _on_exploration_session(self, msg):
        if not self._p().get('task_completion_enabled', False) or self._stopped:
            return
        try:
            session = json.loads(msg.data)
            token = session['session_id']
            if session['state'] != 'exploring' or not isinstance(token, str) or len(token) != 32:
                return
            if any(c not in '0123456789abcdef' for c in token):
                return
        except (ValueError, TypeError, KeyError):
            return
        if self._progress is not None and token == self._progress.session_id:
            return
        # A replacement G3 session must first release any old navigation ownership.
        self._discard_brain('new_session')
        self._cancel_goal(reason='new_session')
        self._progress = MissionProgress(token, self._progress_limits, now=time.monotonic())
        self._progress.advance(time.monotonic(), active=self._mission_state == MISSION_EXPLORING)
        self._completion_pending = None
        self._selector.reset_mission()
        self._selector.params.max_region_failures = self._progress_limits.max_region_failures
        self._sm.revive()
        self._event('exploration_started', session_id=token)

    def _watch_progress(self):
        progress = getattr(self, '_progress', None)
        if progress is None or self._stopped:
            return
        now = time.monotonic()
        progress.advance(now, active=self._mission_state == MISSION_EXPLORING)
        if self._mission_state != MISSION_EXPLORING:
            return
        if self._completion_pending is None:
            xy = self._robot_xy()
            healthy = xy is not None and self._map_pose_healthy(xy)
            if healthy:
                progress.observe_area(float(np.count_nonzero(self._grid.data >= 0))*self._grid.resolution**2)
            result = progress.evaluate(healthy=healthy, selection_current=self._selection_frame_id == self._map_frame_id)
            if result is None:
                return
            self._completion_pending = result
            self._discard_brain('exploration_ended')
            self._sm.on_mission_state('standby')
            self._cancel_goal(reason='exploration_ended')
            self._event('exploration_ended', **result)
        if now-self._completion_published_at >= .5:
            self._completion_pub.publish(String(data=json.dumps(self._completion_pending)))
            self._completion_published_at = now

    # ------------------------------------------------------------------
    # 控制循环
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if not rclpy.ok():
            return
        now = self._now()
        self._watch_navigation()
        if self._p().get('require_path_clearance',False) and self._mission_state==MISSION_EXPLORING:
            xy=self._robot_xy()
            if xy is not None and self._map_pose_healthy(xy):
                self._selector.observe_position(*xy,now)
        if self._stopped or self._attempt is not None or self._handoff_token is not None or self._brain_pending is not None:
            return
        if self._p().get('task_completion_enabled', False) and (self._progress is None or self._completion_pending is not None):
            return
        status=getattr(self,'_recon_status',None)
        if (self._p().get('track_observation',False) and status and status.get('state')=='searching'
            and status.get('observation_window_pending') is True and time.monotonic()-self._recon_status_at<5.):
            return
        cmd = self._sm.next_command()
        if cmd is Command.SEND_GOAL:
            self._select_and_send(now)
        elif cmd is Command.CANCEL_GOAL:
            self._cancel_goal(now)
        # Command.NONE：什么都不做（绝大多数 tick 都是这个）

    def _select_and_send(self, now: float) -> None:
        self._last_select_status = None
        robot_xy = self._robot_xy()
        if robot_xy is None or not self._map_pose_healthy(robot_xy):
            self._sm.on_select_failed(False)
            self._record_select_status(SelectStatus.NO_ROBOT_POSE if robot_xy is None else None)
            self._track_select_failure()
            return
        if self._p().get('track_observation',False):
            status=getattr(self,'_recon_status',None)
            if not status or time.monotonic()-self._recon_status_at>5.:
                self._sm.on_select_failed(False);self._event('search_unavailable',reason='awaiting_recon_status');return
            if status.get('targets') and not self._observation_goals(robot_xy,now):
                self._sm.on_select_failed(False);self._event('search_unavailable',reason='no_safe_target_viewpoint');return
        if self._p()["use_mission_brain"]:
            self._request_brain(robot_xy, now)
            return
        goal = select_and_report(
            self._selector, self._sm, self._grid, robot_xy, now,
            on_status=self._record_select_status,
            confirm_exhaustion=getattr(self, '_progress', None) is None)
        if goal is not None:
            self._send_goal(goal, now)
        else:
            self._track_select_failure()

    def _on_recon_status(self,msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict) or not isinstance(value.get('missing_counts'),dict):return
            if not all(isinstance(c,int) and not isinstance(c,bool) and c>=0 for c in value['missing_counts'].values()):return
            if not isinstance(value.get('targets',{}),dict):return
            for target in value.get('targets',{}).values():
                position=target['position_m']
                if len(position)!=3 or not all(math.isfinite(float(v)) for v in position):return
            self._recon_status=value;self._recon_status_at=time.monotonic()
        except (ValueError,TypeError,KeyError,AttributeError):return

    def _observation_goals(self,robot_xy,now):
        if self._p().get('track_observation',False):
            status=getattr(self,'_recon_status',None)
            if status and status.get('state')=='searching' and time.monotonic()-self._recon_status_at<5.:
                anchors=[t['position_m'][:2] for t in status.get('targets',{}).values()
                         if 0<=now-t.get('position_stamp_ns',0)*1e-9<=10.]
                goals=tracked_observation_candidates(self._grid,robot_xy,anchors=anchors,
                    safety_radius_m=self._selector.params.safety_radius_m,max_candidates=int(self._p()['brain_max_candidates']))
                valid=tuple(g for g in goals if self._selector.is_goal_valid(self._grid,robot_xy,g,now))
                if valid:return valid
        if not self._p().get('active_observation',False):return ()
        status=getattr(self,'_recon_status',None)
        if not status or status.get('state')!='searching' or time.monotonic()-self._recon_status_at>5.:return ()
        if not any(count>0 for count in status['missing_counts'].values()):return ()
        anchors=[t['position_m'][:2] for t in status.get('targets',{}).values()]
        self._selector._expire(now)
        visited=[(v.x,v.y) for v in self._selector._visits]
        goals=observation_candidates(self._grid,robot_xy,anchors=anchors,visited=visited,
            safety_radius_m=self._selector.params.safety_radius_m,max_candidates=int(self._p()['brain_max_candidates']))
        valid=tuple(g for g in goals if self._selector.is_goal_valid(self._grid,robot_xy,g,now))
        if valid:self._cached_observation=(valid,time.monotonic())
        elif getattr(self,'_cached_observation',None) is not None:
            cached,at=self._cached_observation
            if time.monotonic()-at<8.:
                valid=tuple(g for g in cached if self._selector.is_goal_valid(self._grid,robot_xy,g,now)
                            and not any(math.hypot(g.x-v[0],g.y-v[1])<.75 for v in visited))
        return valid

    def _map_pose_healthy(self, robot_xy):
        if self._grid is None or time.monotonic()-self._grid_stamp > self._p()['map_timeout_s']:
            return False
        row, col = (math.floor(v) for v in self._grid.world_to_grid(*robot_xy))
        return bool(self._grid.in_bounds(row, col) and self._grid.traversable[row, col])

    def _request_brain(self, robot_xy, now):
        observation=self._observation_goals(robot_xy,now)
        result = (GoalSelection(SelectStatus.GOAL,observation[0],observation) if observation else
                  self._selector.select(self._grid, robot_xy, now,
                    max_candidates=int(self._p()['brain_max_candidates'])))
        self._record_select_status(result.status)
        if result.goal is None:
            self._sm.on_select_failed(result.is_complete and getattr(self, '_progress', None) is None)
            self._track_select_failure()
            return
        if result.goal.purpose == 'scan':
            # An in-place measurement action has no frontier ranking decision.
            self._sm.on_goal_sent()
            self._send_goal(result.goal, now)
            return
        request_id = uuid.uuid4().hex
        goals = {f'f{i}': goal for i, goal in enumerate(result.candidates)}
        budget = float(self._p()['brain_timeout_s'])
        self._brain_pending = dict(id=request_id, goals=goals, robot_xy=robot_xy,
                                   deadline=time.monotonic()+budget)
        context = dict(
            mission_state='EXPLORE',
            localization_healthy=True,
            operator_instruction=self._p()['mission_instruction'],
            task='Rank only the supplied safe frontiers for the operator mission; choose '
                 'SELECT_FRONTIER. Balance mission relevance, unknown-space gain, and travel '
                 'cost. Never invent coordinates, observations, or hidden people.',
        )
        if observation and observation[0].purpose=='observe_targets':
            context.update(task='Select one supplied safe viewpoint to observe the confirmed moving people. Prefer the highest deterministic score to keep the camera facing them during travel. Preserve standoff; do not invent targets.',
                           candidate_views={k:dict(purpose=g.purpose,look_at=g.look_at) for k,g in goals.items()})
        elif observation:
            context.update(task='Select one supplied safe observation viewpoint to expose unknown space behind a mapped occluder. '
                                'Prioritize visibility gain, then travel cost. These are visibility hypotheses, not detections.',
                           missing_counts=self._recon_status['missing_counts'],
                           candidate_views={k:dict(purpose=g.purpose,look_at=g.look_at,occluder=g.occluder) for k,g in goals.items()})
        payload = dict(request_id=request_id, context=context, evidence_ids=[], timeout_s=budget,
            candidates=[dict(id=key, x=g.x, y=g.y, information_gain=g.unknown_fraction,
                             path_cost=g.geodesic_dist_m, failure_count=0,
                             deterministic_score=g.gain) for key,g in goals.items()])
        self._context_pub.publish(String(data=json.dumps(context)))
        self._candidates.publish(String(data=json.dumps(payload)))
        self._event('brain_requested', request_id=request_id, candidate_count=len(goals),
                    purpose=observation[0].purpose if observation else 'frontier')

    def _discard_brain(self, reason):
        pending = getattr(self, '_brain_pending', None)
        if pending is None:
            return
        self._brain_pending = None
        self._sm.on_select_failed(False)
        self._event('brain_discarded', request_id=pending['id'], reason=reason)

    def _on_brain_decision(self, message):
        pending = self._brain_pending
        if pending is None:
            return
        try:
            decision = json.loads(message.data)
            if decision.get('request_id') != pending['id']:
                return
        except (ValueError, AttributeError):
            return
        if time.monotonic() >= pending['deadline']:
            self._resolve_brain(None, 'deadline')
            return
        # Only the matching candidate ID is executable; model coordinates are ignored.
        key = decision.get('candidate_id')
        if decision.get('action') == 'SELECT_FRONTIER' and isinstance(key,str) and key in pending['goals'] and decision.get('fallback') is False:
            self._resolve_brain(key, 'deepseek', decision)
        else:
            self._resolve_brain(None, 'invalid_or_fallback', decision)

    def _resolve_brain(self, key, source, decision=None):
        pending, self._brain_pending = self._brain_pending, None
        if pending is None:
            return
        if self._stopped or not self._sm.should_explore or self._handoff_token is not None:
            self._sm.on_select_failed(False)
            self._event('brain_discarded', request_id=pending['id'], reason='mission_changed')
            return
        goal = pending['goals'].get(key) if key is not None else next(iter(pending['goals'].values()))
        robot_xy = self._robot_xy()
        valid = (robot_xy is not None and self._map_pose_healthy(robot_xy)
                 and math.hypot(robot_xy[0]-pending['robot_xy'][0], robot_xy[1]-pending['robot_xy'][1]) <= .5
                 and self._selector.is_goal_valid(self._grid, robot_xy, goal, self._now()))
        if not valid:
            self._sm.on_select_failed(False)
            self._event('brain_discarded', request_id=pending['id'], reason='latest_map_or_pose_invalid')
            return
        self._event('brain_selected', request_id=pending['id'], candidate_id=key or 'f0',
                    source=source, fallback=source!='deepseek', decision=decision)
        self._sm.on_goal_sent()
        self._send_goal(goal, self._now())

    def _record_select_status(self, status) -> None:
        self._last_select_status = status
        if getattr(self, '_progress', None) is not None:
            self._selection_frame_id = self._map_frame_id
            self._progress.selection(status.value if status is not None else None,
                                     frame_id=self._map_frame_id)

    def _track_select_failure(self) -> None:
        status = self._last_select_status
        self._event('search_unavailable',reason=status.value if status is not None else 'map_or_pose_unavailable')
        if status is None:
            self.get_logger().debug("本轮没发点（无地图）")
        elif status is SelectStatus.SMALL_FRONTIERS:
            self._small_frontiers_count += 1
            if self._small_frontiers_count == 1:
                self.get_logger().warning(
                    f"frontier 簇全部小于阈值 "
                    f"({self._selector.params.min_frontier_area_m2:.4f} m²)"
                )
            elif self._small_frontiers_count % 10 == 0:
                self.get_logger().warning(
                    f"SMALL_FRONTIERS 已连续 {self._small_frontiers_count} 次"
                )
        elif status is SelectStatus.NO_CANDIDATE:
            self._no_candidate_count += 1
            if self._no_candidate_count % 10 == 0:
                self.get_logger().warning(
                    f"NO_CANDIDATE 已 {self._no_candidate_count} 次"
                )
        elif status is SelectStatus.NO_ROBOT_POSE:
            self._no_pose_count += 1
            if self._no_pose_count % 10 == 0:
                self.get_logger().warning(
                    f"NO_ROBOT_POSE 已 {self._no_pose_count} 次"
                )
        else:
            self.get_logger().debug(f"本轮没发点（{status.value}）")

        anomalies = self._sm.anomalies
        if len(anomalies) > self._last_anomaly_count:
            for a in anomalies[self._last_anomaly_count:]:
                self.get_logger().warning(f"状态机异常: {a}")
            self._last_anomaly_count = len(anomalies)

    def _event(self, status, attempt=None, **details):
        payload = {'status': status, 'monotonic_time': time.monotonic(), **details}
        if getattr(self, '_progress', None) is not None:
            payload['session_id'] = self._progress.session_id
        if attempt is not None:
            goal = attempt['goal']
            payload.update(attempt_id=attempt['id'], target={'x':goal.x,'y':goal.y,'yaw':goal.yaw},
                           gain=goal.gain, geodesic_dist_m=goal.geodesic_dist_m,
                           purpose=goal.purpose,look_at=goal.look_at,occluder=goal.occluder)
        self._events.publish(String(data=json.dumps(payload)))
        self.get_logger().info(json.dumps(payload))

    def _send_goal(self, goal: Goal, now: float) -> None:
        if not self._nav.server_is_ready():
            self._sm.on_goal_result(False)
            self._event('server_unavailable')
            if getattr(self, '_progress', None) is not None:
                self._progress.navigation_result(False)
            return
        msg = NavigateToPose.Goal()
        if goal.purpose == 'scan':
            msg.behavior_tree = self._p()['scan_bt_path']
        elif goal.purpose in ('observe_occlusion','observe_targets'):
            path=self._p().get('observation_bt_path','')
            if not path:
                self._sm.on_goal_result(False)
                self._event('search_unavailable',reason='observation_tree_unconfigured')
                return
            msg.behavior_tree=path
        msg.pose.header.frame_id = self._p()['map_frame']
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(goal.x)
        msg.pose.pose.position.y = float(goal.y)
        qx, qy, qz, qw = yaw_to_quaternion(goal.yaw)
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = qx, qy
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        self._sequence += 1
        attempt = {'id':self._sequence,'goal':goal,'handle':None,'cancel':False,
                   'cancel_sent':False,'started':time.monotonic(),'reason':None}
        self._attempt = attempt
        self._goal, self._goal_handle, self._goal_sent_at = goal, None, now
        self._event('sending', attempt)
        try:
            self._nav.send_goal_async(msg).add_done_callback(partial(self._on_goal_accepted, attempt))
        except Exception as exc:
            self._finish_attempt(attempt, 'send_error', error=str(exc))

    def _on_goal_accepted(self, attempt, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self._finish_attempt(attempt, 'send_error', error=str(exc))
            return
        if handle is None or not handle.accepted:
            self._finish_attempt(attempt, 'rejected')
            return
        attempt['handle'] = handle
        if self._attempt is attempt:
            self._goal_handle = handle
        self._event('accepted', attempt)
        try:
            handle.get_result_async().add_done_callback(partial(self._on_goal_done, attempt))
        except Exception as exc:
            # Without a terminal result, navigation ownership cannot be released.
            self._event('result_error', attempt, error=str(exc))
            attempt['cancel'] = True
        if attempt['cancel'] or self._attempt is not attempt:
            self._cancel_handle(attempt)

    def _on_goal_done(self, attempt, future) -> None:
        try:
            status = future.result().status
        except Exception as exc:
            self._event('result_error', attempt, error=str(exc))
            attempt['cancel'] = True
            self._cancel_handle(attempt)
            return
        self._finish_attempt(attempt, {4:'succeeded',5:'canceled',6:'aborted'}.get(status,'unknown'))

    def _finish_attempt(self, attempt, status, **details):
        self._event(status, attempt, **details)
        if self._attempt is not attempt:
            return
        success = status == 'succeeded'
        if not attempt['cancel'] or attempt['reason'] == 'timeout':
            self._selector.on_result(attempt['goal'], success, self._now())
            if getattr(self, '_progress', None) is not None and (attempt['goal'].purpose != 'scan' or not success):
                self._progress.navigation_result(success)
            if success and attempt['goal'].purpose != 'scan':
                self._selector.mark_visited(attempt['goal'].x,attempt['goal'].y,self._now())
        self._attempt = self._goal = self._goal_handle = None
        self._sm.on_goal_result(success)

    def _cancel_handle(self, attempt):
        if attempt['cancel_sent'] or attempt['handle'] is None:
            return
        attempt['cancel_sent'] = True
        try:
            attempt['handle'].cancel_goal_async().add_done_callback(partial(self._on_cancel, attempt))
        except Exception as exc:
            self._event('cancel_error', attempt, error=str(exc))

    def _on_cancel(self, attempt, future):
        try:
            status = 'cancel_accepted' if future.result().goals_canceling else 'cancel_rejected'
            self._event(status, attempt)
        except Exception as exc:
            self._event('cancel_error', attempt, error=str(exc))

    def _cancel_goal(self, now=None, reason='mission') -> None:
        attempt = self._attempt
        if attempt is None or attempt['cancel']:
            return
        attempt['cancel'], attempt['reason'] = True, reason
        self._event('cancel_requested', attempt, reason=reason)
        self._cancel_handle(attempt)

    def _on_handoff(self, msg):
        try:
            token = json.loads(msg.data)['token']
            if not isinstance(token,str) or not token or len(token)>128:
                return
        except (ValueError,KeyError,TypeError):
            return
        self._discard_brain('handoff')
        self._handoff_token = token
        self._mission_state = 'returning'
        self._sm.on_mission_state('returning')
        self._cancel_goal(reason='handoff')

    def _watch_navigation(self):
        self._watch_progress()
        pending = getattr(self, '_brain_pending', None)
        if pending is not None and time.monotonic() >= pending['deadline']:
            self._resolve_brain(None, 'deadline')
        attempt = self._attempt
        if (attempt is not None and not attempt['cancel'] and attempt['goal'].purpose=='frontier'
            and self._sm.should_explore and self._p().get('active_observation',False)
            and time.monotonic()-getattr(self,'_view_check_at',-math.inf)>2.):
            self._view_check_at=time.monotonic()
            xy=self._robot_xy()
            if xy is not None and self._map_pose_healthy(xy) and self._observation_goals(xy,self._now()):
                self._cancel_goal(reason='observation_available')
        if attempt is not None:
            if time.monotonic()-attempt['started'] > self._p()['max_nav_timeout_s']:
                self._cancel_goal(reason='timeout')
            elif not self._sm.should_explore:
                self._cancel_goal(reason='mission:'+self._mission_state)
        elif self._handoff_token is not None:
            token = self._handoff_token
            self._handoff_ack.publish(String(data=json.dumps({'token':token,'released':True})))
            self._event('handoff_released', token=token)
            self._handoff_token = None

    def stop_navigation(self):
        self._discard_brain('shutdown')
        self._stopped = True
        self._sm.on_mission_state('standby')
        self._cancel_goal(reason='shutdown')

    # ------------------------------------------------------------------
    # 杂项
    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _robot_xy(self) -> tuple[float, float] | None:
        """Fail closed if map-frame TF is missing or stale; never invent a pose."""
        p = self._p()
        try:
            from rclpy.time import Time
            from tf2_ros import Buffer, TransformListener
            if not hasattr(self, '_tf'):
                self._tf = Buffer()
                self._tf_listener = TransformListener(self._tf, self)
            tr = self._tf.lookup_transform(p['map_frame'],p['robot_frame'],Time())
            age = (self.get_clock().now()-Time.from_msg(tr.header.stamp)).nanoseconds*1e-9
            if age > 1.0:
                return None
            t = tr.transform.translation
            if not (math.isfinite(t.x) and math.isfinite(t.y)):
                return None
            return float(t.x),float(t.y)
        except Exception:
            return None


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = VisionExplorer()
    stopping = False
    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True
    # Do not raise an asynchronous Python exception inside a native DDS take.
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_navigation()
        deadline = time.monotonic()+2
        while rclpy.ok() and node._attempt is not None and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
