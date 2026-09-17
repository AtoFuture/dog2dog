"""探索决策：从 frontier 里选出一个目标点，并给出**抵达朝向**。

本模块把 grid / frontier / geodesic 串起来，产出可以直接填进
``NavigateToPose`` 的 ``(x, y, yaw)``。

四个容易做错、且做错后**不会报错、只表现为「探索很慢」或「原地打转」**的点
（见 docs/框架规划.md §4.3）：

1. **目标点必须测地可达**，不能用欧氏直线判断 —— 墙另一侧的 frontier 看着近，
   实际过不去。

2. **frontier 格紧贴未知区，clearance 天然只有一格**。直接对它做安全半径校验，
   在「一个房间、四周未知」这种最常见场景下会**一个目标点都选不出来**。
   必须先把候选点从边界退回自由空间（``goal_pullback_m``）。

3. **朝向不能用整簇质心**。矩形房间的 frontier 是一个**闭环**，它的质心恰好是
   **房间正中心** —— 据此算出的朝向会指向房间内部，狗走到边界后会面朝房间，
   白跑一趟。必须用**目标点附近的局部 frontier** 定方向。

4. **黑名单和访问惩罚必须比对候选点本身**，不能比对簇质心。同上，
   在闭环 frontier 下簇质心是房间中心，比对它等于永远不生效。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from .frontier import FrontierCluster, detect_frontiers
from .geodesic import geodesic_distance
from .grid import GridMap


@dataclass
class Goal:
    """一个探索目标点。"""

    x: float
    y: float
    yaw: float
    cluster_label: int
    gain: float
    geodesic_dist_m: float
    unknown_fraction: float


@dataclass
class _BlacklistEntry:
    x: float
    y: float
    expires_at: float


@dataclass
class _Visit:
    x: float
    y: float
    at: float


@dataclass
class ExplorerParams:
    """选点参数。默认值都是**起点**，需按实际地图与 Go2 尺寸实测调整。"""

    # --- 安全 ---
    safety_radius_m: float = 0.45
    """目标点周围必须留出的空隙半径（米）。

    必须与 G1 的 Nav2 ``inflation_radius`` / ``robot_radius`` **同源**。
    若这里比 Nav2 的膨胀半径小，G2 会把点选在 Nav2 判定为致命代价的格子上，
    表现为 Nav2 拒绝目标 -> G2 超时拉黑 -> 换点 -> 覆盖率上不去，
    而**现象上会像是「G1 导航不行」**，造成跨组扯皮。
    该值应写进接口契约作为冻结参数。Go2 机身约 70x31 cm，0.45 是偏保守的起步值。
    """

    # --- frontier ---
    min_frontier_area_m2: float = 0.25
    merge_gap_cells: int = 0
    """见 ``frontier.detect_frontiers`` —— 默认不合并，合并会把整个 frontier 环
    并成一个簇且改变其几何。"""

    goal_pullback_m: float = 0.8
    """候选点从 frontier 边界**退回自由空间**的距离（米），应对上面第 2 点。

    取值应 >= ``safety_radius_m``，否则退回后仍然过不了安全校验。
    """

    # --- 增益 ---
    gain_window_radius_m: float = 2.0
    lambda_decay: float = 0.15
    """距离衰减系数（1/m），收益按 ``exp(-lambda * d)`` 折算。

    用**指数衰减**而非幂律 ``1/d^α``：λ 有明确的 1/m 量纲、可按物理意义选，
    而 α 是纯经验参数、取值无依据。0.15 意味着 2 m 处权重 0.74、10 m 处 0.22。
    **起步值，需实测调整。**
    """

    # --- 朝向 ---
    yaw_local_radius_m: float = 1.5
    """用目标点周围这个半径内的 frontier 格来定朝向，应对上面第 3 点。"""

    # --- 震荡抑制 ---
    visit_history_s: float = 60.0
    visit_penalty_radius_m: float = 2.0
    visit_penalty_factor: float = 0.3

    # --- 失败黑名单 ---
    blacklist_ttl_s: float = 30.0
    """失败点拉黑时长。**不能永久** —— 地图更新后该点可能重新变得可达。"""
    blacklist_radius_m: float = 1.0

    # --- 性能 ---
    max_geodesic_radius_cells: int | None = 400
    """测地展开半径上限（格）。地图很大时限制算力，也让选点偏向近处。"""


class GoalSelector:
    """有状态的探索目标选择器。

    用法::

        selector = GoalSelector()
        goal = selector.select(grid, robot_xy=(0.0, 0.0), now=t)
        if goal is None:
            ...  # 探索完成
        else:
            send_navigate_to_pose(goal.x, goal.y, goal.yaw)
            selector.on_result(goal, success=True, now=t2)
    """

    def __init__(self, params: ExplorerParams | None = None):
        self.params = params or ExplorerParams()
        self._blacklist: list[_BlacklistEntry] = []
        self._visits: list[_Visit] = []

    # ------------------------------------------------------------------
    # 状态维护
    # ------------------------------------------------------------------
    def _expire(self, now: float) -> None:
        self._blacklist = [e for e in self._blacklist if e.expires_at > now]
        self._visits = [v for v in self._visits if now - v.at < self.params.visit_history_s]

    def on_result(self, goal: Goal, success: bool, now: float) -> None:
        """回报一次导航结果。

        失败（含被拒绝、超时、被抢占）都应传 ``False``，该点会被临时拉黑。

        注意：**被 G3 返航抢占**也走 ``False`` 进来，但按契约，抢占后 G2 应整体
        转入 PASSIVE 而不是继续选下一个点 —— 那个判断属于状态机，
        不在本模块（见 docs/框架规划.md §4.3⑤ 与 §10.1）。
        """
        if success:
            self._visits.append(_Visit(goal.x, goal.y, now))
        else:
            self._blacklist.append(
                _BlacklistEntry(goal.x, goal.y, now + self.params.blacklist_ttl_s)
            )

    def mark_visited(self, x: float, y: float, now: float) -> None:
        """外部直接标记一个已访问位置（例如机器人实际走到的地方）。"""
        self._visits.append(_Visit(x, y, now))

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------
    def _filter_blacklisted(self, grid: GridMap, candidates: np.ndarray) -> np.ndarray:
        """从候选格里剔除落在黑名单半径内的（向量化，避免逐格 Python 循环）。"""
        if not self._blacklist or candidates.size == 0:
            return candidates

        wx, wy = grid.grid_to_world(candidates[:, 0].astype(float),
                                    candidates[:, 1].astype(float))
        radius2 = self.params.blacklist_radius_m ** 2
        blocked = np.zeros(len(candidates), dtype=bool)
        for e in self._blacklist:
            blocked |= (wx - e.x) ** 2 + (wy - e.y) ** 2 < radius2

        return candidates[~blocked]

    def _local_frontier_centroid(
        self, grid: GridMap, cluster: FrontierCluster, row: int, col: int
    ) -> tuple[float, float]:
        """目标点**附近**那片 frontier 的质心 —— 用它定朝向。

        不能直接用 ``cluster.centroid_world``：矩形房间的 frontier 是闭环，
        其质心是房间正中心，朝向会指反（见模块 docstring 第 3 点）。
        """
        cells = cluster.cells
        r = max(1, int(round(self.params.yaw_local_radius_m / grid.resolution)))

        d = np.hypot(cells[:, 0] - row, cells[:, 1] - col).astype(np.float64)
        near = cells[d <= r]
        if near.size == 0:
            return cluster.centroid_world

        # 距离加权，而不是简单平均。
        # 在房间**角落**附近，窗口内截到的是一段 L 形的边界：
        # 简单平均会把 L 的另一条边（可能已经横跨到房间的另一侧）也拉进来，
        # 质心反而落向房间内部，朝向又指反了。加权让最近的 frontier 格主导。
        w = 1.0 / (np.hypot(near[:, 0] - row, near[:, 1] - col) + 1.0)
        cr = float((near[:, 0] * w).sum() / w.sum())
        cc = float((near[:, 1] * w).sum() / w.sum())

        cx = grid.origin[0] + (cc + 0.5) * grid.resolution
        cy = grid.origin[1] + (cr + 0.5) * grid.resolution
        return cx, cy

    def _unknown_fraction_map(self, grid: GridMap) -> np.ndarray:
        """预先算出**全图每格**周围窗口内的未知格占比。

        增益评分要对候选集里的每一格算一次窗口均值。若逐格去切片统计，
        几百个候选就是几百次小切片，慢且啰嗦。用一次盒式滤波把整张占比图
        算出来（内部走积分图，O(N)），之后每格查表 O(1)。
        """
        r_cells = max(1, int(round(self.params.gain_window_radius_m / grid.resolution)))
        k = 2 * r_cells + 1
        return cv2.boxFilter(
            grid.unknown.astype(np.float32),
            -1,
            (k, k),
            normalize=True,
            borderType=cv2.BORDER_REPLICATE,
        )

    def _visit_penalty(self, grid: GridMap, candidates: np.ndarray) -> np.ndarray:
        """每个候选格的访问惩罚系数（向量化）。

        惩罚**随距离渐变**，不能是「半径内一律乘系数」的平坦形式：
        对称场景下几个最近的候选彼此相距往往小于惩罚半径，
        平坦惩罚会把它们**同比例**打折，排序完全不变 —— 惩罚形同虚设，
        机器人照旧在两个 frontier 之间来回震荡。
        线性渐变下，正踩在旧目标上的候选吃满惩罚，旁边的候选几乎不受影响。

        比对的是**候选点本身**而非簇质心（见模块 docstring 第 4 点）。
        """
        p = self.params
        if not self._visits or candidates.size == 0:
            return np.ones(len(candidates))

        wx, wy = grid.grid_to_world(
            candidates[:, 0].astype(float), candidates[:, 1].astype(float)
        )
        penalty = np.ones(len(candidates))
        for v in self._visits:
            d = np.hypot(wx - v.x, wy - v.y)
            graded = p.visit_penalty_factor + (1.0 - p.visit_penalty_factor) * np.clip(
                d / p.visit_penalty_radius_m, 0.0, 1.0
            )
            penalty = np.minimum(penalty, np.where(d < p.visit_penalty_radius_m, graded, 1.0))
        return penalty

    # ------------------------------------------------------------------
    # 主入口
    # ------------------------------------------------------------------
    def select(
        self,
        grid: GridMap,
        robot_xy: tuple[float, float],
        now: float,
    ) -> Goal | None:
        """选出一个目标点；没有可用的 frontier 时返回 ``None``（= 探索完成）。"""
        p = self.params
        self._expire(now)

        start = grid.nearest_traversable_index(robot_xy[0], robot_xy[1])
        if start is None:
            return None

        clusters = detect_frontiers(
            grid,
            min_area_m2=p.min_frontier_area_m2,
            merge_gap_cells=p.merge_gap_cells,
        )
        if not clusters:
            return None

        dist = geodesic_distance(grid.traversable, start, p.max_geodesic_radius_cells)

        # 可作目标点的格：可通行，且周围留得下机器人（与 Nav2 膨胀半径同源）
        clearance = grid.clearance_m()
        valid_goal = grid.traversable & (clearance >= p.safety_radius_m)

        uf_map = self._unknown_fraction_map(grid)

        pullback_cells = max(0, int(round(p.goal_pullback_m / grid.resolution)))
        kernel = np.ones((2 * pullback_cells + 1, 2 * pullback_cells + 1), np.uint8)

        best: Goal | None = None
        for cluster in clusters:
            # frontier 格紧贴未知区，clearance 天然只有一格 —— 直接校验必然全灭。
            # 先把簇在自由空间内向外扩张，让候选点从边界退回到房间内部。
            cell_mask = np.zeros((grid.height, grid.width), dtype=np.uint8)
            cell_mask[cluster.cells[:, 0], cluster.cells[:, 1]] = 1
            if pullback_cells > 0:
                cell_mask = cv2.dilate(cell_mask, kernel)

            candidates = np.argwhere((cell_mask > 0) & valid_goal & (dist >= 0))
            candidates = self._filter_blacklisted(grid, candidates)
            if candidates.size == 0:
                continue

            # 对**整个候选集**评分取最优，而不是只取测地最近的那一格。
            #
            # 这一点很关键：只取最近的话，距离项永远压过其他因素，
            # 增益评分实际上从未参与决策 —— 那就退化成了它本要取代的朴素行为。
            # 增益 = 未知格占比 × 距离衰减 × 访问惩罚
            rows, cols = candidates[:, 0], candidates[:, 1]
            d_m = dist[rows, cols] * grid.resolution
            gains = (
                uf_map[rows, cols]
                * np.exp(-p.lambda_decay * d_m)
                * self._visit_penalty(grid, candidates)
            )

            k = int(np.argmax(gains))
            gain = float(gains[k])
            if gain <= 0.0:
                continue

            row, col = int(rows[k]), int(cols[k])
            wx, wy = grid.grid_to_world(row, col)

            if best is None or gain > best.gain:
                # 朝向：从目标点指向**附近**那片 frontier 的质心。
                # 那片区域在未知区一侧，面向它才能看到新东西。
                cx, cy = self._local_frontier_centroid(grid, cluster, row, col)
                yaw = math.atan2(cy - wy, cx - wx)
                best = Goal(
                    x=wx,
                    y=wy,
                    yaw=yaw,
                    cluster_label=cluster.label,
                    gain=gain,
                    geodesic_dist_m=float(dist[row, col]) * grid.resolution,
                    unknown_fraction=float(uf_map[row, col]),
                )

        return best
