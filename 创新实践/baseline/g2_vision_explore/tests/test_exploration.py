"""探索链路测试：栅格、frontier、测地距离、目标点选择。

重点覆盖三个**做错了不会报错、只会表现为「探索很慢」**的坑：

1. **frontier 格紧贴未知区，clearance 天然只有一格** —— 直接对它做安全半径校验
   会导致「一个房间、四周未知」这种最常见的场景下**一个目标点都选不出来**。
   必须先把候选点从边界退回自由空间（``goal_pullback_m``）。
2. **测地距离不能用欧氏距离代替** —— 墙另一侧的 frontier 看着近，实际过不去。
3. **目标点必须带朝向 yaw** —— 否则狗走到 frontier 可能面朝墙，这一趟白跑。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from g2_core.explorer import ExplorerParams, GoalSelector, SelectStatus
from g2_core.frontier import detect_frontiers, has_frontier_cells
from g2_core.geodesic import geodesic_distance, geodesic_nearest_cell
from g2_core.grid import FREE, OCCUPIED, UNKNOWN, GridMap

RES = 0.1

ROOM_CENTRE_XY = (2.0, 2.0)
"""room_map() 里机器人常站的位置。"""


def must_pick(selector, grid, robot_xy=ROOM_CENTRE_XY, now=0.0):
    """断言「确实选到了目标」并返回它。

    大多数测试只关心「选出来的那个点对不对」，不关心状态枚举，
    所以用这个把 `GoalSelection` 拆包，避免每处都写两行断言。
    """
    sel = selector.select(grid, robot_xy=robot_xy, now=now)
    assert sel.status is SelectStatus.GOAL, f"预期选到目标，实际得到 {sel.status}"
    assert sel.goal is not None
    return sel.goal


def pick_status(selector, grid, robot_xy=ROOM_CENTRE_XY, now=0.0):
    """只取状态，不假设结果。"""
    return selector.select(grid, robot_xy=robot_xy, now=now).status


def room_map(size=60, lo=10, hi=50, res=RES) -> GridMap:
    """一个四周全是未知区域的矩形房间 —— 最典型的探索起始状态。"""
    data = np.full((size, size), UNKNOWN, dtype=np.int8)
    data[lo:hi, lo:hi] = FREE
    return GridMap(data, res, origin=(0.0, 0.0))


# ----------------------------------------------------------------------
# GridMap
# ----------------------------------------------------------------------
def test_masks_and_ranges():
    grid = room_map()
    assert grid.free[30, 30]
    assert grid.unknown[0, 0]
    assert not grid.occupied.any()

    assert grid.unknown_ratio() == pytest.approx(1 - (40 * 40) / (60 * 60))


def test_world_grid_roundtrip():
    grid = GridMap(np.zeros((10, 10), dtype=np.int8), 0.5, origin=(1.0, 2.0))

    row, col = grid.world_to_grid(1.0 + 2.5, 2.0 + 1.5)
    assert (row, col) == pytest.approx((3.0, 5.0))

    wx, wy = grid.grid_to_world(3, 5)
    assert (wx, wy) == pytest.approx((1.0 + 5.5 * 0.5, 2.0 + 3.5 * 0.5))


def test_nearest_traversable_snaps_out_of_unknown():
    grid = room_map()
    # 房间外一点，应吸附到房间边缘的自由格
    assert grid.nearest_traversable_index(0.05, 0.05) is not None
    # 房间中心已在自由格上，应原地返回
    assert grid.nearest_traversable_index(2.0, 2.0) == (20, 20)


def test_clearance_is_larger_in_the_middle():
    grid = room_map()
    cl = grid.clearance_m()
    assert cl[30, 30] > cl[11, 11]


def test_clearance_rejects_bad_resolution():
    with pytest.raises(ValueError):
        GridMap(np.zeros((4, 4), dtype=np.int8), 0.0)
    with pytest.raises(ValueError):
        GridMap(np.zeros(4, dtype=np.int8), 0.1)


# ----------------------------------------------------------------------
# frontier
# ----------------------------------------------------------------------
def test_frontier_is_the_ring_around_the_room():
    grid = room_map()
    clusters = detect_frontiers(grid, min_area_m2=0.0, merge_within_cells=0)

    assert len(clusters) == 1, "四周未知，房间边缘 8 连通成一个闭环"

    cells = clusters[0].cells
    rows, cols = cells[:, 0], cells[:, 1]
    # 全部落在房间边界那一圈上
    assert rows.min() == 10 and rows.max() == 49
    assert cols.min() == 10 and cols.max() == 49
    # 房间内部不应出现在 frontier 里
    assert not ((rows > 10) & (rows < 49) & (cols > 10) & (cols < 49)).any()


def test_ring_centroid_is_the_room_centre():
    """记录「为什么朝向不能用整簇质心」—— 这是本次实现中发现的真问题。

    矩形房间的 frontier 是一整个闭环，它的质心恰好是**房间正中心**。
    若用簇质心定朝向，狗走到边界后会面朝房间内部，这一趟白跑，
    而且不会有任何报错。
    """
    grid = room_map()
    cluster = detect_frontiers(grid, min_area_m2=0.0, merge_within_cells=0)[0]

    cx, cy = cluster.centroid_world
    room_centre_x, room_centre_y = grid.grid_to_world(29.5, 29.5)

    assert cx == pytest.approx(room_centre_x, abs=0.1)
    assert cy == pytest.approx(room_centre_y, abs=0.1)


def test_min_area_filters_noise_clusters():
    data = np.full((40, 40), UNKNOWN, dtype=np.int8)
    data[10:30, 10:30] = FREE

    grid = GridMap(data, RES, origin=(0, 0))
    assert len(detect_frontiers(grid, min_area_m2=0.0, merge_within_cells=0)) == 1

    # 环的面积 = 20x20 - 18x18 = 76 格 = 0.76 m²；门槛抬到 1.0 m² 应全部过滤
    ring_area_m2 = (20 * 20 - 18 * 18) * RES ** 2
    assert ring_area_m2 == pytest.approx(0.76)

    assert detect_frontiers(grid, min_area_m2=1.0, merge_within_cells=0) == []


def test_merge_preserves_cluster_geometry():
    """⚠️ 这条测试原先断言的是**错误行为**（「合并会让面积变大」），
    2026-09-17 重写。

    原先合并用形态学膨胀实现，副作用是**改变 frontier 本身的几何**：
    区域被向内扩张、面积被放大、质心偏移。那条测试把副作用当成了规格 ——
    于是按 `frontier.py` 自己 docstring 推荐的「按质心距离合并」去修，
    测试反而会失败（一个挡住正确修法的枷锁）。

    现在合并只改「分组」，不动几何。这条测试断言的正是**这个契约**。
    """
    grid = room_map()
    raw = detect_frontiers(grid, min_area_m2=0.0, merge_within_cells=0)[0]

    # 阈值很小 -> 只有一个簇，合并应当**完全不变**
    same = detect_frontiers(grid, min_area_m2=0.0, merge_within_cells=1)[0]

    assert same.area_m2 == pytest.approx(raw.area_m2), "合并不该改变面积"
    assert same.centroid_world == pytest.approx(raw.centroid_world), "合并不该移动质心"
    assert same.cells.shape == raw.cells.shape, "合并不该增删格点"


def test_merge_joins_only_nearby_clusters():
    """合并的**唯一**效果是把靠得近的簇归为一组 —— 不碰任何簇的内部几何。"""
    size = 60
    data = np.full((size, size), UNKNOWN, dtype=np.int8)
    # 两个相距较远的自由岛，各带一圈 frontier
    data[10:20, 10:20] = FREE
    data[40:50, 40:50] = FREE
    grid = GridMap(data, RES, origin=(0.0, 0.0))

    raw = detect_frontiers(grid, min_area_m2=0.0, merge_within_cells=0)
    assert len(raw) == 2, "两个岛应当是两个独立的 frontier 环"

    total_area = sum(c.area_m2 for c in raw)

    # 阈值远大于两岛间距 -> 应当并成一个
    merged = detect_frontiers(grid, min_area_m2=0.0, merge_within_cells=1000)
    assert len(merged) == 1
    assert merged[0].area_m2 == pytest.approx(total_area), "合并后面积应当是两者之和"
    assert len(merged[0].cells) == sum(len(c.cells) for c in raw), "格点应当是并集"


def test_map_border_counts_as_unknown():
    """⭐ 语义统一后的行为：**地图外 = 未知**，所以全自由图的边界就是 frontier。

    原先 cv2.dilate 用默认边界值 0（图外当作「非未知」），于是全自由图
    检测不到任何 frontier。但 SLAM 的占据栅格通常是**裁到已探明区**的 ——
    地图边界正是「已知与未知的交界」，也就是 frontier 的定义本身。
    漏掉它会让探索提前判完成。

    见 grid.clearance_m / explorer._unknown_fraction_map —— 三处统一为同一语义。
    """
    data = np.full((20, 20), FREE, dtype=np.int8)
    grid = GridMap(data, RES, origin=(0, 0))
    clusters = detect_frontiers(grid, min_area_m2=0.0)

    assert len(clusters) == 1, "全自由图的边界应当构成一整圈 frontier"
    cells = clusters[0].cells
    rows, cols = cells[:, 0], cells[:, 1]
    assert rows.min() == 0 and rows.max() == 19, "应当是外圈"
    assert cols.min() == 0 and cols.max() == 19


def test_no_frontier_when_map_is_fully_enclosed():
    """真正探完的情形：自由区被占用区**完全包住**，边界不再是未知。"""
    data = np.full((24, 24), OCCUPIED, dtype=np.int8)
    data[2:22, 2:22] = FREE
    grid = GridMap(data, RES, origin=(0, 0))
    assert detect_frontiers(grid, min_area_m2=0.0) == []


# ----------------------------------------------------------------------
# 测地距离
# ----------------------------------------------------------------------
def test_geodesic_is_blocked_by_a_wall():
    """一堵贯通整列的墙 —— 墙另一侧必须不可达。

    这正是「欧氏距离会骗人」的场景：右侧看着很近，实际过不去。
    """
    data = np.full((20, 20), FREE, dtype=np.int8)
    data[:, 10] = OCCUPIED  # 整列墙，无门

    grid = GridMap(data, RES, origin=(0, 0))
    dist = geodesic_distance(grid.traversable, (10, 5))

    assert dist[10, 5] == 0
    assert dist[10, 8] == 3
    assert dist[10, 15] == -1, "墙另一侧必须不可达"


def test_geodesic_goes_around_through_a_door():
    """门开在**远离机器人所在行**的地方，路径必须绕行。

    若把门正好开在机器人那一行，测地距离会恰好等于欧氏距离，
    这个测试就失去意义了 —— 那也正是「欧氏距离何时骗人」的边界。
    """
    data = np.full((30, 30), FREE, dtype=np.int8)
    data[:, 10] = OCCUPIED
    data[2:5, 10] = FREE  # 门在最上方，离机器人（row 15）很远

    grid = GridMap(data, RES, origin=(0, 0))
    dist = geodesic_distance(grid.traversable, (15, 5))

    assert dist[15, 15] > 0, "透过门应当可达"
    # 必须北上绕到 row 3 再南下，距离显著大于欧氏直线
    assert dist[15, 15] > 3 * (15 - 5)


def test_geodesic_nearest_cell_skips_unreachable():
    data = np.full((20, 20), FREE, dtype=np.int8)
    data[:, 10] = OCCUPIED

    grid = GridMap(data, RES, origin=(0, 0))
    dist = geodesic_distance(grid.traversable, (10, 2))

    cells = np.array([[10, 15], [10, 8], [10, 7]])  # 一个不可达 + 两个可达
    assert geodesic_nearest_cell(dist, cells) == (10, 7)


def test_geodesic_returns_all_unreachable_when_start_blocked():
    data = np.full((10, 10), OCCUPIED, dtype=np.int8)
    grid = GridMap(data, RES, origin=(0, 0))
    dist = geodesic_distance(grid.traversable, (5, 5))
    assert (dist == -1).all()


def test_geodesic_respects_max_radius():
    data = np.full((100, 100), FREE, dtype=np.int8)
    grid = GridMap(data, RES, origin=(0, 0))
    dist = geodesic_distance(grid.traversable, (50, 50), max_radius_cells=5)

    assert dist[50, 50] == 0
    assert dist[50, 55] == 5
    assert dist[50, 60] == -1, "超出半径的格应记为不可达"


# ----------------------------------------------------------------------
# 目标点选择
# ----------------------------------------------------------------------
def test_raw_frontier_cells_have_inadequate_clearance():
    """记录「为什么需要 pullback」—— 这是本次实现中发现的真问题。

    frontier 格紧贴未知区，它一侧就是未知，clearance 只有一格。
    若直接对这些格做安全半径校验，房间场景下**全部会被拒绝**。
    """
    grid = room_map()
    clusters = detect_frontiers(grid, min_area_m2=0.0)
    clearance = grid.clearance_m()

    frontier_clearance = clearance[clusters[0].cells[:, 0], clusters[0].cells[:, 1]]

    assert frontier_clearance.max() <= 2 * RES
    assert (frontier_clearance < ExplorerParams().safety_radius_m).all()


def test_goal_is_selected_despite_frontier_clearance():
    """有了 pullback，房间场景必须能选出目标点（否则探索根本起不来）。"""
    grid = room_map()
    selector = GoalSelector()
    goal = must_pick(selector, grid)


def test_goal_satisfies_safety_radius():
    grid = room_map()
    params = ExplorerParams()
    selector = GoalSelector(params)
    goal = must_pick(selector, grid)
    assert goal.geodesic_dist_m > 0

    # 目标格本身必须是自由格，且 clearance 达标
    row, col = grid.world_to_grid(goal.x, goal.y)
    row, col = int(math.floor(row)), int(math.floor(col))
    assert grid.traversable[row, col]
    assert grid.clearance_m()[row, col] >= params.safety_radius_m


def test_goal_is_reachable():
    """选出的目标点必须真的能走到 —— 用测地距离验证。"""
    grid = room_map()
    selector = GoalSelector()
    goal = must_pick(selector, grid)

    start = grid.nearest_traversable_index(2.0, 2.0)
    dist = geodesic_distance(grid.traversable, start)

    row, col = grid.world_to_grid(goal.x, goal.y)
    assert dist[int(math.floor(row)), int(math.floor(col))] >= 0


def test_goal_yaw_faces_the_unknown_region():
    """朝向必须**背离房间中心**，指向外圈的未知区。

    若用整簇质心定朝向，闭环 frontier 的质心是房间中心，
    结果会是朝向指反 —— 狗到了边界却面朝房间，这一趟白跑且不报错。
    """
    grid = room_map()
    centre_x, centre_y = grid.grid_to_world(29.5, 29.5)

    selector = GoalSelector()
    goal = must_pick(selector, grid, robot_xy=(centre_x, centre_y))

    # 目标点相对房间中心的方向（即「向外」）
    ox, oy = goal.x - centre_x, goal.y - centre_y
    assert math.hypot(ox, oy) > 0.3, "目标点应确实离开了房间中心区域"

    fx, fy = math.cos(goal.yaw), math.sin(goal.yaw)
    assert (ox * fx + oy * fy) > 0, "朝向应指向房间外侧的未知区，而不是指回房间内部"


def test_goal_avoids_blacklisted_area():
    grid = room_map()
    selector = GoalSelector()

    first = must_pick(selector, grid)

    selector.on_result(first, success=False, now=0.0)
    second = must_pick(selector, grid, now=1.0)
    gap = math.hypot(second.x - first.x, second.y - first.y)
    assert gap >= ExplorerParams().blacklist_radius_m, "失败点应被避开"


def test_blacklist_expires():
    """黑名单不能永久生效 —— 地图更新后该点可能重新可达。"""
    p = ExplorerParams(blacklist_ttl_s=10.0)
    grid = room_map()
    selector = GoalSelector(p)

    first = must_pick(selector, grid)
    selector.on_result(first, success=False, now=0.0)

    # 过期之后应当允许重新选择同一个点
    assert pick_status(selector, grid, now=100.0) is SelectStatus.GOAL


def test_visit_penalty_discourages_revisiting():
    """访问过的区域应被打折，避免在两个 frontier 之间来回震荡。"""
    p = ExplorerParams(visit_penalty_radius_m=3.0, visit_penalty_factor=0.2)
    grid = room_map()
    selector = GoalSelector(p)

    first = must_pick(selector, grid)
    selector.on_result(first, success=True, now=0.0)

    sel2 = selector.select(grid, robot_xy=(2.0, 2.0), now=1.0)
    # ⚠️ 这里**不再**用 `if sel2.goal is not None` 包住断言 ——
    # 条件断言会让整条测试在最该报警的时候静默通过
    # （审查的变异测试证实：把访问惩罚恒置 0 时这条测试本来会空过）。
    assert sel2.status is SelectStatus.GOAL, "访问惩罚不该让选择器选不出点"
    gap = math.hypot(sel2.goal.x - first.x, sel2.goal.y - first.y)
    assert gap > 0.5, "成功访问后不应立刻回到同一点"


def test_gain_actually_influences_selection():
    """增益评分必须真的参与决策 —— 不能退化成「永远选测地最近的那个」。

    回归测试。初版实现里，每个簇只取「测地最近的一格」再打分，
    于是距离项永远压过一切，增益评分实际上从未参与过决策 ——
    那就退化成了它本要取代的朴素行为（找最近的 frontier）。

    构造：左右上下四个方向的候选**测地距离完全相等**，但左侧边界外是
    一整片实心占用块（窗口里几乎没有未知格），右侧是开阔未知区。
    只按距离选会挑中左侧那个毫无信息量的点。
    """
    size = 60
    data = np.full((size, size), UNKNOWN, dtype=np.int8)
    data[10:50, 10:50] = FREE
    data[:, 0:9] = OCCUPIED     # 左：整片实心
    data[0:9, :] = OCCUPIED     # 上：整片实心
    data[51:60, :] = OCCUPIED   # 下：整片实心

    grid = GridMap(data, RES, origin=(0, 0))
    centre_x, centre_y = grid.grid_to_world(29.5, 29.5)

    goal = must_pick(GoalSelector(), grid, robot_xy=(centre_x, centre_y))
    assert goal.x > centre_x + 0.5, "应选右侧开阔区，而不是左侧被实心块挡住的边界"
    assert goal.unknown_fraction > 0.15, "选中的目标点周围应当确实有未知区可探"


def test_no_goal_when_fully_explored():
    """自由区被占用区完全包住 -> 没有 frontier -> 探索完成。

    注意不能再用「整张图全自由」来构造这个场景了：
    统一语义之后图外算未知，全自由图的边界本身就是 frontier。
    """
    data = np.full((40, 40), OCCUPIED, dtype=np.int8)
    data[5:35, 5:35] = FREE
    grid = GridMap(data, RES, origin=(0, 0))
    assert pick_status(GoalSelector(), grid, robot_xy=(2.0, 2.0)) is SelectStatus.NO_FRONTIER


def test_no_goal_when_robot_trapped():
    """机器人所在格完全被占 -> 没有起点，返回 None 而不是崩溃。"""
    data = np.full((30, 30), OCCUPIED, dtype=np.int8)
    grid = GridMap(data, RES, origin=(0, 0))
    assert pick_status(GoalSelector(), grid, robot_xy=(1.5, 1.5)) is SelectStatus.NO_ROBOT_POSE


def test_wall_separated_frontier_is_not_chosen_as_nearest():
    """两个房间被墙隔开、只有远处一个门 —— 不应选中墙那侧的「近」frontier。

    这是 v1「直线回退」会犯的错：欧氏距离下墙那侧的 frontier 看着最近，
    实际要绕远路甚至根本过不去。
    """
    size = 60
    data = np.full((size, size), UNKNOWN, dtype=np.int8)
    data[10:50, 10:50] = FREE
    data[10:50, 29] = OCCUPIED      # 竖墙把房间一分为二
    data[10:14, 29] = FREE          # 门开在上方，离机器人很远

    grid = GridMap(data, RES, origin=(0, 0))
    selector = GoalSelector()

    # 机器人贴墙站在左半边
    goal = must_pick(selector, grid, robot_xy=(2.55, 3.0))
    # 无论选中哪边，目标点都必须真实可达
    start = grid.nearest_traversable_index(2.55, 3.0)
    dist = geodesic_distance(grid.traversable, start)
    row, col = grid.world_to_grid(goal.x, goal.y)
    assert dist[int(math.floor(row)), int(math.floor(col))] >= 0


# ----------------------------------------------------------------------
# 增益评分的回归保护
#
# ⚠️ 这一节是**补审查发现的一个大洞**：变异测试显示，把
# `exp(-p.lambda_decay * d_m)` 整个从增益里删掉，原来的 85 个测试
# **全部照过** —— 增益评分里最核心的距离项当时完全没有约束。
# visit penalty 的「距离渐变」同样（变异 M9 存活）。
# ----------------------------------------------------------------------
def test_gain_includes_the_distance_decay():
    """增益必须包含 exp(-λd)。删掉它这条测试必须失败。

    做法：用目标自身带的 `unknown_fraction` 与 `geodesic_dist_m` 独立复算期望值。
    新选择器没有访问记录，所以惩罚为 1，期望值只由「未知占比 × 距离衰减」构成。
    """
    grid = room_map()
    p = ExplorerParams()
    goal = must_pick(GoalSelector(p), grid)

    assert goal.geodesic_dist_m > 0, "距离必须为正，否则这条测试没有判别力"

    expected = goal.unknown_fraction * math.exp(-p.lambda_decay * goal.geodesic_dist_m)
    assert goal.gain == pytest.approx(expected, rel=1e-9), "增益里缺少距离衰减项"

    assert goal.gain < goal.unknown_fraction, "λ>0 时衰减项必然小于 1"


def test_distance_decay_direction_favours_nearer():
    """λ 越大越偏好近处 —— 验证衰减方向没写反。"""
    grid = room_map()
    far_goal = must_pick(GoalSelector(ExplorerParams(lambda_decay=0.01)), grid)
    near_goal = must_pick(GoalSelector(ExplorerParams(lambda_decay=1.0)), grid)
    assert near_goal.geodesic_dist_m <= far_goal.geodesic_dist_m


def test_visit_penalty_gradient():
    """访问惩罚必须**随距离渐变**，不能是「半径内一律乘系数」。

    平坦惩罚在对称场景下把所有邻近候选同比例打折、排序不变 ——
    惩罚形同虚设，机器人照旧来回震荡。对应变异 M9（原先存活）。
    """
    grid = room_map()
    p = ExplorerParams(visit_penalty_radius_m=2.0, visit_penalty_factor=0.3)
    sel = GoalSelector(p)
    sel.mark_visited(2.0, 2.0, now=0.0)

    r0, c0 = grid.world_to_grid(2.0, 2.0)
    r0, c0 = int(r0), int(c0)
    # 把访问点放在**格心**上，这样第一个候选到它的距离正好是 0
    # （放在 cell 角上的话距离是 0.07 m，惩罚就不是满值了）
    wx0, wy0 = grid.grid_to_world(r0, c0)
    sel._visits.clear()
    sel.mark_visited(wx0, wy0, now=0.0)
    candidates = np.array([[r0, c0], [r0 + 1, c0], [r0 + 30, c0]])

    pen = sel._visit_penalty(grid, candidates)

    assert pen[0] == pytest.approx(p.visit_penalty_factor), "正踩在上面应吃满惩罚"
    assert pen[0] < pen[1] < 1.0, "稍远一点惩罚应当更轻（渐变）"
    assert pen[2] == pytest.approx(1.0), "半径外不该受惩罚"


def test_param_invariants_are_enforced():
    """非法参数组合必须报错，不能静默变成「永远选不出点」的选择器。"""
    grid = room_map()

    bad = ExplorerParams(goal_pullback_m=0.3, safety_radius_m=0.45)
    with pytest.raises(ValueError, match="goal_pullback_m"):
        GoalSelector(bad).select(grid, robot_xy=ROOM_CENTRE_XY, now=0.0)

    with pytest.raises(ValueError):
        GoalSelector(ExplorerParams(visit_penalty_factor=0.0)).select(
            grid, robot_xy=ROOM_CENTRE_XY, now=0.0
        )

    with pytest.raises(ValueError):
        GoalSelector(ExplorerParams(lambda_decay=-1.0)).select(
            grid, robot_xy=ROOM_CENTRE_XY, now=0.0
        )


def test_radius_limited_search_falls_back_to_full_map():
    """所有 frontier 都在测地半径外时，必须兜底用全图再搜一次。

    ⚠️ 回归测试：`geodesic_distance` 把「搜索窗口外」与「真实不可达」
    共用同一个 -1，而候选过滤是 `dist >= 0`。窗口太小时 select 会返回
    「没有候选」—— 在旧接口下就是 None，被状态机当成探索完成**永久停机**。

    构造：42 m 长走廊（0.1 m/格），机器人在一端，唯一 frontier 在另一端，
    远超故意设小的 50 格窗口。
    """
    size = 460
    # 关键：走廊上下都填**占用**（不是未知），否则整条走廊的上下边缘
    # 全都是 frontier，最近的那个就在机器人旁边，窗口再小也够得着。
    # 只有远端之外留未知，frontier 才会唯一且很远。
    data = np.full((30, size), OCCUPIED, dtype=np.int8)
    data[10:20, 5:452] = FREE
    data[:, 452:] = UNKNOWN
    grid = GridMap(data, 0.1, origin=(0.0, 0.0))

    # min_area 要调小：远端开口只有 10 格（0.1 m²），默认的 0.25 m² 会把它滤掉
    sel = GoalSelector(
        ExplorerParams(max_geodesic_radius_cells=50, min_frontier_area_m2=0.05)
    )
    result = sel.select(grid, robot_xy=(0.6, 1.5), now=0.0)

    assert result.status is SelectStatus.GOAL, "窗口太小时应兜底全图搜索，而不是判「选不出点」"
    assert result.goal.used_full_map_search is True, "应标记走了全图兜底"


# ----------------------------------------------------------------------
# SMALL_FRONTIERS：把「有格但每簇都太小」与「真探完了」分开（审核 P1-⑤）
# ----------------------------------------------------------------------
def narrow_doorway_map(res=0.1, corridor_cells=2):
    """一个已探明的房间 + 一条**很窄**的开口通向未知区。

    开口宽 ``corridor_cells`` 格。在 0.1 m/格下，2 格 = 0.2 m 宽 ——
    比真实门洞还窄，但足以把「有 frontier 格」和「没有 frontier 格」分开。

    这就是审核实测的那个场景：真实门洞（1 m 宽 × 1 格深 ≈ 0.1 m²）
    在 ``min_area_m2=0.25`` 下会被整簇丢掉。
    """
    data = np.full((40, 40), OCCUPIED, dtype=np.int8)
    data[10:30, 10:30] = FREE                       # 已探明的房间
    data[10:30, 30:32] = UNKNOWN                    # 右边一片未知
    mid = 20
    data[mid - corridor_cells // 2: mid + corridor_cells // 2 + 1, 28:30] = FREE   # 窄通道
    return GridMap(data, res, origin=(0.0, 0.0))


def test_narrow_doorway_reports_small_frontiers_not_complete():
    """⭐ P1-⑤ 回归：有 frontier 格但每簇都太小 —— **不能说成「探索完成」**。

    原先 ``detect_frontiers`` 的**空列表**把两种情况混在一起，
    ``select()`` 一律映射成 ``NO_FRONTIER``，调用方据此进入不可逆的 ``DONE``。
    后果是机器人**站在还没进去过的门口宣布探索完成**，然后永久停机 ——
    直接打覆盖率指标。

    而小簇恰恰是真实门洞的样子：``min_frontier_area_m2=0.25`` 在 0.1 m/格下
    等于 25 格，而 frontier 条带通常只有 1 格深 —— 一簇要 2.5 米宽才算数。
    """
    grid = narrow_doorway_map()
    sel = GoalSelector(ExplorerParams(min_frontier_area_m2=0.25))

    assert has_frontier_cells(grid), "前置条件：这张图上确实有 frontier 格"
    status = pick_status(sel, grid, robot_xy=(2.0, 2.0))

    assert status is SelectStatus.SMALL_FRONTIERS, (
        f"有 frontier 格却报 {status} —— 「有格但太小」不能等于「探完了」"
    )
    assert not sel.select(grid, robot_xy=(2.0, 2.0), now=0.0).is_complete, \
        "SMALL_FRONTIERS 不是探索完成"


def test_fully_explored_map_reports_complete():
    """一块未知都没有（自由区被占用格完整包住）—— 这才是真的探完了。

    ⚠️ 注意不能拿「整张图全是自由格」来构造：``frontier_mask`` 的
    ``borderValue=1`` 把**地图外当未知**，所以那样构造出来的图在数组边界
    仍然有 frontier 格（这是刻意的语义，见 frontier_mask 的说明）。
    要让「一块未知都没有」成立，自由区必须被**占用格**完整包住。
    """
    data = np.full((40, 40), OCCUPIED, dtype=np.int8)
    data[10:30, 10:30] = FREE
    grid = GridMap(data, 0.1, origin=(0.0, 0.0))
    sel = GoalSelector(ExplorerParams())

    assert not has_frontier_cells(grid), "占用格包住的自由区不产生 frontier"
    assert pick_status(sel, grid, robot_xy=(2.0, 2.0)) is SelectStatus.NO_FRONTIER
    assert sel.select(grid, robot_xy=(2.0, 2.0), now=0.0).is_complete


def test_small_frontiers_disappears_once_the_threshold_fits():
    """把阈值调到装得下那簇，就应当正常选出目标 —— 状态是真的可恢复的。

    （这一条同时说明：``SMALL_FRONTIERS`` **持续**出现时该调 ``min_frontier_area_m2``，
    而不是永远空转重试。文档里写明了这一点。）
    """
    grid = narrow_doorway_map()
    sel = GoalSelector(ExplorerParams(min_frontier_area_m2=0.01))   # 1 格 = 0.01 m²

    assert pick_status(sel, grid, robot_xy=(2.0, 2.0)) is SelectStatus.GOAL


def test_has_frontier_cells_is_false_when_unknown_is_fully_enclosed():
    """未知区被占用格完全包住时，没有「与未知相邻的自由格」= 没有 frontier。"""
    data = np.full((20, 20), OCCUPIED, dtype=np.int8)
    data[8:12, 8:12] = UNKNOWN          # 一块未知，但四周都是占用
    grid = GridMap(data, 0.1, origin=(0.0, 0.0))

    assert has_frontier_cells(grid) is False
