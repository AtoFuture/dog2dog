"""测地距离（geodesic distance）—— 沿可通行区域的真实行进距离。

为什么不能直接用欧氏距离（见 docs/框架规划.md §4.3②）：

    frontier 常常位于**墙的另一侧**（透过门洞看到的未知区）。
    此时「机器人 -> frontier 质心」的欧氏直线**穿墙**，基于它做的回退选点
    会落在墙那边，机器人根本过不去。
    更隐蔽的是：欧氏距离在绕墙时会**严重低估**代价，导致增益评分选出一个
    实际要走很远的 frontier。

    正确做法是在自由空间上做 BFS / Dijkstra 展开，得到「沿可通行区域的步数」。
    explore_lite 的 ``min_distance`` 就是这么算的。

实现说明：用 numpy 波前（wavefront）迭代而不是 Python 队列 BFS。
每轮迭代对整幅掩膜做一次向量化的四邻域传播，轮数等于图的直径。
在 1000x1000 的地图上直径约数百格，即数百次 O(N) 的 numpy 运算 —— 1 Hz
的探索循环完全够用。若地图继续增大，应只对机器人周围的 ROI 子图计算
（见 ``max_radius_cells`` 参数）。
"""

from __future__ import annotations

import numpy as np

UNREACHABLE = -1


def geodesic_distance(
    traversable: np.ndarray,
    start_rc: tuple[int, int],
    max_radius_cells: int | None = None,
) -> np.ndarray:
    """从起点在可通行区域上做测地展开。

    Parameters
    ----------
    traversable : np.ndarray
        形状 ``(height, width)`` 的布尔掩膜，``True`` 表示该格可通行。
    start_rc : tuple[int, int]
        起点 ``(row, col)``。若该格不可通行则返回全 ``-1``。
    max_radius_cells : int | None
        只计算起点周围这么大范围内的距离，超出记为 ``-1``。
        ``None`` 表示算全图。长时探索后地图很大时建议设一个有限值
        （例如 300 格），既省算力也符合「只在附近选目标点」的实际需要。

    Returns
    -------
    np.ndarray
        与 ``traversable`` 同形状的 ``int32`` 数组，值为到起点的**格数**
        （4 连通步数），不可达为 ``-1``。乘 ``resolution`` 可得米。

    Notes
    -----
    采用 **4 连通**。8 连通虽然更接近欧氏距离，但允许沿两个对角障碍的
    缝隙「斜穿」过去，而那条缝在机器人footprint 下未必真的能过。
    保守起见与 explore_lite 保持一致。
    """
    traversable = np.asarray(traversable, dtype=bool)
    height, width = traversable.shape
    dist = np.full((height, width), UNREACHABLE, dtype=np.int32)

    r0, c0 = int(start_rc[0]), int(start_rc[1])
    if not (0 <= r0 < height and 0 <= c0 < width):
        return dist
    if not traversable[r0, c0]:
        return dist

    dist[r0, c0] = 0

    # 可选：把计算限制在起点周围的一个方形窗口内
    if max_radius_cells is not None:
        r_lo = max(0, r0 - max_radius_cells)
        r_hi = min(height, r0 + max_radius_cells + 1)
        c_lo = max(0, c0 - max_radius_cells)
        c_hi = min(width, c0 + max_radius_cells + 1)
        work = np.zeros_like(traversable)
        work[r_lo:r_hi, c_lo:c_hi] = traversable[r_lo:r_hi, c_lo:c_hi]
    else:
        work = traversable

    frontier = np.zeros_like(work)
    frontier[r0, c0] = True

    d = 0
    while frontier.any():
        d += 1
        nxt = np.zeros_like(frontier)
        # 四邻域传播
        nxt[1:, :] |= frontier[:-1, :]
        nxt[:-1, :] |= frontier[1:, :]
        nxt[:, 1:] |= frontier[:, :-1]
        nxt[:, :-1] |= frontier[:, 1:]
        # 只保留可通行且尚未访问的格
        nxt &= work & (dist < 0)
        if not nxt.any():
            break
        dist[nxt] = d
        frontier = nxt

    return dist


def geodesic_nearest_cell(
    dist: np.ndarray,
    cells: np.ndarray,
) -> tuple[int, int] | None:
    """在一组候选格中，取测地距离最小的那个。

    ⚠️ **``GoalSelector.select()`` 已经不用它了**（2026-09-17 注明）。

    早期版本的选点是「每个 frontier 簇取测地最近的一格再打分」，
    那就是用这个函数。但对抗性审查指出：只取最近的一格，
    距离项会永远压过其他因素，**增益评分实际上从未参与决策** ——
    退化成它本要取代的朴素行为。现在 `select()` 改成了对整个候选集向量化打分取最优。

    保留这个函数是因为它对**测试和诊断**仍有用（构造已知可达性、验证 BFS 结果），
    但**不要**再把它接回选点路径 —— 那会把已经修掉的问题带回来。


    这是选目标点的核心步骤：对一个 frontier 簇，不是取它的质心（质心可能
    在墙里或不可达），而是取**簇内到机器人测地距离最小**的那个格。

    Parameters
    ----------
    dist : np.ndarray
        ``geodesic_distance()`` 的输出。
    cells : np.ndarray
        形状 ``(N, 2)`` 的 ``(row, col)`` 候选格（例如 ``FrontierCluster.cells``）。

    Returns
    -------
    (row, col) 或 None（该簇内没有可达格）。
    """
    if cells.size == 0:
        return None

    rows, cols = cells[:, 0], cells[:, 1]
    d = dist[rows, cols]

    reachable = d >= 0
    if not reachable.any():
        return None

    # argmin 只在可达项上取，避免 -1 被误当成最小值
    idx = np.argmin(np.where(reachable, d, np.iinfo(d.dtype).max))
    return int(rows[idx]), int(cols[idx])
