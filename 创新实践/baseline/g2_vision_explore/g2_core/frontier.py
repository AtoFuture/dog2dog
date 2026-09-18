"""frontier（探索边界）检测。

定义：**自由格与未知格相邻**的位置即为 frontier —— 那是「已知」与「未知」的交界，
走过去就能看到新东西。

实现要点（见 docs/框架规划.md §4.3①）：

* 用 ``cv2.dilate`` + ``cv2.connectedComponents``，比纯 Python 遍历快几个数量级。
* 得到的是**簇**而不是单个格子 —— 后续每个簇出一个候选目标点。
* **必须有最小簇过滤**。栅格噪声会产出大量单格小簇，每个都会被当成候选目标点，
  导致机器人来回白跑。explore_lite 的 ``min_frontier_size`` 默认 0.5 m 就是干这个的。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from .grid import GridMap


@dataclass
class FrontierCluster:
    """一个 frontier 簇。

    Attributes
    ----------
    label : int
        连通域标号（仅用于调试）。
    cells : np.ndarray
        形状 ``(N, 2)`` 的 ``(row, col)`` 索引数组。
    centroid_world : tuple[float, float]
        簇的几何中心（``map`` 系）。**注意这不是可通行点**，只是用来定方向。
    area_m2 : float
        簇占用的地图面积（米²）。
    """

    label: int
    cells: np.ndarray
    centroid_world: tuple[float, float]
    area_m2: float
    _cell_array: np.ndarray = field(default=None, repr=False)


def frontier_mask(grid: GridMap) -> np.ndarray:
    """frontier 格掩膜（uint8，0/1）：**与未知区相邻的自由格**。

    抽成独立函数是为了让「有没有 frontier 格」这个判据
    （``has_frontier_cells``）与真正的簇检测**共用同一份定义** ——
    各写一遍迟早会漂移，而漂移的后果是「有 frontier 却说没有」这种静默错误。

    语义边界（2026-09-17 统一）：**地图外当作未知**。
    """
    # 未知区膨胀 1 格，与自由区求交 = 与未知相邻的自由格。
    #
    # ⚠️ borderValue=1：把**地图外**当作未知。
    # cv2.dilate 默认的边界值是 0，等于把图外当成「非未知」——
    # 于是地图被裁剪到已探明区时，边界上的真实 frontier **检测不到**，
    # 探索会提前判完成。SLAM 的图通常正是裁到已探明区的，所以这不是边角情况。
    dil = cv2.dilate(
        grid.unknown.astype(np.uint8),
        np.ones((3, 3), np.uint8),
        borderType=cv2.BORDER_CONSTANT,
        borderValue=1,
    )
    return (grid.free & (dil > 0)).astype(np.uint8)


def has_frontier_cells(grid: GridMap) -> bool:
    """地图上**有没有** frontier 格（不管簇多小）。

    用来把两种情况分开 —— 它们原先都被 ``detect_frontiers`` 的**空列表**表示
    （见 ``explorer.SelectStatus.SMALL_FRONTIERS``）：

        没有 frontier 格        -> 真的探完了
        有格、但每簇都太小      -> 只是小而已（真实门洞就长这样），**没探完**

    不分开的后果不是「漏了一个点」，而是**把没探完说成探完了**，
    状态机据此进入不可逆的 ``DONE``。
    """
    return bool(frontier_mask(grid).any())


def detect_frontiers(
    grid: GridMap,
    min_area_m2: float = 0.25,
    merge_within_cells: int = 0,
) -> list[FrontierCluster]:
    """检测全部 frontier 簇。

    Parameters
    ----------
    grid : GridMap
    min_area_m2 : float
        最小簇面积（米²）。小于此值的簇被丢弃。
        默认 0.25 m² 是个偏保守的起点 —— 在 0.05 m/格 的地图上等于 100 格。
        太小会引入噪声簇，太大会漏掉真实的小门洞，**需按实际地图分辨率实测调整**。
    merge_within_cells : int
        把质心距离小于这么多格的簇**并成一个**。**默认 0（不合并）**。

        ⚠️ 这里刻意**不用形态学膨胀**来实现合并（2026-09-17 重写）。

        膨胀的做法是「把 frontier 掩膜膨胀后再与自由区求交」，它有两个副作用：

        1. **改变 frontier 本身的几何** —— 区域被向内扩张、面积被放大，
           而簇的 `cells` / `centroid_world` / `area_m2` 就不再是真实的 frontier 了。
        2. 在「一个房间、四周未知」这种最常见场景下，整个环会被并成**一个**簇，
           其质心恰好是**房间中心** —— 据此算出的抵达朝向会指向房间内部，完全反了。

        现在的做法是：先按连通域得到真实簇，再**按质心距离合并**。
        合并只改变「哪些簇算一个」，不改变任何簇的几何。

    Returns
    -------
    list[FrontierCluster]，按面积从大到小排序。
    """
    fmask = frontier_mask(grid)
    if not fmask.any():
        return []

    n_labels, labels = cv2.connectedComponents(fmask, connectivity=8)

    # 用 bincount 一次性统计各标号的格数，避免逐簇布尔索引
    counts = np.bincount(labels.ravel(), minlength=n_labels)
    min_cells = max(1, int(np.ceil(min_area_m2 / (grid.resolution ** 2))))

    clusters: list[FrontierCluster] = []
    for label in range(1, n_labels):  # 0 是背景
        if counts[label] < min_cells:
            continue

        cells = np.argwhere(labels == label)  # (N, 2) 的 (row, col)
        rows, cols = cells[:, 0], cells[:, 1]

        # 栅格中心的世界坐标均值
        cx = grid.origin[0] + (cols.mean() + 0.5) * grid.resolution
        cy = grid.origin[1] + (rows.mean() + 0.5) * grid.resolution

        clusters.append(
            FrontierCluster(
                label=int(label),
                cells=cells,
                centroid_world=(float(cx), float(cy)),
                area_m2=float(counts[label]) * grid.resolution ** 2,
            )
        )

    if merge_within_cells > 0 and len(clusters) > 1:
        clusters = _merge_by_centroid(clusters, grid.resolution * merge_within_cells)

    clusters.sort(key=lambda c: c.area_m2, reverse=True)
    return clusters


def _merge_by_centroid(
    clusters: list[FrontierCluster], max_dist_m: float
) -> list[FrontierCluster]:
    """把质心距离小于 ``max_dist_m`` 的簇并成一个（并查集）。

    合并只动「分组」，**不动任何簇的格点** ——
    合并后的簇保留各成员的全部 cells，面积是成员之和。
    """
    n = len(clusters)
    parent = list(range(n))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i: int, j: int) -> None:
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[rj] = ri

    for i in range(n):
        xi, yi = clusters[i].centroid_world
        for j in range(i + 1, n):
            xj, yj = clusters[j].centroid_world
            if math.hypot(xi - xj, yi - yj) <= max_dist_m:
                union(i, j)

    groups: dict[int, list[int]] = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)

    merged: list[FrontierCluster] = []
    for root, members in groups.items():
        if len(members) == 1:
            merged.append(clusters[root])
            continue

        cells = np.vstack([clusters[m].cells for m in members])

        # 合并后的质心 = 成员质心按**格数**加权平均。
        # 这与「直接对所有格点求均值」等价（各簇质心本就是其格点的均值），
        # 而按格数加权是必须的 —— 直接平均成员质心会让小簇的权重被高估。
        weights = np.array([len(clusters[m].cells) for m in members], dtype=np.float64)
        wsum = weights.sum()
        cx = sum(w * clusters[m].centroid_world[0] for w, m in zip(weights, members)) / wsum
        cy = sum(w * clusters[m].centroid_world[1] for w, m in zip(weights, members)) / wsum

        merged.append(
            FrontierCluster(
                label=clusters[root].label,
                cells=cells,
                centroid_world=(float(cx), float(cy)),
                area_m2=sum(clusters[m].area_m2 for m in members),
            )
        )
    return merged
