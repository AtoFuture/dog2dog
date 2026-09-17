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


def detect_frontiers(
    grid: GridMap,
    min_area_m2: float = 0.25,
    merge_gap_cells: int = 0,
) -> list[FrontierCluster]:
    """检测全部 frontier 簇。

    Parameters
    ----------
    grid : GridMap
    min_area_m2 : float
        最小簇面积（米²）。小于此值的簇被丢弃。
        默认 0.25 m² 是个偏保守的起点 —— 在 0.05 m/格 的地图上等于 100 格。
        太小会引入噪声簇，太大会漏掉真实的小门洞，**需按实际地图分辨率实测调整**。
    merge_gap_cells : int
        先对 frontier 掩膜做一次膨胀再连通，把相隔很近的碎片并成一个簇。
        **默认 0（不合并）**。

        ⚠️ 慎用：合并用的是「膨胀后再与自由区求交」，它**会把 frontier 区域本身
        向内扩张**，于是 (a) 簇的面积被放大、几何被改变；(b) 在「一个房间、四周未知」
        这种最常见场景下，整个环会被并成**一个**簇 —— 于是探索决策退化成
        「只有一个候选目标点」，而且该簇的质心是**房间中心**，
        据此算出的抵达朝向会**指向房间内部**，完全反了。

        如果确实需要合并碎片，更稳妥的做法是按簇质心距离合并，而不是形态学膨胀。

    Returns
    -------
    list[FrontierCluster]，按面积从大到小排序。
    """
    free = grid.free
    unknown = grid.unknown

    # 未知区膨胀 1 格，与自由区求交 = 与未知相邻的自由格
    dil = cv2.dilate(unknown.astype(np.uint8), np.ones((3, 3), np.uint8))
    frontier_mask = (free & (dil > 0)).astype(np.uint8)

    if merge_gap_cells > 0:
        k = 2 * merge_gap_cells + 1
        merged = cv2.dilate(frontier_mask, np.ones((k, k), np.uint8))
        # 膨胀后再与 free 求交，避免把膨胀出来的非自由格算进簇
        frontier_mask = (merged > 0).astype(np.uint8) & free.astype(np.uint8)

    if not frontier_mask.any():
        return []

    n_labels, labels = cv2.connectedComponents(frontier_mask, connectivity=8)

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

    clusters.sort(key=lambda c: c.area_m2, reverse=True)
    return clusters
