"""占据栅格地图的解析与几何查询。

接口契约（见 docs/框架规划.md §4.3①）：
    G2 **只消费** ``nav_msgs/OccupancyGrid`` 的规范取值域
        -1 = 未知 / 0 = 自由 / 100 = 占用
    **不消费 Nav2 costmap** —— 它的内部取值域是 0-255（NO_INFORMATION=255,
    LETHAL_OBSTACLE=254, INSCRIBED_INFLATED_OBSTACLE=253, FREE_SPACE=0），
    语义与 OccupancyGrid 完全不同。若误把 costmap 当 OccupancyGrid，
    ``grid == -1`` 永远不成立，所有判定静默失效。

坐标约定：
    数组布局为 ``data[y, x]``，与 OccupancyGrid 的 row-major 一致；
    ``y`` 指向世界坐标的 +y 方向，索引 0 位于地图 origin 处（即左下角）。
    OpenCV 的图像惯例是 y 向下，但本模块只做逐元素运算（膨胀、连通域、
    距离变换），这些操作与数组朝向无关，因此不需要翻转。
    只有在 世界坐标 <-> 栅格索引 互转时才需要留意，见 world_to_grid()。
"""

from __future__ import annotations

import cv2
import numpy as np

# OccupancyGrid 规范取值域
UNKNOWN = -1
FREE = 0
OCCUPIED = 100


class GridMap:
    """对一帧 ``nav_msgs/OccupancyGrid`` 的轻量封装。

    Parameters
    ----------
    data : np.ndarray
        形状 ``(height, width)`` 的二维数组，值域 -1/0/100。
    resolution : float
        每格边长（米）。
    origin : tuple[float, float]
        地图左下角（索引 [0,0] 格的外角）在 ``map`` 系中的 (x, y)。
    """

    def __init__(self, data: np.ndarray, resolution: float, origin=(0.0, 0.0)):
        data = np.asarray(data)
        if data.ndim != 2:
            raise ValueError(f"data 必须是二维数组，得到 shape={data.shape}")
        if resolution <= 0:
            raise ValueError(f"resolution 必须为正，得到 {resolution}")

        self.data = data
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.height, self.width = data.shape

    # ------------------------------------------------------------------
    # 取值域判定
    # ------------------------------------------------------------------
    @property
    def free(self) -> np.ndarray:
        """自由格掩膜（bool）。"""
        return self.data == FREE

    @property
    def occupied(self) -> np.ndarray:
        """占用格掩膜（bool）。"""
        return self.data >= 100

    @property
    def unknown(self) -> np.ndarray:
        """未知格掩膜（bool）。"""
        return self.data < 0

    @property
    def traversable(self) -> np.ndarray:
        """可通行掩膜 —— **仅指自由格**。

        注意未知格不算可通行。规划目标点时必须只落在 ``free`` 上，
        不能落在 ``unknown`` 上（那正是「把目标点选进墙里」的成因之一）。
        """
        return self.free

    # ------------------------------------------------------------------
    # 坐标转换
    # ------------------------------------------------------------------
    def world_to_grid(self, wx: float, wy: float):
        """世界坐标 -> 栅格索引 ``(row, col)``。

        返回浮点索引，调用方自行取整；越界不报错（由调用方 clamp 或校验）。
        """
        col = (wx - self.origin[0]) / self.resolution
        row = (wy - self.origin[1]) / self.resolution
        return row, col

    def grid_to_world(self, row: float, col: float):
        """栅格索引 -> 该格**中心**的世界坐标 ``(wx, wy)``。"""
        wx = self.origin[0] + (col + 0.5) * self.resolution
        wy = self.origin[1] + (row + 0.5) * self.resolution
        return wx, wy

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def nearest_traversable_index(self, wx: float, wy: float, max_radius_cells: int = 10):
        """把世界坐标吸附到最近的可通行格。

        机器人位姿可能落在栅格边界外或未知格上（尤其在刚启动、地图还小的时候），
        直接索引会越界或判为不可通行。此方法在给定半径内螺旋搜索最近的自由格。

        Returns
        -------
        (row, col) 或 None（半径内找不到自由格）。
        """
        row, col = self.world_to_grid(wx, wy)
        row, col = int(np.floor(row)), int(np.floor(col))

        if self.in_bounds(row, col) and self.traversable[row, col]:
            return row, col

        for r in range(1, max_radius_cells + 1):
            for dr in range(-r, r + 1):
                for dc in range(-r, r + 1):
                    # 只看当前这一圈的外环
                    if max(abs(dr), abs(dc)) != r:
                        continue
                    rr, cc = row + dr, col + dc
                    if self.in_bounds(rr, cc) and self.traversable[rr, cc]:
                        return rr, cc
        return None

    # ------------------------------------------------------------------
    # 几何量
    # ------------------------------------------------------------------
    def clearance_m(self, precise: bool = True) -> np.ndarray:
        """每格到最近**非可通行格**的距离（米）。

        用于可通行性校验：目标点必须满足 ``clearance_m() >= 安全半径``。

        三个容易写错的约定（见 docs/框架规划.md §4.3②）：

        1. 传给 ``cv2.distanceTransform`` 的必须是 **traversable 掩膜**。
           该函数的语义是「每个非零像素到最近 **0** 像素的距离」——
           传可通行掩膜，得到的才是「离障碍/未知的距离」；传反了会得到
           「离自由空间的距离」，正好相反。
        2. 结果单位是**格**，必须乘 resolution 才是米。
        3. 默认的 ``DIST_L2`` + 3x3 掩膜是**近似**欧氏距离；
           要精确必须用 ``DIST_MASK_PRECISE``（代价是慢一些）。
        """
        mask = self.traversable.astype(np.uint8)
        flag = cv2.DIST_MASK_PRECISE if precise else cv2.DIST_MASK_3
        dist_cells = cv2.distanceTransform(mask, cv2.DIST_L2, flag)
        return dist_cells * self.resolution

    def unknown_ratio(self) -> float:
        """未知格占比。用于地图质量监控（突变说明建图异常）。"""
        return float(self.unknown.mean())
