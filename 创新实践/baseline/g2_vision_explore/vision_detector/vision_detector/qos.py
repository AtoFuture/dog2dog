"""QoS 配置集中管理。

--------------------------------------------------------------------------------
为什么单独成文件

**QoS 不匹配是 ROS2 视觉节点最常见的「一帧都收不到」，而且它不报错。**

rclpy 订阅默认是 RELIABLE，而相机/雷达话题几乎都是 BEST_EFFORT
（``qos_profile_sensor_data``）。两者不兼容时运行时**不抛异常、不打错误**，
只在启动时打一行 ``incompatible QoS ... RELIABILITY`` 警告，
然后订阅者一帧都收不到。

排查起来极其费时：节点起来了、话题名对、``ros2 topic hz`` 也有数据流，
就是回调不触发。所以这里把 QoS 显式写死、集中一处，不允许散落在各个订阅里。

--------------------------------------------------------------------------------
各话题的约定（见 docs/框架规划.md §2.2）

============================  ==================================================
图像 / 深度 / 点云             ``BEST_EFFORT`` + ``VOLATILE`` + ``KEEP_LAST(1~5)``
``/map``                      durability 与 G1 约定（建议 ``TRANSIENT_LOCAL``）
我们发布出去的检测消息          ``RELIABLE``（G3 不能丢检测）
============================  ==================================================

**联调第一件事**：``ros2 topic info <话题> --verbose`` 看实际 QoS，别猜。
"""

from __future__ import annotations

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

# 传感器数据：BEST_EFFORT。相机、深度、点云一律用这个。
#
# 与 rclpy 自带的 `qos_profile_sensor_data` 等价，但显式写出来 ——
# 免得有人以为默认的 RELIABLE 也行。
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# 准静态数据（内参等）：TRANSIENT_LOCAL，晚启动的订阅者也能拿到最后一帧。
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# 我们自己发出去的检测结果：RELIABLE。
# G3 用它做指标统计，丢一条就是指标少一条 —— 这里不能省。
RESULT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
)
