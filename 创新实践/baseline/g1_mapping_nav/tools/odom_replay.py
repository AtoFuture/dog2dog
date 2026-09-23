#!/usr/bin/env python3
"""离线复现 QuadrupedOdometryNode 的积分逻辑，用来定位并验证修复。

为什么要有这个：
  足端里程计的问题在实机上每次验证都要重跑仿真（一轮约 90 s 挂钟），
  而算法本身是纯函数式的——给定同样的 joint 命令、foot_contact 和时钟，
  输出完全确定。所以把输入录一次，之后在离线反复试修法。

  先用 --variant current 复现实测值（校验复现器本身没写错），
  再换其它 variant 看哪个能把漂移压下来。

用法（容器内）：
  python3 tools/odom_replay.py <bag> [--variant current|count|prev|count+prev|noelse]
                                 [--fk-path <robot_FK.py 所在目录>]

variant 说明：
  current    原样复现：contact_count += 0.65、prev 只在接地时更新、无接地时按指令速度
  count      只把 0.65 换成真实计数
  prev       只让 prev 每拍都更新（脚落地时不再拿离地前的陈旧参考）
  count+prev 两者都改
  noelse     在 count+prev 基础上，去掉「按指令速度积分」的兜底
"""

import argparse
import math
import sys
from collections import deque

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

GT_TOPIC = "/g1/ground_truth"
ODOM_TOPIC = "/robot1/odom"
CLOCK_TOPIC = "/clock"
CONTACT_TOPIC = "/robot1/foot_contact"
JOINT_TOPIC = "/robot1/joint_group_controller/commands"
VELOCITY_TOPIC = "/robot1/robot_velocity"

# 与节点构造参数一致
BODY_DIMENSIONS = [0.3762, 0.0935]
LEG_DIMENSIONS = [0.0, 0.0955, 0.213, 0.213]
FILTER_WINDOW = 14
PUBLISH_RATE = 50


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def load(bag_path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id="sqlite3"),
        ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    clk = get_message(types[CLOCK_TOPIC])
    gt_t = get_message(types[GT_TOPIC])
    odom_t = get_message(types[ODOM_TOPIC])
    contact_t = get_message(types[CONTACT_TOPIC])
    joint_t = get_message(types[JOINT_TOPIC])
    vel_t = get_message(types[VELOCITY_TOPIC])

    # 单趟读取。注意时间基：
    #   /clock、/g1/ground_truth、/robot1/odom 用消息自带时间戳（仿真时间）
    #   其余输入节点只保留「最新值」，用 bag 接收时间做先后顺序即可
    clock, gt, odom, contacts, joints, velocities = [], [], [], [], [], []
    while reader.has_next():
        topic, raw, bag_t = reader.read_next()
        if topic == CLOCK_TOPIC:
            m = deserialize_message(raw, clk)
            # 存 (bag接收时间, 仿真时间) 对：bag 时间戳是挂钟，
            # 节点跑的是仿真时间，两者差一个实时率，必须建映射才能对齐输入。
            clock.append(
                (
                    bag_t * 1e-9,
                    float(m.clock.sec) + float(m.clock.nanosec) * 1e-9,
                )
            )
        elif topic == GT_TOPIC:
            m = deserialize_message(raw, gt_t)
            t = float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9
            gt.append(
                (
                    t,
                    m.pose.pose.position.x,
                    m.pose.pose.position.y,
                    yaw_from_quaternion(m.pose.pose.orientation),
                )
            )
        elif topic == ODOM_TOPIC:
            m = deserialize_message(raw, odom_t)
            t = float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9
            odom.append((t, m.pose.pose.position.x, m.pose.pose.position.y))
        elif topic == CONTACT_TOPIC:
            m = deserialize_message(raw, contact_t)
            contacts.append((bag_t * 1e-9, [bool(c) for c in m.contacts]))
        elif topic == JOINT_TOPIC:
            m = deserialize_message(raw, joint_t)
            joints.append((bag_t * 1e-9, list(m.data)))
        elif topic == VELOCITY_TOPIC:
            m = deserialize_message(raw, vel_t)
            velocities.append(
                (bag_t * 1e-9, m.cmd_vel.linear.x, m.cmd_vel.linear.y)
            )

    for series in (clock, gt, odom, contacts, joints, velocities):
        series.sort(key=lambda s: s[0])
    return clock, gt, odom, contacts, joints, velocities


def latest_at(series, t, default):
    """取时间 <= t 的最后一条（模拟订阅回调只保留最新值）。"""
    value = default
    for item in series:
        if item[0] <= t:
            value = item
        else:
            break
    return value


def replay(bag_path, fk_dir, variant):
    sys.path.insert(0, fk_dir)
    import robot_FK  # noqa: E402

    clock, gt, odom, contacts, joints, velocities = load(bag_path)
    if not clock:
        raise SystemExit("bag 里没有 /clock，无法复现")
    fk = robot_FK.ForwardKinematics(BODY_DIMENSIONS, LEG_DIMENSIONS)

    # 仿真时间 → 挂钟 的分段线性映射（输入话题按挂钟索引）
    sim_times = [c[1] for c in clock]
    wall_times = [c[0] for c in clock]

    def sim_to_wall(sim_t):
        if sim_t <= sim_times[0]:
            return wall_times[0]
        if sim_t >= sim_times[-1]:
            return wall_times[-1]
        lo, hi = 0, len(sim_times) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if sim_times[mid] <= sim_t:
                lo = mid
            else:
                hi = mid
        span = sim_times[hi] - sim_times[lo]
        if span <= 0:
            return wall_times[lo]
        frac = (sim_t - sim_times[lo]) / span
        return wall_times[lo] + frac * (wall_times[hi] - wall_times[lo])

    use_real_count = variant in ("count", "count+prev", "noelse")
    update_prev_always = variant in ("prev", "count+prev", "noelse")
    keep_else = variant != "noelse"

    t_start, t_end = sim_times[0], sim_times[-1]
    dt_tick = 1.0 / PUBLISH_RATE
    heading_series = [(g[0], g[3]) for g in gt]

    x = y = 0.0
    dxq, dyq = deque(maxlen=FILTER_WINDOW), deque(maxlen=FILTER_WINDOW)
    prev = [None, None, None, None]
    last_t = t_start
    traj = []

    # prev 在「前一拍」的足端位置，落地判定需要
    prev_all = [None, None, None, None]

    t = t_start
    while t <= t_end:
        dt = t - last_t
        last_t = t
        t += dt_tick
        if dt <= 0.0:
            continue

        # 输入话题按挂钟索引，先换算
        wall = sim_to_wall(t)
        joint_entry = latest_at(joints, wall, None)
        contact_entry = latest_at(contacts, wall, None)
        vel_entry = latest_at(velocities, wall, None)
        if joint_entry is None or len(joint_entry[1]) != 12:
            continue
        foot_positions = fk.forward_kinematics_all_legs(joint_entry[1])
        flags = contact_entry[1] if contact_entry else [False] * 4

        dx_total = dy_total = 0.0
        contact_count = 0.0
        for i in range(4):
            fx, fy = foot_positions[i][0], foot_positions[i][1]
            if flags[i]:
                ref = prev_all[i] if update_prev_always else prev[i]
                if ref is not None:
                    dx_total += fx - ref[0]
                    dy_total += -(fy - ref[1])
                    contact_count += 1.0 if use_real_count else 0.65
                prev[i] = (fx, fy)
            if update_prev_always:
                prev_all[i] = (fx, fy)
            elif flags[i]:
                prev_all[i] = (fx, fy)

        # 用真值航向代替 IMU（直线行驶时两者接近；隔离出 delta 算法的误差）
        heading = latest_at(heading_series, t, (0, 0.0))[1]

        if contact_count > 0:
            dx = dx_total / contact_count
            dy = dy_total / contact_count
        elif keep_else and vel_entry is not None:
            dx = vel_entry[1] * dt
            dy = vel_entry[2] * dt
        else:
            dx = dy = 0.0

        dxq.append(dx)
        dyq.append(dy)
        avg_dx = sum(dxq) / len(dxq)
        avg_dy = sum(dyq) / len(dyq)
        x += avg_dx * math.cos(heading) - avg_dy * math.sin(heading)
        y += avg_dx * math.sin(heading) + avg_dy * math.cos(heading)
        traj.append((t, x, y))

    return traj, gt, odom


def summarize(traj, gt, label):
    if len(traj) < 2:
        print(f"[{label}] 样本不足")
        return None
    _, x0, y0 = traj[0]
    _, xn, yn = traj[-1]
    net = math.hypot(xn - x0, yn - y0)
    path = sum(
        math.hypot(traj[i][1] - traj[i - 1][1], traj[i][2] - traj[i - 1][2])
        for i in range(1, len(traj))
    )
    print(f"[{label}] 净位移={net:.4f} m  路径长={path:.4f} m  n={len(traj)}")
    return net


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument(
        "--variant",
        default="current",
        choices=["current", "count", "prev", "count+prev", "noelse"],
    )
    parser.add_argument(
        "--fk-path",
        default="/home/wy/go2_sim_ws/ROS2-Gazebo-GO2/src/quadropted_controller/scripts/ForwardKinematics",
    )
    args = parser.parse_args()

    traj, gt, odom = replay(args.bag, args.fk_path, args.variant)

    print(f"variant = {args.variant}")
    summarize(traj, gt, "replay")
    if len(odom) >= 2:
        summarize(
            [(o[0], o[1], o[2]) for o in odom], gt, "实测 odom"
        )
    if len(gt) >= 2:
        _, gx0, gy0, _ = gt[0]
        _, gxn, gyn, _ = gt[-1]
        print(
            f"[真值]     净位移={math.hypot(gxn - gx0, gyn - gy0):.4f} m"
        )


if __name__ == "__main__":
    main()
