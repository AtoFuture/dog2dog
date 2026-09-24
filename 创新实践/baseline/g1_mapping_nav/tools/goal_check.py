#!/usr/bin/env python3
"""从录好的 rosbag 里按**真值**判定「这一轮到底到没到」，并顺带查冻结。

为什么要有它：
  1) `goal_test.sh` 原来只 echo 一次真值到 `goal_end_gt.txt`，**不跟目标点比**，
     那个文件还出现过 0 字节（真值读失败）而脚本照样 exit 0。P1 的出口标准
     要求「不得再仅以 action 的 SUCCEEDED 判定真实到达」，但判定一直是人在看。
  2) 更要紧的是「有指令但不动」这种失败：2026-09-24 查出 amcl_G5 那一轮
     真值原地冻结 44 s，同期 Nav2 在正常下发路径与速度，只有把
     `cmd_vel_scaled_out` 和真值摆在一起才看得出来。

判据（对齐 P1 出口标准）：
  · 到点误差 ≤ 0.30 m               —— 真值终点 vs 目标点
  · 冻结时长 ≤ 5 s（且当时 Nav2 有非零线速度指令）
 任一不满足则以 rc=1 退出，可直接当流程闸门。

用法（容器内，需要 rosbag2_py）：
    python3 tools/goal_check.py <bag目录> <目标x> <目标y> [--yaw 0.0] [--json out.json]
"""

import argparse
import json
import math
import sys

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

GT_TOPIC = "/g1/ground_truth"
CMD_TOPIC = "/robot1/cmd_vel_scaled_out"
AMCL_TOPIC = "/robot1/amcl_pose"

# P1 出口标准
GOAL_ERROR_LIMIT_M = 0.30
FROZEN_LIMIT_S = 5.0
# 「没在动」的真值速度阈值 / 「Nav2 在要求它动」的指令阈值
STILL_SPEED = 0.01
CMD_EPS = 0.005
# 短于此值的冻结段是采样噪声，不报（起步/收尾时常见）
MIN_WINDOW_S = 0.2
# 取终点位的尾窗，压掉抖动
TAIL_WINDOW_S = 1.0


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def read_bag(bag):
    """单趟读取，返回 {(topic): [(t, x, y, z, yaw)]}；t 用 bag 接收时间统一。"""
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag, storage_id="sqlite3"),
        ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {
        name: get_message(types[name])
        for name in (GT_TOPIC, CMD_TOPIC, AMCL_TOPIC)
        if name in types
    }
    missing = [n for n in (GT_TOPIC, CMD_TOPIC) if n not in types]
    if missing:
        print(f"⚠️  bag 里缺话题：{missing}", file=sys.stderr)

    tracks = {GT_TOPIC: [], CMD_TOPIC: [], AMCL_TOPIC: []}
    t0 = None
    while reader.has_next():
        topic, raw, bag_time = reader.read_next()
        msg_type = wanted.get(topic)
        if msg_type is None:
            continue
        t = bag_time * 1e-9
        if t0 is None:
            t0 = t
        t -= t0
        msg = deserialize_message(raw, msg_type)
        if topic == CMD_TOPIC:
            tracks[topic].append((t, msg.linear.x, msg.linear.y, 0.0, 0.0))
            continue
        p = msg.pose.pose
        tracks[topic].append(
            (t, p.position.x, p.position.y, p.position.z,
             yaw_from_quaternion(p.orientation))
        )
    for series in tracks.values():
        series.sort(key=lambda s: s[0])
    return tracks


def freeze_windows(gt, cmd):
    """找「真值没动、但 Nav2 要求它在动」的连续区间，返回 [(起, 止, 秒)]。"""
    windows, start = [], None
    for i in range(1, len(gt)):
        t, x, y = gt[i][0], gt[i][1], gt[i][2]
        dt = t - gt[i - 1][0]
        if dt <= 0.0 or dt > 0.5:
            continue
        moving = math.hypot(x - gt[i - 1][1], y - gt[i - 1][2]) / dt >= STILL_SPEED
        commanded = max(
            (abs(c[1]) + abs(c[2]) for c in cmd if abs(c[0] - t) < 0.25),
            default=0.0,
        ) > CMD_EPS
        if not moving and commanded:
            if start is None:
                start = gt[i - 1][0]
        elif start is not None:
            windows.append((start, gt[i - 1][0]))
            start = None
    if start is not None:
        windows.append((start, gt[-1][0]))
    return [
        (a, b, b - a) for a, b in windows if b - a >= MIN_WINDOW_S
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="rosbag2 目录")
    parser.add_argument("goal_x", type=float)
    parser.add_argument("goal_y", type=float)
    parser.add_argument("--yaw", type=float, default=None,
                        help="目标朝向（度）。注意：包里**没有** goal 的朝向"
                             "（goal 走 service，录不到），必须自己填发目标时用的值；"
                             "填错这行就无意义，所以它不参与判定")
    parser.add_argument("--json", dest="json_out")
    args = parser.parse_args()

    tracks = read_bag(args.bag)
    gt = tracks[GT_TOPIC]
    if len(gt) < 2:
        print(f"ERROR: {GT_TOPIC} 样本不足（{len(gt)} 条），bag 里没录到真值？",
              file=sys.stderr)
        return 2

    # 终点取最后 1 秒的均值
    t_end = gt[-1][0]
    tail = [g for g in gt if g[0] >= t_end - TAIL_WINDOW_S] or [gt[-1]]
    ex = sum(g[1] for g in tail) / len(tail)
    ey = sum(g[2] for g in tail) / len(tail)
    error = math.hypot(ex - args.goal_x, ey - args.goal_y)
    closest = min(math.hypot(g[1] - args.goal_x, g[2] - args.goal_y) for g in gt)
    zs = [g[3] for g in gt]

    windows = freeze_windows(gt, tracks[CMD_TOPIC])
    longest = max((w[2] for w in windows), default=0.0)

    # AMCL 对真值的偏差（地图与世界恒等对齐时可直接比）
    amcl_dev = 0.0
    for t, x, y, _, _ in tracks[AMCL_TOPIC]:
        near = min(gt, key=lambda g: abs(g[0] - t))
        amcl_dev = max(amcl_dev, math.hypot(x - near[1], y - near[2]))

    report = {
        "bag": args.bag,
        "goal": [args.goal_x, args.goal_y],
        "end_ground_truth": [round(ex, 4), round(ey, 4)],
        "goal_error_m": round(error, 4),
        "goal_error_limit_m": GOAL_ERROR_LIMIT_M,
        "closest_approach_m": round(closest, 4),
        "z_range": [round(min(zs), 4), round(max(zs), 4)],
        "duration_s": round(gt[-1][0] - gt[0][0], 2),
        "frozen_windows": [
            {"start_s": round(w[0], 2), "end_s": round(w[1], 2),
             "seconds": round(w[2], 2)} for w in windows
        ],
        "frozen_longest_s": round(longest, 2),
        "frozen_limit_s": FROZEN_LIMIT_S,
        "amcl_max_deviation_m": round(amcl_dev, 4),
    }
    if args.yaw is not None:
        yaw_err = math.degrees(abs(
            math.remainder(gt[-1][4] - math.radians(args.yaw), 2 * math.pi)
        ))
        report["yaw_error_deg"] = round(yaw_err, 2)

    print(f"bag: {args.bag}")
    print(f"目标 ({args.goal_x}, {args.goal_y})   终点真值 ({ex:.3f}, {ey:.3f})")
    print(f"  到点误差   {error:.3f} m   （限值 {GOAL_ERROR_LIMIT_M}）")
    print(f"  最近接近   {closest:.3f} m")
    if args.yaw is not None:
        print(f"  朝向误差   {report['yaw_error_deg']:.1f}°")
    print(f"  z 范围     {min(zs):.3f} ~ {max(zs):.3f}")
    print(f"  AMCL 偏差  {amcl_dev:.4f} m")
    if windows:
        print(f"  ⚠️ 冻结    共 {len(windows)} 段，最长 {longest:.1f} s "
              f"（限值 {FROZEN_LIMIT_S}）—— 有指令但真值不动：")
        for w in windows:
            print(f"              t={w[0]:6.1f}~{w[1]:<6.1f} {w[2]:5.1f} s")
    else:
        print(f"  冻结       无（限值 {FROZEN_LIMIT_S} s）")

    ok = error <= GOAL_ERROR_LIMIT_M and longest <= FROZEN_LIMIT_S
    print(f"\n判定：{'通过 ✅' if ok else '不通过 ❌'}")

    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as fh:
            json.dump(report, fh, ensure_ascii=False, indent=2)
        print(f"报告已写入 {args.json_out}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
