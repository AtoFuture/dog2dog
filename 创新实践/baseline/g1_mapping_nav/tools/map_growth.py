#!/usr/bin/env python3
"""从 rosbag 里量地图「边走边更新」——这是 P1 第 5 项的判据。

为什么要有它：
  P1 前 4 项用的是 AMCL + 静态地图，那张图进 bag 之后只有 1 条消息、永远不变，
  所以「地图在长」这件事根本没被量过。要证明 SLAM 在边跑边建图，必须把
  `/robot1/map` 按时间摊开看：已知栅格数、可通行栅格数、边界是否在扩。

判据（对齐 P1 第 5 项 / P2 出口标准）：
  · 地图更新数 > 1                  —— 至少不是一条静态图
  · 已知栅格数的末值 > 首值          —— 地图确实在长
  · 增长率非恒定（有若干段在长）      —— 「覆盖率曲线非恒定」的原话

用法（容器内，需要 rosbag2_py）：
    python3 tools/map_growth.py <bag目录> [--json out.json] [--every 5]
    python3 tools/map_growth.py <bag目录> --diff   # 另存首/末两张图的差

口径说明：
  OccupancyGrid 的 data 里 -1 = 未知，0 = free，100 = occupied。
  「已知」= 非 -1 的栅格；「可通行」= 值 < occupied_thresh 的栅格。
  两条曲线都会打印，判据用「已知」——free 会被动态障碍来回改，不如已知稳。
"""

import argparse
import json
import sys

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

MAP_TOPIC = "/robot1/map"
UNKNOWN = -1


def read_maps(bag):
    """返回 [(t, width, height, resolution, known, free, occupied)]，t 为包内相对秒。"""
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag, storage_id="sqlite3"),
        ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if MAP_TOPIC not in types:
        return None
    msg_type = get_message(types[MAP_TOPIC])

    rows, t0 = [], None
    while reader.has_next():
        topic, raw, bag_time = reader.read_next()
        if topic != MAP_TOPIC:
            continue
        t = bag_time * 1e-9
        if t0 is None:
            t0 = t
        msg = deserialize_message(raw, msg_type)
        values = msg.data
        known = sum(1 for v in values if v != UNKNOWN)
        free = sum(1 for v in values if v != UNKNOWN and v < 65)
        rows.append(
            (t - t0, msg.info.width, msg.info.height, msg.info.resolution,
             known, free, known - free)
        )
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--json", dest="json_out")
    parser.add_argument("--every", type=float, default=5.0,
                        help="采样打印间隔（秒，按包内相对时间）")
    args = parser.parse_args()

    rows = read_maps(args.bag)
    if rows is None:
        print(f"ERROR: 包里没有 {MAP_TOPIC}", file=sys.stderr)
        return 2
    if not rows:
        print(f"ERROR: {MAP_TOPIC} 没有消息", file=sys.stderr)
        return 2

    cell = rows[0][3] ** 2
    print(f"bag: {args.bag}")
    print(f"地图消息 {len(rows)} 条，"
          f"{rows[0][1]}x{rows[0][2]} @ {rows[0][3]} m/pix")
    print(f"\n{'t(s)':>7} {'已知(m²)':>10} {'可通行(m²)':>11} {'占比':>7}")
    last = -1e9
    for t, w, h, res, known, free, occ in rows:
        if t - last < args.every and t != rows[-1][0]:
            continue
        last = t
        print(f"{t:7.1f} {known * cell:10.1f} {free * cell:11.1f} "
              f"{known / (w * h) * 100:6.1f}%")

    first, end = rows[0], rows[-1]
    known_gain = end[4] - first[4]
    # 相邻两次更新里「在长」的比例，用来判曲线是否恒定
    growing = sum(
        1 for a, b in zip(rows, rows[1:]) if b[4] > a[4]
    )
    steps = max(len(rows) - 1, 1)

    print(f"\n已知栅格 {first[4]} → {end[4]}（{known_gain:+d} 格，"
          f"{known_gain * cell:+.1f} m²）")
    print(f"可通行   {first[5] * cell:.1f} → {end[5] * cell:.1f} m²")
    print(f"更新中在长的比例 {growing}/{steps}")

    checks = {
        "updates_gt_1": {"value": len(rows), "pass": len(rows) > 1},
        "known_grows": {"value": known_gain, "pass": known_gain > 0},
        "not_constant": {"value": growing, "pass": 0 < growing < steps},
    }
    labels = {
        "updates_gt_1": "地图更新数 > 1（不是一条静态图）",
        "known_grows": "已知栅格在增长（地图确实在长）",
        "not_constant": "增长率非恒定（覆盖率曲线非恒定）",
    }
    print()
    for key, res in checks.items():
        print(f"  {'✅' if res['pass'] else '❌'} {labels[key]}"
              f"（{res['value']}）")

    ok = all(c["pass"] for c in checks.values())
    print(f"\n判定：{'通过 ✅' if ok else '不通过 ❌'}")
    if not ok and len(rows) == 1:
        print("  提示：只有 1 条地图消息，说明跑的是 AMCL + 静态地图，"
              "不是 SLAM 边跑边建图。要验 P1 第 5 项得起 Nav2 时带 slam:=true。")

    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as fh:
            json.dump({
                "bag": args.bag,
                "updates": len(rows),
                "size": [first[1], first[2]],
                "resolution": first[3],
                "known_cells": [first[4], end[4]],
                "free_cells": [first[5], end[5]],
                "growing_updates": growing,
                "checks": checks,
                "pass": ok,
            }, fh, ensure_ascii=False, indent=2)
        print(f"报告已写入 {args.json_out}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
