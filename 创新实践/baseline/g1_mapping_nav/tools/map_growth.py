#!/usr/bin/env python3
"""从 rosbag 里量地图「边走边更新」——这是 P1 第 5 项的判据。

## 为什么要这么判（2026-09-27 重写，对照问题清单 H1/H2）

前一版的判定是 `0 < growing < steps`，要求「至少有一次相邻更新没长」，
标签却写「增长率非恒定」——**方向正好反了**：狗一路走、地图每次更新都在长
（正是要证明的事）会被判**不通过**；而「只长一次、之后一直不变」反而**通过**。

而且它只比首末两条、**不读真值**，所以答的是「地图长过没有」，
不是 P1 第 5 项要的「**边走边**建图」。触发场景是项目已经吃过一次的失败：
控制器没激活、狗不动（README 坑 #1）——那时地图照样可能「长过」。

## 现在的判据

  · 地图更新数 > 1
  · 已知栅格末值 > 首值
  · **至少有一次增长发生在狗真的在动的时段**（相邻两次地图更新之间，
    真值路径长 > MOVE_MIN_M）—— 这条是「边走边建图」的正身
  · 覆盖率曲线非恒定（已知栅格出现过不止一个取值）

**包里没有真值就直接 rc=2 不通过**：没有真值就证明不了「增长与运动同期」，
这轮什么都没验到，不能静默放过（对照问题清单 H4/H5 那类静默判通过）。

用法（容器内，需要 rosbag2_py）：
    python3 tools/map_growth.py <bag目录> [--json out.json] [--every 5]

口径说明：
  OccupancyGrid 的 data 里 -1 = 未知，0 = free，100 = occupied。
  「已知」= 非 -1 的栅格；两条曲线都打印，判据用「已知」——free 会被动态
  障碍来回改，不如已知稳。地图与真值都按 **bag 接收时间**对齐（同一时基）。
"""

import argparse
import json
import sys

# rosbag2/rclpy 只在真读包时才需要 —— --selftest 不依赖 ROS，放模块顶层会让
# 自检在宿主机上直接 ImportError（本来就是给"不装 ROS 也能复核判定方向"用的）。
MAP_TOPIC = "/robot1/map"
GT_TOPIC = "/g1/ground_truth"
UNKNOWN = -1
# 相邻两次地图更新之间真值要走够这么多，才算这次增长是「边走边长」的
MOVE_MIN_M = 0.5


def _open(bag):
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag, storage_id="sqlite3"),
        ConverterOptions("", ""),
    )
    return reader, {t.name: t.type for t in reader.get_all_topics_and_types()}


def read_bag(bag):
    """一趟读回地图与真值，两个序列都用 bag 接收时间（同一时基）。

    返回 (maps, gt)；maps 为 [(t, w, h, res, known, free)]，gt 为 [(t, x, y)]。
    缺哪个就返回 None。
    """
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader, types = _open(bag)
    if MAP_TOPIC not in types:
        return None, None
    map_type = get_message(types[MAP_TOPIC])
    gt_type = get_message(types[GT_TOPIC]) if GT_TOPIC in types else None

    maps, gt, t0 = [], [], None
    while reader.has_next():
        topic, raw, bag_time = reader.read_next()
        if topic not in (MAP_TOPIC, GT_TOPIC):
            continue
        t = bag_time * 1e-9
        if t0 is None:
            t0 = t
        t -= t0
        if topic == MAP_TOPIC:
            msg = deserialize_message(raw, map_type)
            values = msg.data
            known = sum(1 for v in values if v != UNKNOWN)
            free = sum(1 for v in values if v != UNKNOWN and v < 65)
            maps.append((t, msg.info.width, msg.info.height,
                         msg.info.resolution, known, free))
        elif gt_type is not None:
            p = deserialize_message(raw, gt_type).pose.pose.position
            gt.append((t, p.x, p.y))
    return maps, gt


def path_between(gt, t_a, t_b):
    """真值在 [t_a, t_b] 之间累积的路径长（不是净位移：来回磨蹭也算动过）"""
    seg = [g for g in gt if t_a <= g[0] <= t_b]
    return sum(
        ((seg[i][1] - seg[i - 1][1]) ** 2 + (seg[i][2] - seg[i - 1][2]) ** 2) ** 0.5
        for i in range(1, len(seg))
    )


def judge(rows, gt):
    """判定核心。抽成函数是为了能用合成数据复现（见 --selftest）。

    rows: [(t, w, h, res, known, free)]，gt: [(t, x, y)]
    返回 (metrics, checks, ok)
    """
    first, end = rows[0], rows[-1]
    grew_while_moving, grew_total, moved_total = 0, 0, 0
    for a, b in zip(rows, rows[1:]):
        dist = path_between(gt, a[0], b[0])
        if dist > MOVE_MIN_M:
            moved_total += 1
        if b[4] > a[4]:
            grew_total += 1
            if dist > MOVE_MIN_M:
                grew_while_moving += 1

    # 「至少一半」：只长一次、之后狗一直走却不再长，不算「边走边建图」
    need = max(1, (moved_total + 1) // 2)
    metrics = {
        "updates": len(rows),
        "known_gain": end[4] - first[4],
        "distinct_known": len({r[4] for r in rows}),
        "grew_total": grew_total,
        "moved_total": moved_total,
        "grew_while_moving": grew_while_moving,
        "need": need,
        "gt_path": round(path_between(gt, gt[0][0], gt[-1][0]), 3),
    }
    checks = {
        "updates_gt_1": {"value": metrics["updates"],
                         "pass": metrics["updates"] > 1},
        "known_grows": {"value": metrics["known_gain"],
                        "pass": metrics["known_gain"] > 0},
        "grew_while_moving": {"value": grew_while_moving,
                              "pass": grew_while_moving >= need},
        "curve_non_constant": {"value": metrics["distinct_known"],
                               "pass": metrics["distinct_known"] > 1},
    }
    return metrics, checks, all(c["pass"] for c in checks.values())


def selftest():
    """用合成数据复现问题清单 H1/H2 的三档场景，验证判定方向不再反。"""
    def mk(known_seq, dt=10.0, step=1.0):
        """known_seq 给出每次地图更新的已知栅格数；真值按 step m/间隔匀速前进"""
        rows = [(i * dt, 100, 100, 0.05, k, k) for i, k in enumerate(known_seq)]
        gt = [(i * dt / 4, i * dt / 4 * step, 0.0)
              for i in range(len(known_seq) * 4 + 1)]
        return rows, gt

    cases = [
        ("狗一路走、地图每次都长（正是要证明的）",
         mk([1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000]), True),
        ("只长一次、之后狗继续走但地图不再长",
         mk([1000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000]), False),
        ("只有 2 条地图消息、确实长了",
         mk([1000, 1200]), True),
        ("狗一步没走、地图却长过（控制器没激活那类）",
         mk([1000, 1500, 2000], step=0.0), False),
    ]
    bad = 0
    for name, (rows, gt), want in cases:
        _, checks, ok = judge(rows, gt)
        flag = "✅" if ok == want else "❌ 判定与预期不符"
        if ok != want:
            bad += 1
        print(f"  {flag}  预期{'通过' if want else '不通过'} / 实得{'通过' if ok else '不通过'}"
              f"  —— {name}")
        if ok != want:
            for k, v in checks.items():
                print(f"        {k}: pass={v['pass']} value={v['value']}")
    print(f"\n自检：{len(cases) - bad}/{len(cases)} 档符合预期")
    return 0 if bad == 0 else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", nargs="?", help="rosbag2 目录")
    parser.add_argument("--json", dest="json_out")
    parser.add_argument("--every", type=float, default=5.0,
                        help="采样打印间隔（秒，按包内相对时间）")
    parser.add_argument("--selftest", action="store_true",
                        help="用合成数据复核判定方向（不需要 bag）")
    args = parser.parse_args()

    if args.selftest:
        return selftest()
    if not args.bag:
        parser.error("要么给 bag 目录，要么用 --selftest")

    rows, gt = read_bag(args.bag)
    if rows is None:
        print(f"ERROR: 包里没有 {MAP_TOPIC}", file=sys.stderr)
        return 2
    if not rows:
        print(f"ERROR: {MAP_TOPIC} 没有消息", file=sys.stderr)
        return 2
    if not gt:
        print(f"ERROR: 包里没有 {GT_TOPIC} —— 没有真值就证明不了「增长与运动同期」，"
              "这轮等于没验（对照问题清单 H4/H5：缺数据不许静默判通过）。",
              file=sys.stderr)
        return 2

    cell = rows[0][3] ** 2
    print(f"bag: {args.bag}")
    print(f"地图消息 {len(rows)} 条，{rows[0][1]}x{rows[0][2]} @ {rows[0][3]} m/pix；"
          f"真值 {len(gt)} 条")
    print(f"\n{'t(s)':>7} {'已知(m²)':>10} {'可通行(m²)':>11} {'占比':>7}")
    last = -1e9
    for t, w, h, res, known, free in rows:
        if t - last < args.every and t != rows[-1][0]:
            continue
        last = t
        print(f"{t:7.1f} {known * cell:10.1f} {free * cell:11.1f} "
              f"{known / (w * h) * 100:6.1f}%")

    metrics, checks, ok = judge(rows, gt)
    first = rows[0]
    print(f"\n已知栅格 {first[4]} → {rows[-1][4]}"
          f"（{metrics['known_gain']:+d} 格，{metrics['known_gain'] * cell:+.1f} m²）")
    print(f"可通行   {first[5] * cell:.1f} → {rows[-1][5] * cell:.1f} m²")
    print(f"地图更新 {metrics['updates']} 次；其中「长了」{metrics['grew_total']} 次，"
          f"「狗在动」{metrics['moved_total']} 次，"
          f"**两者同时** {metrics['grew_while_moving']} 次（要求 ≥{metrics['need']}）")
    print(f"真值路径长 {metrics['gt_path']:.2f} m")

    labels = {
        "updates_gt_1": "地图更新数 > 1（不是一条静态图）",
        "known_grows": "已知栅格在增长",
        "grew_while_moving": f"狗在动的时段里至少一半同时看到地图在长"
                             f"（>{MOVE_MIN_M} m 才算在动）",
        "curve_non_constant": "覆盖率曲线非恒定（已知栅格不止一个取值）",
    }
    print()
    for key, res in checks.items():
        print(f"  {'✅' if res['pass'] else '❌'} {labels[key]}"
              f"（{res['value']}）")

    print(f"\n判定：{'通过 ✅' if ok else '不通过 ❌'}")
    if len(rows) == 1:
        print("  提示：只有 1 条地图消息，说明跑的是 AMCL + 静态地图，"
              "不是 SLAM 边跑边建图。要验 P1 第 5 项得起 Nav2 时带 slam:=true。")
    elif not checks["grew_while_moving"]["pass"] and checks["known_grows"]["pass"]:
        print("  提示：地图确实长过，但**不是狗在动的时候长的** —— "
              "这不算「边走边建图」，对照问题清单 H2。")

    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as fh:
            json.dump({
                "bag": args.bag,
                "updates": len(rows),
                "size": [first[1], first[2]],
                "resolution": first[3],
                "known_cells": [first[4], end[4]],
                "free_cells": [first[5], end[5]],
                "grew_total": grew_total,
                "moved_total": moved_total,
                "grew_while_moving": grew_while_moving,
                "gt_path_m": round(gt_path, 3),
                "checks": checks,
                "pass": ok,
            }, fh, ensure_ascii=False, indent=2)
        print(f"报告已写入 {args.json_out}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
