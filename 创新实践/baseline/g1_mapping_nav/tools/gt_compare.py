#!/usr/bin/env python3
"""比较 rosbag 里的真值 / odom / EKF 轨迹，量化「里程计幻觉」。

用途：G1 P1 的出口标准里，「静止 120 s 时虚假位移 ≤ 0.02 m」和
「ground truth 上到点误差 ≤ 0.30 m」都不能只看 action 的 SUCCEEDED，
必须拿真值对。本脚本把三条轨迹摆在一起算同一组量。

用法（容器内，需要 rosbag2_py）：
    python3 tools/gt_compare.py <bag目录> [--robot robot1_my_bot] [--json 输出.json]

判据口径：
  · 净位移  = |终点 - 起点|          —— 走了多远
  · 最大偏离 = max|各点 - 起点|       —— 静止测试里这个就是「虚假位移」
  · 路径长度 = Σ|相邻点差|            —— 抖动会让它远大于净位移
"""

import argparse
import json
import math
import sys

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

GT_TOPIC = "/g1/ground_truth"
ODOM_TOPIC = "/robot1/odom"
FILTERED_TOPIC = "/robot1/odometry/filtered"

# 出口标准（P1）
STATIC_LIMIT_M = 0.02
GOAL_ERROR_LIMIT_M = 0.30
# 出口标准写的是「静止 **120 s** 时虚假位移 ≤ 0.02 m」。原来只要 ≥2 个样本就给
# pass，不看时长 —— 仓库里的 static120.json 实际只有 86.8 s 也照样 pass
# （问题清单 M4）。时长也要判。
STATIC_MIN_DURATION_S = 120.0
# 必需话题：没有它就无法判定；缺了要 rc=2，不能「样本不足，跳过」了事
REQUIRED_TOPICS = (GT_TOPIC,)


def stamp_seconds(header):
    """用消息头的时间戳（仿真时间），拿不到就返回 None。"""
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def read_bag(bag_path):
    """返回 {来源: [(t, x, y, yaw)]}。"""
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id="sqlite3"),
        ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    wanted = {
        name: get_message(types[name])
        for name in (GT_TOPIC, ODOM_TOPIC, FILTERED_TOPIC)
        if name in types
    }
    missing = [
        n for n in (GT_TOPIC, ODOM_TOPIC, FILTERED_TOPIC) if n not in types
    ]
    if missing:
        print(f"⚠️  bag 里缺话题：{missing}", file=sys.stderr)

    tracks = {"ground_truth": [], "odom": [], "filtered": []}
    counts = {GT_TOPIC: 0, ODOM_TOPIC: 0, FILTERED_TOPIC: 0}
    zero_stamps = {}

    while reader.has_next():
        topic, raw, bag_time = reader.read_next()
        msg_type = wanted.get(topic)
        if msg_type is None:
            continue
        counts[topic] += 1
        msg = deserialize_message(raw, msg_type)

        key = {
            GT_TOPIC: "ground_truth",
            ODOM_TOPIC: "odom",
            FILTERED_TOPIC: "filtered",
        }[topic]
        t = stamp_seconds(msg.header)
        if t is None or t <= 0.0:
            # 真值节点用仿真时钟打戳；万一拿到 0 戳，退回 bag 时间并告警，
            # 而不是默默算出错位的假数。
            t = bag_time * 1e-9
            zero_stamps[key] = zero_stamps.get(key, 0) + 1
        p = msg.pose.pose
        tracks[key].append(
            (t, p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        )

    for key in tracks:
        tracks[key].sort(key=lambda s: s[0])
    if zero_stamps:
        print(
            f"⚠️  有 {zero_stamps} 条消息时间戳为 0，已退回 bag 时间；"
            "这会让时间基不一致，结论不可信",
            file=sys.stderr,
        )
    return tracks, counts


def summarize(samples):
    if len(samples) < 2:
        return None
    t0, x0, y0, _ = samples[0]
    tn, xn, yn, _ = samples[-1]
    max_excursion = 0.0
    path_length = 0.0
    for i in range(1, len(samples)):
        _, x, y, _ = samples[i]
        max_excursion = max(max_excursion, math.hypot(x - x0, y - y0))
        _, px, py, _ = samples[i - 1]
        path_length += math.hypot(x - px, y - py)
    return {
        "sample_count": len(samples),
        "duration_s": round(tn - t0, 3),
        "start": [round(x0, 4), round(y0, 4)],
        "end": [round(xn, 4), round(yn, 4)],
        "net_displacement_m": round(math.hypot(xn - x0, yn - y0), 4),
        "max_excursion_m": round(max_excursion, 4),
        "path_length_m": round(path_length, 4),
    }


def nearest(samples, t):
    if not samples:
        return None
    best = min(samples, key=lambda s: abs(s[0] - t))
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--robot", default="robot1_my_bot")
    parser.add_argument("--json", dest="json_out")
    args = parser.parse_args()

    tracks, counts = read_bag(args.bag)

    report = {
        "bag": args.bag,
        "robot": args.robot,
        "message_counts": counts,
        "tracks": {},
        "static_test": {},
    }

    print(f"bag: {args.bag}")
    print(f"robot: {args.robot}")
    print(f"消息数: {counts}\n")

    # 哪些轨迹根本没录到 —— 必须显式记下来并影响退出码。
    # 原来只是打印「样本不足，跳过」、static_test 里干脆没这一项、JSON 无标记，
    # 于是「没录到」和「没这项指标」在报告里长得一样（问题清单 M3）。
    absent = [k for k in ("ground_truth", "odom", "filtered")
              if len(tracks[k]) < 2]
    report["tracks_absent"] = absent

    for key in ("ground_truth", "odom", "filtered"):
        stats = summarize(tracks[key])
        report["tracks"][key] = stats
        if stats is None:
            print(f"[{key}] ⚠️ 没录到（{len(tracks[key])} 条）—— "
                  f"这条轨迹的判据**不适用**，不是通过")
            continue
        print(
            f"[{key}] n={stats['sample_count']} 时长={stats['duration_s']}s "
            f"净位移={stats['net_displacement_m']}m "
            f"最大偏离={stats['max_excursion_m']}m "
            f"路径长={stats['path_length_m']}m"
        )

    # 静止测试：物理上没动时，odom/EKF 报出来的最大偏离就是「虚假位移」。
    # 时长也要判：出口标准是「静止 **120 s**」，原来只要 ≥2 个样本就给 pass。
    print("\n== 静止测试判定（限值 %.2f m，时长需 ≥ %.0f s）=="
          % (STATIC_LIMIT_M, STATIC_MIN_DURATION_S))
    # 时长按**仿真时间**算（消息头的时间戳）。这个项目反复在挂钟/仿真时间上
    # 栽过（RTF 0.3 时同一段录制两种口径差 3 倍），所以写清楚口径。
    print("（时长按消息头的仿真时间算，不是挂钟）")
    verdicts = []
    for key in ("ground_truth", "odom", "filtered"):
        stats = report["tracks"].get(key)
        if not stats:
            report["static_test"][key] = {"pass": None,
                                          "reason": "没录到，判据不适用"}
            print(f"  {key:14s} —— 没录到，判据不适用（不是通过）")
            continue
        dur_ok = stats["duration_s"] >= STATIC_MIN_DURATION_S
        exc_ok = stats["max_excursion_m"] <= STATIC_LIMIT_M
        ok = dur_ok and exc_ok
        report["static_test"][key] = {
            "max_excursion_m": stats["max_excursion_m"],
            "duration_s": stats["duration_s"],
            "min_duration_s": STATIC_MIN_DURATION_S,
            "limit_m": STATIC_LIMIT_M,
            "pass": ok,
        }
        why = []
        if not exc_ok:
            why.append("超过位移限值")
        if not dur_ok:
            why.append(f"时长不足（{stats['duration_s']}s < "
                       f"{STATIC_MIN_DURATION_S:.0f}s）")
        print(
            f"  {key:14s} 最大偏离={stats['max_excursion_m']:.4f} m  "
            f"时长={stats['duration_s']}s → "
            f"{'通过' if ok else '不通过：' + '、'.join(why)}"
        )
        verdicts.append(ok)

    # 与真值对比。
    # ⚠️ 必须按**位移**比，不能按绝对坐标比：odom 以自身起点为原点（(0,0)），
    #    而狗在 Gazebo 世界里的生成点未必在原点（实测 x=0.0285）。
    #    直接相减会把「坐标原点偏移」误报成「里程计漂移」。
    gt = tracks["ground_truth"]
    if gt and len(gt) >= 2:
        _, gx0, gy0, _ = gt[0]
        for key in ("odom", "filtered"):
            series = tracks[key]
            if len(series) < 2:
                continue
            _, ox0, oy0, _ = series[0]
            worst = 0.0
            for t, gx, gy, _ in gt:
                near = nearest(series, t)
                if near is None:
                    continue
                _, ox, oy, _ = near
                drift = math.hypot(
                    (ox - ox0) - (gx - gx0),
                    (oy - oy0) - (gy - gy0),
                )
                worst = max(worst, drift)
            report.setdefault("drift_vs_ground_truth", {})[key] = {
                "max_drift_m": round(worst, 4),
                "goal_error_limit_m": GOAL_ERROR_LIMIT_M,
            }
            verdict = "在限值内" if worst <= GOAL_ERROR_LIMIT_M else "超过限值"
            print(
                f"\n位移口径：{key} 相对真值的最大漂移 {worst:.4f} m（{verdict}，"
                f"限值 {GOAL_ERROR_LIMIT_M} m）"
            )

    # 退出码要能区分三种情况（原来恒 0，问题清单 M3）：
    #   2 = 必需话题（真值）没录到 —— 这轮没验到，不给结论
    #   1 = 判了，且有不通过的项
    #   0 = 判了，全过
    missing_required = [k for k in REQUIRED_TOPICS if k in absent]
    if missing_required:
        verdict = 2
        report["verdict"] = "数据不足，未判定"
        print(f"\n**未判定**：必需话题没录到 {missing_required} —— 这轮什么都没验到，"
              f"不要当成通过。")
    elif absent:
        # 缺轨迹不能让结论变成「通过」。原来的表现是：odom/EKF 没录到 →
        # 只剩 ground_truth 一项（真值对自己当然 pass）→ 整体显示通过 ——
        # README 坑 #2（话题名写错导致 odom 没录到）正是这个场景（问题清单 M3）。
        verdict = 1
        report["verdict"] = "判据不完整"
        print(f"\n**不通过（判据不完整）**：{'、'.join(absent)} 没录到，"
              f"P1 出口标准里那条「虚假位移」验的是 odom/EKF，不是真值自己 —— "
              f"缺了就说明这轮没验到，不能算过。")
    elif verdicts and not all(verdicts):
        verdict = 1
        report["verdict"] = "不通过"
        print("\n**不通过**：静止测试有项未达标（见上）。")
    else:
        verdict = 0
        report["verdict"] = "通过"
        print("\n**通过**：已判的项全部达标。")
    if absent:
        print(f"（注意：{'、'.join(absent)} 没录到，相关判据不适用 —— "
              f"报告里记的是 null 而不是 0）")

    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as fh:
            json.dump(report, fh, ensure_ascii=False, indent=2)
        print(f"\n报告已写入 {args.json_out}")
    return verdict


if __name__ == "__main__":
    sys.exit(main())
