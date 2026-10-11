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
# 「Nav2 在要求它动」的指令阈值。
# ⚠️ 必须与 cmd_vel_scaler.py 的 `nav_deadband`（默认 0.005）保持一致：
# 缩放器对它输入域（就是本脚本读的 cmd_vel_scaled_out）里小于该值的指令
# **必然输出 0**，狗因此物理停住 —— 那是设计如此，不算冻结。
# 两个常量耦合却各自写死，改一个忘另一个就会「对死区整段失明」
# （问题清单 M1）。可用 --cmd-eps 覆盖，改缩放器参数时记得一起改。
CMD_EPS = 0.005
# 冻结判定用**窗口净位移**，不用单帧位移（问题清单 M2）：
# 真值约 285 Hz、dt≈3.5 ms，单帧位移在「原地蹬腿」时反而很大
# （路径 1.92 m / 净位移 0.03 m 那种），会把卡住判成在动。
FROZEN_WINDOW_S = 1.0     # 判定窗口
FROZEN_NET_M = 0.05       # 窗口内净位移小于此值 = 这一段没在动
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
    # 这两个话题**缺一不可**：没有真值就判不了到点，没有指令就判不了冻结。
    # 原来是只警告不中止，于是缺 cmd 时冻结判据恒假 → 报「冻结 无」→ 判通过
    # （问题清单 H4，实测 bag_static120 就是这样「通过」的）。
    return_missing = [n for n in (GT_TOPIC, CMD_TOPIC) if n not in types]

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
    return tracks, return_missing


def freeze_windows(gt, cmd, cmd_eps=CMD_EPS):
    """找「真值没动、但 Nav2 要求它在动」的连续区间，返回 [(起, 止, 秒)]。

    按**窗口净位移**判，不按单帧位移（问题清单 M2）：单帧位移量的是瞬时速度，
    原地蹬腿时每帧都很大 → 会把卡住判成在动。这里改成「1 秒窗口内净位移
    < FROZEN_NET_M 且这期间有非零指令」才算冻结。
    """
    if len(gt) < 2:
        return []

    def commanded(ta, tb):
        return any(abs(c[1]) + abs(c[2]) > cmd_eps
                   for c in cmd if ta <= c[0] <= tb)

    frozen_flags = []          # (窗口起点, 窗口终点, 是否冻结)
    i = 0
    while i < len(gt) - 1:
        ta = gt[i][0]
        tb = ta + FROZEN_WINDOW_S
        j = i
        while j + 1 < len(gt) and gt[j + 1][0] <= tb:
            j += 1
        if j == i:             # 真值太稀，凑不出窗口
            i += 1
            continue
        net = math.hypot(gt[j][1] - gt[i][1], gt[j][2] - gt[i][2])
        frozen_flags.append((ta, gt[j][0], net < FROZEN_NET_M and commanded(ta, gt[j][0])))
        i = j

    windows, start = [], None
    for ta, tb, frozen in frozen_flags:
        if frozen and start is None:
            start = ta
        elif not frozen and start is not None:
            windows.append((start, ta))
            start = None
    if start is not None:
        windows.append((start, frozen_flags[-1][1]))
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
    parser.add_argument("--cmd-eps", type=float, default=CMD_EPS,
                        help=f"「Nav2 要求它动」的指令阈值，默认 {CMD_EPS}。"
                             "⚠️ 必须与 cmd_vel_scaler.py 的 nav_deadband 一致，"
                             "否则冻结判据对死区失明（问题清单 M1）")
    args = parser.parse_args()

    tracks, missing = read_bag(args.bag)
    # 缺真值或缺指令 → 这轮判不了，rc=2 响亮失败。
    # 不这样就会出问题清单 H4 那个错：缺 cmd 时冻结判据恒假、报「冻结 无」、
    # 然后判「通过」——验收标准里「不出现留存速度指令」那条等于没判。
    if missing:
        print(f"ERROR: 包里缺话题 {missing} —— 缺任何一个都判不了：\n"
              f"       {GT_TOPIC} 用于到点误差，{CMD_TOPIC} 用于冻结段。\n"
              f"       这不是「没问题」，是「没验到」，不给结论（rc=2）。",
              file=sys.stderr)
        return 2

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

    windows = freeze_windows(gt, tracks[CMD_TOPIC], args.cmd_eps)
    longest = max((w[2] for w in windows), default=0.0)

    # AMCL 对真值的偏差（地图与世界恒等对齐时可直接比）。
    # 包里没有 amcl_pose 时必须是 None（「没测到」），不能是 0.0 ——
    # 0.0 看上去是「完美」，实际是没数据（问题清单 H5；README 为 P1 第 5 项
    # 规定的 slam 流程里根本没有 amcl_pose，那份报告必然打 0.0000）。
    amcl_dev = None
    if tracks[AMCL_TOPIC]:
        amcl_dev = 0.0
        for t, x, y, _, _ in tracks[AMCL_TOPIC]:
            near = min(gt, key=lambda g: abs(g[0] - t))
            amcl_dev = max(amcl_dev, math.hypot(x - near[1], y - near[2]))
        amcl_dev = round(amcl_dev, 4)

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
        "frozen_cmd_eps": args.cmd_eps,
        # None = 包里没这个数据（没测到）；不要读成 0
        "amcl_max_deviation_m": amcl_dev,
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
    if amcl_dev is None:
        print(f"  AMCL 偏差  未测到（包里没有 {AMCL_TOPIC}）"
              f"　—— 不是 0，是不适用（slam 模式没有 AMCL）")
    else:
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
