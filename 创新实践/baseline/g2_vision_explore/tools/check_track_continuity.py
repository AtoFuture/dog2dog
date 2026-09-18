#!/usr/bin/env python3
"""测**跨帧跟踪 ID 连续性** —— 倒地时间判据的地基。

--------------------------------------------------------------------------------
为什么单独有这个工具

倒地判据（``g2_core/fall_tracker.py``）建立在「同一个 ``id`` 的多帧观测序列」之上：

    见过直立 -> 3s 内转为水平 -> 水平保持 2s 以上  =>  一次倒地事件

也就是说，**同一个人的 id 必须连续存活至少 ~2 秒**，判据才可能触发。
而在此之前，这条假设**从来没有在真实视频上验过** ——
唯一的证据来自一个帧间隔 5 秒的数据集，结论是负面的（id 全程拿不到）。

那个负面结论很可能是数据的问题（5 秒里人早移动了），但「很可能是」不等于「验过了」。

--------------------------------------------------------------------------------
它量什么

1. **``track_id=None`` 的帧占比** —— 跟踪器还没确认轨迹。
   这部分帧**进不了判据**，占比高就等于判据大部分时间在闭眼。

2. **id 连续存活的帧数分布** —— 判据要求 ≥ ``persist_s`` 秒，
   30 fps 下就是 60 帧。看有多少条轨迹能到这个量级。

3. **总 id 数 vs 同时在画面里的人数** —— 粗略反映「同一个人被反复分配新 id」
   （id switch）的严重程度。没有标注，所以这是个**下界估计**，
   不是精确的 switch 计数。

--------------------------------------------------------------------------------
用法::

    python3 tools/check_track_continuity.py \\
        --video /path/to/video.mp4 --weights ~/g2_testdata/yolo11n-pose.pt
"""

from __future__ import annotations

import argparse
import os
import sys
from collections import Counter, defaultdict

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--weights", default=os.path.expanduser("~/g2_testdata/yolo11n-pose.pt"))
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--stride", type=int, default=1, help="每隔几帧处理一次")
    ap.add_argument("--max-frames", type=int, default=0, help="0 = 全部")
    ap.add_argument("--fps-for-persist", type=float, default=0.0,
                    help="判据需要的 fps（默认取视频自身 fps），用来算 persist 帧数")
    ap.add_argument("--persist-s", type=float, default=2.0)
    args = ap.parse_args()

    from ultralytics import YOLO
    from g2_core.detector import Detector, DetectorConfig

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"打不开视频：{args.video}")
        return 1

    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    fps = args.fps_for_persist or src_fps
    eff_fps = fps / max(1, args.stride)
    need_frames = int(round(args.persist_s * eff_fps))

    print(f"视频 {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
          f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ {src_fps:.1f} fps ｜ "
          f"stride={args.stride} -> 有效 {eff_fps:.1f} fps")
    print(f"判据需要 id 连续存活 ≥ {args.persist_s}s = **{need_frames} 帧**\n")

    det = Detector(DetectorConfig(weights=args.weights, conf=args.conf,
                                  imgsz=640, device="cpu"))

    # 当前连续段：id -> 已连续出现的帧数
    run_len: Counter = Counter()
    # 每个 id 的最长连续段
    best_run: dict[int, int] = defaultdict(int)
    # 该 id 上一次出现的帧号（判连续性用）
    last_seen: dict[int, int] = {}
    n_frames = n_with_person = n_unconfirmed = n_person_total = 0

    fi = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if fi % args.stride:
            fi += 1
            continue
        if args.max_frames and n_frames >= args.max_frames:
            break

        dets = det.track(frame)
        n_frames += 1
        if dets:
            n_with_person += 1
        n_person_total += len(dets)

        seen = set()
        for d in dets:
            if d.track_id is None:
                n_unconfirmed += 1
                continue
            tid = d.track_id
            if last_seen.get(tid) == n_frames - 1:
                run_len[tid] += 1
            else:
                run_len[tid] = 1
            last_seen[tid] = n_frames
            best_run[tid] = max(best_run[tid], run_len[tid])
            seen.add(tid)

        if n_frames % 200 == 0:
            print(f"  ...已处理 {n_frames} 帧，累计 {n_person_total} 个目标，"
                  f"在用 id {len(run_len)} 个")

        fi += 1

    report(n_frames, n_with_person, n_person_total, n_unconfirmed, best_run, need_frames)
    return 0


def report(n_frames, n_with_person, n_person_total, n_unconfirmed, best_run, need_frames):
    print("\n" + "=" * 70)
    print("跟踪 ID 连续性结果")
    print("=" * 70)
    if not n_frames:
        print("没有读到帧")
        return

    print(f"\n处理 {n_frames} 帧，其中 {n_with_person} 帧画面里有人"
          f"（{100*n_with_person/max(1,n_frames):.0f}%）")
    print(f"累计检出目标 {n_person_total} 个")

    if not n_person_total:
        print("\n整个视频没检出人 —— 换个视频或调低 --conf")
        return

    pct = 100 * n_unconfirmed / n_person_total
    print(f"\n[① 轨迹未确认] {n_unconfirmed}/{n_person_total} = **{pct:.1f}%** 的目标没有 id")
    print("   这些帧进不了倒地判据。占比高 = 判据大部分时间在闭眼。")

    runs = sorted(best_run.values(), reverse=True)
    print(f"\n[② id 连续存活] 共 {len(runs)} 条轨迹")
    if runs:
        ok_n = sum(1 for r in runs if r >= need_frames)
        print(f"   最长 {runs[0]} 帧 ｜ 中位 {runs[len(runs)//2]} 帧")
        print(f"   存活 ≥ {need_frames} 帧（判据的最低要求）的有 **{ok_n}** 条"
              f"（{100*ok_n/len(runs):.0f}%）")
        buckets = [(1, 5), (5, 15), (15, 60), (60, 300), (300, 10**9)]
        for lo, hi in buckets:
            n = sum(1 for r in runs if lo <= r < hi)
            bar = "#" * int(50 * n / len(runs))
            hi_s = f"{hi}" if hi < 10**8 else "∞"
            print(f"     {lo:>4}~{hi_s:<4} 帧: {n:>4}  {bar}")

    print("\n" + "-" * 70)
    ok = n_person_total and pct < 20 and runs and sum(
        1 for r in runs if r >= need_frames) > 0
    if ok:
        print("✅ 跟踪能稳定给出 id，且存在足够长的连续段 —— 时间判据可以跑")
    else:
        print("❌ 跟踪连续性不满足时间判据的要求（见上面两个指标）")
    print("⚠️ 本工具只看**二维跟踪**，不涉及深度，因此不验证倒地判据本身。")


if __name__ == "__main__":
    raise SystemExit(main())
