#!/usr/bin/env python3
"""严谨版的躺姿召回评估：用数据集自带标注，而不是靠肉眼猜。

--------------------------------------------------------------------------------
为什么需要「严谨版」

第一版（check_liepose_recall.py）默认**每张图里都有人**。
这对视频抽帧完全不成立 —— 一段连续录像里，人在画面外、或者还站着、
或者已经走开的帧占大多数。按那个假设算出来的「召回」是假的：
把「人不在画面里」也当成漏检，白白拉低数字；反过来若只看抽到的帧，
又可能把大量不含人的帧排除在外，说不清分母是什么。

这个脚本用数据集自带的标注定真值：

    label 0 = 正常
    label 1 = 倒地**发生中**（过渡态）  -> 不计入分母
    label 2/3 = 已倒地（两种类型）      -> 这才是分母

--------------------------------------------------------------------------------
用法

    python3 tools/eval_liepose_recall.py \\
        --images  ~/g2_testdata/low_angle \\
        --annot   ~/g2_testdata/corner1_low_annotation.csv \\
        --weights yolo11n.pt

RGB 与深度是两路独立的流，时间戳差几十毫秒，所以按**最近时间戳**对齐。
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

TS_RE = re.compile(r"(\d{8})-(\d{6})_(\d{6})")
FALLEN_LABELS = {"2", "3"}
TRANSITION_LABEL = "1"


def _ts_seconds(name: str) -> float | None:
    m = TS_RE.search(name)
    if not m:
        return None
    d, t, us = m.groups()
    hh, mm, ss = int(t[:2]), int(t[2:4]), int(t[4:6])
    return hh * 3600 + mm * 60 + ss + int(us) / 1e6


def main() -> int:
    ap = argparse.ArgumentParser(description="用数据集标注做躺姿召回评估")
    ap.add_argument("--images", required=True)
    ap.add_argument("--annot", required=True, help="标注 CSV")
    ap.add_argument("--weights", default="yolo11n.pt")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--ts-column", default="realsense_depth", help="CSV 里带时间戳的列")
    ap.add_argument("--label-column", default="label")
    ap.add_argument("--save", default=None)
    args = ap.parse_args()

    import cv2

    from g2_core.detector import Detector, DetectorConfig, DetectorUnavailableError

    # --- 读真值 ---
    gt: list[tuple[float, str]] = []
    with open(args.annot, newline="") as f:
        for row in csv.DictReader(f):
            t = _ts_seconds(row.get(args.ts_column) or "")
            if t is not None:
                gt.append((t, row.get(args.label_column, "")))
    gt.sort()
    if not gt:
        print("标注里没解析出任何时间戳，检查 --ts-column")
        return 1
    gt_times = np.array([t for t, _ in gt])
    print(f"真值帧 {len(gt)} 条")

    # --- 图片 ---
    img_dir = Path(args.images).expanduser()
    files = sorted(
        p for p in img_dir.iterdir()
        if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"}
    )
    if not files:
        print(f"{img_dir} 里没有图")
        return 1

    try:
        det = Detector(DetectorConfig(
            weights=args.weights, conf=args.conf, imgsz=args.imgsz, classes=None,
        ))
    except DetectorUnavailableError as exc:
        print(exc)
        return 1

    if args.save:
        Path(args.save).mkdir(parents=True, exist_ok=True)

    # --- 逐帧 ---
    stats: dict[str, dict[str, int]] = defaultdict(lambda: {"total": 0, "hit": 0})
    detail: list[tuple[str, str, int]] = []

    for f in files:
        t = _ts_seconds(f.name)
        if t is None:
            print(f"  跳过（文件名无时间戳）: {f.name}")
            continue
        # 最近时间戳对齐
        idx = int(np.argmin(np.abs(gt_times - t)))
        if abs(gt_times[idx] - t) > 0.5:
            print(f"  跳过（与真值差 {abs(gt_times[idx] - t):.2f}s）: {f.name}")
            continue
        label = gt[idx][1]

        img = cv2.imread(str(f))
        if img is None:
            continue
        dets = det.detect(img)
        persons = [d for d in dets if d.class_name == "person"]

        if label != TRANSITION_LABEL:
            stats[label]["total"] += 1
            if persons:
                stats[label]["hit"] += 1
        detail.append((f.name, label, len(persons)))

        if args.save:
            out = img.copy()
            for d in dets:
                x1, y1, x2, y2 = (int(v) for v in d.bbox_xyxy)
                c = (0, 200, 0) if d.class_name == "person" else (140, 140, 140)
                cv2.rectangle(out, (x1, y1), (x2, y2), c, 2)
                if d.class_name == "person":
                    cv2.putText(out, f"person {d.confidence:.2f}", (x1, max(18, y1 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)
            cv2.putText(out, f"label={label}", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            cv2.imwrite(str(Path(args.save) / f.name), out)

    # --- 报告 ---
    print("=" * 68)
    print(f"{'label':>6} {'含义':<12} {'命中/总数':>12} {'召回':>8}")
    names = {"0": "正常", "2": "已倒地(A)", "3": "已倒地(B)"}
    for lab in sorted(stats):
        s = stats[lab]
        rate = s["hit"] / s["total"] if s["total"] else 0.0
        print(f"{lab:>6} {names.get(lab, '?'):<12} {s['hit']:>5}/{s['total']:<6} {rate:>7.0%}")

    fallen_total = sum(stats[l]["total"] for l in FALLEN_LABELS if l in stats)
    fallen_hit = sum(stats[l]["hit"] for l in FALLEN_LABELS if l in stats)
    if fallen_total:
        r = fallen_hit / fallen_total
        print("-" * 68)
        print(f"**倒地帧召回: {fallen_hit}/{fallen_total} = {r:.0%}**")
        if r >= 0.85:
            print("  达到计划的检出率指标（>85%）")
        else:
            print("  ⚠️ 未达计划的检出率指标（>85%）")
            print("     建议：① 降 conf / 提高 imgsz 重测；② 评估少量数据微调的成本")

    normal = stats.get("0")
    if normal and normal["total"]:
        print(f"正常帧误报（检出 person）: {normal['hit']}/{normal['total']}"
              f" = {normal['hit'] / normal['total']:.0%}"
              "   <- 注意：正常帧里人本来就站着，检出 person 不算误报")

    print()
    print("提示：本评估只统计「画面里有没有被检出的人」。")
    print("      真正的倒地**判据**（姿态、长宽比）是下一步，不在本脚本范围内。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
