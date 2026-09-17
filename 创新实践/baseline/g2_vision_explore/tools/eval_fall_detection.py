#!/usr/bin/env python3
"""端到端倒地识别评估：检出人 -> 判倒地 -> 对真值算指标。

--------------------------------------------------------------------------------
和 eval_liepose_recall.py 的区别

那个只测「有没有检出人」。这个测的是**完整交付物**：
检出人之后，**判不判得出他倒地了**。

这才是交付物 3 的验收口径 —— 只检出人不够，
「侦查」要的是从「找到人」升级到「判断异常」。

--------------------------------------------------------------------------------
指标口径

    label 0    = 正常           -> 判为倒地即**误报**
    label 2/3  = 已倒地          -> 判为非倒地即**漏报**
    label 1    = 倒地发生中       -> 过渡态，不参与统计

对应项目文档 §五：
    目标检出率 = 正确检出数 / 场景内全部目标数  -> 本脚本的「倒地召回」
    误检率     = 错误检出数 / 全部检出数        -> 本脚本的「倒地误报率」

--------------------------------------------------------------------------------
输出里最关键的不是数字，是**分布**

若两类样本的比率分布严重重叠，说明**任何阈值都不可能分开它们** ——
那不是调参问题，是方法 A（长宽比）本身不够用，必须上方法 B（三维姿态）。
这比一个孤零零的准确率数字有用得多。
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

TS_RE = re.compile(r"(\d{8})-(\d{6})_(\d{6})")
FALLEN_LABELS = ("2", "3")
NORMAL_LABEL = "0"
TRANSITION_LABEL = "1"
SWEEP = (0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.2)


def _ts(name: str) -> float | None:
    m = TS_RE.search(name)
    if not m:
        return None
    _, t, us = m.groups()
    return int(t[:2]) * 3600 + int(t[2:4]) * 60 + int(t[4:6]) + int(us) / 1e6


def _describe(a: np.ndarray) -> str:
    if a.size == 0:
        return "n/a"
    q1, q2, q3 = np.percentile(a, [25, 50, 75])
    return f"中位数 {q2:.2f}  [四分位 {q1:.2f}~{q3:.2f}]  [{a.min():.2f}, {a.max():.2f}]"


def main() -> int:
    ap = argparse.ArgumentParser(description="端到端倒地识别评估")
    ap.add_argument("--images", required=True)
    ap.add_argument("--annot", required=True)
    ap.add_argument("--weights", default="yolo26n.pt")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--pitch-deg", type=float, default=0.0,
                    help="相机下俯角（度），用于长宽比的俯仰归一化")
    ap.add_argument("--fallen-ratio", type=float, default=0.6,
                    help="低于此高宽比判为倒地")
    args = ap.parse_args()

    import cv2

    from g2_core.anomaly import bbox_aspect_is_fallen
    from g2_core.detector import Detector, DetectorConfig, DetectorUnavailableError

    # --- 真值 ---
    gt: list[tuple[float, str]] = []
    with open(args.annot, newline="") as f:
        for row in csv.DictReader(f):
            t = _ts(row.get("realsense_depth") or "")
            if t is not None:
                gt.append((t, row.get("label", "")))
    gt.sort()
    gt_t = np.array([g[0] for g in gt])

    try:
        det = Detector(DetectorConfig(
            weights=args.weights, conf=args.conf, imgsz=args.imgsz, classes=["person"],
        ))
    except DetectorUnavailableError as exc:
        print(exc)
        return 1

    img_dir = Path(args.images).expanduser()
    files = sorted(p for p in img_dir.iterdir() if p.suffix.lower() in {".jpg", ".jpeg", ".png"})

    pitch = np.radians(args.pitch_deg)

    # --- 只跑一遍检测，把每帧的比率缓存下来，阈值扫描在缓存上做 ---
    # （每扫一个阈值就重跑一遍检测是 8 倍浪费，且结果完全一样）
    records: list[tuple[str, tuple[float, ...]]] = []
    for f in files:
        t = _ts(f.name)
        if t is None:
            continue
        i = int(np.argmin(np.abs(gt_t - t)))
        if abs(gt_t[i] - t) > 0.5:
            continue
        label = gt[i][1]
        if label == TRANSITION_LABEL:
            continue

        img = cv2.imread(str(f))
        if img is None:
            continue
        persons = det.detect(img)

        frame_ratios = []
        for d in persons:
            _, ratio = bbox_aspect_is_fallen(d.bbox_xyxy, camera_pitch_rad=pitch)
            frame_ratios.append(ratio)
        records.append((label, tuple(frame_ratios)))

    if not records:
        print("没有可用的帧")
        return 1

    # --- 分布 ---
    print("=" * 72)
    print("高宽比分布（每个检出的 person 一条；越大越「竖」，越小越「扁」）")
    per_class: dict[str, list[float]] = {}
    for label, ratios in records:
        per_class.setdefault(label, []).extend(ratios)

    for lab in (NORMAL_LABEL,) + FALLEN_LABELS:
        if lab in per_class:
            arr = np.array(per_class[lab])
            name = {"0": "正常", "2": "已倒地A", "3": "已倒地B"}[lab]
            print(f"  {name}(label {lab})  n={arr.size:3d}  {_describe(arr)}")

    # --- 阈值扫描（帧级判定：帧里任一人被判倒地则本帧报倒地）---
    print()
    print("=" * 72)
    print("阈值扫描 —— 判据：归一化高宽比 < 阈值 即判倒地")
    print(f"{'阈值':>6}  {'倒地召回':>14}  {'正常帧误报':>14}")

    def sweep(thr: float) -> tuple[int, int, int, int]:
        rec_hit = rec_tot = fp_hit = fp_tot = 0
        for label, ratios in records:
            fired = any(r < thr for r in ratios)
            if label in FALLEN_LABELS:
                rec_tot += 1
                rec_hit += int(fired)
            elif label == NORMAL_LABEL:
                fp_tot += 1
                fp_hit += int(fired)
        return rec_hit, rec_tot, fp_hit, fp_tot

    for thr in SWEEP:
        rh, rt, fh, ft = sweep(thr)
        mark = "  <- 当前" if abs(thr - args.fallen_ratio) < 1e-6 else ""
        print(f"{thr:>6.1f}  {rh:>5}/{rt:<5} {rh/rt if rt else 0:>5.0%}  "
              f"{fh:>5}/{ft:<5} {fh/ft if ft else 0:>5.0%}{mark}")

    # --- 可分性 ---
    print()
    print("=" * 72)
    fall_arr = np.concatenate([np.array(per_class[l]) for l in FALLEN_LABELS
                               if per_class.get(l)]) if any(per_class.get(l) for l in FALLEN_LABELS) else np.array([])
    norm_arr = np.array(per_class.get(NORMAL_LABEL, []))

    if norm_arr.size and fall_arr.size:
        below = float((norm_arr < np.median(fall_arr)).mean())
        print(f"正常样本中，有 {below:.0%} 的比率低于倒地样本的中位数")
        if below > 0.25:
            print("  ⚠️ 两类**重叠严重** —— 这不是调阈值能解决的。")
            print("     单靠长宽比分不开站着和躺着，必须上方法 B（三维姿态），")
            print(f"     或先用相机俯仰角归一化（当前 --pitch-deg={args.pitch_deg:g}）")
        else:
            print("  区分度尚可，长宽比可作粗筛用。")

    print()
    print("边界：固定机位、单人、模拟跌倒。Go2 是移动平台且机身晃动，")
    print("      会改变投影；「沿光轴躺倒」姿态也未覆盖。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
