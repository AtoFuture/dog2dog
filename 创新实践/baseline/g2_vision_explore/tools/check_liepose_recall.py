#!/usr/bin/env python3
"""实测：预训练检测器能不能检出**躺着的人**。

--------------------------------------------------------------------------------
为什么这个脚本很重要

整个「人员倒地识别」交付物都建立在一个**未经验证的假设**上：

    预训练的 COCO YOLO 能稳定检出躺姿的人。

而 COCO 数据集里几乎没有躺姿样本。已有文献报告 YOLO 会把蹲着/躺着的人
预测成 ``dog``，且这类检测错误是端到端倒地检测误报的**主要来源**。

「零训练成本」这条路线在这里恰恰是最脆弱的一环 —— 所以要么尽早用真实数据实测，
要么就在第 10 周交付时才发现整套判据没输入可用。

这个脚本零成本（几秒到几分钟），结论却决定要不要走「少量数据微调」——
而那与课题「几乎不占卡」的前提冲突，需要早提。

--------------------------------------------------------------------------------
用法

    python3 tools/check_liepose_recall.py --images ~/g2_testdata/laying
    python3 tools/check_liepose_recall.py --images DIR --weights yolo11n.pt --save out/

输出会明确告诉你：这些人被认成了什么（person？dog？还是根本没检出）。
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

import numpy as np

# 允许从仓库直接跑，不必先安装
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def main() -> int:
    ap = argparse.ArgumentParser(description="实测预训练检测器在躺姿上的召回")
    ap.add_argument("--images", required=True, help="躺姿图片所在目录")
    ap.add_argument("--weights", default="yolo11n.pt", help="权重（会自动下载）")
    ap.add_argument("--conf", type=float, default=0.25, help="置信度阈值")
    ap.add_argument("--imgsz", type=int, default=640, help="推理分辨率")
    ap.add_argument("--save", default=None, help="把标注后的图存到这个目录")
    args = ap.parse_args()

    try:
        import cv2
    except ImportError:
        print("需要 opencv：pip install opencv-python-headless")
        return 1

    from g2_core.detector import Detector, DetectorConfig, DetectorUnavailableError

    img_dir = Path(args.images).expanduser()
    files = sorted(p for p in img_dir.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)
    if not files:
        print(f"{img_dir} 里没有图片")
        return 1

    print(f"图片 {len(files)} 张 ｜ 权重 {args.weights} ｜ conf={args.conf} ｜ imgsz={args.imgsz}")
    print("=" * 72)

    try:
        # classes=None：**不过滤类别**。就是要看它到底认成了什么 ——
        # 如果只关注 person，那"被认成 dog"这种情况就被悄悄过滤掉了，
        # 而那恰恰是这个测试要找的问题。
        det = Detector(DetectorConfig(
            weights=args.weights, conf=args.conf, imgsz=args.imgsz, classes=None,
        ))
    except DetectorUnavailableError as exc:
        print(exc)
        return 1

    if args.save:
        Path(args.save).mkdir(parents=True, exist_ok=True)

    hit_person = 0
    hit_nothing = 0
    class_counter: Counter[str] = Counter()
    confs: list[float] = []

    for f in files:
        img = cv2.imread(str(f))
        if img is None:
            print(f"  读取失败: {f.name}")
            continue

        dets = det.detect(img)
        names = [d.class_name for d in dets]
        class_counter.update(names)

        persons = [d for d in dets if d.class_name == "person"]
        if persons:
            hit_person += 1
            confs.extend(d.confidence for d in persons)
            mark = "✅"
        else:
            hit_nothing += 1
            mark = "❌"

        others = [n for n in names if n != "person"]
        detail = f"person x{len(persons)}" if persons else "无 person"
        if others:
            detail += f"  ｜ 其它: {dict(Counter(others))}"
        print(f"  {mark} {f.name:20s} {detail}")

        if args.save:
            out = img.copy()
            for d in dets:
                x1, y1, x2, y2 = (int(v) for v in d.bbox_xyxy)
                color = (0, 200, 0) if d.class_name == "person" else (0, 140, 255)
                cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
                cv2.putText(out, f"{d.class_name} {d.confidence:.2f}", (x1, max(20, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.imwrite(str(Path(args.save) / f.name), out)

    total = hit_person + hit_nothing
    print("=" * 72)
    print(f"检出 person 的图片: {hit_person}/{total}"
          f"  ({100.0 * hit_person / total:.0f}%)" if total else "无有效图片")
    if confs:
        print(f"person 置信度: 中位数 {np.median(confs):.2f}  "
              f"最低 {min(confs):.2f}  最高 {max(confs):.2f}")
    print(f"全部检出类别统计: {dict(class_counter)}")
    print()

    # 结论
    rate = hit_person / total if total else 0.0
    if rate >= 0.9:
        print("结论：预训练权重在躺姿上召回良好 —— 「零训练成本」路线成立。")
        print("      下一步：把 conf 阈值和 imgsz 调到能覆盖实际侦查距离。")
    elif rate >= 0.5:
        print("结论：召回**明显不足**。有相当比例的躺姿没被检出。")
        print("      若这部分正是需要报警的目标，检出率指标（>85%）会有压力。")
        print("      建议：① 降 conf 阈值看能否救回；② 评估少量数据微调的成本。")
    else:
        print("结论：预训练权重在躺姿上**基本失效**。")
        print("      ⚠️ 这直接动摇「人员倒地识别」这个交付物的前提。")
        print("      必须尽快在组内提出：要么接受一个很低的目标检出率，")
        print("      要么走少量数据微调（与课题「几乎不占卡」的前提冲突，需早提）。")

    if args.save:
        print(f"\n标注后的图已存到 {args.save}，可人工核对误报/漏报。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
