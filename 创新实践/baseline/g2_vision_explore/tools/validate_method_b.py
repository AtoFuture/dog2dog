#!/usr/bin/env python3
"""在真实数据上验证**方法 B（三维躯干倾角倒地判据）**。

--------------------------------------------------------------------------------
为什么要有这个脚本

方法 B 是交付物 3（人员倒地识别）**唯一剩下的判据** ——
方法 A（检测框长宽比）已在低机位下实测失败（最优工作点召回仅 43%，见
``docs/倒地判据实测-方法A失败.md``）。而方法 B 从写出来起**从没在真实数据上跑过**。

一个从没跑过的判据不能写进交付物。这个脚本就是给它做判决的。

--------------------------------------------------------------------------------
方法 B 成立需要三个条件同时满足

1. **pose 模型能在躺姿上出关键点** —— 没有二维关键点就无从谈起；
2. **肩/髋像素处的深度有效** —— 躺着的人往往在画面边缘、距离又远；
3. **知道重力在相机系里指向哪** —— 倾角是相对重力的，不知道重力就没有倾角。

第 3 条有个容易误解的地方，值得写下来：

    方法 B 的躯干倾角是**旋转不变量**。``angle(torso, up)`` 只要求两个向量
    在**同一个坐标系**里表达，不要求那个坐标系是 map 系。
    所以**不需要完整的外参标定** —— 只需要「重力在相机光学系里指向哪」。
    本脚本用**深度图拟合地面平面**取重力，自足、不依赖 IMU，而且这正是
    真机上 SLAM/IMU 会给的东西。

--------------------------------------------------------------------------------
真值语义（``corner1_low_annotation.csv`` 的 ``label`` 列）

从抽帧目视确认：

    label 0   正常活动（站立 / 坐 / 蹲 / 搬东西）
    label 1   **正在倒下**（过渡过程，躯干倾角本来就在中间）
    label 2   已倒地（躺在地上）
    label 3   已倒地（另一段，人在动）

⚠️ **label 1 必须排除出评分**：那一瞬间躯干本来就介于竖直和水平之间，
   判它「倒地」或「没倒地」都是任意的 —— 把它算进分母会凭空制造
   一批假阳性或假阴性，得出的指标不反映判据好坏。
   本脚本对 label 1 **单独统计、不并入混淆矩阵**。

⚠️ label 0 里有一部分帧**画面中根本没有第二个人**（空房间）。
   这类帧上「没检出轨迹」是正确行为，不是判据失败。
   评分只统计「确实检出了轨迹、并且给出了倾角」的帧；
   检出率是**检测器**的指标，不是**判据**的指标，两者不能混。

--------------------------------------------------------------------------------
用法::

    python3 tools/validate_method_b.py \
        --data ~/g2_testdata \
        --pose-model ~/g2_testdata/yolo11n-pose.pt \
        --out docs/

输出：终端表格 + ``--out`` 下的 ``方法B验证.json``。
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import re
import sys
from collections import Counter, defaultdict

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from g2_core.anomaly import (  # noqa: E402
    FALLEN_MAX_DEG,
    FALLEN_MIN_DEG,
    assess_fall_from_keypoints_3d,
    check_reprojection,
    midpoints_from_keypoints_2d,
)
from g2_core.projector import depth_to_meters, sample_depth_near  # noqa: E402

# ⚠️ 数据集**没有提供相机标定**，这是本次验证最大的已知误差源。
# 倾角对焦距是**一阶敏感**的（躯干向量 ∝ (A, B, C·f)），所以 --fx 是个可调参数，
# 报告里的内参扫描就是靠它跑的。
# 参考：D435 彩色 HFOV=69.4° -> fx = 320/tan(34.7°) ≈ 462。
FX = FY = 615.0
CX, CY = 320.0, 240.0

# COCO 17 关键点索引
SH_L, SH_R, HIP_L, HIP_R = 5, 6, 11, 12

TS_RE = re.compile(r"(\d{8})-(\d{6})_(\d{6})")


def _ts(name: str):
    """从文件名抽 (日期, 时分秒, 微秒)。彩色与深度是两路独立流，微秒不同。"""
    m = TS_RE.search(os.path.basename(name))
    return None if not m else (m.group(1), m.group(2), int(m.group(3)))


# ----------------------------------------------------------------------
# 重力：从深度图拟合地面
# ----------------------------------------------------------------------
def backproject(depth_mm: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """整帧反投影到相机光学系 (x 右, y 下, z 前)。返回 (点云, 有效掩膜)。"""
    h, w = depth_mm.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    z = depth_mm.astype(np.float64) / 1000.0
    ok = z > 0
    x = (u - CX) * z / FX
    y = (v - CY) * z / FY
    return np.stack([x, y, z], axis=-1), ok


def ransac_floor_normal(depth_mm: np.ndarray, thr: float = 0.02,
                        iters: int = 400, step: int = 5, seed: int = 0):
    """拟合地面平面，返回（相机光学系下的「上」方向, 内点率, 相机离地高度）。

    「上」的定号：光学系 +y 朝下，所以朝上的法向其 y 分量必为负。
    """
    P, ok = backproject(depth_mm)
    pts = P[ok][::step]
    if len(pts) < 100:
        return None, 0.0, float("nan")

    rng = np.random.default_rng(seed)
    n = len(pts)
    best = (0, None, None)
    for _ in range(iters):
        i = rng.choice(n, 3, replace=False)
        p0, p1, p2 = pts[i]
        nv = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(nv)
        if ln < 1e-9:
            continue
        nv = nv / ln
        d = -float(nv @ p0)
        c = int((np.abs(pts @ nv + d) < thr).sum())
        if c > best[0]:
            best = (c, nv, d)

    cnt, nv, d = best
    if nv is None:
        return None, 0.0, float("nan")
    if nv[1] > 0:            # 定号：朝上
        nv, d = -nv, -d
    return nv, cnt / n, abs(d)


# ----------------------------------------------------------------------
# 关键点 -> 三维
# ----------------------------------------------------------------------
def keypoint_to_3d(depth_m: np.ndarray, u: float, v: float):
    """像素 + 深度 -> 相机光学系三维点。深度无效时返回 None。

    用 ``sample_depth_near``（窗口近端分位数）而不是逐像素或中位滤波 ——
    理由见该函数的 docstring：关键点可能落在身体轮廓之外，
    而**人一定比背景近**，取近端才拿得到人的那一层。
    """
    z = sample_depth_near(depth_m, u, v)
    if not np.isfinite(z) or z <= 0:
        return None
    return np.array([(u - CX) * z / FX, (v - CY) * z / FY, z])


# ----------------------------------------------------------------------
def build_pairs(data_dir: str):
    """把 彩色 / 深度 / 标签 配成三元组。"""
    rows = list(csv.DictReader(open(os.path.join(data_dir, "corner1_low_annotation.csv"))))
    label_by_depth = {r["realsense_depth"]: int(r["label"])
                      for r in rows if r["realsense_depth"]}

    rgbs = glob.glob(os.path.join(data_dir, "low_angle_big", "*.png"))
    pairs = []
    for dp in sorted(glob.glob(os.path.join(data_dir, "depth_big", "*.npy"))):
        base = os.path.basename(dp)
        if base not in label_by_depth:
            continue
        k = _ts(dp)
        if k is None:
            continue
        cand = [p for p in rgbs if _ts(p)[:2] == k[:2]]
        if not cand:
            continue
        rgb = min(cand, key=lambda p: abs(_ts(p)[2] - k[2]))
        pairs.append((rgb, dp, label_by_depth[base]))
    return pairs


def main() -> int:
    global FX, FY
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=os.path.expanduser("~/g2_testdata"))
    ap.add_argument("--pose-model", default=os.path.expanduser("~/g2_testdata/yolo11n-pose.pt"))
    ap.add_argument("--out", default=None)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--fx", type=float, default=FX,
                    help="焦距（像素）。数据集无标定，报告里的内参扫描靠它复现。")
    args = ap.parse_args()

    FX = FY = float(args.fx)

    from ultralytics import YOLO

    pairs = build_pairs(args.data)
    print(f"配到 {len(pairs)} 帧（彩色 + 深度 + 标签）")
    if not pairs:
        print("没有可用数据，检查 --data")
        return 1

    model = YOLO(args.pose_model)

    # 每帧一条记录
    recs = []
    for rgb_path, depth_path, label in pairs:
        im = cv2.imread(rgb_path)
        depth_mm = np.load(depth_path)
        # 地面拟合要毫米（它内部除以 1000），取深度要米 —— 分开传，别混
        up, inlier, cam_h = ransac_floor_normal(depth_mm)
        depth_m = depth_to_meters(depth_mm, "16UC1")
        res = model.predict(im, verbose=False, conf=args.conf)[0]

        persons = []
        if res.keypoints is not None and len(res.keypoints):
            kd = res.keypoints.data.cpu().numpy()      # (n, 17, 3)
            for kp in kd:
                _, _, confs = midpoints_from_keypoints_2d(kp)
                pts3d = {}
                for idx, name in ((SH_L, "sh_l"), (SH_R, "sh_r"),
                                  (HIP_L, "hip_l"), (HIP_R, "hip_r")):
                    pts3d[name] = keypoint_to_3d(depth_m, kp[idx][0], kp[idx][1])

                missing = [k for k, v in pts3d.items() if v is None]
                if missing:
                    persons.append(dict(ok=False, missing=missing, confs=confs))
                    continue

                # 人体尺度：单独记一份，好统计这道门丢了多少
                proportions = check_reprojection(
                    pts3d["sh_l"], pts3d["sh_r"], pts3d["hip_l"], pts3d["hip_r"])

                a = assess_fall_from_keypoints_3d(
                    pts3d["sh_l"], pts3d["sh_r"], pts3d["hip_l"], pts3d["hip_r"],
                    confs, up=up,
                )
                persons.append(dict(
                    ok=True, method=a.method, is_fallen=a.is_fallen,
                    tilt_deg=a.tilt_deg, confidence=a.confidence, reason=a.reason,
                    confs=list(confs), proportions_ok=proportions is None,
                ))

        recs.append(dict(
            rgb=os.path.basename(rgb_path), depth=os.path.basename(depth_path),
            label=label, n_person=len(persons),
            floor_inlier=inlier, cam_height=cam_h,
            up=None if up is None else [float(x) for x in up],
            persons=persons,
        ))
        tilts = []
        for p in persons:
            if not p.get("ok"):
                tilts.append(f"无深度{'/'.join(p['missing'])}")
            elif p["tilt_deg"] is None:
                tilts.append("丢弃")
            else:
                tilts.append(f"{p['tilt_deg']:.0f}°")
        print(f"  {os.path.basename(rgb_path)[:29]}  label={label}  "
              f"人数={len(persons)}  倾角={tilts}")

    report(recs)
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        p = os.path.join(args.out, "方法B验证.json")
        json.dump(recs, open(p, "w"), ensure_ascii=False, indent=1)
        print(f"\n明细已写入 {p}")
    return 0


def report(recs) -> None:
    print("\n" + "=" * 72)
    print("方法 B 验证结果")
    print("=" * 72)

    # 地面拟合质量
    inl = np.array([r["floor_inlier"] for r in recs if r["up"]])
    hs = np.array([r["cam_height"] for r in recs if r["up"]])
    print(f"\n[地面拟合] {len(inl)}/{len(recs)} 帧成功  "
          f"内点率 {inl.mean()*100:.1f}%  相机高度 {hs.mean():.2f}±{hs.std():.2f} m")

    by_label = defaultdict(list)
    for r in recs:
        by_label[r["label"]].append(r)

    print("\n[逐标签：检出与倾角]")
    print(f"{'label':>6} {'帧数':>5} {'检出轨迹':>9} {'给出倾角':>9} "
          f"{'倾角中位':>9} {'倾角范围':>18}")
    for lb in sorted(by_label):
        rs = by_label[lb]
        n_traj = sum(r["n_person"] for r in rs)
        ok = [p for r in rs for p in r["persons"] if p.get("ok")]
        tilts = [p["tilt_deg"] for p in ok if p["tilt_deg"] is not None]
        rng = f"{min(tilts):.0f}~{max(tilts):.0f}" if tilts else "-"
        med = f"{np.median(tilts):.1f}" if tilts else "-"
        print(f"{lb:>6} {len(rs):>5} {n_traj:>9} {len(tilts):>9} {med:>9} {rng:>18}")

    # 人体尺度门的丢弃率 —— 这道门丢太多就说明流程本身不可用
    print("\n[人体尺度质检门]")
    for lb, name in ((0, "正常"), (1, "过渡"), (2, "已倒地"), (3, "已倒地")):
        ps = [p for r in recs if r["label"] == lb for p in r["persons"] if p.get("ok")]
        if not ps:
            continue
        bad = sum(1 for p in ps if not p["proportions_ok"])
        print(f"  label {lb}（{name}）：{bad}/{len(ps)} 条被丢弃"
              f"（{100*bad/len(ps):.0f}%）")

    # 混淆矩阵：label 0 = 正常，label 2/3 = 倒地，label 1 排除
    # 只统计**通过了质检门、且真的给出了倾角**的轨迹 ——
    # 「丢弃」不是判据的结论，算进分母会把判据的账和质检的账混在一起
    def usable(r):
        return [p for p in r["persons"] if p.get("ok") and p["tilt_deg"] is not None]

    pos = [p for r in recs if r["label"] in (2, 3) for p in usable(r)]
    neg = [p for r in recs if r["label"] == 0 for p in usable(r)]
    # 阈值判定：方法 B 已经给了 is_fallen，但这里额外做一次显式阈值分析
    print(f"\n[混淆矩阵] 判定阈值 = 倾角 ∈ [{FALLEN_MIN_DEG:.0f}°, {FALLEN_MAX_DEG:.0f}°]")
    print(f"  正类（label 2/3，已倒地）：{len(pos)} 条轨迹，"
          f"判倒地 {sum(p['is_fallen'] for p in pos)} 条")
    print(f"  负类（label 0，正常）    ：{len(neg)} 条轨迹，"
          f"判倒地 {sum(p['is_fallen'] for p in neg)} 条")

    if pos:
        rec = sum(p["is_fallen"] for p in pos) / len(pos)
        print(f"\n  **召回率 = {rec*100:.1f}%**  ({sum(p['is_fallen'] for p in pos)}/{len(pos)})")
    if neg:
        fp = sum(p["is_fallen"] for p in neg) / len(neg)
        print(f"  **误报率 = {fp*100:.1f}%**  ({sum(p['is_fallen'] for p in neg)}/{len(neg)})")

    # label 1 单独看
    mid = [p["tilt_deg"] for r in recs if r["label"] == 1 for p in usable(r)]
    if mid:
        print(f"\n  label 1（正在倒下，**不计入评分**）：{len(mid)} 条，"
              f"倾角 {min(mid):.0f}~{max(mid):.0f}°，中位 {np.median(mid):.0f}°")

    # 最重要的一条诊断：两类分得开吗
    if pos and neg:
        tp = [p["tilt_deg"] for p in pos]
        tn = [p["tilt_deg"] for p in neg]
        print("\n[可分性] ← 这一条比任何阈值指标都重要")
        print(f"  正类倾角: {sorted(round(t) for t in tp)}")
        print(f"  负类倾角: {sorted(round(t) for t in tn)}")
        print(f"  正类最小值 {min(tp):.0f}°   负类最大值 {max(tn):.0f}°")
        if max(tn) > min(tp):
            print(f"  >>> **两类重叠** —— 有任何阈值都分不开它们，"
                  f"因为有一个正常帧（{max(tn):.0f}°）比最轻的倒地（{min(tp):.0f}°）还高")
        else:
            print("  >>> 两类不重叠")

    # 阈值扫描：看有没有更好的工作点
    allpos = [(p["tilt_deg"], True) for p in pos]
    allneg = [(p["tilt_deg"], False) for p in neg]
    if allpos and allneg:
        print("\n[阈值扫描] 只改「判倒地」的下限（上限固定 120°）：")
        print(f"{'下限':>6} {'召回':>8} {'误报':>8}")
        best = None
        for thr in range(20, 96, 5):
            r = sum(1 for t, _ in allpos if thr <= t <= FALLEN_MAX_DEG) / len(allpos)
            f = sum(1 for t, _ in allneg if thr <= t <= FALLEN_MAX_DEG) / len(allneg)
            mark = ""
            if r >= 0.85 and f <= 0.10:
                mark = "  ← 达标"
                if best is None:
                    best = thr
            print(f"{thr:>6} {r*100:>7.0f}% {f*100:>7.0f}%{mark}")
        if best is not None:
            print(f"\n  ⚠️ 有阈值（{best}°）满足 召回>85%/误报<10%，但见上面的 [可分性] ——"
                  f"\n     两类若重叠，这个工作点是**过拟合**，不能当作结论。")
        else:
            print("\n  结论：**没有**任何阈值同时满足 召回>85% / 误报<10%")

    # 丢弃原因
    fails = Counter()
    for r in recs:
        for p in r["persons"]:
            if not p.get("ok"):
                fails["深度无效"] += 1
            elif p["method"] == "insufficient":
                fails[p["reason"].split("（")[0]] += 1
    if fails:
        print(f"\n[丢弃统计] {dict(fails)}")


if __name__ == "__main__":
    raise SystemExit(main())
