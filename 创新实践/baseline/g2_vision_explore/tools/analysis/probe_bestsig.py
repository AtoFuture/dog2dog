"""既然髋高单独分不开，试四个候选信号，看哪个把「倒地」和「正常」分得最开：

  ① 髋高          —— fall_tracker 现在用的
  ② 肩高          —— 蹲/跪的人肩仍高，躺的人肩低
  ③ 肩高 − 髋高   —— 躯干的**竖直跨度**：直立 ~0.5 m，水平 ~0.05 m
  ④ 倾角          —— 已知两类重叠（方法 B）
每个都给「召回 100% 时的误报率」，以及最佳工作点。
"""
import sys
import os
import glob
import csv
import re

import cv2
import numpy as np

sys.path.insert(0, '/home/wy/project/baseline/g2_vision_explore')
from g2_core.anomaly import TORSO_KEYPOINT_IDS, torso_tilt_from_vertical  # noqa: E402
from g2_core.detector import Detector, DetectorConfig                     # noqa: E402
from g2_core.floor import fit_floor_plane                                 # noqa: E402
from g2_core.projector import (CameraIntrinsics, depth_to_meters,          # noqa: E402
                               sample_depth_near)

DATA = os.path.expanduser('~/g2_testdata')
K = CameraIntrinsics(fx=615.0, fy=615.0, cx=320.0, cy=240.0)
TS_RE = re.compile(r'(\d{8})-(\d{6})_(\d{6})')


def _ts(p):
    m = TS_RE.search(os.path.basename(p))
    return None if not m else (m.group(1), m.group(2), int(m.group(3)))


def build_pairs():
    rows = list(csv.DictReader(open(f'{DATA}/corner1_low_annotation.csv')))
    lab = {r['realsense_depth']: int(r['label']) for r in rows if r['realsense_depth']}
    rgbs = glob.glob(f'{DATA}/low_angle_big/*.png')
    out = []
    for dp in sorted(glob.glob(f'{DATA}/depth_big/*.npy')):
        b = os.path.basename(dp)
        if b not in lab or _ts(dp) is None:
            continue
        k = _ts(dp)
        cand = [p for p in rgbs if _ts(p)[:2] == k[:2]]
        if cand:
            out.append((min(cand, key=lambda p: abs(_ts(p)[2] - k[2])), dp, lab[b]))
    return out


det = Detector(DetectorConfig(weights=f'{DATA}/yolo11n-pose.pt', conf=0.25))
rows = []
for rgb, dp, label in build_pairs():
    img = cv2.imread(rgb)
    dep = np.load(dp)
    if img is None or dep is None:
        continue
    d_m = depth_to_meters(dep, '16UC1')
    z = dep.astype(np.float64) / 1000.0
    ok = z > 0
    u, v = np.meshgrid(np.arange(dep.shape[1]), np.arange(dep.shape[0]))
    P = np.stack([(u - K.cx) * z / K.fx, (v - K.cy) * z / K.fy, z], -1)
    fit = fit_floor_plane(P[ok], step=5)
    if not fit.ok:
        continue
    dets = [d for d in det.detect(img)
            if d.class_name == 'person' and d.keypoints is not None]
    if not dets:
        continue
    d0 = max(dets, key=lambda x: x.confidence)
    kp = np.asarray(d0.keypoints, dtype=np.float64)
    per = []
    for i in TORSO_KEYPOINT_IDS:
        zz = sample_depth_near(d_m, kp[i][0], kp[i][1])
        if not np.isfinite(zz) or zz <= 0:
            per = None
            break
        per.append(np.array([(kp[i][0] - K.cx) * zz / K.fx,
                             (kp[i][1] - K.cy) * zz / K.fy, zz]))
    if per is None:
        continue
    up, off = fit.normal, fit.offset
    sm, hm = (per[0] + per[1]) / 2.0, (per[2] + per[3]) / 2.0
    hh = float(hm @ up + off)
    sh = float(sm @ up + off)
    rows.append(dict(label=label, hip=hh, sh=sh, span=sh - hh,
                     tilt=float(np.degrees(torso_tilt_from_vertical(sm, hm, up=up)))))

fall = [r for r in rows if r['label'] in (2, 3)]
norm = [r for r in rows if r['label'] == 0]
print(f'倒地 n={len(fall)}  正常 n={len(norm)}\n')

SIGS = [('① 髋高', 'hip', False), ('② 肩高', 'sh', False),
        ('③ 肩高−髋高（躯干竖直跨度）', 'span', True), ('④ 倾角', 'tilt', False)]
print(f"{'信号':<26} {'方向':>4} | {'两类中位':>16} | "
      f"{'召回100%时误报':>14} | {'最佳工作点':>22}")
print('-' * 92)
for name, key, descending in SIGS:
    f = np.array([r[key] for r in fall])
    n = np.array([r[key] for r in norm])
    # 找「召回 100%」的边界
    if descending:      # 值越小越像倒地
        thr0 = f.max()
        fpr = float((n <= thr0).mean())
        best = None
        for thr in np.linspace(min(f.min(), n.min()), max(f.max(), n.max()), 400):
            rec = float((f <= thr).mean())
            fp = float((n <= thr).mean())
            # 工作点：误报 ≤10% 时召回最高
            if fp <= 0.10 and (best is None or rec > best[1]):
                best = (thr, rec, fp)
        bp = f'≤{best[0]:.2f} → 召回 {best[1]:.0%} / 误报 {best[2]:.0%}' if best else '误报 ≤10% 无解'
        med = f'{np.median(f):+.2f} / {np.median(n):+.2f}'
        print(f'{name:<26} {"越小":>4} | {med:>16} | {fpr:>14.0%} | {bp:>22}')
    else:
        thr0 = f.min()
        fpr = float((n >= thr0).mean())
        best = None
        for thr in np.linspace(min(f.min(), n.min()), max(f.max(), n.max()), 400):
            rec = float((f >= thr).mean())
            fp = float((n >= thr).mean())
            if fp <= 0.10 and (best is None or rec > best[1]):
                best = (thr, rec, fp)
        bp = f'≥{best[0]:.2f} → 召回 {best[1]:.0%} / 误报 {best[2]:.0%}' if best else '误报 ≤10% 无解'
        med = f'{np.median(f):+.2f} / {np.median(n):+.2f}'
        print(f'{name:<26} {"越大":>4} | {med:>16} | {fpr:>14.0%} | {bp:>22}')

print('\n=== 细节：三个信号在两类的范围 ===')
for name, key in (('髋高', 'hip'), ('肩高', 'sh'), ('竖直跨度', 'span')):
    f = np.array([r[key] for r in fall])
    n = np.array([r[key] for r in norm])
    print(f'  {name:<8} 倒地 {f.min():+.2f}~{f.max():+.2f}   正常 {n.min():+.2f}~{n.max():+.2f}')
