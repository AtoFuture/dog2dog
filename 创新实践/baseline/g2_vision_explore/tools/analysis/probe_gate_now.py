"""既然旧数拿不回来，就直接问：**现在**这个离地高度门能不能标定？

给出每类的分位数分布 + 阈值扫描。关键要看「正常活动」那一类的**低端** ——
label 0 的定义是「站立/坐/蹲/搬东西」，坐和蹲的髋高本来就低。
如果低端和「躺」重叠，那么高度门失败的**原因就不是标定，而是前提**：
「正常活动」这一类里本来就包含髋高与躺姿相当的姿态。
"""
import sys
import os
import glob
import csv
import re

import cv2
import numpy as np

sys.path.insert(0, '/home/wy/project/baseline/g2_vision_explore')
from g2_core.anomaly import (TORSO_KEYPOINT_IDS, check_reprojection,   # noqa: E402
                             torso_tilt_from_vertical)
from g2_core.detector import Detector, DetectorConfig                 # noqa: E402
from g2_core.floor import fit_floor_plane                             # noqa: E402
from g2_core.projector import (CameraIntrinsics, depth_to_meters,      # noqa: E402
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
    hm = (per[2] + per[3]) / 2.0
    sm = (per[0] + per[1]) / 2.0
    # 跪/蹲 的特征：髋低 + 倾角也低（躯干仍竖直）
    hip_h = float(hm @ up + off)
    sh_h = float(sm @ up + off)
    tilt = float(np.degrees(torso_tilt_from_vertical(sm, hm, up=up)))
    rows.append(dict(label=label, hip_h=hip_h, sh_h=sh_h, tilt=tilt,
                     conf=float(d0.confidence)))

print(f'{len(rows)} 帧有人\n')
print('=== 每类髋高的分位数 ===')
hdr = (f"{'标签':>4} {'n':>4} | {'最小':>7} {'p10':>7} {'p25':>7} {'中位':>7} "
       f"{'p75':>7} {'p90':>7} {'最大':>7} | {'倾角中位':>8}")
print(hdr)
print('-' * len(hdr))
LBL = {0: '正常', 1: '过渡', 2: '倒地', 3: '倒地'}
for lb in (0, 1, 2, 3):
    sub = [r for r in rows if r['label'] == lb]
    if not sub:
        continue
    h = np.array([r['hip_h'] for r in sub])
    t = np.array([r['tilt'] for r in sub])
    q = lambda p: np.percentile(h, p)   # noqa: E731
    print(f'{LBL[lb]:>4} {len(sub):>4} | {h.min():>+7.2f} {q(10):>+7.2f} {q(25):>+7.2f} '
          f'{np.median(h):>+7.2f} {q(75):>+7.2f} {q(90):>+7.2f} {h.max():>+7.2f} | '
          f'{np.median(t):>7.0f}°')

print('\n=== 阈值扫描：用髋高把「倒地(label2/3)」和「正常(label0)」分开 ===')
fall = np.array([r['hip_h'] for r in rows if r['label'] in (2, 3)])
norm = np.array([r['hip_h'] for r in rows if r['label'] == 0])
print(f"{'阈值':>5} | {'召回':>6} {'误报':>6} | 说明")
print('-' * 52)
for thr in (0.30, 0.35, 0.40, 0.45, 0.50, 0.60):
    tp = int((fall < thr).sum())
    fp = int((norm < thr).sum())
    print(f'{thr:>5.2f} | {tp/len(fall):>6.0%} {fp/len(norm):>6.0%} | '
          f'倒地 {tp}/{len(fall)} 判对，正常 {fp}/{len(norm)} 被误杀')

print('\n=== 关键：「正常活动」里髋高偏低的是些什么姿态？===')
low = sorted([r for r in rows if r['label'] == 0 and r['hip_h'] < 0.35],
             key=lambda r: r['hip_h'])
print(f'label 0 里髋高 < 0.35 m 的有 {len(low)}/{len(norm)} 条：')
for r in low:
    print(f"   髋高 {r['hip_h']:+.2f}  肩高 {r['sh_h']:+.2f}  "
          f"倾角 {r['tilt']:>3.0f}°  conf={r['conf']:.2f}   "
          f"{'← 躯干竖直，是蹲/跪而不是躺' if r['tilt'] < 45 else ''}")
