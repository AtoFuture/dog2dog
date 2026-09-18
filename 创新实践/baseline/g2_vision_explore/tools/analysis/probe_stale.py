"""验证：文档里那个「真倒地髋高 0.28~1.58 m」是不是**过时测量**。

假设：它是在 `fit_floor_plane` 加上校验**之前**量的 —— 当时由
validate_method_b.py 自己实现一份**没有校验**的 RANSAC（只取内点最多的平面，
天花板/床面/墙面都可能被当成地面）。文档自己也记了那份实现"已删除"。

同一批帧、同一套关键点、同一套取深度，只换地面拟合，各算一遍髋高：
  A 旧口径：无校验 RANSAC + 内点率门槛 0.08（照 /tmp 里当时脚本的写法）
  B 新口径：g2_core.floor.fit_floor_plane（两条校验）
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


def old_ransac(pts, iters=300, thr=0.03, seed=0):
    """当时 validate_method_b.py 里那份**没有校验**的 RANSAC（照 /tmp 脚本复刻）。"""
    rng = np.random.default_rng(seed)
    best = (0, None, None)
    for _ in range(iters):
        i = rng.choice(len(pts), 3, replace=False)
        p0, p1, p2 = pts[i]
        nv = np.cross(p1 - p0, p2 - p0)
        L = np.linalg.norm(nv)
        if L < 1e-9:
            continue
        nv /= L
        d = -float(nv @ p0)
        c = int((np.abs(pts @ nv + d) < thr).sum())
        if c > best[0]:
            best = (c, nv, d)
    c, nv, d = best
    if nv is None:
        return None, None, 0.0, None, None
    if nv[1] > 0:
        nv, d = -nv, -d
    return nv, d, c / len(pts), float(d), float(c / len(pts))


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
pairs = build_pairs()

recs = []
for rgb, dp, label in pairs:
    img = cv2.imread(rgb)
    dep = np.load(dp)
    if img is None or dep is None:
        continue
    d_m = depth_to_meters(dep, '16UC1')
    z = dep.astype(np.float64) / 1000.0
    ok = z > 0
    u, v = np.meshgrid(np.arange(dep.shape[1]), np.arange(dep.shape[0]))
    P = np.stack([(u - K.cx) * z / K.fx, (v - K.cy) * z / K.fy, z], -1)
    pts_all = P[ok]
    dets = [d for d in det.detect(img)
            if d.class_name == 'person' and d.keypoints is not None]
    if not dets:
        continue
    d0 = max(dets, key=lambda x: x.confidence)
    kp = np.asarray(d0.keypoints, dtype=np.float64)

    # 关键点三维（两种口径共用同一套取深度）
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
    hl, hr = per[2], per[3]
    hip_mid = (hl + hr) / 2.0

    rec = dict(label=label, rgb=os.path.basename(rgb))

    # A 旧口径
    nv, d, inl, camh, _ = old_ransac(pts_all[::5])
    rec['old'] = None
    if nv is not None and inl >= 0.08:
        rec['old'] = (float(nv @ hip_mid + d), float(camh), float(inl))

    # B 新口径
    fit = fit_floor_plane(pts_all, step=5)
    rec['new'] = None
    if fit.ok:
        rec['new'] = (float(hip_mid @ fit.normal + fit.offset),
                      float(fit.camera_height_m), float(fit.inlier_ratio))
    recs.append(rec)

print(f'配到 {len(recs)} 帧有人的\n')

for tag, key in (('A 旧口径（无校验 RANSAC）', 'old'), ('B 新口径（fit_floor_plane）', 'new')):
    print(f'=== {tag} ===')
    print(f"{'标签':>4} {'帧':>4} {'有高度':>6} | {'髋高中位':>8} {'髋高范围':>20} "
          f"{'相机高中位':>10}")
    for lb in (0, 1, 2, 3):
        sub = [r for r in recs if r['label'] == lb and r[key] is not None]
        n = sum(1 for r in recs if r['label'] == lb)
        if not sub:
            print(f'{lb:>4} {n:>4} {0:>6} | 无')
            continue
        h = np.array([r[key][0] for r in sub])
        c = np.array([r[key][1] for r in sub])
        print(f'{lb:>4} {n:>4} {len(sub):>6} | {np.median(h):>+8.2f} '
              f'{f"{h.min():+.2f} ~ {h.max():+.2f}":>20} {np.median(c):>+10.2f}')
    print()

print('=== 文档里那张表（倒地 = label 2/3，正常 = label 0）===')
for tag, key in (('旧口径', 'old'), ('新口径', 'new')):
    fall = [r[key][0] for r in recs if r['label'] in (2, 3) and r[key] is not None]
    norm = [r[key][0] for r in recs if r['label'] == 0 and r[key] is not None]
    f = f'{min(fall):.2f} ~ {max(fall):.2f}' if fall else '--'
    n = f'{min(norm):.2f} ~ {max(norm):.2f}' if norm else '--'
    print(f'  {tag}: 真倒地 {f}    正常活动 {n}')

print('\n  文档原值: 真倒地 0.28 ~ 1.58    正常活动 0.22 ~ 1.51')
