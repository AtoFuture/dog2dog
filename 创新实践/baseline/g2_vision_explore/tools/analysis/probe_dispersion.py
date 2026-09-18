"""分位数该取多少：滤掉飞点之后，重新量各分位数的「偏置 vs 离散度」。

背景：percentile=5.0 是当初在 corner1_low（RealSense）上被实测选出来的 ——
它的卖点是**离散度**（肩宽 IQR 0.13，比小窗口中位滤波的 0.86 好 7 倍），
代价是**偏置**（中位 0.24，真值 0.40，小了 40%）。

现在要决定要不要调大它。判据两条，缺一不可：

  ① 偏置：肩宽中位应当靠近真值 0.40
  ② 离散度：IQR 不能变大 —— 质检门用的是**区间**，
     「偏置但精确」还救得回来（区间挪一挪），「噪声大」救不回来
  ③ 抓人能力：关键点落到身体外时，近端分位数才拿得到人。
     用「同侧两点深度差」当代理指标 —— 取到背景时它会炸。

corner1_low 没有飞点（700 以下零像素），所以这次改动对它无影响 ——
正好当**干净的对照组**：分位数的效果在这里能单独看见。
"""
import sys
import os
import glob
import csv
import re

import cv2
import numpy as np

sys.path.insert(0, '/home/wy/project/baseline/g2_vision_explore')
from g2_core.anomaly import (TORSO_KEYPOINT_IDS, check_reprojection,  # noqa: E402
                             torso_tilt_from_vertical)
from g2_core.detector import Detector, DetectorConfig                # noqa: E402
from g2_core.floor import fit_floor_plane                            # noqa: E402
from g2_core.projector import (CameraIntrinsics, depth_to_meters,     # noqa: E402
                               sample_depth_near)

DATA = os.path.expanduser('~/g2_testdata')
K = CameraIntrinsics(fx=615.0, fy=615.0, cx=320.0, cy=240.0)
TS_RE = re.compile(r'(\d{8})-(\d{6})_(\d{6})')
PCTS = [5, 15, 25, 50]


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
        if not cand:
            continue
        out.append((min(cand, key=lambda p: abs(_ts(p)[2] - k[2])), dp, lab[b]))
    return out


def measure(kp, d_m, pct):
    """按给定分位数反投影四点。返回 (四点三维, 四点深度) 或 (None, None)。"""
    pts, zs = [], []
    for i in TORSO_KEYPOINT_IDS:
        z = sample_depth_near(d_m, kp[i][0], kp[i][1], percentile=float(pct))
        if not np.isfinite(z) or z <= 0:
            return None, None
        pts.append(np.array([(kp[i][0] - K.cx) * z / K.fx,
                             (kp[i][1] - K.cy) * z / K.fy, z]))
        zs.append(z)
    return np.stack(pts), np.array(zs)


det = Detector(DetectorConfig(weights=f'{DATA}/yolo11n-pose.pt', conf=0.25))
pairs = build_pairs()
print(f'corner1_low: {len(pairs)} 对（RGB+深度+标签）')

recs = {p: [] for p in PCTS}
skipped = 0
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
    fit = fit_floor_plane(P[ok], step=5)
    if not fit.ok:
        skipped += 1
    dets = [d for d in det.detect(img) if d.class_name == 'person' and d.keypoints is not None]
    if not dets:
        continue
    d0 = max(dets, key=lambda x: x.confidence)
    kp = np.asarray(d0.keypoints, dtype=np.float64)
    for pct in PCTS:
        pts, zs = measure(kp, d_m, pct)
        if pts is None:
            continue
        sl, sr, hl, hr = pts
        rec = dict(
            rgb=os.path.basename(rgb), label=label, conf=float(d0.confidence),
            sw=float(np.linalg.norm(sl - sr)),
            hw=float(np.linalg.norm(hl - hr)),
            tl=float(np.linalg.norm((sl + sr) / 2 - (hl + hr) / 2)),
            dz=max(abs(sl[2] - sr[2]), abs(hl[2] - hr[2])),
            gate=check_reprojection(*pts),
            tilt=None, height=None, kpconf=min(float(kp[i][2]) for i in TORSO_KEYPOINT_IDS),
        )
        if fit.ok:
            up, off = fit.normal, fit.offset
            sm, hm = (sl + sr) / 2, (hl + hr) / 2
            rec['tilt'] = float(np.degrees(torso_tilt_from_vertical(sm, hm, up=up)))
            rec['height'] = float(((sm + hm) / 2) @ up + off)
        recs[pct].append(rec)

print(f'地面不合格 {skipped} 帧\n')

print('=== 偏置 vs 离散度（真值：肩宽 0.40 / 髋宽 ~0.32 / 躯干长 ~0.50 m）===')
hdr = (f"{'分位':>5} {'n':>4} | {'肩宽中位':>9} {'肩宽IQR':>8} | "
       f"{'髋宽中位':>9} {'髋宽IQR':>8} | {'躯干长中位':>10} {'IQR':>6} | "
       f"{'深度差中位':>10} {'p90':>6}")
print(hdr)
print('-' * len(hdr))
for pct in PCTS:
    r = recs[pct]
    if not r:
        continue
    sw = np.array([x['sw'] for x in r])
    hw = np.array([x['hw'] for x in r])
    tl = np.array([x['tl'] for x in r])
    dz = np.array([x['dz'] for x in r])
    q = lambda a: np.percentile(a, 75) - np.percentile(a, 25)   # noqa: E731
    print(f'{pct:>5} {len(r):>4} | {np.median(sw):>9.3f} {q(sw):>8.3f} | '
          f'{np.median(hw):>9.3f} {q(hw):>8.3f} | '
          f'{np.median(tl):>10.3f} {q(tl):>6.3f} | '
          f'{np.median(dz):>10.3f} {np.percentile(dz, 90):>6.3f}')

print('\n=== 质检门留存（新的深度差门为主）===')
print(f"{'分位':>5} {'总样本':>7} {'通过':>6} {'留存率':>8}   主要拒绝原因")
for pct in PCTS:
    r = recs[pct]
    if not r:
        continue
    passed = [x for x in r if x['gate'] is None]
    from collections import Counter
    reasons = Counter(x['gate'].split(' ')[0] + x['gate'].split(' ')[1]
                      for x in r if x['gate'] is not None)
    top = '  '.join(f'{k}×{v}' for k, v in reasons.most_common(3))
    print(f'{pct:>5} {len(r):>7} {len(passed):>6} {len(passed)/len(r):>8.1%}   {top}')

print('\n=== 关键点置信度分布（决定 min_keypoint_conf 该不该动）===')
for pct in PCTS:
    r = recs[pct]
    if r:
        kc = np.array([x['kpconf'] for x in r])
        print(f'  p{pct:<3}: 中位 {np.median(kc):.2f}  '
              f'p10 {np.percentile(kc,10):.2f}  低于0.5的占 {np.mean(kc<0.5):.1%}')
