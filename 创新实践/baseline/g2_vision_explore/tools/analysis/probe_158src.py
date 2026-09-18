"""追查「真倒地 0.28 ~ 1.58 m」这个数到底怎么来的。

已排除：地面拟合口径（新旧都算过，几乎一致）。
剩下的候选差异源，逐个试：
  ① 只取置信度最高的人  vs  取**每一帧所有**检出的人
  ② 过质检门  vs  不过门
  ③ 不同的 fx
  ④ 半径 radius=6（当时 /tmp 脚本用的）vs 默认 17
每换一个口径就打一次「真倒地 / 正常活动」的髋高范围，看哪一个能落在文档值附近。
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
                             keypoints_to_torso_3d)
from g2_core.detector import Detector, DetectorConfig                 # noqa: E402
from g2_core.floor import fit_floor_plane                             # noqa: E402
from g2_core.projector import (CameraIntrinsics, depth_to_meters,      # noqa: E402
                               sample_depth_near)

DATA = os.path.expanduser('~/g2_testdata')
TS_RE = re.compile(r'(\d{8})-(\d{6})_(\d{6})')
DOC_FALL, DOC_NORM = (0.28, 1.58), (0.22, 1.51)


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


def collect(fx, radius, all_persons, gate_only, kconf):
    K = CameraIntrinsics(fx=fx, fy=fx, cx=320.0, cy=240.0)
    det = Detector(DetectorConfig(weights=f'{DATA}/yolo11n-pose.pt', conf=0.25))
    out = []
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
        use = dets if all_persons else [max(dets, key=lambda x: x.confidence)]
        for d in use:
            kp = np.asarray(d.keypoints, dtype=np.float64)
            if min(float(kp[i][2]) for i in TORSO_KEYPOINT_IDS) < kconf:
                continue
            per = []
            for i in TORSO_KEYPOINT_IDS:
                zz = sample_depth_near(d_m, kp[i][0], kp[i][1], radius=radius)
                if not np.isfinite(zz) or zz <= 0:
                    per = None
                    break
                per.append(np.array([(kp[i][0] - K.cx) * zz / K.fx,
                                     (kp[i][1] - K.cy) * zz / K.fy, zz]))
            if per is None:
                continue
            if gate_only and check_reprojection(*per) is not None:
                continue
            hm = (per[2] + per[3]) / 2.0
            out.append((label, float(hm @ fit.normal + fit.offset)))
    return out


def rng_of(rows, labels):
    v = [h for lb, h in rows if lb in labels]
    return (f'{min(v):.2f} ~ {max(v):.2f}', len(v)) if v else ('--', 0)


CASES = [
    ('基线（最高分人 / r17 / 过门 / kconf0.5）', 615, 17, False, True, 0.5),
    ('① 每一帧所有人', 615, 17, True, True, 0.5),
    ('② 不过质检门', 615, 17, False, False, 0.5),
    ('②b 不过门 + 不过关键点置信度', 615, 17, False, False, 0.0),
    ('④ radius=6', 615, 6, False, True, 0.5),
    ('③ fx=350', 350, 17, False, True, 0.5),
    ('③b fx=1000', 1000, 17, False, True, 0.5),
]
print(f"{'口径':<36} {'真倒地':>18} {'正常活动':>18}")
print('-' * 76)
print(f"{'**文档原值**':<36} {f'{DOC_FALL[0]:.2f} ~ {DOC_FALL[1]:.2f}':>18} "
      f"{f'{DOC_NORM[0]:.2f} ~ {DOC_NORM[1]:.2f}':>18}")
for name, fx, r, ap, go, kc in CASES:
    rows = collect(fx, r, ap, go, kc)
    f, nf = rng_of(rows, (2, 3))
    n, nn = rng_of(rows, (0,))
    print(f'{name:<36} {f:>18} {n:>18}   (n={nf}/{nn})')
