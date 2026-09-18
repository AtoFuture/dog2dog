"""定位 corner1_low 上「真倒地却量出 1.58 m 髋高」那个异常。

三类原因在日志里长得一模一样（都只是「高度是个大数」），必须分开：
  A 关键点落错  -> 该点的深度与其余三点明显不一致（深度差门会亮）
  B 平面锁错    -> 四个点被**整体**平移，相机高度/内点率也会异常
  C 深度取错    -> 窗口里取到了不该取的东西
  D 标签本身错  -> 人根本不在那（躺在别的家具上、或已起身）

对每个 label 2/3（已倒地）且髋高偏高的帧，把四个关键点的像素、置信度、
深度、尺寸、地面参数全打出来，并渲染成图。
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
                             keypoints_to_torso_3d, torso_tilt_from_vertical)
from g2_core.detector import Detector, DetectorConfig                 # noqa: E402
from g2_core.floor import fit_floor_plane                             # noqa: E402
from g2_core.projector import (CameraIntrinsics, depth_to_meters,      # noqa: E402
                               sample_depth_near)

DATA = os.path.expanduser('~/g2_testdata')
K = CameraIntrinsics(fx=615.0, fy=615.0, cx=320.0, cy=240.0)
TS_RE = re.compile(r'(\d{8})-(\d{6})_(\d{6})')
NAMES = ['L肩', 'R肩', 'L髋', 'R髋']


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
pairs = build_pairs()

rows = []
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
    dets = [d for d in det.detect(img)
            if d.class_name == 'person' and d.keypoints is not None]
    if not dets:
        continue
    d0 = max(dets, key=lambda x: x.confidence)
    kp = np.asarray(d0.keypoints, dtype=np.float64)
    rec = dict(rgb=os.path.basename(rgb), label=label, conf=float(d0.confidence),
               fit_ok=fit.ok, cam_h=fit.camera_height_m, inlier=fit.inlier_ratio,
               up=fit.normal, off=fit.offset, kp=kp, img=img, dep=dep, d_m=d_m,
               pts=[], zs=[], hts=[], kconf=[], sizes=None, tilt=None,
               gate=None, hip_h=None, sh_h=None)
    if not fit.ok:
        rows.append(rec)
        continue
    per = []
    for i in TORSO_KEYPOINT_IDS:
        zz = sample_depth_near(d_m, kp[i][0], kp[i][1])
        if not np.isfinite(zz) or zz <= 0:
            per.append(None)
            continue
        p3 = np.array([(kp[i][0] - K.cx) * zz / K.fx,
                       (kp[i][1] - K.cy) * zz / K.fy, zz])
        per.append(p3)
        rec['zs'].append(float(zz))
        rec['hts'].append(float(fit.normal @ p3 + fit.offset))
        rec['kconf'].append(float(kp[i][2]))
    if any(p is None for p in per):
        rows.append(rec)
        continue
    rec['pts'] = per
    sl, sr, hl, hr = per
    rec['sizes'] = (float(np.linalg.norm(sl - sr)), float(np.linalg.norm(hl - hr)),
                    float(np.linalg.norm((sl + sr) / 2 - (hl + hr) / 2)))
    sm, hm = (sl + sr) / 2, (hl + hr) / 2
    rec['tilt'] = float(np.degrees(torso_tilt_from_vertical(sm, hm, up=fit.normal)))
    rec['hip_h'] = float(((hl + hr) / 2) @ fit.normal + fit.offset)
    rec['sh_h'] = float(sm @ fit.normal + fit.offset)
    rec['gate'] = check_reprojection(*per)
    rows.append(rec)

print('=== 按标签：髋高分布 ===')
print(f"{'标签':>4} {'帧':>4} {'有高度':>6} | {'髋高中位':>8} {'范围':>18} | {'肩高中位':>8}")
print('-' * 60)
for lb in (0, 1, 2, 3):
    sub = [r for r in rows if r['label'] == lb and r['hip_h'] is not None]
    if not sub:
        print(f'{lb:>4} {sum(1 for r in rows if r["label"]==lb):>4} {0:>6} | 无')
        continue
    h = np.array([r['hip_h'] for r in sub])
    s = np.array([r['sh_h'] for r in sub])
    n = sum(1 for r in rows if r['label'] == lb)
    print(f'{lb:>4} {n:>4} {len(sub):>6} | {np.median(h):>+8.2f} '
          f'{f"{h.min():+.2f} ~ {h.max():+.2f}":>18} | {np.median(s):>+8.2f}')

print('\n=== label 2/3（已倒地）里髋高 > 1.0 m 的帧 —— 逐帧诊断 ===')
bad = [r for r in rows if r['label'] in (2, 3) and r['hip_h'] is not None
       and r['hip_h'] > 1.0]
print(f'共 {len(bad)} 帧\n')
for r in bad:
    print(f"--- {r['rgb']}  label={r['label']}  conf={r['conf']:.2f} ---")
    print(f"  地面: 相机高 {r['cam_h']:+.2f} m  内点 {r['inlier']:.1%}  "
          f"法向 ({r['up'][0]:+.2f},{r['up'][1]:+.2f},{r['up'][2]:+.2f})")
    print(f"  髋高 {r['hip_h']:+.2f}  肩高 {r['sh_h']:+.2f}   倾角 {r['tilt']:.0f}°")
    sw, hw, tl = r['sizes']
    print(f"  尺寸: 肩宽 {sw:.2f} 髋宽 {hw:.2f} 躯干长 {tl:.2f}  "
          f"（真值 0.40 / 0.32 / 0.50）")
    for j, nm in enumerate(NAMES):
        px = r['kp'][TORSO_KEYPOINT_IDS[j]]
        print(f"    {nm}: px=({px[0]:6.1f},{px[1]:6.1f}) conf={px[2]:.2f}  "
              f"深度 {r['zs'][j]:.2f} m  离地 {r['hts'][j]:+.2f} m")
    print(f"  质检门: {r['gate'] or '通过'}")

print('\n=== 对照：label 2/3 里髋高正常的帧 ===')
good = [r for r in rows if r['label'] in (2, 3) and r['hip_h'] is not None
        and r['hip_h'] <= 1.0]
print(f'共 {len(good)} 帧，髋高 {min(r["hip_h"] for r in good):+.2f} ~ '
      f'{max(r["hip_h"] for r in good):+.2f} m' if good else '  无')

# 渲染异常帧
if bad:
    tiles = []
    for r in bad[:4]:
        img = r['img'].copy()
        for j, i in enumerate(TORSO_KEYPOINT_IDS):
            u_, v_ = r['kp'][i][0], r['kp'][i][1]
            cv2.circle(img, (int(u_), int(v_)), 5, (0, 0, 255), 2)
            cv2.putText(img, f"{r['hts'][j]:+.2f}", (int(u_) + 6, int(v_)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
        cv2.putText(img, f"{r['rgb'][:24]} L{r['label']} hip{r['hip_h']:+.2f}",
                    (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        tiles.append(img)
    while len(tiles) < 4:
        tiles.append(np.zeros_like(tiles[0]))
    grid = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
    cv2.imwrite('/tmp/hip158.png', grid)
    print('\n渲染 -> /tmp/hip158.png')
