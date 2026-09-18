"""诊断：幸存观测的离地高度系统性偏高约 1.5 m，到底是哪一环。

对每条通过质检的观测，同时用**同一个关键点、同一帧地面**，
只换深度取样方式算一遍高度：

  p5   —— 生产现状（sample_depth_near 默认）
  p25 / p50 / p95 —— 只动分位数
  single —— 关键点所在的那一个像素（关键点若真在身体上，这才是对的）

如果高度偏高是「近端偏置」造成的，p50/single 应当把躺姿拉回 ~0.2 m。
如果不是，说明偏置在别处（关键点位置 / 内参 / 平面）。
"""
import sys
import cv2
import numpy as np

sys.path.insert(0, '/home/wy/project/baseline/g2_vision_explore')
from g2_core.anomaly import TORSO_KEYPOINT_IDS, check_reprojection   # noqa: E402
from g2_core.detector import Detector, DetectorConfig                # noqa: E402
from g2_core.floor import fit_floor_plane                            # noqa: E402
from g2_core.projector import (CameraIntrinsics, depth_to_meters,     # noqa: E402
                               sample_depth_near)

W, H, N = 320, 240, 1378
K = CameraIntrinsics(fx=262.0, fy=262.0, cx=160.0, cy=120.0)
VIDEO = '/home/wy/data1/创新实践/baseline/g2_vision_explore/1378_rgbd_45s.mp4'
depth_raw = np.memmap('/tmp/d1378.raw', dtype='<u2', mode='r', shape=(N, H, W))
U, V = np.meshgrid(np.arange(W), np.arange(H))
NAMES = {5: 'L肩', 6: 'R肩', 11: 'L髋', 12: 'R髋'}


def backproject(dmm):
    z = dmm.astype(np.float64) / 1000.0
    ok = z > 0
    return np.stack([(U - K.cx) * z / K.fx, (V - K.cy) * z / K.fy, z], axis=-1)[ok]


def z_single(depth_m, u, v):
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= ui < W and 0 <= vi < H):
        return float('nan')
    return float(depth_m[vi, ui])


det = Detector(DetectorConfig(weights='/home/wy/g2_testdata/yolo11n-pose.pt', conf=0.25))
cap = cv2.VideoCapture(VIDEO)


def measure(kp, depth_m, sampler):
    """按给定取样器反投影四点，返回 (四点三维, 四点深度)。"""
    pts, zs = [], []
    for i in TORSO_KEYPOINT_IDS:
        u, v = kp[i][0], kp[i][1]
        z = sampler(depth_m, u, v)
        if not np.isfinite(z) or z <= 0:
            return None, None
        pts.append(np.array([(u - K.cx) * z / K.fx,
                             (v - K.cy) * z / K.fy, z]))
        zs.append(z)
    return np.stack(pts), np.array(zs)


SAMPLERS = {
    'p5':     lambda d, u, v: sample_depth_near(d, u, v),
    'p25':    lambda d, u, v: sample_depth_near(d, u, v, percentile=25.0),
    'p50':    lambda d, u, v: sample_depth_near(d, u, v, percentile=50.0),
    'p95':    lambda d, u, v: sample_depth_near(d, u, v, percentile=95.0),
    'single': z_single,
}

rows = []
nofit = 0
for fr in range(0, N, 5):
    pts_bg = backproject(depth_raw[fr])
    fit = fit_floor_plane(pts_bg, step=5)
    if not fit.ok:
        nofit += 1
        continue
    cap.set(cv2.CAP_PROP_POS_FRAMES, fr)
    ok, bgr = cap.read()
    if not ok:
        continue
    d_m = depth_to_meters(depth_raw[fr], "16UC1")
    for d in det.detect(bgr[:, :320]):
        if d.class_name != 'person' or d.keypoints is None:
            continue
        kp = d.keypoints
        base_pts, base_z = measure(kp, d_m, SAMPLERS['p5'])
        if base_pts is None:
            continue
        # 只用生产质检判定「是否幸存」，但四条判据都记下来
        why = check_reprojection(*base_pts)
        if why is not None:
            continue

        up, off = fit.normal, fit.offset
        rec = dict(frame=fr, conf=d.confidence, cam_h=fit.camera_height_m,
                   tilt=None, heights={}, zs={}, kp={}, why=None,
                   kp_conf=[float(d.keypoints[i][2]) for i in TORSO_KEYPOINT_IDS])
        for name, fn in SAMPLERS.items():
            p, z = measure(kp, d_m, fn)
            if p is None:
                rec['heights'][name] = float('nan')
                rec['zs'][name] = [float('nan')] * 4
                continue
            mid = (p[0] + p[1]) / 2.0
            midh = (p[2] + p[3]) / 2.0
            rec['heights'][name] = float(up @ ((mid + midh) / 2.0) + off)
            rec['zs'][name] = [float(x) for x in z]
        # 倾角（用 p5，和生产一致）
        p = base_pts
        tv = (p[0] + p[1]) / 2.0 - (p[2] + p[3]) / 2.0
        nv = np.linalg.norm(tv)
        rec['tilt'] = float(np.degrees(np.arccos(
            np.clip(float(tv / nv @ up), -1, 1)))) if nv > 1e-9 else float('nan')
        for j, i in enumerate(TORSO_KEYPOINT_IDS):
            rec['kp'][NAMES[i]] = (float(kp[i][0]), float(kp[i][1]))
        rows.append(rec)

print(f'地面不合格 {nofit} 帧，幸存观测 {len(rows)} 条\n')

hdr = f"{'帧':>5} {'倾角':>5} {'相高':>5} | " + ' '.join(
    f'{n:>7}' for n in SAMPLERS) + '   kp_conf'
print(hdr)
print('-' * len(hdr))
for r in sorted(rows, key=lambda x: -x['tilt']):
    hs = ' '.join(f"{r['heights'][n]:>+7.2f}" for n in SAMPLERS)
    print(f"{r['frame']:>5} {r['tilt']:>4.0f}° {r['cam_h']:>5.2f} | {hs}   "
          f"min={min(r['kp_conf']):.2f}")

print()
for lo, hi, label in ((60, 999, '躺姿(≥60°)'), (0, 30, '直立(<30°)')):
    sub = [r for r in rows if lo <= r['tilt'] < hi]
    if not sub:
        continue
    print(f'{label} n={len(sub)}')
    for n in SAMPLERS:
        v = np.array([r['heights'][n] for r in sub])
        v = v[np.isfinite(v)]
        if v.size:
            print(f'   {n:>7}: 中位 {np.median(v):+.2f}  ({v.min():+.2f}~{v.max():+.2f})')

print('\n各观测的四点深度（p5 vs single）与像素位置:')
for r in sorted(rows, key=lambda x: x['frame']):
    z5 = r['zs']['p5']
    zs_ = r['zs']['single']
    print(f"  帧{r['frame']:>5} 倾角{r['tilt']:>4.0f}°  p5={['%.2f'%z for z in z5]}  "
          f"single={['%.2f'%z for z in zs_]}")
    for nm, uv in r['kp'].items():
        print(f'        {nm} px=({uv[0]:6.1f},{uv[1]:6.1f})', end='')
    print()
