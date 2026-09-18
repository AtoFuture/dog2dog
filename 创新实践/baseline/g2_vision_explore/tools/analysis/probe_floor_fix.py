"""决定性实验：给取深度加一道**有效量程下限**，看躺/直立倒挂是否消失、双峰是否出现。

一次检测、多次评估（检测是慢的那一步，缓存起来）。
对每个下限 dmin：窗口内先丢掉 < dmin 的像素，再取 5 分位。
dmin=0 即生产现状（只丢 0）。
"""
import sys
import cv2
import numpy as np

sys.path.insert(0, '/home/wy/project/baseline/g2_vision_explore')
from g2_core.anomaly import TORSO_KEYPOINT_IDS, check_reprojection   # noqa: E402
from g2_core.detector import Detector, DetectorConfig                # noqa: E402
from g2_core.floor import fit_floor_plane                            # noqa: E402
from g2_core.projector import CameraIntrinsics, depth_to_meters       # noqa: E402

W, H, N = 320, 240, 1378
K = CameraIntrinsics(fx=262.0, fy=262.0, cx=160.0, cy=120.0)
depth_raw = np.memmap('/tmp/d1378.raw', dtype='<u2', mode='r', shape=(N, H, W))
U, V = np.meshgrid(np.arange(W), np.arange(H))
VIDEO = '/home/wy/data1/创新实践/baseline/g2_vision_explore/1378_rgbd_45s.mp4'
R = 17


def backproject(dmm):
    z = dmm.astype(np.float64) / 1000.0
    ok = z > 0
    return np.stack([(U - K.cx) * z / K.fx, (V - K.cy) * z / K.fy, z], axis=-1)[ok]


def sample(patch_raw, u, v, dmin_raw):
    """dmin_raw: 有效量程下限（原始整数单位）。0 表示只丢 0。"""
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= ui < W and 0 <= vi < H):
        return float('nan')
    p = patch_raw[max(0, vi - R):vi + R + 1, max(0, ui - R):ui + R + 1].astype(np.float64)
    ok = (p > dmin_raw)
    if ok.sum() < 10:
        return float('nan')
    return float(np.percentile(p[ok], 5.0)) / 1000.0


det = Detector(DetectorConfig(weights='/home/wy/g2_testdata/yolo11n-pose.pt', conf=0.25))
cap = cv2.VideoCapture(VIDEO)

# ---- 第一遍：缓存检测 + 地面，只做一次 ----
cache = []
nofit = 0
for fr in range(0, N, 5):
    fit = fit_floor_plane(backproject(depth_raw[fr]), step=5)
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
        kp = np.asarray(d.keypoints, dtype=np.float64)
        if min(float(kp[i][2]) for i in TORSO_KEYPOINT_IDS) < 0.5:
            continue
        cache.append((fr, fit, depth_raw[fr], kp, d.confidence))

print(f'地面不合格 {nofit} 帧，缓存检测 {len(cache)} 条 '
      f'（已按生产 min_keypoint_conf=0.5 预筛）\n')

FLOORS = [0, 300, 400, 500, 600, 700, 1000]
print(f"{'下限':>6} {'通过质检':>8} {'躺姿n':>6} {'躺姿中位':>8} {'直立n':>6} "
      f"{'直立中位':>8} {'倒挂?':>6}")
print('-' * 62)
detail = {}
for dmin in FLOORS:
    recs = []
    for fr, fit, rawf, kp, conf in cache:
        pts = []
        good = True
        for i in TORSO_KEYPOINT_IDS:
            z = sample(rawf, kp[i][0], kp[i][1], dmin)
            if not np.isfinite(z) or z <= 0:
                good = False
                break
            pts.append(np.array([(kp[i][0] - K.cx) * z / K.fx,
                                 (kp[i][1] - K.cy) * z / K.fy, z]))
        if not good:
            continue
        pts = np.stack(pts)
        if check_reprojection(*pts) is not None:
            continue
        up, off = fit.normal, fit.offset
        tv = (pts[0] + pts[1]) / 2.0 - (pts[2] + pts[3]) / 2.0
        nv = np.linalg.norm(tv)
        tilt = float(np.degrees(np.arccos(np.clip(float(tv / nv @ up), -1, 1)))) if nv > 1e-9 else np.nan
        h = float(up @ ((pts[0] + pts[1] + pts[2] + pts[3]) / 4.0) + off)
        recs.append((fr, tilt, h))
    detail[dmin] = recs
    lying = np.array([h for _, t, h in recs if t >= 60])
    up_ = np.array([h for _, t, h in recs if t < 30])
    lm = f'{np.median(lying):+.2f}' if lying.size else '  --  '
    um = f'{np.median(up_):+.2f}' if up_.size else '  --  '
    inv = ('是 ✗' if lying.size and up_.size and np.median(lying) > np.median(up_)
           else ('否 ✓' if lying.size and up_.size else '--'))
    print(f'{dmin:>6} {len(recs):>8} {lying.size:>6} {lm:>8} {up_.size:>6} {um:>8} {inv:>6}')

print('\n=== 双峰检查：躺姿离地高度直方图（床 ~0.6 / 地面 ~0.2 应分两堆）===')
for dmin in (0, 500, 700):
    lying = [h for _, t, h in detail[dmin] if t >= 60]
    print(f'\n  下限 {dmin}  (躺姿 n={len(lying)})')
    if not lying:
        print('    无')
        continue
    for lo in np.arange(-0.4, 1.4, 0.2):
        c = sum(1 for h in lying if lo <= h < lo + 0.2)
        if c:
            print(f'    {lo:+.1f}~{lo+0.2:+.1f} m | {"#"*c} {c}')
