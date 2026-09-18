"""二维扫描：有效量程下限 × 取深度的分位数。

在垃圾值被滤掉之后，p5 的近端偏置还剩多少？双峰（床 0.6 / 地面 0.2）在哪一格出现？
另外打关键帧真值对照（114 站立 / 340 躺床 0.6 / 1032 躺地 0.2 / 1263 站立）。
"""
import sys
import cv2
import numpy as np

sys.path.insert(0, '/home/wy/project/baseline/g2_vision_explore')
from g2_core.anomaly import TORSO_KEYPOINT_IDS, check_reprojection   # noqa: E402
from g2_core.detector import Detector, DetectorConfig                # noqa: E402
from g2_core.floor import fit_floor_plane                            # noqa: E402
from g2_core.projector import CameraIntrinsics                      # noqa: E402

W, H, N = 320, 240, 1378
K = CameraIntrinsics(fx=262.0, fy=262.0, cx=160.0, cy=120.0)
raw = np.memmap('/tmp/d1378.raw', dtype='<u2', mode='r', shape=(N, H, W))
U, V = np.meshgrid(np.arange(W), np.arange(H))
VIDEO = '/home/wy/data1/创新实践/baseline/g2_vision_explore/1378_rgbd_45s.mp4'
R = 17


def backproject(dmm):
    z = dmm.astype(np.float64) / 1000.0
    ok = z > 0
    return np.stack([(U - K.cx) * z / K.fx, (V - K.cy) * z / K.fy, z], axis=-1)[ok]


def sample(rawf, u, v, dmin, pct):
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= ui < W and 0 <= vi < H):
        return float('nan')
    p = rawf[max(0, vi - R):vi + R + 1, max(0, ui - R):ui + R + 1].astype(np.float64)
    ok = p > dmin
    if ok.sum() < 10:
        return float('nan')
    return float(np.percentile(p[ok], pct)) / 1000.0


det = Detector(DetectorConfig(weights='/home/wy/g2_testdata/yolo11n-pose.pt', conf=0.25))
cap = cv2.VideoCapture(VIDEO)
cache, nofit = [], 0
for fr in range(0, N, 5):
    fit = fit_floor_plane(backproject(raw[fr]), step=5)
    if not fit.ok:
        nofit += 1
        continue
    cap.set(cv2.CAP_PROP_POS_FRAMES, fr)
    ok, bgr = cap.read()
    if not ok:
        continue
    for d in det.detect(bgr[:, :320]):
        if d.class_name != 'person' or d.keypoints is None:
            continue
        kp = np.asarray(d.keypoints, dtype=np.float64)
        if min(float(kp[i][2]) for i in TORSO_KEYPOINT_IDS) < 0.5:
            continue
        cache.append((fr, fit, raw[fr], kp))
print(f'地面不合格 {nofit} 帧，缓存检测 {len(cache)} 条\n')


def run(dmin, pct):
    out = []
    for fr, fit, rawf, kp in cache:
        pts = []
        for i in TORSO_KEYPOINT_IDS:
            z = sample(rawf, kp[i][0], kp[i][1], dmin, pct)
            if not np.isfinite(z) or z <= 0:
                pts = None
                break
            pts.append(np.array([(kp[i][0] - K.cx) * z / K.fx,
                                 (kp[i][1] - K.cy) * z / K.fy, z]))
        if pts is None:
            continue
        pts = np.stack(pts)
        if check_reprojection(*pts) is not None:
            continue
        up, off = fit.normal, fit.offset
        tv = (pts[0] + pts[1]) / 2.0 - (pts[2] + pts[3]) / 2.0
        nv = np.linalg.norm(tv)
        tilt = float(np.degrees(np.arccos(np.clip(float(tv / nv @ up), -1, 1)))) if nv > 1e-9 else np.nan
        out.append((fr, tilt, float(up @ ((pts[0] + pts[1] + pts[2] + pts[3]) / 4.0) + off)))
    return out


print('=== 下限 × 分位数：躺姿中位 / 直立中位 / 观测数 ===')
PCTS = [5, 15, 25, 50]
print(f"{'下限':>6} | " + ' | '.join(f'{f"p{p}":>22}' for p in PCTS))
print(f"{'':>6} | " + ' | '.join(f"{'躺/直/倒挂':>22}" for _ in PCTS))
print('-' * 104)
best = {}
for dmin in (0, 600, 700, 1000, 1400):
    cells = []
    for pct in PCTS:
        recs = run(dmin, pct)
        lying = np.array([h for _, t, h in recs if t >= 60])
        up_ = np.array([h for _, t, h in recs if t < 30])
        best[(dmin, pct)] = recs
        if lying.size and up_.size:
            lm, um = np.median(lying), np.median(up_)
            cells.append(f'{lm:+.2f}/{um:+.2f}/{"✗" if lm > um else "✓":>1} ({lying.size:>2},{up_.size:>2})')
        else:
            cells.append(f'{"--":>7}        ')
    print(f'{dmin:>6} | ' + ' | '.join(f'{c:>22}' for c in cells))

print('\n=== 关键帧真值对照（真值：1263 站立 / 340 躺床~0.6 / 1032 躺地~0.2）===')
for dmin, pct in ((0, 5), (700, 5), (700, 15), (700, 25), (700, 50), (1000, 15)):
    recs = {fr: (t, h) for fr, t, h in best[(dmin, pct)]}
    parts = []
    for want, truth in ((1263, '站立'), (340, '躺床0.6'), (1032, '躺地0.2'), (918, '躺地')):
        hit = [recs[f] for f in recs if abs(f - want) <= 10]
        parts.append(f'{want}:{"%.2f"%hit[0][1] if hit else "--"}')
    print(f'  下限{dmin:>5} p{pct:<3}: ' + '  '.join(parts))
