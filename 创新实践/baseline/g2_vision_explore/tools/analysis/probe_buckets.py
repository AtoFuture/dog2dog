"""双峰没出现的真正原因：是「偏置」还是「根本没有躺地上的样本」？

把 276 帧扫描里的每条观测按**素材本身的真值活动**归类（文档 §1 的内容表）：
    #114 站立行走 / #229 坐床 / #344 躺床 / #459 下床跪地 / #574 躺床
    #689 地上爬 / #803 坐床 / #918 躺地上 / #1148 地上起身 / #1263 站立

床与地面是两个真值模式，看观测到底落在哪些段上、高度各是多少。
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
raw = np.memmap('/tmp/d1378.raw', dtype='<u2', mode='r', shape=(N, H, W))
U, V = np.meshgrid(np.arange(W), np.arange(H))
VIDEO = '/home/wy/data1/创新实践/baseline/g2_vision_explore/1378_rgbd_45s.mp4'

# 素材真值：帧号区间 -> 活动。区间边界取两段之间的中点。
SEGS = [(0, 170, '站立行走'), (170, 286, '坐床'), (286, 400, '躺床'),
        (400, 516, '下床跪地'), (516, 630, '躺床'), (630, 745, '地上爬'),
        (745, 860, '坐床'), (860, 1030, '躺地上'), (1030, 1200, '地上起身'),
        (1200, 1378, '站立')]
FLOOR_SEGS = {'躺地上', '地上爬', '下床跪地', '地上起身'}


def seg_of(fr):
    for lo, hi, name in SEGS:
        if lo <= fr < hi:
            return name
    return '?'


def backproject(dmm):
    z = dmm.astype(np.float64) / 1000.0
    ok = z > 0
    return np.stack([(U - K.cx) * z / K.fx, (V - K.cy) * z / K.fy, z], axis=-1)[ok]


det = Detector(DetectorConfig(weights='/home/wy/g2_testdata/yolo11n-pose.pt', conf=0.25))
cap = cv2.VideoCapture(VIDEO)

rows = []
nofit = 0
for fr in range(0, N, 5):
    fit = fit_floor_plane(backproject(raw[fr]), step=5)
    if not fit.ok:
        nofit += 1
        continue
    cap.set(cv2.CAP_PROP_POS_FRAMES, fr)
    ok, bgr = cap.read()
    if not ok:
        continue
    d_m = depth_to_meters(raw[fr], "16UC1")
    dets = [d for d in det.detect(bgr[:, :320])
            if d.class_name == 'person' and d.keypoints is not None]
    if not dets:
        rows.append((fr, seg_of(fr), None, None, None, '检测器没出人'))
        continue
    d0 = max(dets, key=lambda x: x.confidence)
    kp = np.asarray(d0.keypoints, dtype=np.float64)
    kc = min(float(kp[i][2]) for i in TORSO_KEYPOINT_IDS)
    if kc < 0.5:
        rows.append((fr, seg_of(fr), float(d0.confidence), None, kc, '关键点置信度低'))
        continue
    from g2_core.anomaly import keypoints_to_torso_3d
    torso, why = keypoints_to_torso_3d(kp, d_m, K)
    if torso is None:
        rows.append((fr, seg_of(fr), float(d0.confidence), None, kc, why))
        continue
    up, off = fit.normal, fit.offset
    sm = (torso.shoulder_left + torso.shoulder_right) / 2.0
    hm = (torso.hip_left + torso.hip_right) / 2.0
    rows.append((fr, seg_of(fr), float(d0.confidence),
                 (float(torso.tilt_deg(up)), float(((sm + hm) / 2.0) @ up + off)),
                 kc, '通过'))

print(f'地面不合格 {nofit} 帧\n')

print('=== 按真值活动分段：观测是否产出 ===')
seen = {}
for fr, seg, conf, res, kc, why in rows:
    s = seen.setdefault(seg, dict(n=0, ok=0, hs=[], tilts=[]))
    s['n'] += 1
    if res is not None:
        s['ok'] += 1
        s['hs'].append(res[1])
        s['tilts'].append(res[0])
order = [s[2] for s in SEGS]
print(f"{'真值活动':>10} {'帧数':>5} {'出观测':>6} {'产出率':>7} | "
      f"{'倾角中位':>8} {'高度中位':>9} {'高度范围':>18}")
print('-' * 78)
for name in order:
    s = seen.get(name)
    if not s:
        continue
    if s['hs']:
        h = np.array(s['hs'])
        t = np.array(s['tilts'])
        rng = f'{h.min():+.2f} ~ {h.max():+.2f}'
        print(f'{name:>10} {s["n"]:>5} {s["ok"]:>6} {s["ok"]/s["n"]:>7.0%} | '
              f'{np.median(t):>7.0f}° {np.median(h):>+9.2f} {rng:>18}')
    else:
        print(f'{name:>10} {s["n"]:>5} {s["ok"]:>6} {s["ok"]/s["n"]:>7.0%} | '
              f'{"--":>8} {"--":>9} {"--":>18}')

print('\n=== 躺姿(倾角≥60°)的来源：床上还是地上 ===')
for fr, seg, conf, res, kc, why in sorted(rows):
    if res is not None and res[0] >= 60:
        tag = '地面' if seg in FLOOR_SEGS else '床上'
        print(f'  帧{fr:>5} {seg:<8} [{tag}]  倾角 {res[0]:>3.0f}°  '
              f'离地 {res[1]:+.2f} m  conf={conf:.2f}')

print('\n=== 地面段被拒的原因（这是双峰缺一半的地方）===')
from collections import Counter
c = Counter()
for fr, seg, conf, res, kc, why in rows:
    if seg in FLOOR_SEGS and res is None:
        c[why.split('：')[0].split('（')[0][:34]] += 1
for k, v in c.most_common(8):
    print(f'  {v:>3}×  {k}')
