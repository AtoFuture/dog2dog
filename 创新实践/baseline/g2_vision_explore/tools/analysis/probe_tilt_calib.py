"""标定「倾角 ≥ 60° ⇒ 倒地」这个前提 —— 它从没在带真值的数据上验过。

真值来自素材自带的活动分段（文档 §1 的内容表）。**必须在锚点附近取帧**，
因为锚点之间是过渡过程（例如"坐床→躺床"中间那几帧，姿态本来就在变），
把过渡帧当某一类会凭空制造错误样本。

对每个锚点 ±12 帧、每 2 帧取一帧，同时记录：
  * 生产口径的结论（过质检门 -> 倾角 / 不过 -> 原因）
  * **绕过质检门的原始倾角** —— 用来判断「信号本身在不在」，
    还是「信号在、只是被门挡住了」。两者修法完全不同。

⚠️ 局限：这份素材里**只有一个明确的「躺地上」锚点（#918）**。
   正样本基数很小，结论只能当方向，不能当定论。
"""
import sys
import cv2
import numpy as np

sys.path.insert(0, '/home/wy/project/baseline/g2_vision_explore')
from g2_core.anomaly import (TORSO_KEYPOINT_IDS, check_reprojection,   # noqa: E402
                             keypoints_to_torso_3d, torso_tilt_from_vertical)
from g2_core.detector import Detector, DetectorConfig                 # noqa: E402
from g2_core.floor import fit_floor_plane                             # noqa: E402
from g2_core.projector import (CameraIntrinsics, depth_to_meters,      # noqa: E402
                               sample_depth_near)

W, H, N = 320, 240, 1378
K = CameraIntrinsics(fx=262.0, fy=262.0, cx=160.0, cy=120.0)
raw = np.memmap('/tmp/d1378.raw', dtype='<u2', mode='r', shape=(N, H, W))
U, V = np.meshgrid(np.arange(W), np.arange(H))
VIDEO = '/home/wy/data1/创新实践/baseline/g2_vision_explore/1378_rgbd_45s.mp4'

# (帧, 活动, 类别)。pos=应当报倒地；neg=不应报；bed=躺在床上（需与倒地区分）；trans=过渡
ANCHORS = [
    (114,  '站立行走', 'neg'),
    (229,  '坐床',     'neg'),
    (344,  '躺床',     'bed'),
    (459,  '下床跪地', 'trans'),
    (574,  '躺床',     'bed'),
    (689,  '地上爬',   'neg'),
    (803,  '坐床',     'neg'),
    (918,  '躺地上',   'pos'),
    (1148, '地上起身', 'trans'),
    (1263, '站立',     'neg'),
]
HALF, STEP = 12, 2


def backproject(dmm):
    z = dmm.astype(np.float64) / 1000.0
    ok = z > 0
    return np.stack([(U - K.cx) * z / K.fx, (V - K.cy) * z / K.fy, z], axis=-1)[ok]


def raw_tilt(kp, d_m, up):
    """绕过质检门，只按关键点+深度算倾角。返回 (倾角, 高度, 同侧深度差) 或 None。"""
    pts = []
    for i in TORSO_KEYPOINT_IDS:
        z = sample_depth_near(d_m, kp[i][0], kp[i][1])
        if not np.isfinite(z) or z <= 0:
            return None
        pts.append(np.array([(kp[i][0] - K.cx) * z / K.fx,
                             (kp[i][1] - K.cy) * z / K.fy, z]))
    sl, sr, hl, hr = pts
    sm, hm = (sl + sr) / 2.0, (hl + hr) / 2.0
    dz = max(abs(sl[2] - sr[2]), abs(hl[2] - hr[2]))
    return float(np.degrees(torso_tilt_from_vertical(sm, hm, up=up))), (sm + hm) / 2.0, dz


det = Detector(DetectorConfig(weights='/home/wy/g2_testdata/yolo11n-pose.pt', conf=0.25))
cap = cv2.VideoCapture(VIDEO)

rows = []
for anchor, name, cls in ANCHORS:
    for fr in range(max(0, anchor - HALF), min(N, anchor + HALF + 1), STEP):
        fit = fit_floor_plane(backproject(raw[fr]), step=5)
        cap.set(cv2.CAP_PROP_POS_FRAMES, fr)
        ok, bgr = cap.read()
        if not ok:
            continue
        d_m = depth_to_meters(raw[fr], "16UC1")
        dets = [d for d in det.detect(bgr[:, :320])
                if d.class_name == 'person' and d.keypoints is not None]
        base = dict(anchor=anchor, name=name, cls=cls, fr=fr,
                    conf=None, tilt=None, height=None, dz=None,
                    gate=None, fit_ok=fit.ok)
        if not dets:
            base['gate'] = '检测器没出人'
            rows.append(base)
            continue
        d0 = max(dets, key=lambda x: x.confidence)
        base['conf'] = float(d0.confidence)
        kp = np.asarray(d0.keypoints, dtype=np.float64)
        if not fit.ok:
            base['gate'] = '地面不合格'
            rows.append(base)
            continue
        up, off = fit.normal, fit.offset
        rt = raw_tilt(kp, d_m, up)
        if rt is not None:
            base['tilt'], mid, base['dz'] = rt
            base['height'] = float(up @ mid + off)
        # 生产口径
        torso, why = keypoints_to_torso_3d(kp, d_m, K)
        base['gate'] = '通过' if torso is not None else why
        if torso is not None:
            sm = (torso.shoulder_left + torso.shoulder_right) / 2.0
            hm = (torso.hip_left + torso.hip_right) / 2.0
            base['prod_tilt'] = float(torso.tilt_deg(up))
            base['prod_height'] = float(((sm + hm) / 2.0) @ up + off)
        rows.append(base)

print(f'共 {len(rows)} 个采样\n')
print('=== 逐锚点（生产口径）===')
print(f"{'锚点':>5} {'活动':>8} {'类':>5} | {'出观测':>6} | "
      f"{'生产倾角':>16} {'生产高度':>16} | 主要拒绝原因")
print('-' * 92)
for anchor, name, cls in ANCHORS:
    sub = [r for r in rows if r['anchor'] == anchor]
    prod = [r for r in sub if r.get('prod_tilt') is not None]
    t = f"{np.median([r['prod_tilt'] for r in prod]):.0f}°" if prod else '--'
    h = f"{np.median([r['prod_height'] for r in prod]):+.2f}" if prod else '--'
    from collections import Counter
    rej = Counter(r['gate'] for r in sub if r.get('prod_tilt') is None)
    top = '  '.join(f'{k[:16]}×{v}' for k, v in rej.most_common(2))
    print(f'{anchor:>5} {name:>8} {cls:>5} | {len(prod):>3}/{len(sub):<2} | '
          f'{t:>16} {h:>16} | {top}')

print('\n=== 按类别汇总：**绕过质检门**的原始倾角（看信号本身在不在）===')
print(f"{'类别':>6} {'n':>4} | {'原始倾角中位':>10} {'范围':>16} | "
      f"{'质检通过':>8}")
print('-' * 62)
for cls, label in (('pos', '倒地'), ('bed', '躺床'), ('neg', '正常'), ('trans', '过渡')):
    sub = [r for r in rows if r['cls'] == cls and r['tilt'] is not None]
    if not sub:
        print(f'{label:>6} {0:>4} | 无有效倾角')
        continue
    t = np.array([r['tilt'] for r in sub])
    ok = sum(1 for r in sub if r.get('prod_tilt') is not None)
    print(f'{label:>6} {len(sub):>4} | {np.median(t):>10.0f}° '
          f'{f"{t.min():.0f}~{t.max():.0f}":>16} | {ok/len(sub):>8.0%}')

print('\n=== 阈值扫描：倾角门取多少能把「倒地」和其余分开？ ===')
print('（用绕过质检门的原始倾角，正类=躺地上）')
print(f"{'阈值':>5} {'TP':>4} {'FN':>4} {'FP':>4} {'TN':>4} | {'召回':>6} {'误报':>6}")
print('-' * 46)
pos = [r for r in rows if r['cls'] == 'pos' and r['tilt'] is not None]
oth = [r for r in rows if r['cls'] in ('neg', 'bed') and r['tilt'] is not None]
for thr in (30, 45, 60, 75, 90):
    tp = sum(1 for r in pos if 60 <= r['tilt'] <= 120 and r['tilt'] >= thr)
    tp = sum(1 for r in pos if thr <= r['tilt'] <= 120)
    fn = len(pos) - tp
    fp = sum(1 for r in oth if thr <= r['tilt'] <= 120)
    tn = len(oth) - fp
    rec = f'{tp/len(pos):.0%}' if pos else '--'
    fpr = f'{fp/len(oth):.0%}' if oth else '--'
    print(f'{thr:>5} {tp:>4} {fn:>4} {fp:>4} {tn:>4} | {rec:>6} {fpr:>6}')

print('\n=== 关键：躺床 vs 躺地上，倾角分得开吗？===')
for cls, label in (('pos', '躺地上'), ('bed', '躺床')):
    sub = [r for r in rows if r['cls'] == cls and r['tilt'] is not None]
    if sub:
        t = np.array([r['tilt'] for r in sub])
        h = np.array([r['height'] for r in sub if r['height'] is not None])
        print(f'  {label}: 倾角 {t.min():.0f}~{t.max():.0f}° (中位 {np.median(t):.0f}°)'
              f'   高度 ' + (f'{h.min():+.2f}~{h.max():+.2f} (中位 {np.median(h):+.2f})'
                             if h.size else '--'))
