"""把原始深度 1~2000 细粒度直方图打出来，找「垃圾值」与「真实测距」之间的空档。

同时把垃圾像素的**空间分布**画出来 —— 是均匀撒的，还是集中在物体边缘。
"""
import sys
import numpy as np
import cv2

W, H, N = 320, 240, 1378
raw = np.memmap('/tmp/d1378.raw', dtype='<u2', mode='r', shape=(N, H, W))

print('=== 原始深度细粒度直方图（全序列每 10 帧抽样）===')
bins = [(1, 10), (10, 50), (50, 100), (100, 200), (200, 300), (300, 400),
        (400, 500), (500, 600), (600, 700), (700, 800), (800, 900),
        (900, 1000), (1000, 1200), (1200, 1500), (1500, 2000)]
counts = {b: 0 for b in bins}
total = 0
for fr in range(0, N, 10):
    v = raw[fr].astype(np.int64).ravel()
    total += v.size
    for lo, hi in bins:
        counts[(lo, hi)] += int(((v >= lo) & (v < hi)).sum())
print(f'  {len(range(0,N,10))} 帧, {total} 像素')
for (lo, hi), c in counts.items():
    bar = '#' * int(c / max(1, total) * 2000)
    print(f'    {lo:>5}~{hi:<5}: {c:>8}  {c/total:7.4%}  {bar}')

print()
print('=== 垃圾像素的空间分布（帧 1305 / 545 / 270）===')
tiles = []
for fr in (1305, 545, 270, 1263):
    v = raw[fr].astype(np.int64)
    vis = np.zeros((H, W, 3), dtype=np.uint8)
    valid = (v >= 700)
    vis[valid] = np.clip((v[valid] - 700) / 3500 * 200 + 40, 0, 255).astype(np.uint8)[:, None]
    vis[v == 0] = (0, 0, 0)
    vis[(v > 0) & (v < 500)] = (0, 0, 255)      # 垃圾：红
    vis[(v >= 500) & (v < 700)] = (0, 200, 255)  # 可疑：黄
    cv2.putText(vis, f'f{fr} red=<0.5m yellow=0.5-0.7m', (4, 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1)
    tiles.append(cv2.resize(vis, (W * 2, H * 2), interpolation=cv2.INTER_NEAREST))
grid = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
cv2.imwrite('/tmp/depth_garbage.png', grid)
print('  wrote /tmp/depth_garbage.png')

print()
print('=== 关键帧的 35x35 窗口里，垃圾占比（决定 p5 是否被污染）===')
sys.path.insert(0, '/home/wy/project/baseline/g2_vision_explore')
CASES = [
    (1305, (206, 143), 'L肩'), (1305, (207, 130), 'R肩'),
    (545, (213, 146), 'L髋'), (545, (213, 138), 'R髋'),
    (270, (218, 154), 'L髋'), (270, (192, 149), 'R髋'),
    (1263, (190, 88), '肩(直立)'), (114, (196, 84), '肩(直立)'),
]
print(f"  {'帧':>5} {'位置':>10} {'非零':>5} {'<500':>5} {'<700':>5} "
      f"{'垃圾%':>7}  p5是否被污染")
for fr, (u, v), label in CASES:
    patch = raw[fr, max(0, v - 17):v + 18, max(0, u - 17):u + 18].astype(np.int64).ravel()
    nz = int((patch > 0).sum())
    small5 = int(((patch > 0) & (patch < 500)).sum())
    small7 = int(((patch > 0) & (patch < 700)).sum())
    frac = small7 / max(1, nz)
    print(f'  {fr:>5} {label:>10} {nz:>5} {small5:>5} {small7:>5} '
          f'{frac:>7.2%}  {"是" if frac >= 0.05 else "否"}')
