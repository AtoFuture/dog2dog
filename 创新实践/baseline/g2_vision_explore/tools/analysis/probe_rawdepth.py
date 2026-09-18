"""查原始深度图的取值分布：0 以外还有没有别的「无效编码」混在里面。

如果有大量 1~700 的小值，它们会被 depth_to_meters 当成 0.001~0.7 m 的
合法深度，而 sample_depth_near 取的又是**近端 5 分位** —— 正好把这些垃圾全兜住。
"""
import sys
import numpy as np

W, H, N = 320, 240, 1378
raw = np.memmap('/tmp/d1378.raw', dtype='<u2', mode='r', shape=(N, H, W))

print('=== 全序列 raw 值分布（每 50 帧抽样）===')
tot = {}
for fr in range(0, N, 50):
    v = raw[fr].astype(np.int64).ravel()
    for lo, hi, name in ((0, 0, '0(未测到)'), (1, 50, '1~50'), (51, 200, '51~200'),
                         (201, 500, '201~500'), (501, 700, '501~700'),
                         (701, 4000, '701~4000'), (4001, 65535, '>4000')):
        tot[name] = tot.get(name, 0) + int(((v > lo - 1) & (v <= hi)).sum() if lo else (v == 0).sum())
n = len(range(0, N, 50)) * W * H
print(f'  抽样 {len(range(0,N,50))} 帧，共 {n} 像素')
for name in ('0(未测到)', '1~50', '51~200', '201~500', '501~700', '701~4000', '>4000'):
    c = tot.get(name, 0)
    print(f'    {name:>10}: {c:>10}  {c/n:6.2%}')

print()
print('=== 非零值里的最小值分布（这些会被当成有效深度）===')
for fr in (1305, 545, 270, 1263, 114):
    v = raw[fr].astype(np.int64)
    nz = v[v > 0]
    print(f'  帧{fr:>5}: 非零 {nz.size:>6} 个   最小 {nz.min():>5}   '
          f'p1 {np.percentile(nz,1):>6.0f}   p5 {np.percentile(nz,5):>6.0f}   '
          f'中位 {np.median(nz):>6.0f}   <700 的占非零 {np.mean(nz<700):6.2%}')

print()
print('=== 具体窗口：帧 1305 左肩 px=(206.1,143.1) 的 35x35 原始值 ===')
for fr, (u, v), label in ((1305, (206, 143), 'L肩'), (545, (213, 146), 'L髋'),
                          (270, (218, 154), 'L髋'), (1263, (0, 0), None)):
    if label is None:
        continue
    patch = raw[fr, max(0, v - 17):v + 18, max(0, u - 17):u + 18].astype(np.int64)
    print(f'  帧{fr} {label} 窗口 {patch.shape}:')
    vals = patch.ravel()
    print(f'    零值 {np.sum(vals==0):>4}  非零 {np.sum(vals>0):>4}   '
          f'非零最小 {vals[vals>0].min() if (vals>0).any() else -1}')
    print(f'    p5(含零)={np.percentile(vals,5):.0f}  '
          f'p5(非零)={np.percentile(vals[vals>0],5):.0f}  '
          f'中位(非零)={np.median(vals[vals>0]):.0f}' if (vals > 0).any() else '    全零')
    small = vals[(vals > 0) & (vals < 700)]
    print(f'    <700 的非零值: {small.size} 个，样例 {sorted(small)[:10]}')
