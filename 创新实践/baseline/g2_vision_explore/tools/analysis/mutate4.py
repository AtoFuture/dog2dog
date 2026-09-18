"""变异测试：故意改坏 projector.py 里新加的「有效量程下限」，看测试拦不拦得住。

重点验证 test_sample_depth_near_rejects_flying_pixels 不是空测试 ——
把下限拿掉之后它**必须**失败。
"""
import subprocess
import shutil
import sys
import os

ROOT = '/home/wy/project/baseline/g2_vision_explore'
PROJ = f'{ROOT}/g2_core/projector.py'

MUTS = [
    ('M1 默认下限改成 0（等于没修）',
     'min_depth_m: float = MIN_VALID_DEPTH_M,', 'min_depth_m: float = 0.0,'),
    ('M2 常量本身改成 0',
     'MIN_VALID_DEPTH_M = 0.6', 'MIN_VALID_DEPTH_M = 0.0'),
    ('M3 完全不过滤（只查有限性）',
     'valid = patch[np.isfinite(patch) & (patch >= min_depth_m)]',
     'valid = patch[np.isfinite(patch)]'),
    ('M4 比较方向写反（留下飞点、丢掉真值）',
     '(patch >= min_depth_m)', '(patch <= min_depth_m)'),
    ('M5 下限设成 3.0（把真值也挡掉）',
     'MIN_VALID_DEPTH_M = 0.6', 'MIN_VALID_DEPTH_M = 3.0'),
]


def run_tests():
    p = subprocess.run(
        [sys.executable, '-B', '-m', 'pytest',
         'tests/test_projector.py', 'tests/test_anomaly.py',
         '-q', '--no-header'],
        cwd=ROOT, capture_output=True, text=True,
    )
    return p.returncode != 0, (p.stdout + p.stderr)


survived = []
for label, old, new in MUTS:
    src = open(PROJ).read()
    if old not in src:
        print(f'  ?? {label}: 找不到待改字符串，跳过')
        survived.append((label, 'PATTERN NOT FOUND'))
        continue
    shutil.copy(PROJ, PROJ + '.bak')
    try:
        open(PROJ, 'w').write(src.replace(old, new, 1))
        for d in ('g2_core/__pycache__', 'tests/__pycache__'):
            shutil.rmtree(f'{ROOT}/{d}', ignore_errors=True)
        caught, out = run_tests()
    finally:
        shutil.move(PROJ + '.bak', PROJ)
        for d in ('g2_core/__pycache__', 'tests/__pycache__'):
            shutil.rmtree(f'{ROOT}/{d}', ignore_errors=True)

    if caught:
        line = [l for l in out.splitlines() if l.startswith('FAILED') or ' failed' in l]
        print(f'  ✓ 被拦下  {label}   {line[0][:90] if line else ""}')
    else:
        print(f'  ✗ 存活！  {label}')
        survived.append((label, ''))

print()
if survived:
    print(f'有 {len(survived)} 个变异存活：')
    for lb, why in survived:
        print(f'   - {lb} {why}')
else:
    print('全部变异都被拦下。')
