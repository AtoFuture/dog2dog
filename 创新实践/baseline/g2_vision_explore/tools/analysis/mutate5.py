"""变异：把 floor.py 的相机高度下限改回去 / 改过头，看测试拦不拦得住。"""
import subprocess, shutil, sys
ROOT = '/home/wy/project/baseline/g2_vision_explore'
F = f'{ROOT}/g2_core/floor.py'
MUTS = [
    ('M1 下限调回 1.0（本次要修的那个 bug）',
     'min_camera_height_m: float = 0.2,', 'min_camera_height_m: float = 1.0,'),
    ('M2 下限放到 0.0（校验形同虚设）',
     'min_camera_height_m: float = 0.2,', 'min_camera_height_m: float = 0.0,'),
    ('M3 上限收紧到 0.5（把数据集素材挡掉）',
     'max_camera_height_m: float = 3.0,', 'max_camera_height_m: float = 0.5,'),
]
def run():
    p = subprocess.run([sys.executable,'-B','-m','pytest','tests/test_floor.py','-q','-p','no:warnings','--tb=line'],
                       cwd=ROOT, capture_output=True, text=True)
    return p.returncode != 0, (p.stdout+p.stderr)
surv = []
for label, old, new in MUTS:
    src = open(F).read()
    if old not in src:
        print(f'  ?? {label}: 找不到模式'); surv.append(label); continue
    shutil.copy(F, F+'.bak')
    try:
        open(F,'w').write(src.replace(old,new,1))
        for d in ('g2_core/__pycache__','tests/__pycache__'):
            shutil.rmtree(f'{ROOT}/{d}', ignore_errors=True)
        caught, out = run()
    finally:
        shutil.move(F+'.bak', F)
        for d in ('g2_core/__pycache__','tests/__pycache__'):
            shutil.rmtree(f'{ROOT}/{d}', ignore_errors=True)
    if caught:
        line = [l for l in out.splitlines() if 'FAILED' in l or 'assert' in l]
        print(f'  ✓ 被拦下  {label}')
        if line: print(f'            {line[0][:100]}')
    else:
        print(f'  ✗ 存活！  {label}'); surv.append(label)
print()
print('全部拦下。' if not surv else f'{len(surv)} 个存活：{surv}')
