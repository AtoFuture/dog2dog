#!/usr/bin/env python3
"""filatov_metrics.py — Filatov 等(arXiv:1708.02354) 的无真值 2D 地图质量三指标。
无需真值地图/轨迹，直接在占据栅格上计算，用于量化"毛刺/重影/闭环失败"。
  1) occupied_ratio  = occupied / (occupied + free)   —— 墙壁变厚/重影会升高
  2) enclosed_areas  = 不接触地图边界的 free 连通域数 —— 闭环失败/房间重叠会增多
  3) corner_count    = Harris 角点数(结构图)          —— 伪影/错位会增多
用法: python3 filatov_metrics.py --map M.pgm --method NAME [--out filatov.csv]
"""
import argparse, os, csv
import numpy as np

OCC, FREE = 100, 250  # 205=unknown 不算 free


def read_pgm(path):
    with open(path, "rb") as f:
        d = f.read()
    i, n, toks = 0, len(d), []
    while len(toks) < 4:
        while i < n and d[i:i+1].isspace():
            i += 1
        if d[i:i+1] == b"#":
            while i < n and d[i:i+1] != b"\n":
                i += 1
            continue
        s = i
        while i < n and not d[i:i+1].isspace():
            i += 1
        toks.append(d[s:i]); i = s + (i - s)
    W, H = int(toks[1]), int(toks[2]); i += 1
    return np.frombuffer(d[i:i+W*H], np.uint8).reshape(H, W)


def label(mask):
    try:
        from scipy import ndimage
        lab, n = ndimage.label(mask, structure=np.ones((3, 3), np.int32))
        return lab, n
    except Exception:
        H, W = mask.shape
        lab = np.zeros((H, W), np.int32); n = 0
        from collections import deque
        for r, c in np.argwhere(mask):
            if lab[r, c]:
                continue
            n += 1; q = deque([(int(r), int(c))]); lab[r, c] = n
            while q:
                y, x = q.popleft()
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = y+dy, x+dx
                        if 0 <= ny < H and 0 <= nx < W and mask[ny, nx] and not lab[ny, nx]:
                            lab[ny, nx] = n; q.append((ny, nx))
        return lab, n


def harris(img, k=0.04, thr=1e4):
    f = img.astype(np.float64)
    gx = np.zeros_like(f); gy = np.zeros_like(f)
    gx[:, 1:-1] = f[:, 2:] - f[:, :-2]
    gy[1:-1, :] = f[2:, :] - f[:-2, :]
    Ixx, Iyy, Ixy = gx*gx, gy*gy, gx*gy
    S = np.ones((3, 3))
    def box(a):
        out = np.zeros_like(a)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                out += np.roll(np.roll(a, dy, 0), dx, 1)
        return out
    Sxx, Syy, Sxy = box(Ixx), box(Iyy), box(Ixy)
    R = (Sxx*Syy - Sxy*Sxy) - k*(Sxx + Syy)**2
    return int((R > thr).sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--method", required=True)
    ap.add_argument("--out", default="filatov.csv")
    a = ap.parse_args()
    g = read_pgm(a.map)
    occ = g < OCC
    free = g > FREE
    o, fr = int(occ.sum()), int(free.sum())
    ratio = o / (o + fr) if (o + fr) else float("nan")

    lab, ncomp = label(free)
    border = set(np.unique(np.concatenate([lab[0, :], lab[-1, :], lab[:, 0], lab[:, -1]])))
    border.discard(0)
    enclosed = sum(1 for i in range(1, ncomp + 1) if i not in border)

    corners = harris(occ.astype(np.uint8))

    row = {"method": a.method, "W": g.shape[1], "H": g.shape[0],
           "occ_px": o, "free_px": fr, "unknown_px": int((~occ & ~free).sum()),
           "occupied_ratio": round(ratio, 4), "free_components": ncomp,
           "enclosed_areas": enclosed, "corner_count": corners}
    ex = os.path.isfile(a.out)
    with open(a.out, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not ex:
            w.writeheader()
        w.writerow(row)
    print(f"[filatov] {a.method}: occ_ratio={ratio:.4f} enclosed={enclosed} "
          f"corners={corners} free_comp={ncomp}")


if __name__ == "__main__":
    main()
