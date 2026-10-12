#!/usr/bin/env python3
"""map_metrics.py — 2D 地图量化评价 (est vs GT)。
raw map_saver PGM: 0=occupied, 205=unknown, 254=free。
输出 IoU / precision / recall / F1 / cell accuracy / GT completeness%, 可选 Chamfer。
用法:
  python3 map_metrics.py --est E.pgm --gt G.pgm [--gt-pcd G.pcd] --method NAME --out map_metrics.csv
"""
import argparse, os, csv, sys
import numpy as np

OCC_THR, FREE_THR = 100, 200


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    i, n, toks = 0, len(data), []
    while len(toks) < 4:
        while i < n and data[i:i+1].isspace():
            i += 1
        if data[i:i+1] == b"#":
            while i < n and data[i:i+1] != b"\n":
                i += 1
            continue
        s = i
        while j_ok(data, i, n):
            i += 1
        toks.append(data[s:i])
    return _finish(data, i, toks)


def j_ok(data, i, n):
    return i < n and not data[i:i+1].isspace()


def _finish(data, i, toks):
    magic = toks[0].decode()
    W, H = int(toks[1]), int(toks[2])
    i += 1
    if magic == "P5":
        arr = np.frombuffer(data[i:i + W * H], dtype=np.uint8).reshape(H, W)
    else:
        arr = np.array(data[i:].split(), dtype=int).reshape(H, W).astype(np.uint8)
    return arr


def read_pcd(path):
    pts, npts = [], None
    with open(path, "r", errors="ignore") as f:
        header = True
        for ln in f:
            if header:
                if ln.startswith("POINTS"):
                    npts = int(ln.split()[1])
                elif ln.startswith("DATA"):
                    if "ascii" not in ln:
                        raise SystemExit("only ASCII .pcd supported")
                    header = False
                continue
            p = ln.split()
            if len(p) >= 3:
                pts.append((float(p[0]), float(p[1]), float(p[2])))
    return np.array(pts, dtype=float)


def boundary(m):
    return m & ~(np.roll(m, 1, 0) & np.roll(m, -1, 0) & np.roll(m, 1, 1) & np.roll(m, -1, 1))


def chamfer(A, B):
    if len(A) == 0 or len(B) == 0:
        return float("nan")
    try:
        from scipy.spatial import cKDTree
        dA = cKDTree(B).query(A, k=1)[0]
        dB = cKDTree(A).query(B, k=1)[0]
    except Exception:
        def nn(P, Q, ch=2000):
            o = np.empty(len(P))
            for s in range(0, len(P), ch):
                blk = P[s:s+ch]
                o[s:s+ch] = np.sqrt(((blk[:, None, :] - Q[None, :, :])**2).sum(-1)).min(1)
            return o
        dA, dB = nn(A, B), nn(B, A)
    return float(np.concatenate([dA, dB]).mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--est", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--gt-pcd", default=None)
    ap.add_argument("--method", required=True)
    ap.add_argument("--out", default="map_metrics.csv")
    ap.add_argument("--boundary", action="store_true")
    ap.add_argument("--res", type=float, default=0.05)
    a = ap.parse_args()

    est, gt = read_pgm(a.est), read_pgm(a.gt)
    H, W = min(est.shape[0], gt.shape[0]), min(est.shape[1], gt.shape[1])
    if est.shape != gt.shape:
        print(f"[warn] shape mismatch est={est.shape} gt={gt.shape}; crop {H}x{W}", file=sys.stderr)
    est, gt = est[:H, :W], gt[:H, :W]

    eo, ef = est < OCC_THR, est > FREE_THR
    go, gf = gt < OCC_THR, gt > FREE_THR
    ek, gk = eo | ef, go | gf

    tp = int((eo & go).sum()); fp = int((eo & ~go).sum()); fn = int((~eo & go).sum())
    union = int((eo | go).sum())
    iou = tp / union if union else float("nan")
    prec = tp / (tp + fp) if (tp + fp) else float("nan")
    rec = tp / (tp + fn) if (tp + fn) else float("nan")
    f1 = 2 * prec * rec / (prec + rec) if prec and rec and not np.isnan(prec + rec) else float("nan")
    known = ek & gk
    correct = (eo & go) | (ef & gf)
    acc = int((correct & known).sum()) / int(known.sum()) if known.sum() else float("nan")
    gocc = int(go.sum())
    comp = int((go & ek).sum()) / gocc * 100 if gocc else float("nan")

    cd = float("nan")
    if a.gt_pcd and os.path.isfile(a.gt_pcd):
        gp = read_pcd(a.gt_pcd)
        m = boundary(eo) if a.boundary else eo
        idx = np.argwhere(m)
        ex = (idx[:, 1] + 0.5) * a.res
        ey = (H - 1 - idx[:, 0] + 0.5) * a.res
        ep = np.column_stack([ex, ey, np.zeros(len(idx))])
        cd = chamfer(ep, gp[:, :3])

    row = {"method": a.method, "resolution": a.res, "est_occ_px": int(eo.sum()), "gt_occ_px": gocc,
           "iou": round(iou, 4), "precision": round(prec, 4), "recall": round(rec, 4),
           "f1": round(f1, 4), "cell_accuracy": round(acc, 4),
           "gt_completeness_pct": round(comp, 2),
           "chamfer_sym_m": round(cd, 4) if cd == cd else ""}
    ex_file = os.path.isfile(a.out)
    with open(a.out, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not ex_file:
            w.writeheader()
        w.writerow(row)
    print(f"[map_metrics] {a.method}: IoU={iou:.4f} F1={f1:.4f} acc={acc:.4f} "
          f"completeness={comp:.2f}% ch={row['chamfer_sym_m']}")


if __name__ == "__main__":
    main()
