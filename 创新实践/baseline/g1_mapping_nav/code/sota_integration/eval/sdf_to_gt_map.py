#!/usr/bin/env python3
"""sdf_to_gt_map.py — 把 Gazebo .sdf 世界里的静态盒状碰撞体栅格化成 2D 真值栅格。
用法: python3 sdf_to_gt_map.py --sdf W.sdf --ref_yaml est.yaml -o gt_map [--pcd gt.pcd]
网格几何完全复用 est 地图的 resolution/origin/尺寸 -> 可与 est 逐格比对(IoU)。
假设 map 帧 ≈ world 帧(单机器人、从 spawn 起步)。
"""
import argparse, os, math
import xml.etree.ElementTree as ET
import numpy as np


def pose6(e, dflt=(0, 0, 0, 0, 0, 0)):
    p = e.find("pose")
    if p is None or not p.text:
        return list(dflt)
    v = [float(x) for x in p.text.split()]
    if len(v) == 6:
        return v
    if len(v) == 3:
        return v + [0, 0, 0]
    return list(dflt)


def se2(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1]])


def read_pgm_dims(path):
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
    return int(toks[1]), int(toks[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sdf", required=True)
    ap.add_argument("--ref_yaml", required=True)
    ap.add_argument("--exclude", default=r"ground|floor|terrain|plane")
    ap.add_argument("--min_height", type=float, default=0.2)
    ap.add_argument("--pcd", default=None)
    ap.add_argument("-o", "--out", required=True)
    a = ap.parse_args()

    import yaml
    y = yaml.safe_load(open(a.ref_yaml))
    res = float(y["resolution"])
    ox, oy, oyaw = [float(v) for v in y["origin"]]
    img = y["image"]
    imgp = img if os.path.isabs(img) else os.path.join(os.path.dirname(a.ref_yaml), img)
    W, H = read_pgm_dims(imgp)
    print(f"[gt] ref grid {W}x{H} res={res} origin=({ox},{oy},{oyaw})")

    cols = np.arange(W); rows = np.arange(H)
    wx = ox + (cols + 0.5) * res
    wy = oy + (H - 1 - rows + 0.5) * res     # row0 = top (max y)
    WX, WY = np.meshgrid(wx, wy)

    # 若 ref 地图有 yaw，先在网格采样点上反向旋转
    if abs(oyaw) > 1e-9:
        lx = WX - ox; ly = WY - oy
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        WX = ox + c*lx - s*ly; WY = oy + s*lx + c*ly

    imp = __import__("re").compile(a.exclude, __import__("re").IGNORECASE)
    root = ET.parse(a.sdf).getroot()
    world = root.find("world") if root.tag == "sdf" else root
    occ = np.zeros((H, W), dtype=bool)
    n_box = 0
    for model in world.findall("model"):
        nm = model.get("name", "")
        if imp.search(nm):
            continue
        if (model.findtext("static", "false").strip().lower() not in ("1", "true")):
            continue
        mp = pose6(model)
        Tm = se2(mp[0], mp[1], mp[5])
        for link in model.findall("link"):
            lp = pose6(link)
            Tl = Tm @ se2(lp[0], lp[1], lp[5])
            for col in link.findall("collision"):
                g = col.find("geometry")
                if g is None:
                    continue
                box = g.find("box")
                if box is None:
                    continue
                sx, sy, sz = [float(v) for v in box.find("size").text.split()]
                if sz < a.min_height:
                    continue
                cp = pose6(col)
                Tc = Tl @ se2(cp[0], cp[1], cp[5])
                cx, cy = Tc[0, 2], Tc[1, 2]
                yaw = math.atan2(Tc[1, 0], Tc[0, 0])
                dx, dy = WX - cx, WY - cy
                c, s = math.cos(-yaw), math.sin(-yaw)
                lx = c*dx - s*dy; ly = s*dx + c*dy
                occ |= (np.abs(lx) <= sx/2 + 1e-9) & (np.abs(ly) <= sy/2 + 1e-9)
                n_box += 1
    grid = np.full((H, W), 254, np.uint8)
    grid[occ] = 0
    print(f"[gt] boxes={n_box} occupied_cells={int(occ.sum())}")

    with open(a.out + ".pgm", "wb") as f:
        f.write(b"P5\n"); f.write(f"{W} {H}\n255\n".encode()); f.write(grid.tobytes())
    with open(a.out + ".yaml", "w") as f:
        f.write(f"image: {os.path.basename(a.out)}.pgm\nresolution: {res}\n")
        f.write(f"origin: [{ox}, {oy}, {oyaw}]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n")
    print(f"[gt] wrote {a.out}.pgm/.yaml")

    if a.pcd:
        idx = np.argwhere(occ)
        xs = ox + (idx[:, 1] + 0.5) * res
        ys = oy + (H - 1 - idx[:, 0] + 0.5) * res
        with open(a.pcd, "w") as f:
            f.write("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
            f.write(f"WIDTH {len(idx)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(idx)}\nDATA ascii\n")
            for x, yy in zip(xs, ys):
                f.write(f"{x:.4f} {yy:.4f} 0.0000\n")
        print(f"[gt] wrote {a.pcd} ({len(idx)} pts)")


if __name__ == "__main__":
    main()
