import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image

NS = "{http://www.collada.org/2005/11/COLLADASchema}"
BASE = S = 0.6 * 0.01               # cm -> m with scale 0.6
OFF = np.array([7.19009, 1.09982])  # depot model pose


def parse(path):
    root = ET.parse(path).getroot()
    SA = {}
    for src in root.iter(NS + "source"):
        fa = src.find(NS + "float_array")
        if fa is not None:
            SA[src.get("id")] = np.fromstring(fa.text, sep=" ")
    geos = {}
    for geom in root.iter(NS + "geometry"):
        mesh = geom.find(NS + "mesh")
        if mesh is None:
            continue
        verts = mesh.find(NS + "vertices")
        vpos = None
        for inp in verts.findall(NS + "input"):
            if inp.get("semantic") == "POSITION":
                a = SA.get(inp.get("source").lstrip("#"))
                if a is not None and a.size % 3 == 0:
                    vpos = a.reshape(-1, 3)
                break
        if vpos is None:
            continue
        tris = []
        for prim in list(mesh.findall(NS + "triangles")) + list(mesh.findall(NS + "polylist")):
            off = 0
            for inp in prim.findall(NS + "input"):
                if inp.get("semantic") == "VERTEX":
                    off = int(inp.get("offset"))
            stride = max(int(i.get("offset")) for i in prim.findall(NS + "input")) + 1
            p = prim.find(NS + "p")
            if p is None:
                continue
            idx = np.fromstring(p.text, sep=" ", dtype=np.int64).reshape(-1, stride)[:, off]
            tris.append(idx.reshape(-1, 3))
        if tris:
            geos[geom.get("id")] = (vpos, np.vstack(tris))
    # scene graph
    allv = []
    scene = root.find(f"{NS}library_visual_scenes/{NS}visual_scene")
    nodes_by_id = {n.get("id"): n for n in root.iter(NS + "node") if n.get("id")}

    def mat_of(n):
        for ch in n:
            tag = ch.tag.replace(NS, "")
            if tag == "matrix":
                return np.array([float(x) for x in ch.text.split()]).reshape(4, 4).T
            if tag in ("translate", "rotate", "scale"):
                pass
        # compose TRS
        M = np.eye(4)
        for ch in n:
            tag = ch.tag.replace(NS, "")
            v = [float(x) for x in (ch.text or "").split()]
            if tag == "translate":
                T = np.eye(4); T[:3, 3] = v; M = M @ T
            elif tag == "scale":
                Sc = np.eye(4); Sc[0, 0], Sc[1, 1], Sc[2, 2] = v; M = M @ Sc
            elif tag == "rotate":
                ax = np.array(v[:3]); ang = np.radians(v[3]); ax = ax / (np.linalg.norm(ax) + 1e-12)
                c, s = np.cos(ang), np.sin(ang); x, y, z = ax
                R = np.eye(4)
                R[:3, :3] = np.array([
                    [c + x*x*(1-c), x*y*(1-c) - z*s, x*z*(1-c) + y*s],
                    [y*x*(1-c) + z*s, c + y*y*(1-c), y*z*(1-c) - x*s],
                    [z*x*(1-c) - y*s, z*y*(1-c) + x*s, c + z*z*(1-c)]])
                M = M @ R
        return M

    def walk(n, M):
        M2 = M @ mat_of(n)
        for ig in n.findall(NS + "instance_geometry"):
            gid = ig.get("url").lstrip("#")
            if gid in geos:
                vpos, tris = geos[gid]
                V = (M2[:3, :3] @ vpos.T).T + M2[:3, 3]
                allv.append(V[tris])
        for ch in n.findall(NS + "node"):
            walk(ch, M2)
        for iN in n.findall(NS + "instance_node"):
            tgt = iN.get("url").lstrip("#")
            if tgt in nodes_by_id:
                walk(nodes_by_id[tgt], M2)

    for n in scene.findall(NS + "node"):
        walk(n, np.eye(4))
    return np.vstack(allv) if allv else np.zeros((0, 3, 3))


trisA = parse(r"C:\Users\19373\AppData\Local\Temp\opencode\Depot.dae")
trisB = parse(r"C:\Users\19373\AppData\Local\Temp\opencode\Crates.dae")
print("Depot tris", len(trisA), "Crates tris", len(trisB))
T = np.vstack([trisA, trisB])
# to meters + offset
T = T * BASE
T[:, :, 0] += OFF[0]; T[:, :, 1] += OFF[1]
print("world x", T[:,:,0].min().round(2), T[:,:,0].max().round(2),
      "y", T[:,:,1].min().round(2), T[:,:,1].max().round(2))

res = 0.05
x0, x1, y0, y1 = -16.0, 24.0, -9.0, 10.0
W = int((x1 - x0) / res); H = int((y1 - y0) / res)
g = np.zeros((H, W), bool)


def mark(ax, ay, bx, by):
    d = np.hypot(bx - ax, by - ay)
    n = max(2, int(d / (res * 0.5)))
    xs = np.linspace(ax, bx, n); ys = np.linspace(ay, by, n)
    cx = ((xs - x0) / res).astype(int); cy = ((ys - y0) / res).astype(int)
    ok = (cx >= 0) & (cx < W) & (cy >= 0) & (cy < H)
    g[cy[ok], cx[ok]] = True


zmin = T[:,:,2].min(1); zmax = T[:,:,2].max(1)
keep = (zmin < 2.5) & (zmax > 0.15)
print('kept tris', int(keep.sum()), 'of', len(T))
for tri in T[keep]:
    mark(tri[0, 0], tri[0, 1], tri[1, 0], tri[1, 1])
    mark(tri[1, 0], tri[1, 1], tri[2, 0], tri[2, 1])
    mark(tri[2, 0], tri[2, 1], tri[0, 0], tri[0, 1])
for _ in range(2):
    g = g | np.roll(g, 1, 0) | np.roll(g, -1, 0) | np.roll(g, 1, 1) | np.roll(g, -1, 1)
Image.fromarray(np.flipud(np.where(g, 0, 255).astype(np.uint8))).save(
    r"C:\Users\19373\AppData\Local\Temp\opencode\depot_full.png")

xs = ((np.arange(-6.0, 20.0, 0.1) - x0) / res).astype(int)
print("free rows (x -6..20):")
for yy in np.arange(-6.6, 8.7, 0.2):
    cy = int((yy - y0) / res)
    if 0 <= cy < H:
        frac = g[cy, xs].mean()
        print(f"  y={yy:5.2f} occ={frac:.2f} {'FREE' if frac<0.1 else ('part' if frac<0.5 else 'OCC')}")
