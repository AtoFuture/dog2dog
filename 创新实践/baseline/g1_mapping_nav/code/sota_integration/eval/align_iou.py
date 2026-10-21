#!/usr/bin/env python3
"""align_iou.py — 用带 GT 的 bag 求 map<->world 的 SE(2)(Umeyama)，把 SDF 真值反变换到
est 网格后计算 IoU/F1/accuracy/completeness（GT 自由空间 flood-fill，其余标未知）。
用法:
  python3 align_iou.py --bag BAGDIR --sdf W.sdf --est MAP.pgm --est-yaml MAP.yaml --method NAME
"""
import argparse, os, math, re
import numpy as np
import xml.etree.ElementTree as ET
import yaml


# ---------- PGM ----------
def read_pgm(path):
    d = open(path, "rb").read()
    i, t = 0, []
    while len(t) < 4:
        while d[i:i+1].isspace():
            i += 1
        if d[i:i+1] == b"#":
            while d[i:i+1] != b"\n":
                i += 1
            continue
        s = i
        while not d[i:i+1].isspace():
            i += 1
        t.append(d[s:i])
    W, H = int(t[1]), int(t[2]); i += 1
    return np.frombuffer(d[i:i+W*H], np.uint8).reshape(H, W)


# ---------- SDF ----------
def pose6(e):
    p = e.find("pose")
    if p is None or not p.text:
        return [0, 0, 0, 0, 0, 0]
    v = [float(x) for x in p.text.split()]
    return v + [0, 0, 0] if len(v) == 3 else v


def se2(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1]])


def sdf_walls(sdf, exclude, min_h=0.2):
    """返回 world 系下的 (cx,cy,yaw,sx,sy) 列表。"""
    imp = re.compile(exclude, re.I)
    root = ET.parse(sdf).getroot()
    world = root.find("world") if root.tag == "sdf" else root
    out = []
    for m in world.findall("model"):
        if imp.search(m.get("name", "")):
            continue
        if m.findtext("static", "false").strip().lower() not in ("1", "true"):
            continue
        mp = pose6(m); Tm = se2(mp[0], mp[1], mp[5])
        for lk in m.findall("link"):
            lp = pose6(lk); Tl = Tm @ se2(lp[0], lp[1], lp[5])
            for col in lk.findall("collision"):
                g = col.find("geometry")
                if g is None or g.find("box") is None:
                    continue
                sx, sy, sz = [float(v) for v in g.find("box").find("size").text.split()]
                if sz < min_h:
                    continue
                cp = pose6(col); Tc = Tl @ se2(cp[0], cp[1], cp[5])
                out.append((Tc[0, 2], Tc[1, 2], math.atan2(Tc[1, 0], Tc[0, 0]), sx, sy))
    return out


# ---------- bag: GT 轨迹 + map->base_link ----------
def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def traj_from_bag(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer
    from rclpy.duration import Duration as RclDuration
    from rclpy.time import Time as RclTime
    try:
        import yaml as _y
        storage = _y.safe_load(open(os.path.join(bag, "metadata.yaml")))
        storage = storage.get("rosbag2_bagfile_information", storage).get("storage_identifier", "sqlite3")
    except Exception:
        storage = "sqlite3"
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    gt, tfs, statics = [], [], []
    while rd.has_next():
        topic, data, _ = rd.read_next()
        if topic == "/gt/pose" and "TFMessage" in types.get(topic, ""):
            m = deserialize_message(data, TFMessage)
            for tr in m.transforms:
                nm = tr.child_frame_id.lstrip("/")
                if nm == "robot1_my_bot" or nm.endswith("/robot1_my_bot"):
                    p, q = tr.transform.translation, tr.transform.rotation
                    gt.append((tr.header.stamp.sec + tr.header.stamp.nanosec*1e-9, p.x, p.y, yaw_of(q)))
        elif topic == "/tf_static" and "TFMessage" in types.get(topic, ""):
            statics.append(data)
        elif topic == "/tf" and "TFMessage" in types.get(topic, ""):
            tfs.append(data)
    # 名不全时退一步：取 GT 里出现的第一个非 link 顶层名
    if not gt:
        rd2 = rosbag2_py.SequentialReader()
        rd2.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage), rosbag2_py.ConverterOptions("", ""))
        names = set()
        while rd2.has_next():
            topic, data, _ = rd2.read_next()
            if topic == "/gt/pose":
                for tr in deserialize_message(data, TFMessage).transforms:
                    names.add(tr.child_frame_id)
        print("  [gt] candidate child frames:", sorted(names)[:10])
        raise SystemExit("no GT transform matched robot1_my_bot")

    buf = Buffer(cache_time=RclDuration(seconds=100000))
    for d in statics:
        for tr in deserialize_message(d, TFMessage).transforms:
            try: buf.set_transform_static(tr, "bag")
            except Exception: pass
    for d in tfs:
        for tr in deserialize_message(d, TFMessage).transforms:
            try: buf.set_transform(tr, "bag")
            except Exception: pass

    est = []
    for (t, x, y, yw) in gt:
        try:
            tf = buf.lookup_transform("map", "base_link",
                                      RclTime(seconds=int(t), nanoseconds=int((t-int(t))*1e9)),
                                      RclDuration(seconds=0.1))
            p, q = tf.transform.translation, tf.transform.rotation
            est.append((t, p.x, p.y, yaw_of(q)))
        except Exception:
            continue
    return gt, est


def umeyama2d(P, Q):
    """求 R(2x2),t 使 Q ~= R P + t（无尺度）。P=map, Q=world"""
    pc, qc = P.mean(0), Q.mean(0)
    H = (P-pc).T @ (Q-qc)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, d]) @ U.T
    t = qc - R @ pc
    return R, t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--sdf", required=True)
    ap.add_argument("--est", required=True)
    ap.add_argument("--est-yaml", required=True)
    ap.add_argument("--method", default="est")
    ap.add_argument("--exclude", default=r"ground|floor|terrain|plane")
    ap.add_argument("--out-pgm", default="/tmp/gt_aligned.pgm")
    a = ap.parse_args()

    gtd, estd = traj_from_bag(a.bag)
    n = min(len(gtd), len(estd))
    print(f"[traj] gt={len(gtd)} est={len(estd)} matched={n}")
    if n < 10:
        raise SystemExit("too few matched poses; is slam+GT running together?")
    # 按时间就近配对
    gt_map = {round(t, 2): (x, y, yw) for (t, x, y, yw) in gtd}
    P, Q = [], []
    for (t, x, y, yw) in estd:
        k = min(gt_map, key=lambda kk: abs(kk - t))
        if abs(k - t) < 0.05:
            gx, gy, gyw = gt_map[k]
            P.append([x, y]); Q.append([gx, gy])
    P, Q = np.array(P), np.array(Q)
    print(f"[traj] paired={len(P)}")

    # 锚点法(robust)：用起点位姿对确定 SE(2)。避免"近似共线轨迹"导致 Umeyama 旋转退化
    # (实测退化会给出 ~100deg 的伪旋转,残差却=0)。
    K = min(200, len(estd))
    de = np.radians(np.array([e[3] for e in estd[:K]]) - np.array([g[3] for g in gtd[:K]]))
    yaw = math.degrees(math.atan2(np.mean(np.sin(de)), np.mean(np.cos(de))))  # 循环均值
    R = np.array([[math.cos(math.radians(yaw)), -math.sin(math.radians(yaw))],
                  [math.sin(math.radians(yaw)),  math.cos(math.radians(yaw))]])
    t = Q.mean(0) - R @ P.mean(0)
    print(f"[traj] world<-map (anchor): yaw={yaw:.2f}deg  t=({t[0]:.3f},{t[1]:.3f})  "
          f"|t|={np.hypot(*t):.2f}m  K={K}")
    resid = Q - (P @ R.T + t)
    print(f"[traj] residual RMSE={np.sqrt((resid**2).sum(1).mean())*100:.2f} cm  "
          f"(median drift={np.median(np.hypot(*(resid.T)))*100:.2f} cm)")

    # ---- 把 SDF 墙(世界) 反变换到 map 帧，栅格化到 est 网格 ----
    y = yaml.safe_load(open(a.est_yaml))
    res = float(y["resolution"]); ox, oy, oyaw = [float(v) for v in y["origin"]]
    est = read_pgm(a.est)
    H, W = est.shape
    cols = np.arange(W); rows = np.arange(H)
    mx = ox + (cols + 0.5) * res
    my = oy + (H - 1 - rows + 0.5) * res
    MX, MY = np.meshgrid(mx, my)
    if abs(oyaw) > 1e-9:
        lx, ly = MX - ox, MY - oy
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        MX = ox + c*lx - s*ly; MY = oy + s*lx + c*ly

    Rt = R.T; tinv = -R.T @ t     # world -> map:  m = R^T (w - t)
    occ = np.zeros((H, W), bool)
    for (cx, cy, cyaw, sx, sy) in sdf_walls(a.sdf, a.exclude):
        mcx = Rt[0, 0]*cx + Rt[0, 1]*cy + tinv[0]
        mcy = Rt[1, 0]*cx + Rt[1, 1]*cy + tinv[1]
        myaw = cyaw - math.radians(yaw)
        dx, dy = MX - mcx, MY - mcy
        c, s = math.cos(-myaw), math.sin(-myaw)
        lx = c*dx - s*dy; ly = s*dx + c*dy
        occ |= (np.abs(lx) <= sx/2 + 1e-9) & (np.abs(ly) <= sy/2 + 1e-9)

    # GT: 0=occ, 254=free, 205=unknown; 自由空间 flood-fill(从中心连通)
    gt = np.full((H, W), 205, np.uint8)
    gt[occ] = 0
    free = ~occ
    try:
        from scipy import ndimage
        lab, _ = ndimage.label(free, structure=np.ones((3, 3), int))
        c0 = lab[H//2, W//2]
        inner = (lab == c0) if c0 else free
    except Exception:
        inner = free
    gt[inner] = 254
    with open(a.out_pgm, "wb") as f:
        f.write(b"P5\n"); f.write(f"{W} {H}\n255\n".encode()); f.write(gt.tobytes())
    print(f"[gt] aligned GT raster -> {a.out_pgm} (occ={int(occ.sum())} free={int((gt==254).sum())})")

    # ---- IoU etc on aligned pair ----
    eo = est < 100; ef = est >= 250; ek = eo | ef
    go = gt < 100; gf = gt >= 250
    tp = int((eo & go).sum()); fp = int((eo & ~go).sum()); fn = int((~eo & go).sum())
    U = int((eo | go).sum())
    iou = tp/U if U else 0
    prec = tp/(tp+fp) if tp+fp else 0
    rec = tp/(tp+fn) if tp+fn else 0
    f1 = 2*prec*rec/(prec+rec) if prec+rec else 0
    known = ek & (go | gf)
    correct = (eo & go) | (ef & gf)
    acc = int((correct & known).sum())/int(known.sum())*100 if known.sum() else 0
    comp = int((go & ek).sum())/int(go.sum())*100 if go.sum() else 0
    print(f"== {a.method} (ALIGNED)")
    print(f"   IoU={iou:.4f}  P={prec:.4f}  R={rec:.4f}  F1={f1:.4f}")
    print(f"   cell_accuracy={acc:.2f}%  completeness={comp:.1f}%")


if __name__ == "__main__":
    main()
