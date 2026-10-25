#!/usr/bin/env python3
"""align_iou2.py — 正向验证：est 占据点 --(R,t)--> world 栅格，与 SDF(world) 比 IoU。
若此方向 IoU 高 -> 之前的反变换有符号/方向错；若仍低 -> 地图确实与真值不符。
"""
import argparse, os, math, re
import numpy as np
import xml.etree.ElementTree as ET
import yaml


def read_pgm(path):
    d = open(path, "rb").read(); i = 0; t = []
    while len(t) < 4:
        while d[i:i+1].isspace(): i += 1
        if d[i:i+1] == b"#":
            while d[i:i+1] != b"\n": i += 1
            continue
        s = i
        while not d[i:i+1].isspace(): i += 1
        t.append(d[s:i])
    W, H = int(t[1]), int(t[2]); i += 1
    return np.frombuffer(d[i:i+W*H], np.uint8).reshape(H, W)


def pose6(e):
    p = e.find("pose")
    if p is None or not p.text: return [0,0,0,0,0,0]
    v = [float(x) for x in p.text.split()]
    return v + [0,0,0] if len(v)==3 else v


def se2(x,y,yaw):
    c,s = math.cos(yaw), math.sin(yaw)
    return np.array([[c,-s,x],[s,c,y],[0,0,1]])


def sdf_walls(sdf, exclude, min_h=0.2):
    imp = re.compile(exclude, re.I)
    root = ET.parse(sdf).getroot()
    world = root.find("world") if root.tag=="sdf" else root
    out=[]
    for m in world.findall("model"):
        if imp.search(m.get("name","")): continue
        if m.findtext("static","false").strip().lower() not in ("1","true"): continue
        mp=pose6(m); Tm=se2(mp[0],mp[1],mp[5])
        for lk in m.findall("link"):
            lp=pose6(lk); Tl=Tm@se2(lp[0],lp[1],lp[5])
            for col in lk.findall("collision"):
                g=col.find("geometry")
                if g is None or g.find("box") is None: continue
                sx,sy,sz=[float(v) for v in g.find("box").find("size").text.split()]
                if sz<min_h: continue
                cp=pose6(col); Tc=Tl@se2(cp[0],cp[1],cp[5])
                out.append((Tc[0,2],Tc[1,2],math.atan2(Tc[1,0],Tc[0,0]),sx,sy))
    return out


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--yaw_deg",type=float,required=True)
    ap.add_argument("--tx",type=float,required=True)
    ap.add_argument("--ty",type=float,required=True)
    ap.add_argument("--sdf",required=True)
    ap.add_argument("--est",required=True)
    ap.add_argument("--est-yaml",required=True)
    ap.add_argument("--exclude",default=r"ground|floor|terrain|plane")
    a=ap.parse_args()

    y=yaml.safe_load(open(a.est_yaml))
    res=float(y["resolution"]); ox,oy,oyaw=[float(v) for v in y["origin"]]
    est=read_pgm(a.est); H,W=est.shape
    eo=est<100
    iy,ix=np.argwhere(eo).T
    mx=ox+(ix+0.5)*res
    my=oy+(H-1-iy+0.5)*res
    if abs(oyaw)>1e-9:
        lx,ly=mx-ox,my-oy; c,s=math.cos(-oyaw),math.sin(-oyaw)
        MX=ox+c*lx-s*ly; MY=oy+s*lx+c*ly
    else:
        MX,MY=mx,my
    phi=math.radians(a.yaw_deg); c,s=math.cos(phi),math.sin(phi)
    WX=c*MX-s*MY+a.tx
    WY=s*MX+c*MY+a.ty

    walls=sdf_walls(a.sdf,a.exclude)
    xmin=min(WX.min(), min(w[0]-w[3] for w in walls))-1.0
    xmax=max(WX.max(), max(w[0]+w[3] for w in walls))+1.0
    ymin=min(WY.min(), min(w[1]-w[4] for w in walls))-1.0
    ymax=max(WY.max(), max(w[1]+w[4] for w in walls))+1.0
    GW=int((xmax-xmin)/res)+1; GH=int((ymax-ymin)/res)+1
    print(f"[grid] world {GW}x{GH} origin=({xmin:.2f},{ymin:.2f}) est_pts_world_bounds "
          f"x[{WX.min():.1f},{WX.max():.1f}] y[{WY.min():.1f},{WY.max():.1f}]")
    gx=np.arange(GW); gy=np.arange(GH)
    CX=xmin+(gx+0.5)*res
    CY=ymin+(GH-1-gy+0.5)*res
    GX,GY=np.meshgrid(CX,CY)

    occ=np.zeros((GH,GW),bool)
    for (cx,cy,cyaw,sx,sy) in walls:
        dx,dy=GX-cx,GY-cy; c,s=math.cos(-cyaw),math.sin(-cyaw)
        lx=c*dx-s*dy; ly=s*dx+c*dy
        occ|=(np.abs(lx)<=sx/2)&(np.abs(ly)<=sy/2)

    E=np.zeros((GH,GW),bool)
    pi=np.clip(((WX-xmin)/res).astype(int),0,GW-1)
    pj=np.clip(((ymax-WY)/res).astype(int),0,GH-1)
    E[pj,pi]=True

    tp=int((E&occ).sum()); fp=int((E&~occ).sum()); fn=int((~E&occ).sum())
    U=int((E|occ).sum()); iou=tp/U if U else 0
    prec=tp/(tp+fp) if tp+fp else 0; rec=tp/(tp+fn) if tp+fn else 0
    print(f"== forward-align IoU={iou:.4f} P={prec:.4f} R={rec:.4f} "
          f"(est_occ_world={int(E.sum())} sdf_occ={int(occ.sum())} overlap={tp})")


if __name__=="__main__":
    main()
