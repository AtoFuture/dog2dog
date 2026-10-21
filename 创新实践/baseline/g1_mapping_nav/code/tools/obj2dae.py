#!/usr/bin/env python3
"""Minimal OBJ -> COLLADA(.dae) converter (triangles, no skeleton).
Enough to use an .obj mesh as a gz-sim <actor> skin (physics-free moving visual)."""
import sys, os


def parse_obj(path):
    verts, norms, faces = [], [], []
    cur = "mat0"
    mats = set()
    for line in open(path, encoding="utf-8", errors="ignore"):
        t = line.split()
        if not t:
            continue
        if t[0] == "v" and len(t) >= 4:
            verts.append((float(t[1]), float(t[2]), float(t[3])))
        elif t[0] == "vn" and len(t) >= 4:
            norms.append((float(t[1]), float(t[2]), float(t[3])))
        elif t[0] == "usemtl":
            cur = t[1]
            mats.add(cur)
        elif t[0] == "f":
            idx = []
            for tok in t[1:]:
                p = tok.split("/")
                vi = int(p[0]) - 1
                ni = int(p[2]) - 1 if len(p) > 2 and p[2] else -1
                idx.append((vi, ni))
            for k in range(1, len(idx) - 1):
                faces.append((idx[0], idx[k], idx[k + 1]))
    return verts, norms, faces


def write_dae(verts, norms, faces, out):
    src_v = "".join(f"{x:.6f} {y:.6f} {z:.6f} " for (x, y, z) in verts)
    src_n = "".join(f"{x:.6f} {y:.6f} {z:.6f} " for (x, y, z) in norms) if norms else "0 0 1 "
    idx = []
    for a, b, c in faces:
        for (vi, ni) in (a, b, c):
            idx.append(f"{vi} {ni if ni >= 0 else 0}")
    idx = " ".join(idx)
    dae = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset>
  <library_effects><effect id="eff0"><profile_COMMON><technique sid="common">
    <phong><diffuse><color>0.75 0.76 0.78 1</color></diffuse>
    <specular><color>0.3 0.3 0.3 1</color></specular></phong>
  </technique></profile_COMMON></effect></library_effects>
  <library_materials><material id="mat0" name="mat0"><instance_effect url="#eff0"/></material></library_materials>
  <library_geometries><geometry id="geo0" name="geo0"><mesh>
    <source id="pos"><float_array id="pos-a" count="{len(verts)*3}">{src_v}</float_array>
      <technique_common><accessor source="#pos-a" count="{len(verts)}" stride="3">
        <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
      </accessor></technique_common></source>
    <source id="nor"><float_array id="nor-a" count="{len(norms)*3 if norms else 3}">{src_n}</float_array>
      <technique_common><accessor source="#nor-a" count="{len(norms) if norms else 1}" stride="3">
        <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
      </accessor></technique_common></source>
    <vertices id="vtx"><input semantic="POSITION" source="#pos"/></vertices>
    <triangles count="{len(faces)}" material="mat0">
      <input semantic="VERTEX" source="#vtx" offset="0"/>
      <input semantic="NORMAL" source="#nor" offset="1"/>
      <p>{idx}</p>
    </triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="scene">
    <node id="n0" name="mesh"><instance_geometry url="#geo0">
      <bind_material><technique_common>
        <instance_material symbol="mat0" target="#mat0"/>
      </technique_common></bind_material>
    </instance_geometry></node>
  </visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
'''
    open(out, "w", encoding="utf-8").write(dae)
    print("wrote", out, len(verts), "verts", len(faces), "tris")


def rotz(verts, norms, deg):
    import math
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    r = lambda p: (p[0]*c - p[1]*s, p[0]*s + p[1]*c, p[2])
    return [r(p) for p in verts], [r(p) for p in norms]


def scalev(verts, norms, k):
    return [tuple(x * k for x in p) for p in verts], [tuple(x * k for x in p) for p in norms]


if __name__ == "__main__":
    v, n, f = parse_obj(sys.argv[1])
    deg = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    scl = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
    if deg:
        v, n = rotz(v, n, deg)
    if scl != 1.0:
        v, n = scalev(v, n, scl)
    write_dae(v, n, f, sys.argv[2])
