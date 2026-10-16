#!/usr/bin/env python3
"""Animate persons / vehicles in warehouse_cross_objects.sdf via kinematic set_pose.

Each tick: compute the pose of every mover along its closed polyline path and call the
gz `set_pose_vector` service once. Kinematic (no physics) -> stable, but the movers do
NOT react to collisions, so the paths must stay in free space.

Usage (inside the container, same IGN_PARTITION as the sim):
    python3 objects_motion.py --world world_demo --rate 15
"""
import argparse
import math
import subprocess
import sys
import time

# name, waypoints (closed loop), speed m/s, height, yaw_offset rad
#   yaw_offset aligns the model's long axis with travel direction
#   (forklift body long axis is +Y at yaw 0 -> +90deg to drive along X)
MOVERS = [
    ("person_1", [(1.0, -1.15), (11.0, -1.15)], 0.6, 0.0, 0.0),
    ("person_2", [(1.0, 1.25), (11.0, 1.25)], 0.6, 0.0, 0.0),
    ("person_3", [(11.0, 3.85), (1.0, 3.85)], 0.7, 0.0, 0.0),
    ("suv_1", [(-6.5, -6.5), (1.5, -6.5)], 1.2, 0.05, 0.0),
    ("suv_2", [(12.0, -6.5), (4.5, -6.5)], 1.4, 0.05, 0.0),
]


def build_loop(pts):
    segs = []
    n = len(pts)
    for i in range(n):
        a = pts[i]
        b = pts[(i + 1) % n]
        segs.append((a, b, math.hypot(b[0] - a[0], b[1] - a[1])))
    return segs, sum(s[2] for s in segs) or 1e-6


def at(s, segs, total):
    s %= total
    for a, b, L in segs:
        if s <= L:
            u = s / L if L > 0 else 0.0
            return a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, math.atan2(b[1] - a[1], b[0] - a[0])
        s -= L
    a, b, _ = segs[-1]
    return b[0], b[1], math.atan2(b[1] - a[1], b[0] - a[0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="world_demo")
    ap.add_argument("--rate", type=float, default=15.0)
    ap.add_argument("--duration", type=float, default=0.0)
    args = ap.parse_args()

    movers = []
    for name, pts, speed, z, yaw_off in MOVERS:
        segs, total = build_loop(pts)
        movers.append([name, segs, total, speed, z, yaw_off, 0.0])

    svc = f"/world/{args.world}/set_pose_vector"
    dt = 1.0 / args.rate
    n = 0
    print(f"[objects_motion] {len(movers)} movers -> {svc} @ {args.rate}Hz", flush=True)
    while True:
        text = []
        for m in movers:
            name, segs, total, speed, z, yaw_off, s = m
            x, y, heading = at(s, segs, total)
            m[6] = (s + speed * dt) % total
            yaw = heading + yaw_off
            text.append(
                f'pose {{name: "{name}" position {{x: {x:.4f} y: {y:.4f} z: {z:.4f}}} '
                f"orientation {{x: 0 y: 0 z: {math.sin(yaw/2):.6f} w: {math.cos(yaw/2):.6f}}}}}"
            )
        try:
            subprocess.run(
                ["ign", "service", "-s", svc, "--reqtype", "ignition.msgs.Pose_V",
                 "--reptype", "ignition.msgs.Boolean", "--timeout", "1500",
                 "--req", " ".join(text)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3)
        except Exception as e:
            print("[objects_motion] service err", e, file=sys.stderr)
        n += 1
        if n % int(args.rate * 10) == 0:
            print(f"[objects_motion] sent={n}", flush=True)
        time.sleep(dt)


if __name__ == "__main__":
    main()
