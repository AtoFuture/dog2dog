#!/usr/bin/env python3
"""Animate persons / vehicles in warehouse_cross_objects.sdf via kinematic set_pose.

Runs WITHOUT physics dependency: each tick it computes the pose of every mover
along its closed polyline path and calls the gz `set_pose_vector` service once.

Usage (inside the Syslab/Gazebo container, same domain/partition as the sim):
    python3 objects_motion.py --world world_demo --rate 15 --duration 0
"""
import argparse
import math
import subprocess
import sys
import time

# name, waypoints (closed loop), speed m/s, height
MOVERS = [
    ("person_1", [(-2.0, -6.4), (11.5, -6.4)], 0.6, 0.0),
    ("person_2", [(1.2, 1.27), (11.5, 1.27)], 0.6, 0.0),
    ("person_3", [(11.5, 3.86), (1.2, 3.86)], 0.7, 0.0),
    ("suv_1", [(-13.5, -5.0), (-8.5, -5.0), (-8.5, -1.0), (-13.5, -1.0)], 1.5, 0.02),
    ("forklift_1", [(-13.5, 2.0), (-8.5, 2.0), (-8.5, 5.5), (-13.5, 5.5)], 0.9, 0.15),
]


def build_loop(pts):
    segs = []
    n = len(pts)
    for i in range(n):
        a = pts[i]
        b = pts[(i + 1) % n]
        segs.append((a, b, math.hypot(b[0] - a[0], b[1] - a[1])))
    total = sum(s[2] for s in segs) or 1e-6
    return segs, total


def at(s, segs, total):
    s %= total
    for a, b, L in segs:
        if s <= L:
            u = s / L if L > 0 else 0.0
            x = a[0] + (b[0] - a[0]) * u
            y = a[1] + (b[1] - a[1]) * u
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            return x, y, yaw
        s -= L
    a, b, _ = segs[-1]
    return b[0], b[1], math.atan2(b[1] - a[1], b[0] - a[0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="world_demo")
    ap.add_argument("--rate", type=float, default=15.0)
    ap.add_argument("--duration", type=float, default=0.0, help="0 = forever")
    args = ap.parse_args()

    movers = []
    for name, pts, speed, z in MOVERS:
        segs, total = build_loop(pts)
        movers.append([name, segs, total, speed, z, 0.0])

    svc = f"/world/{args.world}/set_pose_vector"
    dt = 1.0 / args.rate
    t0 = time.monotonic()
    n = 0
    print(f"[objects_motion] {len(movers)} movers -> {svc} @ {args.rate}Hz", flush=True)
    while True:
        text = []
        for m in movers:
            name, segs, total, speed, z, s = m
            x, y, yaw = at(s, segs, total)
            m[5] = (s + speed * dt) % total
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            text.append(
                f'pose {{name: "{name}" position {{x: {x:.4f} y: {y:.4f} z: {z:.4f}}} '
                f"orientation {{x: 0 y: 0 z: {qz:.6f} w: {qw:.6f}}}}}"
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
            print(f"[objects_motion] t={time.monotonic()-t0:.1f}s sent={n}", flush=True)
        if args.duration and (time.monotonic() - t0) >= args.duration:
            break
        time.sleep(dt)


if __name__ == "__main__":
    main()
