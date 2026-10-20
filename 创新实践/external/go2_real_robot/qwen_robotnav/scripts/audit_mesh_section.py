#!/usr/bin/env python3
"""Read-only visible-mesh/scan correspondence audit; no ROS dependency."""
import argparse
import hashlib
import json
from pathlib import Path

from qwen_robotnav.mesh_section import mesh_section, nearest_section


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mesh', type=Path, required=True)
    parser.add_argument('--height', type=float, required=True)
    parser.add_argument('--point', type=float, nargs=2, required=True)
    parser.add_argument('--scale', type=float, default=1.)
    parser.add_argument('--translation', type=float, nargs=3, default=(0., 0., 0.))
    parser.add_argument('--node')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    segments = mesh_section(args.mesh, args.height, scale=args.scale,
                            translation=args.translation, node=args.node)
    result = dict(method='Offline horizontal visible-mesh correspondence only; not runtime control or a traversability certificate.',
                  mesh=str(args.mesh.resolve()), mesh_sha256=hashlib.sha256(args.mesh.read_bytes()).hexdigest(),
                  height_m=args.height, scale=args.scale, translation=args.translation, node=args.node,
                  query_xy=args.point, nearest=nearest_section(segments, args.point), segments=segments.tolist())
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'segments'}, indent=2))


if __name__ == '__main__':
    main()
