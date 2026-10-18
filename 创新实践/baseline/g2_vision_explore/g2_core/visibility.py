"""Expected unknown area from a safe vantage, occluded by observed obstacles."""
import math
import numpy as np


def visible_unknown_area(grid, candidates, radius_m, rays=144):
    """Count distinct unknown cells sampled along sight lines up to known walls.

    Outside the map stays unknown. This only estimates observation utility; it
    never fills the map or authorizes travel. Ray sampling undercounts gaps
    between rays instead of counting the same unknown cell multiple times.
    """
    result = np.zeros(len(candidates))
    steps = max(1, math.ceil(radius_m / grid.resolution))
    dr = radius_m / steps
    angles = np.arange(rays) * (2 * math.pi / rays)
    dx, dy = np.cos(angles), np.sin(angles)
    pad = steps + 1
    width = grid.width + 2 * pad
    for begin in range(0, len(candidates), 128):
        block = candidates[begin:begin+128]
        alive = np.ones((len(block), rays), dtype=bool)
        rows, cols = block[:, 0, None], block[:, 1, None]
        hits = []
        for step in range(1, steps + 1):
            distance = step * dr
            rr = rows + np.rint(dy * distance / grid.resolution).astype(int)
            cc = cols + np.rint(dx * distance / grid.resolution).astype(int)
            inside = (rr >= 0) & (rr < grid.height) & (cc >= 0) & (cc < grid.width)
            values = grid.data[np.clip(rr, 0, grid.height-1), np.clip(cc, 0, grid.width-1)]
            alive &= ~((values >= 100) & inside)
            unknown = ~inside | (values < 0)
            hits.append(np.where(alive & unknown, (rr+pad)*width+cc+pad, -1))
        indices = np.sort(np.concatenate(hits, axis=1), axis=1)
        unique = np.sum((indices[:, 1:] != indices[:, :-1]) & (indices[:, 1:] >= 0), axis=1)
        unique += indices[:, 0] >= 0
        result[begin:begin+len(block)] = unique * grid.resolution**2
    return result
