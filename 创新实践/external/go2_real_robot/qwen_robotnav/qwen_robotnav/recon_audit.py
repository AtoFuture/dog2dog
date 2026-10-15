"""Offline one-to-one matching; never import scene truth into the online policy."""
import math
from functools import lru_cache


def match_targets(estimates, truth, limit_m=.5):
    """Maximum-cardinality, minimum-error assignment with explicit unmatched items.

    Small validation scenes only (up to 12 truth targets). A duplicate estimate
    cannot reuse the same ground-truth person.
    """
    if len(truth)>12:raise ValueError('Scene audit supports at most 12 truth targets')
    if not math.isfinite(limit_m) or limit_m<=0:raise ValueError('Invalid matching limit')
    distances=[[math.dist(e[:2],t[:2]) for t in truth] for e in estimates]

    @lru_cache(None)
    def solve(i, used):
        if i==len(estimates):return (0,0.,())
        count,cost,path=solve(i+1,used)
        best=(count,cost,(None,)+path)
        for j,distance in enumerate(distances[i]):
            if used&(1<<j) or not math.isfinite(distance) or distance>limit_m:continue
            count,cost,path=solve(i+1,used|(1<<j))
            candidate=(count+1,cost+distance,(j,)+path)
            if (-candidate[0],candidate[1])<(-best[0],best[1]):best=candidate
        return best

    return list(solve(0,0)[2])
