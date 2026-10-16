"""Evaluation-only scripted trajectories; never imported by navigation or identity."""
import math

SCENARIOS=('moving_crossing','moving_parallel','moving_occlusion')
DURATION_S=28.


def positions(scenario, elapsed):
    if scenario not in SCENARIOS or not math.isfinite(elapsed):
        raise ValueError('Invalid motion scenario/time')
    # Four seconds to establish two identities, ten to traverse, four holding,
    # then ten to return. Crossing paths are separated in depth by 0.6 m.
    t=max(0.,min(DURATION_S,elapsed))
    phase=min(1.,max(0.,(t-4.)/10.)) if t<18 else max(0.,1.-(t-18.)/10.)
    if scenario=='moving_parallel':
        # 0.3 m depth offset and 0.7 m lateral separation: centres stay
        # inside the old 0.8 m merge radius while both silhouettes can separate.
        return [(10.3,5.25+.8*phase),(10.6,5.95+.8*phase)]
    return [(10.2,5.25+1.8*phase),(10.8,7.05-1.8*phase)]
