"""Evidence-based find-and-return objective; no ground truth or LLM coordinates."""
from collections import defaultdict
import math


def confirmed_instances(payload, required_classes, *, min_stamp_ns=0, min_confidence=.5,
                      max_separation_m=.75, max_interval_s=10., identity_radius_m=.8):
    """Require two spatially consistent, separately captured, tracked JPEG observations."""
    groups = defaultdict(list)
    ready = set(payload.get('ready_evidence_ids', []))
    for value in payload.get('observations', []):
        try:
            value=dict(value)
            position = [float(v) for v in value['position_m']]
            value['position_m']=position
            value['confidence']=float(value['confidence'])
            if (value['class_name'] not in required_classes or value['source'] != 'coco'
                or value['frame_id'] != 'map' or value['track_id'] <= 0
                or value['stamp_ns'] < min_stamp_ns or not math.isfinite(value['confidence'])
                or value['confidence'] < min_confidence
                or value['snapshot_status'] != 'available' or value['evidence_id'] not in ready
                or not value['snapshot_file'] or len(position) != 3
                or not all(math.isfinite(float(v)) for v in position)):
                continue
            groups[(value['class_name'],value['track_id'])].append(value)
        except (KeyError,TypeError,ValueError):
            continue
    candidates = []
    for (class_name, track), observations in groups.items():
        ordered = sorted(observations,key=lambda item:item['stamp_ns'])
        for i, first in enumerate(ordered):
            for second in ordered[i+1:]:
                dt = (second['stamp_ns']-first['stamp_ns'])*1e-9
                if dt > max_interval_s:
                    break
                if dt <= 0 or first['evidence_id']==second['evidence_id']:
                    continue
                if math.dist(first['position_m'],second['position_m']) > max_separation_m:
                    continue
                candidates.append(dict(class_name=class_name,track_id=track,
                    evidence_ids=[first['evidence_id'],second['evidence_id']],
                    observations=[first,second],position_m=[(a+b)/2 for a,b in zip(first['position_m'],second['position_m'])]))
                break
            if candidates and candidates[-1]['track_id']==track and candidates[-1]['class_name']==class_name:
                break
    # Stationary-target policy: track fragmentation near an already confirmed
    # location must not manufacture another person. XY avoids camera-height noise.
    # Conservative: people closer than this radius remain unresolved, not counted twice.
    targets = {}
    for candidate in sorted(candidates, key=lambda c:(c['observations'][0]['stamp_ns'], c['track_id'])):
        nearby = [t for t in targets.values() if t['class_name']==candidate['class_name']
                  and math.dist(t['position_m'][:2],candidate['position_m'][:2])<=identity_radius_m]
        if nearby:
            for target in nearby:
                if candidate['track_id'] not in target['track_ids']:
                    target['track_ids'].append(candidate['track_id'])
            continue
        candidate['track_ids']=[candidate['track_id']]
        name=candidate['class_name']
        count=1+sum(t['class_name']==name for t in targets.values())
        targets[name if count==1 else f'{name}:{count}']=candidate
    return targets


def confirmed_targets(payload, required_classes, **kwargs):
    """Compatibility view: the first confirmed instance of each requested class."""
    targets={}
    for value in confirmed_instances(payload,required_classes,**kwargs).values():
        targets.setdefault(value['class_name'],value)
    return targets
