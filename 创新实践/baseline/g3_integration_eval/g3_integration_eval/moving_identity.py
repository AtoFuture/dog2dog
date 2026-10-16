"""Conservative, deterministic replay of evidence into persistent moving identities.

No scene coordinates, simulator truth, model names or detector-ID equality are
identity evidence. One capture can update each identity at most once. Ambiguous
associations cannot create extra people. Histograms describe clothing, not a
biometric identity; long absences without distinctive appearance stay unresolved.
"""
from collections import defaultdict
from functools import lru_cache
import math
from statistics import median
from g3_integration_eval.person_appearance import distance, valid_descriptor


class MovingIdentityResolver:
    def __init__(self, *, max_speed_m_s=2., max_gap_s=10., settle_s=.2,
                 ambiguity_margin=.3, min_separation_m=.3):
        values=(max_speed_m_s,max_gap_s,settle_s,ambiguity_margin,min_separation_m)
        if not all(math.isfinite(x) and x>0 for x in values):
            raise ValueError('Identity gates must be finite and positive')
        self.speed,self.gap,self.settle,self.margin,self.separation=values
        self.diagnostics={}
        self._stable_assignments={}
        self._next_identity=1

    def resolve(self, payload, required_classes, *, min_stamp_ns=0, min_confidence=.5):
        ready=set(payload.get('ready_evidence_ids', []))
        frames=defaultdict(list);seen=set();latest=min_stamp_ns;pending_frames=set()
        for raw in payload.get('observations', []):
            try:
                o=dict(raw);stamp=o['stamp_ns'];track=o['track_id'];eid=o['evidence_id']
                if (not isinstance(stamp,int) or isinstance(stamp,bool) or stamp<min_stamp_ns
                    or not isinstance(track,int) or isinstance(track,bool) or track<=0
                    or not isinstance(eid,int) or isinstance(eid,bool)
                    or o['class_name'] not in required_classes or o['source']!='coco' or o['frame_id']!='map'):
                    continue
                latest=max(latest,stamp)
                if o.get('snapshot_status')=='missing':pending_frames.add(stamp)
                o['position_m']=[float(x) for x in o['position_m']]
                o['confidence']=float(o['confidence'])
                if (eid not in ready or eid in seen or o['snapshot_status']!='available'
                    or not o['snapshot_file'] or len(o['position_m'])!=3
                    or not all(math.isfinite(x) for x in o['position_m'])
                    or not math.isfinite(o['confidence']) or not min_confidence<=o['confidence']<=1):
                    continue
                seen.add(eid);frames[stamp].append(o)
            except (KeyError,ValueError,TypeError):
                continue
        identities=[];ambiguous=[];assignments={};capacity_exceeded=False
        for stamp, observations in sorted(frames.items()):
            # The latest image may still be arriving on independent detection / JPEG
            # topics. Wait for the next image-time watermark before assigning it.
            if stamp>latest-int(self.settle*1e9) or stamp in pending_frames:continue
            observations.sort(key=lambda o:o['evidence_id'])
            # Bound exact assignment. Overflow fails closed, never greedy truncation.
            if len(observations)>8 or len(identities)>12:
                capacity_exceeded=True;ambiguous.extend(o['evidence_id'] for o in observations);continue
            blocked=set()
            for i,a in enumerate(observations):
                for j,b in enumerate(observations[:i]):
                    if a['class_name']==b['class_name'] and (a['track_id']==b['track_id']
                        or math.dist(a['position_m'][:2],b['position_m'][:2])<self.separation):
                        blocked.update((i,j))
            # Block uncertain duplicates rather than letting one make a new identity.
            usable=[o for i,o in enumerate(observations) if i not in blocked]
            ambiguous.extend(o['evidence_id'] for i,o in enumerate(observations) if i in blocked)
            costs={};possible={}
            for i,o in enumerate(usable):
                possible[i]=set()
                for j,t in enumerate(identities):
                    if o['class_name']!=t['class_name']:continue
                    dt=(stamp-t['last']['stamp_ns'])*1e-9
                    if dt<=0:continue
                    previous=t['last']['position_m'][:2]
                    displacement=math.dist(previous,o['position_m'][:2])
                    if displacement>self.speed*dt+.35:continue
                    appearance=distance(o.get('appearance'),t.get('appearance'))
                    # Strong colour disagreement blocks association, but cannot prove
                    # a new person: crops can change after occlusion or camera motion.
                    possible[i].add(j)
                    if appearance is not None and appearance>.65:continue
                    if dt>self.gap:continue
                    prediction=[x+v*dt for x,v in zip(previous,t['velocity'])]
                    residual=math.dist(prediction,o['position_m'][:2])
                    gate=.35+.3*dt
                    if residual>gate:continue
                    score=residual/gate + (appearance*1.5 if appearance is not None else .25)
                    if score<1.2:costs[i,j]=score
            def solve(forbidden=None):
                @lru_cache(None)
                def visit(i,used):
                    if i==len(usable):return 0.,()
                    tail,path=visit(i+1,used);best=(1.2+tail,(-1,)+path)
                    for j in range(len(identities)):
                        if used & (1<<j) or (i,j)==forbidden or (i,j) not in costs:continue
                        tail,path=visit(i+1,used | (1<<j));candidate=(costs[i,j]+tail,(j,)+path)
                        if candidate[0]<best[0]:best=candidate
                    return best
                return visit(0,0)
            total,chosen=solve()
            # Prediction certainty decays across missed captures. Compare complete
            # assignments with a wider ambiguity margin after a gap, rather than
            # trusting the nearest old position through an unseen crossing/turn.
            accepted={i:j for i,j in enumerate(chosen) if j>=0 and solve((i,j))[0]-total
                      >self.margin+.12*max(0.,(stamp-identities[j]['last']['stamp_ns'])*1e-9-1.)}
            # Only unambiguous assignments free an old identity from the birth
            # exclusion set. Ambiguous edges must never manufacture extra people.
            reserved=set(accepted.values())
            for i,o in enumerate(usable):
                if i in accepted:
                    t=identities[accepted[i]]
                    # Median pairwise slopes resist a partially occluded ROI's
                    # position jump; a single two-frame derivative can overshoot
                    # the hidden person's path and cause a later identity swap.
                    history=[v for v in t['samples'][-6:] if stamp-v['stamp_ns']<=3_000_000_000]+[o]
                    if len(history)<2:history=[t['last'],o]
                    slopes=[[(b['position_m'][axis]-a['position_m'][axis])/((b['stamp_ns']-a['stamp_ns'])*1e-9)
                             for axis in (0,1)] for i,a in enumerate(history) for b in history[i+1:]
                            if b['stamp_ns']-a['stamp_ns']>=100_000_000]
                    if slopes:
                        measured=[median(v[axis] for v in slopes) for axis in (0,1)]
                        speed=math.hypot(*measured)
                        if speed>self.speed:measured=[v*self.speed/speed for v in measured]
                        t['velocity']=measured
                elif chosen[i]>=0 or possible[i]-reserved:
                    ambiguous.append(o['evidence_id']);continue
                else:
                    # A detector-ID teleport may be localization noise or ID reuse.
                    # Unless its old identities are concurrently accounted for,
                    # it is insufficient evidence of an additional person.
                    aliases={j for j,t in enumerate(identities) if o['track_id'] in t['track_ids']}
                    if aliases-reserved:
                        ambiguous.append(o['evidence_id']);continue
                    if len(identities)>=12:
                        capacity_exceeded=True;ambiguous.append(o['evidence_id']);continue
                    t=dict(identity_id=len(identities)+1,class_name=o['class_name'],
                           samples=[],velocity=[0.,0.],track_ids=[],appearance=None)
                    identities.append(t)
                t['samples'].append(o);t['last']=o
                if o['track_id'] not in t['track_ids']:t['track_ids'].append(o['track_id'])
                if valid_descriptor(o.get('appearance')):t['appearance']=o['appearance']
                assignments[o['evidence_id']]=t['identity_id']
        # Replay can gain an older JPEG later. Preserve published identity IDs
        # by evidence overlap instead of numbering by the current birth order.
        # Splits/merges conflicting with already published identities fail closed.
        previous_ids={t['identity_id']:{self._stable_assignments[o['evidence_id']]
                      for o in t['samples'] if o['evidence_id'] in self._stable_assignments} for t in identities}
        owners=defaultdict(set)
        for local_id,old_ids in previous_ids.items():
            for old_id in old_ids:owners[old_id].add(local_id)
        stable={};conflicts=set()
        for t in identities:
            local=t['identity_id'];old=previous_ids[local]
            if len(old)>1 or any(len(owners[x])>1 for x in old):
                conflicts.add(local);ambiguous.extend(o['evidence_id'] for o in t['samples']);continue
            if old:stable[local]=next(iter(old))
            else:
                stable[local]=self._next_identity;self._next_identity+=1
            for o in t['samples']:self._stable_assignments[o['evidence_id']]=stable[local]
        assignments={eid:stable[identity] for eid,identity in assignments.items() if identity in stable}
        targets={}
        for t in identities:
            if t['identity_id'] in conflicts:continue
            # Two unambiguous captures, close enough in time to support continuity.
            pair=next(((a,b) for a,b in zip(t['samples'],t['samples'][1:])
                       if 0<(b['stamp_ns']-a['stamp_ns'])*1e-9<=self.gap),None)
            if pair is None:continue
            name=t['class_name'];count=1+sum(x['class_name']==name for x in targets.values())
            targets[name if count==1 else f'{name}:{count}']=dict(
                class_name=name,identity_id=stable[t['identity_id']],track_id=pair[0]['track_id'],
                track_ids=t['track_ids'],evidence_ids=[o['evidence_id'] for o in pair],observations=list(pair),
                position_m=t['last']['position_m'],position_stamp_ns=t['last']['stamp_ns'],
                velocity_xy_m_s=t['velocity'],last_seen_stamp_ns=t['last']['stamp_ns'],
                associated_evidence_ids=[o['evidence_id'] for o in t['samples']])
        self.diagnostics=dict(policy='motion_appearance_v1',ambiguous_evidence_ids=sorted(set(ambiguous)),
            assignments=assignments,identity_conflict_count=len(conflicts),settled_through_stamp_ns=latest-int(self.settle*1e9),
            tentative_identities=sum(len(t['samples'])<2 for t in identities),capacity_exceeded=capacity_exceeded)
        return targets
