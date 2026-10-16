"""Adversarial moving-person sequences; truth labels never reach the resolver."""
import copy
import random
import pytest
from g3_integration_eval.moving_identity import MovingIdentityResolver
from g3_integration_eval.person_appearance import SCHEMA, DIMENSIONS


def colour(index):
    vector=[0.]*DIMENSIONS;vector[index]=1.
    return dict(schema=SCHEMA,vector=vector)


def capture(eid,stamp,x,y=0.,track=1,appearance=None):
    return dict(evidence_id=eid,stamp_ns=int(stamp*1e9),position_m=[x,y,.8],
        track_id=track,class_name='person',source='coco',confidence=.9,frame_id='map',
        snapshot_status='available',snapshot_file=f'{eid}.jpg',appearance=appearance)


def resolve(observations, **kwargs):
    data=dict(observations=observations,ready_evidence_ids=[o['evidence_id'] for o in observations])
    resolver=MovingIdentityResolver(**kwargs)
    targets=resolver.resolve(data,['person'])
    return targets,resolver.diagnostics


def test_moving_person_over_many_static_regions_and_changed_detector_ids_counts_once():
    observations=[capture(i+1,i,.5*i,track=i//3+1,appearance=colour(0)) for i in range(12)]
    targets,diagnostics=resolve(observations)
    assert len(targets)==1
    assert targets['person']['track_ids']==[1,2,3,4]
    assert targets['person']['position_m'][0]==5.
    assert len(set(diagnostics['assignments'].values()))==1


def test_crossing_people_with_detector_id_swap_preserves_clothing_identities():
    observations=[];truth={}
    for frame in range(9):
        for person in range(2):
            eid=len(observations)+1
            x=(frame*.35 if person==0 else 2.8-frame*.35)
            o=capture(eid,frame,x,person*.4,track=(person+1 if frame<4 else 2-person),appearance=colour(person))
            observations.append(o);truth[eid]=person
    targets,diagnostics=resolve(observations)
    assert len(targets)==2
    identities={}
    for eid,identity in diagnostics['assignments'].items():
        identities.setdefault(identity,set()).add(truth[eid])
    assert all(len(labels)==1 for labels in identities.values())
    assert all(t['track_ids']==[i,3-i] for i,t in enumerate(targets.values(),1))


def test_near_parallel_distinct_people_are_not_merged_by_static_radius():
    observations=[]
    for frame in range(8):
        observations.extend(capture(2*frame+p+1,frame,frame*.3,p*.45,track=p+1,appearance=colour(p)) for p in range(2))
    targets,_=resolve(observations)
    assert len(targets)==2


def test_short_occlusion_and_new_track_id_associate_with_prediction():
    observations=[capture(i+1,t,.4*t,track=1 if t<3 else 9,appearance=colour(0)) for i,t in enumerate([0,1,2,5,6,7])]
    targets,diag=resolve(observations)
    assert len(targets)==1 and targets['person']['track_ids']==[1,9]
    assert not diag['ambiguous_evidence_ids']


def test_long_absence_or_impossible_prediction_never_manufactures_duplicate():
    for times in ([0,1,25,26,27],[0,1,4,5,6]):
        observations=[capture(i+1,t,(0 if t<2 else 3.),track=(1 if t<2 else 8)) for i,t in enumerate(times)]
        targets,diag=resolve(observations)
        assert len(targets)==1
        assert diag['ambiguous_evidence_ids']


def test_same_appearance_crossing_is_unresolved_without_extra_births():
    observations=[]
    for frame in range(7):
        for person in range(2):
            x=frame*.3 if person==0 else 1.8-frame*.3
            observations.append(capture(len(observations)+1,frame,x,person*.05,track=person+1))
    targets,diag=resolve(observations)
    assert len(targets)<=2 and len(set(diag['assignments'].values()))<=2
    assert diag['ambiguous_evidence_ids']


def test_out_of_order_snapshots_and_payload_order_replay_identically():
    observations=[capture(i+1,i,i*.4,appearance=colour(0)) for i in range(6)]
    expected=resolve(observations)
    random.Random(7).shuffle(observations)
    assert resolve(observations)==expected
    resolver=MovingIdentityResolver()
    data=dict(observations=observations,ready_evidence_ids=[1,3])
    resolver.resolve(data,['person'])
    data['ready_evidence_ids']=[o['evidence_id'] for o in observations]
    assert resolver.resolve(data,['person'])==expected[0]


def test_latest_capture_waits_for_watermark_and_distinct_images_required():
    assert not resolve([capture(1,1,0),capture(2,1,1,track=2)])[0]
    assert not resolve([capture(1,1,0),capture(2,2,.2)])[0]
    assert len(resolve([capture(1,1,0),capture(2,2,.2),capture(3,3,.4)])[0])==1


def test_same_frame_duplicate_and_id_zero_do_not_confirm():
    observations=[]
    for t in range(4):
        observations.extend([capture(3*t+1,t,0),capture(3*t+2,t,.05,track=2),capture(3*t+3,t,1,track=0)])
    assert not resolve(observations)[0]


def test_reused_detector_id_teleport_is_unresolved_rather_than_extra_person():
    observations=[capture(1,0,0),capture(2,1,0),capture(3,2,20),capture(4,3,20),capture(5,4,20)]
    targets,diag=resolve(observations)
    assert len(targets)==1 and diag['ambiguous_evidence_ids']
    for o in observations[2:]:o['track_id']=2
    assert len(resolve(observations)[0])==2


def test_capacity_overflow_is_explicit_and_fails_closed():
    observations=[capture(t*9+p+1,t,p,track=p+1) for t in range(3) for p in range(9)]
    targets,diag=resolve(observations)
    assert not targets and diag['capacity_exceeded']


@pytest.mark.parametrize('field,value',[('confidence',float('nan')),('position_m',[float('inf'),0,0]),('source','pose'),('snapshot_status','missing'),('track_id',0),('stamp_ns','1')])
def test_invalid_evidence_cannot_confirm(field,value):
    observations=[capture(1,0,0),capture(2,1,.1),capture(3,2,.2)]
    observations[1][field]=value
    assert not resolve(observations)[0]


def test_malformed_appearance_falls_back_to_motion_without_crashing():
    observations=[capture(i+1,i,i*.2,appearance={'schema':SCHEMA,'vector':[float('nan')]}) for i in range(4)]
    assert len(resolve(observations)[0])==1


def test_changed_clothing_descriptor_after_occlusion_does_not_create_extra_person():
    observations=[capture(i+1,i,i*.2,track=(1 if i<3 else 9),appearance=colour(0 if i<3 else 1)) for i in range(6)]
    targets,diag=resolve(observations)
    assert len(targets)==1 and diag['ambiguous_evidence_ids']


def test_known_pending_snapshot_blocks_entire_capture_until_it_arrives():
    observations=[capture(1,0,0),capture(2,0,1.8,track=2),capture(3,1,0),capture(4,1,1.8,track=2),capture(5,2,0)]
    observations[3]['snapshot_status']='missing'
    targets,_=resolve(observations)
    assert not targets
    observations[3]['snapshot_status']='available'
    assert len(resolve(observations)[0])==2


def test_unseen_crossing_after_stop_is_ambiguous_instead_of_identity_swap():
    observations=[]
    for frame in range(10):
        # Two indistinguishable people stop, disappear during an unseen crossing,
        # and reappear near the other's old position. Detector IDs also swap.
        t=frame if frame<3 else frame+3
        for person in range(2):
            x=person*.6
            y=person*1.4 if frame<3 else (1-person)*1.4
            observations.append(capture(len(observations)+1,t,x,y,track=person+1 if frame<3 else 2-person))
    targets,diag=resolve(observations)
    assert len(targets)==2 and diag['ambiguous_evidence_ids']
    truth={o['evidence_id']:(o['position_m'][0]>.3) for o in observations}
    for target in targets.values():
        assert len({truth[eid] for eid in target['associated_evidence_ids']})==1


def test_partial_roi_position_outlier_does_not_corrupt_velocity_prediction():
    observations=[capture(i+1,i*.2,0,-i*.04,appearance=colour(0)) for i in range(8)]
    observations[-1]['position_m'][1]-=.15
    observations.extend(capture(9+i,3.+i*.2,0,-(.6+i*.04),track=8,appearance=colour(0)) for i in range(4))
    targets,_=resolve(observations)
    assert len(targets)==1 and targets['person']['track_ids']==[1,8]
    assert targets['person']['velocity_xy_m_s'][1]==pytest.approx(-.2,abs=.05)


def test_late_earlier_person_does_not_renumber_an_already_published_identity():
    resolver=MovingIdentityResolver()
    first=[capture(i+1,i+2,10.,track=1) for i in range(4)]
    data=dict(observations=first,ready_evidence_ids=[o['evidence_id'] for o in first])
    before=resolver.resolve(data,['person'])['person']['identity_id']
    late=[capture(100+i,i,0.,track=2) for i in range(3)]
    data['observations']+=late;data['ready_evidence_ids']+=[o['evidence_id'] for o in late]
    after=resolver.resolve(data,['person'])
    assert len(after)==2
    assert next(t for t in after.values() if 1 in t['track_ids'])['identity_id']==before
    assert len({t['identity_id'] for t in after.values()})==2
