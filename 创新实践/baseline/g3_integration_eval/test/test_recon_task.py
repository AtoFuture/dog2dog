from g3_integration_eval.recon_task import confirmed_targets


def observation(evidence_id=1, stamp=1_000_000_000):
    return dict(evidence_id=evidence_id,stamp_ns=stamp,track_id=1,class_name='person',
                confidence=.9,source='coco',frame_id='map',position_m=[2.,0.,.8],
                snapshot_status='available',snapshot_file='image.jpg')


def payload():
    return dict(ready_evidence_ids=[1,2],observations=[observation(),observation(2,3_000_000_000)])


def test_confirms_only_separate_consistent_jpeg_observations():
    result=confirmed_targets(payload(),['person'])
    assert result['person']['evidence_ids']==[1,2]
    value=payload();value['observations'][1]['stamp_ns']=1_000_000_000
    assert not confirmed_targets(value,['person'])


def test_rejects_missing_snapshot_wrong_track_old_or_inconsistent_pose():
    for field,value in [('snapshot_status','missing'),('track_id',2),('position_m',[8.,0.,.8]),('confidence',.1),('confidence',float('nan')),('source','pose')]:
        data=payload();data['observations'][1][field]=value
        assert not confirmed_targets(data,['person'])
    assert not confirmed_targets(payload(),['person'],min_stamp_ns=2_000_000_000)
    assert not confirmed_targets(payload(),['car'])
