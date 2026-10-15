import pytest
from g3_integration_eval.detection_evidence import DetectionEvidenceStore

JPEG_A = b'\xff\xd8image-a\xff\xd9'
JPEG_B = b'\xff\xd8image-b\xff\xd9'


def observation(track=1, source='coco'):
    return dict(track_id=track, stamp_ns=100, source=source, position_m=[1,2,3],
                confidence=.9, class_name='person', has_snapshot=True)


def test_out_of_order_snapshot_survives_restart_and_keeps_distinct_sources(tmp_path):
    store = DetectionEvidenceStore(tmp_path)
    store.add_snapshot(1,100,JPEG_A)
    first = store.add_detection(observation())
    assert store.add_detection(observation()) == first
    second = store.add_detection(observation(source='pose'))
    store.close()
    store = DetectionEvidenceStore(tmp_path)
    result = store.export()
    assert result['ready_evidence_ids'] == [first,second]
    assert len(list(tmp_path.glob('*.jpg'))) == 1
    store.close()


def test_untracked_objects_and_ambiguous_snapshots_are_not_merged(tmp_path):
    store = DetectionEvidenceStore(tmp_path)
    a = store.add_detection(observation(track=0))
    store.add_snapshot(0,100,JPEG_A)
    assert not store.export()['ready_evidence_ids']
    b = store.add_detection(observation(track=0))
    store.add_snapshot(0,100,JPEG_A)
    store.add_snapshot(0,100,JPEG_B)
    result = store.export()
    assert a != b and not result['ready_evidence_ids']
    assert all(o['snapshot_status']=='ambiguous' for o in result['observations'])
    store.close()


def test_no_snapshot_means_no_ready_visual_evidence(tmp_path):
    store = DetectionEvidenceStore(tmp_path)
    store.add_detection(observation())
    assert not store.export()['ready_evidence_ids']
    with pytest.raises(ValueError):
        store.add_detection(dict(observation(), position_m=[float('nan'),0,0]))
    with pytest.raises(ValueError):
        store.add_snapshot(1,100,b'not-a-jpeg')
    store.close()


def test_executor_threads_can_record_and_export_safely(tmp_path):
    from concurrent.futures import ThreadPoolExecutor
    store = DetectionEvidenceStore(tmp_path)
    def record(track):
        store.add_detection(observation(track))
        store.add_snapshot(track,100,JPEG_A)
        store.export()
    with ThreadPoolExecutor(max_workers=4) as executor:
        list(executor.map(record, range(1,11)))
    assert len(store.export()['ready_evidence_ids']) == 10
    store.close()
