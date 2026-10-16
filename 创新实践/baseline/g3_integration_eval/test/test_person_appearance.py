import math
import numpy as np
import pytest
from g3_integration_eval.person_appearance import describe, distance, valid_descriptor
from g3_integration_eval.detection_evidence import DetectionEvidenceStore


def test_clothing_descriptor_matches_resizing_but_separates_different_colours():
    red=np.full((80,40,3),(10,10,220),dtype=np.uint8)
    blue=np.full((60,30,3),(220,10,10),dtype=np.uint8)
    a=describe(red);b=describe(blue)
    assert valid_descriptor(a)
    assert distance(a,describe(red[::2,::2]))==pytest.approx(0)
    assert distance(a,b)>.65
    assert distance(a,{'schema':'unknown','vector':[]}) is None
    assert describe(np.zeros((2,2,3),dtype=np.uint8)) is None


def test_descriptor_persists_with_unique_snapshot_and_ambiguity_never_leaks_it(tmp_path):
    JPEG_A=b"\xff\xd8image-a\xff\xd9";JPEG_B=b"\xff\xd8image-b\xff\xd9"
    def observation():return dict(track_id=1,stamp_ns=100,source="coco",position_m=[1,2,3],confidence=.9,class_name="person",has_snapshot=True)
    feature=describe(np.full((30,20,3),127,dtype=np.uint8))
    store=DetectionEvidenceStore(tmp_path)
    store.add_detection(observation())
    store.add_snapshot(1,100,JPEG_A,appearance=feature)
    store.close();store=DetectionEvidenceStore(tmp_path)
    assert store.export()['observations'][0]['appearance']==feature
    store.add_snapshot(1,100,JPEG_B,appearance=feature)
    assert 'appearance' not in store.export()['observations'][0]
    with pytest.raises(ValueError):store.add_snapshot(1,100,JPEG_A,appearance={'vector':[math.nan]})
    store.close()


def test_old_evidence_database_migrates_without_losing_snapshots(tmp_path):
    import sqlite3
    db=sqlite3.connect(tmp_path/'evidence.sqlite3')
    db.execute('CREATE TABLE snapshots(track_id INTEGER,stamp_ns INTEGER,sha256 TEXT,PRIMARY KEY(track_id,stamp_ns,sha256))')
    db.execute("INSERT INTO snapshots VALUES(1,100,'old')");db.commit();db.close()
    store=DetectionEvidenceStore(tmp_path)
    assert store.db.execute('SELECT sha256,appearance FROM snapshots').fetchall()==[('old',None)]
    store.close()
