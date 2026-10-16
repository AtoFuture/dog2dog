"""Persistent detection observations and independently delivered JPEG snapshots.

Track IDs are not evidence IDs. Snapshot messages lack source and bbox fields;
ambiguous (especially id=0) matches must remain explicitly unresolved.
"""
from __future__ import annotations
import hashlib
from functools import wraps
from threading import RLock
import json
import math
from pathlib import Path
import sqlite3


def _locked(method):
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return wrapped


class DetectionEvidenceStore:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self._lock = RLock()
        self.db = sqlite3.connect(self.directory/'evidence.sqlite3', check_same_thread=False)
        self.db.executescript('''
            CREATE TABLE IF NOT EXISTS observations (
                evidence_id INTEGER PRIMARY KEY, track_id INTEGER, stamp_ns INTEGER,
                source TEXT, payload TEXT);
            CREATE TABLE IF NOT EXISTS snapshots (
                track_id INTEGER, stamp_ns INTEGER, sha256 TEXT, appearance TEXT,
                PRIMARY KEY(track_id, stamp_ns, sha256));
        ''')
        if 'appearance' not in {row[1] for row in self.db.execute('PRAGMA table_info(snapshots)')}:
            self.db.execute('ALTER TABLE snapshots ADD COLUMN appearance TEXT')

    @_locked
    def add_detection(self, value):
        value = dict(value)
        if not all(math.isfinite(float(v)) for v in value['position_m']):
            raise ValueError('non-finite detection position')
        if len(value['position_m']) != 3 or not 0 <= float(value['confidence']) <= 1:
            raise ValueError('invalid detection geometry/confidence')
        track, stamp, source = int(value['track_id']), int(value['stamp_ns']), str(value['source'])
        if track < 0 or stamp < 0 or source not in ('coco', 'pose', 'heuristic'):
            raise ValueError('invalid detection identity')
        if not isinstance(value['class_name'], str) or not 1 <= len(value['class_name']) <= 128:
            raise ValueError('invalid class name')
        with self.db:
            if track != 0:
                existing = self.db.execute('SELECT evidence_id FROM observations WHERE track_id=? AND stamp_ns=? AND source=?', (track,stamp,source)).fetchone()
                if existing:
                    return existing[0]
            # Zero is untracked: different objects in one frame must not merge.
            cursor = self.db.execute('INSERT INTO observations(track_id,stamp_ns,source,payload) VALUES(?,?,?,?)',
                                     (track,stamp,source,json.dumps(value, allow_nan=False)))
        return cursor.lastrowid

    @_locked
    def add_snapshot(self, track_id, stamp_ns, jpeg, appearance=None):
        from g3_integration_eval.person_appearance import valid_descriptor
        if appearance is not None and not valid_descriptor(appearance):
            raise ValueError('invalid appearance descriptor')
        jpeg = bytes(jpeg)
        if not 4 <= len(jpeg) <= 4*1024*1024 or not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
            raise ValueError('invalid JPEG snapshot')
        digest = hashlib.sha256(jpeg).hexdigest()
        image = self.directory/(digest+'.jpg')
        if not image.exists():
            temporary = image.with_suffix('.tmp')
            temporary.write_bytes(jpeg)
            temporary.replace(image)
        with self.db:
            self.db.execute('INSERT INTO snapshots(track_id,stamp_ns,sha256,appearance) VALUES(?,?,?,?) '
                            'ON CONFLICT(track_id,stamp_ns,sha256) DO UPDATE SET appearance=COALESCE(excluded.appearance,snapshots.appearance)',
                            (int(track_id),int(stamp_ns),digest,json.dumps(appearance) if appearance is not None else None))

    @_locked
    def export(self):
        values = []
        for evidence_id,track,stamp,source,payload in self.db.execute('SELECT * FROM observations ORDER BY evidence_id'):
            value = json.loads(payload)
            hashes = [row[0] for row in self.db.execute('SELECT sha256 FROM snapshots WHERE track_id=? AND stamp_ns=?', (track,stamp))]
            # id=0 carries no identity. Never temporarily claim a match before
            # additional same-frame objects have arrived on the separate topic.
            ambiguous = len(hashes)>1 or track==0
            status = 'not_requested' if not value['has_snapshot'] else 'ambiguous' if ambiguous else 'available' if hashes else 'missing'
            value.update(evidence_id=evidence_id, snapshot_status=status,
                         snapshot_file=hashes[0]+'.jpg' if status=='available' else None,
                         frame_id='map', position_validation='detector output; no independent ground-truth check')
            if status == 'available':
                row = self.db.execute('SELECT appearance FROM snapshots WHERE track_id=? AND stamp_ns=? AND sha256=?', (track,stamp,hashes[0])).fetchone()
                if row and row[0]:value['appearance'] = json.loads(row[0])
            values.append(value)
        result = dict(observation_count=len(values), observations=values,
                      ready_evidence_ids=[v['evidence_id'] for v in values if v['snapshot_status']=='available'])
        temporary = self.directory/'evidence.json.tmp'
        temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        temporary.replace(self.directory/'evidence.json')
        return result

    @_locked
    def close(self):
        self.db.close()
