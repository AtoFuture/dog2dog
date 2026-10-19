"""Stop only registered child sessions and their descendants on Linux."""
import argparse
import os
from pathlib import Path
import signal
import time
import re


def processes():
    result = {}
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = path.read_text().rsplit(')', 1)[1].split()
            # state, ppid, pgrp, session, ..., starttime (field 22).
            result[int(path.parent.name)] = (fields[0], int(fields[1]), int(fields[3]), fields[19])
        except (OSError, ValueError, IndexError):
            pass
    return result


def stop_owned(roots, grace=5.0, partition=None):
    if partition is not None and not re.fullmatch(r'runtime_[0-9a-f]{12}',partition):
        raise ValueError('Partition cleanup requires a private runtime UUID')
    roots = {int(pid) for pid in roots if int(pid) > 1}
    table = processes()
    ancestors = {os.getpid()}
    parent = os.getppid()
    while parent in table and parent not in ancestors:
        ancestors.add(parent)
        parent = table[parent][1]
    if roots & ancestors:
        raise ValueError('Refusing to stop the caller or an ancestor')
    # Session membership retains children after a short-lived wrapper exits.
    owned = {pid for pid, row in table.items() if pid in roots or row[2] in roots}
    while True:
        children = {pid for pid, row in table.items() if row[1] in owned}
        if children <= owned:
            break
        owned |= children
    identities = {pid: table[pid][3] for pid in owned - ancestors}
    current_signal = None

    def tagged(pid):
        if partition is None or pid in ancestors:return False
        try:
            env=Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
        except OSError:return False
        return any(entry in env for entry in (b'IGN_PARTITION='+partition.encode(),b'SLAM_IGN_PARTITION='+partition.encode()))

    def live():
        current = processes()
        for pid,row in current.items():
            if pid not in identities and tagged(pid):
                identities[pid]=row[3]
                if current_signal is not None:
                    try:os.kill(pid,current_signal)
                    except ProcessLookupError:pass
        return [pid for pid, started in identities.items()
                if pid in current and current[pid][3] == started and current[pid][0] != 'Z']

    for sig, wait in [(signal.SIGINT, grace), (signal.SIGTERM, 1.0), (signal.SIGKILL, 0)]:
        current_signal = sig
        for pid in live():
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + wait
        while live() and time.monotonic() < deadline:
            time.sleep(.05)
        if not live():
            break


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pids', nargs='*', type=int)
    parser.add_argument('--grace', type=float, default=5.0)
    parser.add_argument('--partition')
    args = parser.parse_args()
    stop_owned(args.pids, args.grace,args.partition)
