"""Run moving sensor-fault trials in an isolated Gazebo domain."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import uuid


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output_dir',type=Path)
    p.add_argument('--mode',choices=('short','long','both'),default='both')
    p.add_argument('--start-domain',type=int,default=229)
    p.add_argument('--legacy-validation',action='store_true');a=p.parse_args()
    if not a.legacy_validation:p.error('Historical fault harness requires --legacy-validation; current runtime uses independent velocity_guard')
    cases=['short','long'] if a.mode=='both' else [a.mode]
    if not 1<=a.start_domain<=232-len(cases)+1 or set(range(a.start_domain,a.start_domain+len(cases))) & {42,43}:p.error('Choose isolated valid ROS domains')
    root=a.output_dir.resolve()
    if root.exists() or not str(root).isascii():p.error('Choose a fresh ASCII output directory')
    root.mkdir(parents=True);script=Path(__file__).with_name('run_slam_frontier.sh');body=script.read_text();results=[]
    sources=[script,*(script.parent/name for name in ['validate_slam_frontier.py','fault_recovery_trial.py','evaluate_fault_recovery.py'])]
    (root/'source_manifest.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},indent=2))
    for i,mode in enumerate(cases):
        env=dict(os.environ,SLAM_LEGACY_VALIDATION='1',SLAM_RUNTIME_PROFILE='',SLAM_FAULT_RECOVERY=mode,SLAM_ROS_DOMAIN_ID=str(a.start_domain+i),
            SLAM_IGN_PARTITION='fault_'+uuid.uuid4().hex[:12],SLAM_RECON_TASK='0',SLAM_RECON_SCENARIO='single',
            SLAM_MISSION_BRAIN='0',SLAM_PERCEPTION='0',SLAM_JOINT_RECON='0',SLAM_ACTIVE_OBSERVATION='0',
            SLAM_INJECT_RETURN_HOLD_S='0',SLAM_DYNAMIC_OBSTACLE='')
        print(json.dumps(dict(mode=mode,domain=a.start_domain+i,phase='starting')),flush=True)
        with (root/(mode+'_runner.log')).open('w') as stream:
            child=subprocess.Popen(['bash','-c',body,str(script),str(root/mode),'2'],env=env,
                stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            try:code=child.wait()
            except KeyboardInterrupt:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=45)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=15)
                raise
        path=root/mode/'fault_validation.json';audit=json.loads(path.read_text()) if path.exists() else {}
        results.append(dict(mode=mode,exit_code=code,success=code==0 and audit.get('success') is True,audit=audit))
        summary=dict(success=len(results)==len(cases) and all(r['success'] for r in results),cases=results)
        (root/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(results[-1]),flush=True)
    return 0 if summary['success'] else 1


if __name__=='__main__':raise SystemExit(main())
