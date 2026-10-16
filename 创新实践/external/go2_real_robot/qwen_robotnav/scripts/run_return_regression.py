"""Run independent moving-person RETURN trials and a bounded zero-command fault."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import uuid


CASE_SPECS=[('crossing','moving_crossing',0.),('parallel','moving_parallel',0.),('return_hold','moving_crossing',18.)]


def summarize(root,start_domain,exit_codes=None):
    root=Path(root).resolve()
    prior=json.loads((root/'summary.json').read_text()) if (root/'summary.json').is_file() else {}
    codes={c['name']:c.get('runner_exit_code') for c in prior.get('cases',[])}
    codes.update(exit_codes or {})
    results=[]
    for index,(name,scene,hold) in enumerate(CASE_SPECS):
        directory=root/name
        def read(filename):
            path=directory/filename
            return json.loads(path.read_text()) if path.is_file() else {}
        mission=read('moving_validation.json');recovery=read('return_recovery_validation.json');report=read('report.json')
        hold_ok=(not hold or (recovery.get('hold_injection_success') is True and report.get('return_hold_s')==hold))
        return_ok=recovery.get('success') is True and hold_ok
        results.append(dict(name=name,domain=start_domain+index,runner_exit_code=codes.get(name),
            return_recovery_success=return_ok,
            full_mission_success=return_ok and mission.get('success') is True and codes.get(name)==0,
            identity_success=mission.get('identity_validation_success'),return_success=recovery.get('return_success'),
            hold_injection_success=recovery.get('hold_injection_success'),
            truth_home_error_m=recovery.get('truth_home_error_m'),directory=str(directory)))
    result=dict(return_regression_success=all(c['return_recovery_success'] for c in results),
        full_mission_success=all(c['full_mission_success'] for c in results),cases=results,
        scope='RETURN reliability and fault recovery; full moving-person mission acceptance is reported separately')
    (root/'summary.json').write_text(json.dumps(result,indent=2))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output_dir',type=Path)
    parser.add_argument('--person-model',type=Path,required=True)
    parser.add_argument('--start-domain',type=int,default=224)
    args=parser.parse_args();root=args.output_dir.resolve()
    if not 1<=args.start_domain<=230 or set(range(args.start_domain,args.start_domain+3)) & {42,43}:
        parser.error('Choose three isolated valid ROS domains')
    if root.exists():parser.error('Choose a fresh suite output directory')
    if not (args.person_model/'model.sdf').is_file():parser.error('Person model missing')
    root.mkdir(parents=True);script=Path(__file__).with_name('run_slam_frontier.sh')
    exit_codes={}
    for index,(name,scene,hold) in enumerate(CASE_SPECS):
        directory=root/name;domain=args.start_domain+index
        env=dict(os.environ,SLAM_RECON_TASK='1',SLAM_PERCEPTION='1',SLAM_MISSION_BRAIN='1',
            SLAM_RECON_SCENARIO=scene,SLAM_RECON_IDENTITY_POLICY='motion_appearance_v1',
            SLAM_RECON_REQUIRED_PERSONS='2',SLAM_ACTIVE_OBSERVATION='0',
            SLAM_RECON_PERSON_MODEL=str(args.person_model.resolve()),SLAM_ROS_DOMAIN_ID=str(domain),
            SLAM_IGN_PARTITION=f'return_reg_{domain}_{uuid.uuid4().hex[:8]}',SLAM_INJECT_RETURN_HOLD_S=str(hold))
        print(json.dumps(dict(case=name,domain=domain,phase='starting')),flush=True)
        with (root/f'{name}_runner.log').open('w') as stream:
            # Keep this shell invocation immutable while source edits continue.
            child=subprocess.Popen(['bash','-c',script.read_text(),str(script),str(directory),'1'],env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            try:code=child.wait()
            except KeyboardInterrupt:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=45)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=10)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                raise
        exit_codes[name]=code
        summary=summarize(root,args.start_domain,exit_codes)
        print(json.dumps(summary['cases'][index]),flush=True)
    return 0 if summary['return_regression_success'] else 1



if __name__=='__main__':raise SystemExit(main())
