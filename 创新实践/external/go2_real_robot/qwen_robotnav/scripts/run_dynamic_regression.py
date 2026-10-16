"""Run isolated cart crossing and temporary-route-blocking Nav2 benchmarks."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import uuid

CASES=('blocked','crossing')


def summarize(root, codes):
    cases=[]
    for name in CASES:
        path=Path(root)/name/'dynamic_validation.json'
        audit=json.loads(path.read_text()) if path.is_file() else {}
        cases.append(dict(name=name,runner_exit_code=codes.get(name),
            success=codes.get(name)==0 and audit.get('success') is True,
            audit=audit,directory=str(Path(root)/name)))
    result=dict(success=all(c['success'] for c in cases),cases=cases,
                scope='deterministic simulator Nav2 benchmark; no hardware, person tracking or DeepSeek acceptance')
    (Path(root)/'summary.json').write_text(json.dumps(result,indent=2));return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output_dir',type=Path)
    p.add_argument('--start-domain',type=int,default=226)
    a=p.parse_args();root=a.output_dir.resolve()
    if not 1<=a.start_domain<=231 or {a.start_domain,a.start_domain+1} & {42,43}:
        p.error('Require two valid isolated ROS domains')
    if root.exists():p.error('Choose a fresh output directory')
    root.mkdir(parents=True);script=Path(__file__).with_name('run_slam_frontier.sh')
    # Both trials use the same captured launcher source.
    body=script.read_text();codes={}
    for i,name in enumerate(CASES):
        domain=a.start_domain+i
        env=dict(os.environ,SLAM_DYNAMIC_OBSTACLE=name,SLAM_ROS_DOMAIN_ID=str(domain),
            SLAM_IGN_PARTITION=f'dynamic_{domain}_{uuid.uuid4().hex[:8]}',SLAM_RECON_TASK='0',
            SLAM_MISSION_BRAIN='0',SLAM_PERCEPTION='0',SLAM_RECON_SCENARIO='single',
            SLAM_ACTIVE_OBSERVATION='0',SLAM_INJECT_RETURN_HOLD_S='0')
        print(json.dumps(dict(case=name,domain=domain,phase='starting')),flush=True)
        with (root/(name+'_runner.log')).open('w') as stream:
            child=subprocess.Popen(['bash','-c',body,str(script),str(root/name),'1'],env=env,
                stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            try:codes[name]=child.wait()
            except KeyboardInterrupt:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=45)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=10)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                raise
        summary=summarize(root,codes);print(json.dumps(summary['cases'][i]),flush=True)
    return 0 if summary['success'] else 1


if __name__=='__main__':raise SystemExit(main())
