"""Run simultaneous robot/person motion, cart crossing and autonomous G3 return."""
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
    p.add_argument('--person-model',type=Path,required=True);p.add_argument('--domain',type=int,default=229)
    p.add_argument('--same-clothing',action='store_true',help='Hard same-appearance stress case; baseline uses different clothing materials')
    p.add_argument('--scenario',choices=('moving_parallel','moving_crossing'),default='moving_parallel')
    p.add_argument('--exploration-timeout',type=float,default=300.)
    p.add_argument('--legacy-validation',action='store_true',help='Explicitly reproduce frozen historical experiments')
    a=p.parse_args()
    if not a.legacy_validation:p.error('Frozen historical experiment: add --legacy-validation to reproduce; current runtime is run_navigation_runtime.py')
    root=a.output_dir.resolve()
    if not 1<=a.domain<=232 or a.domain in (42,43):p.error('Choose an isolated valid domain')
    if root.exists() or not str(root).isascii():p.error('Choose a fresh ASCII output directory')
    if not (a.person_model/'model.sdf').is_file():p.error('Person model missing')
    if not 60<=a.exploration_timeout<=600:p.error('Search deadline must be between 60 and 600 wall seconds')
    root.mkdir(parents=True);script=Path(__file__).with_name('run_slam_frontier.sh');body=script.read_text()
    env=dict(os.environ,SLAM_LEGACY_VALIDATION='1',SLAM_RUNTIME_PROFILE='',SLAM_JOINT_RECON='1',SLAM_JOINT_SAME_CLOTHING='1' if a.same_clothing else '0',SLAM_DYNAMIC_OBSTACLE='crossing',SLAM_ROS_DOMAIN_ID=str(a.domain),
        SLAM_IGN_PARTITION=f'joint_{a.domain}_{uuid.uuid4().hex[:8]}',SLAM_RECON_TASK='1',SLAM_MISSION_BRAIN='1',
        SLAM_PERCEPTION='1',SLAM_RECON_SCENARIO=a.scenario,SLAM_RECON_PERSON_MODEL=str(a.person_model.resolve()),
        SLAM_RECON_REQUIRED_PERSONS='2',SLAM_RECON_IDENTITY_POLICY='motion_appearance_v1',SLAM_ACTIVE_OBSERVATION='0',
        SLAM_INJECT_RETURN_HOLD_S='0',SLAM_EXPLORATION_TIMEOUT=str(a.exploration_timeout))
    (root/'launch_manifest.json').write_text(json.dumps(dict(domain=a.domain,scenario=a.scenario,same_clothing=a.same_clothing,
        ign_partition=env['SLAM_IGN_PARTITION'],launcher_sha256=hashlib.sha256(body.encode()).hexdigest(),
        scope='joint real perception/navigation simulator trial; no hardware'),indent=2))
    with (root/'launcher.log').open('w') as stream:
        child=subprocess.Popen(['bash','-c',body,str(script),str(root),'1'],env=env,
                               stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:code=child.wait()
        except KeyboardInterrupt:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=45)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
            raise
    path=root/'joint_validation.json';audit=json.loads(path.read_text()) if path.exists() else {}
    result=dict(success=code==0 and audit.get('success') is True,runner_exit_code=code,audit=audit)
    (root/'joint_result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
