"""Launch the ordinary warehouse trial using the standalone configured runtime."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import uuid
from qwen_robotnav.runtime_profile import load_profile


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output_dir',type=Path)
    p.add_argument('--candidate-selector',choices=('deepseek','deterministic'))
    p.add_argument('--mission-instruction')
    p.add_argument('--autonomous-completion',action='store_true')
    p.add_argument('--domain',type=int,default=226);p.add_argument('--goals',type=int,default=2)
    p.add_argument('--profile',type=Path,default=Path(__file__).parents[1]/'config/sim_runtime.yaml');a=p.parse_args()
    load_profile(a.profile);root=a.output_dir.resolve()
    if root.exists() or not str(root).isascii() or a.domain in (0,42,43) or not 1<=a.domain<=232 or a.goals<1:p.error('Require fresh ASCII output, isolated domain and positive goals')
    root.mkdir(parents=True);script=Path(__file__).with_name('run_slam_frontier.sh')
    env=dict(os.environ,SLAM_RUNTIME_AUTONOMOUS_COMPLETION='1' if a.autonomous_completion else '0',SLAM_LEGACY_VALIDATION='0',SLAM_RUNTIME_INTERACTIVE='0',
      SLAM_RUNTIME_CANDIDATE_SELECTOR=a.candidate_selector or '',SLAM_MISSION_INSTRUCTION=a.mission_instruction or '',SLAM_RUNTIME_PROFILE=str(a.profile.resolve()),SLAM_ROS_DOMAIN_ID=str(a.domain),
      SLAM_IGN_PARTITION='runtime_'+uuid.uuid4().hex[:12],SLAM_RECON_TASK='0',SLAM_MISSION_BRAIN='0',SLAM_PERCEPTION='0',
      SLAM_RECON_SCENARIO='single',SLAM_ACTIVE_OBSERVATION='0',SLAM_JOINT_RECON='0',SLAM_DYNAMIC_OBSTACLE='',SLAM_FAULT_RECOVERY='',SLAM_INJECT_RETURN_HOLD_S='0')
    with (root/'runner.log').open('w') as stream:
        child=subprocess.Popen(['bash','-c',script.read_text(),str(script),str(root/'trial'),str(a.goals)],env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:code=child.wait()
        except KeyboardInterrupt:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=45)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=15)
            raise
    audit=root/'trial/runtime_validation.json';value=json.loads(audit.read_text()) if audit.exists() else {}
    result=dict(success=code==0 and value.get('success') is True,runner_exit_code=code,audit=value)
    (root/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
