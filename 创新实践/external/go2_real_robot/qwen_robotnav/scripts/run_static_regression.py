"""Repeat static Gazebo cases sequentially through the one formal navigation runtime."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import yaml
from qwen_robotnav.runtime_profile import load_profile,profile_hash
from qwen_robotnav.static_scenarios import SCENARIOS


def case_specs(repeats):
    return [(f'{name}_{repeat+1}',name) for repeat in range(repeats) for name in SCENARIOS]


def summarize(root, repeats):
    root=Path(root)
    plan=json.loads((root/'plan.json').read_text())
    expected=case_specs(repeats)
    if plan['cases'] != [dict(name=name,scenario=scenario) for name,scenario in expected]:
        raise ValueError('Suite plan differs from requested case matrix')
    cases=[]
    for name,scenario in expected:
        directory=root/name
        path=directory/'summary.json'
        summary=json.loads(path.read_text()) if path.is_file() else {}
        status=directory/'suite_runner_exit.json'
        runner=json.loads(status.read_text()).get('exit_code') if status.is_file() else None
        audit=summary.get('audit',{})
        success=(runner==0 and summary.get('runner_exit_code')==0 and summary.get('success') is True
                 and summary.get('static_scenario')==scenario and audit.get('success') is True
                 and (audit.get('static_scene') or {}).get('scenario')==scenario
                 and audit.get('profile_sha256')==plan['profile_sha256'])
        cases.append(dict(name=name,scenario=scenario,success=success,runner_exit_code=runner,
            end_reason=(audit.get('exploration_results') or [{}])[0].get('reason'),
            home_error_m=audit.get('home_error_m'),completed_frontiers=audit.get('completed_frontiers'),
            deepseek_selections=audit.get('deepseek_selections'),brain_fallback_selections=audit.get('brain_fallback_selections'),
            known_area_growth_m2=audit.get('known_area_growth_m2'),static_scene=audit.get('static_scene'),directory=str(directory)))
    result=dict(success=all(c['success'] for c in cases),passed=sum(c['success'] for c in cases),
                total=len(cases),repeats=repeats,cases=cases,
                scope='Independent static Gazebo fixtures; sequential repetitions, not a statistical reliability estimate.')
    (root/'summary.json').write_text(json.dumps(result,indent=2))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('output_dir',type=Path)
    p.add_argument('--repeats',type=int,default=2)
    p.add_argument('--domain',type=int,default=226)
    p.add_argument('--profile',type=Path,default=Path(__file__).parents[1]/'config/sim_runtime.yaml')
    p.add_argument('--candidate-selector',choices=('deepseek','deterministic'),default='deepseek')
    p.add_argument('--summarize-only',action='store_true')
    a=p.parse_args();root=a.output_dir.resolve()
    if not 1<=a.repeats<=5 or a.domain in (0,42,43) or not 1<=a.domain<=232:p.error('Require 1..5 repeats and isolated domain')
    if a.summarize_only:
        result=summarize(root,a.repeats);print(json.dumps(result,indent=2));return 0 if result['success'] else 1
    if root.exists() or not str(root).isascii():p.error('Choose a fresh ASCII output directory')
    profile=load_profile(a.profile)
    profile['decision']['candidate_selector']=a.candidate_selector
    profile['explorer'].update(exploration_budget_s=240.,coverage_stall_s=180.,candidate_wait_s=45.)
    # Same profile for every case; only geometry and case-specific assertions differ.
    root.mkdir(parents=True)
    profile_path=root/'profile.yaml';profile_path.write_text(yaml.safe_dump(profile,sort_keys=False))
    plan=dict(schema_version=1,domain=a.domain,profile_sha256=profile_hash(profile),
              cases=[dict(name=name,scenario=scenario) for name,scenario in case_specs(a.repeats)])
    (root/'plan.json').write_text(json.dumps(plan,indent=2))
    script=Path(__file__).with_name('run_runtime_trial.py')
    for name,scenario in case_specs(a.repeats):
        print(json.dumps(dict(case=name,phase='starting')),flush=True)
        with (root/(name+'.log')).open('w') as stream:
            child=subprocess.Popen([sys.executable,str(script),str(root/name),'--profile',str(profile_path),
                '--domain',str(a.domain),'--static-scenario',scenario,'--autonomous-completion'],
                stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            try:code=child.wait()
            except KeyboardInterrupt:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=60)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=15)
                raise
        # Outer exit code is independent of the inner reported success.
        directory=root/name;directory.mkdir(exist_ok=True)
        (directory/'suite_runner_exit.json').write_text(json.dumps(dict(exit_code=code)))
        result=summarize(root,a.repeats)
        print(json.dumps(next(c for c in result['cases'] if c['name']==name)),flush=True)
    return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
