import importlib.util
import json
from pathlib import Path
import pytest

path=Path(__file__).parents[1]/'scripts/run_static_regression.py'
spec=importlib.util.spec_from_file_location('static_regression',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def suite(root):
    (root/'plan.json').write_text(json.dumps(dict(profile_sha256='same',cases=[dict(name=n,scenario=s) for n,s in module.case_specs(2)])))
    for name,scenario in module.case_specs(2):
        d=root/name;d.mkdir()
        audit=dict(success=True,profile_sha256='same',static_scene=dict(scenario=scenario))
        (d/'summary.json').write_text(json.dumps(dict(success=True,static_scenario=scenario,runner_exit_code=0,audit=audit)))
        (d/'suite_runner_exit.json').write_text('{"exit_code":0}')


def test_all_six_independent_cases_are_required(tmp_path):
    suite(tmp_path)
    assert module.summarize(tmp_path,2)['success']
    (tmp_path/'narrow_2/summary.json').unlink()
    r=module.summarize(tmp_path,2)
    assert not r['success'] and r['passed']==5 and r['total']==6


@pytest.mark.parametrize('bad',['outer_exit','inner_exit','audit','geometry','configuration'])
def test_aggregate_rejects_partial_or_wrong_case_success(tmp_path,bad):
    suite(tmp_path);directory=tmp_path/'dead_end_1'
    p=directory/'summary.json';s=json.loads(p.read_text())
    if bad=='outer_exit':(directory/'suite_runner_exit.json').write_text('{"exit_code":1}')
    elif bad=='inner_exit':s['runner_exit_code']=1
    elif bad=='audit':s['audit']['success']=False
    elif bad=='geometry':s['audit']['static_scene']['scenario']='narrow'
    elif bad=='configuration':s['audit']['profile_sha256']='different'
    p.write_text(json.dumps(s))
    assert not module.summarize(tmp_path,2)['success']


def test_scene_plan_cannot_be_silently_reduced(tmp_path):
    suite(tmp_path)
    with pytest.raises(ValueError,match='plan'):module.summarize(tmp_path,1)


def test_source_changes_or_missing_audit_invalidate_success(tmp_path):
    suite(tmp_path)
    plan=tmp_path/'plan.json';value=json.loads(plan.read_text())
    value['source_audit_required']=True;plan.write_text(json.dumps(value))
    assert module.summarize(tmp_path,2)['passed']==0
    for name,_ in module.case_specs(2):
        (tmp_path/name/'source_audit.json').write_text('{"unchanged":true}')
    assert module.summarize(tmp_path,2)['success']
    (tmp_path/'narrow_1/source_audit.json').write_text('{"unchanged":false,"changed":["state_machine_node.py"]}')
    assert module.summarize(tmp_path,2)['passed']==5


def test_source_manifest_detects_additions_deletions_and_changes(tmp_path):
    from qwen_robotnav.source_audit import runtime_sources,changed_sources
    root=tmp_path/'创新实践/baseline/g3_integration_eval';root.mkdir(parents=True)
    first=root/'state.py';first.write_text('before')
    before=runtime_sources(tmp_path)
    first.write_text('after');(root/'new.py').write_text('new')
    after=runtime_sources(tmp_path)
    assert len(changed_sources(before,after))==2
    first.unlink()
    assert any(name.endswith('state.py') for name in changed_sources(after,runtime_sources(tmp_path)))
    # Controller behavior now depends on compiled sources and build settings.
    for filename in ('return_controller.cpp','return_phase.hpp','CMakeLists.txt'):
        (root/filename).write_text('before')
    before=runtime_sources(tmp_path)
    for filename in ('return_controller.cpp','return_phase.hpp','CMakeLists.txt'):
        (root/filename).write_text('after')
    assert len(changed_sources(before,runtime_sources(tmp_path)))==3
