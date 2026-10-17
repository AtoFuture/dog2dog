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
